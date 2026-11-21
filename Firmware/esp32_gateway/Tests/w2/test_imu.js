async function runImuTests(source) {
  const check=(v,s)=>{if(!v)throw new Error(s);};
  const api=new Function("let lastStatus=null;\n"+source+"\nreturn {imuValidate,createImuPanel,setStatus:s=>{lastStatus=s;}};")();
  const good=()=>({received:true,connected:true,stm32_connected:true,paused:false,valid:true,stale:false,state:"valid",
    hal_status:0,i2c_error:0,response_age_ms:0,sample_age_ms:0,max_age_ms:1500,accel_g:[0,0,1],gyro_dps:[0,-1,2]});
  const cases=[];
  async function fixture() {
    let time=0,id=0,data=good(),mode="ok"; const nodes=new Map(),timers=new Map(),requests=[];
    const doc={getElementById(id){if(!nodes.has(id))nodes.set(id,{textContent:""});return nodes.get(id);}};
    class Controller {constructor(){this.signal={};}abort(){this.signal.reject?.(new Error("timeout"));}}
    const fetcher=async(url,options)=>{
      check(url==="/api/imu"&&!options.method&&!options.body&&options.cache==="no-store","IMU only issues local read-only GET");
      requests.push({url,time});
      if(mode==="offline")throw new Error("offline");
      if(mode==="pending")return new Promise((resolve,reject)=>{options.signal.reject=reject;});
      return {ok:mode!=="http",json:async()=>data};
    };
    const flush=async()=>{for(let i=0;i<15;++i)await Promise.resolve();};
    const panel=api.createImuPanel(doc,fetcher,()=>time,(fn,delay)=>{timers.set(++id,{fn,at:time+delay});return id;},n=>timers.delete(n),Controller);
    panel.start();panel.start();await flush();
    async function advance(ms){const end=time+ms;while(true){const next=[...timers].sort((a,b)=>a[1].at-b[1].at)[0];if(!next||next[1].at>end)break;time=next[1].at;timers.delete(next[0]);next[1].fn();await flush();}time=end;await flush();}
    return {node:id=>doc.getElementById(id).textContent,advance,requests,setData:v=>{data=v;},setMode:v=>{mode=v;}};
  }
  api.setStatus(null); let f=await fixture();
  check(f.requests.length===1&&f.node("imu-accel")==="0.000 / 0.000 / 1.000"&&f.node("imu-gyro").includes("-1.000"),"real units and one startup request");
  cases.push("IMU valid signed readings and units render without commands or duplicate polling");
  f.setData({...good(),state:"read_error",hal_status:3,i2c_error:32,valid:false,stale:true,accel_g:[null,null,null],gyro_dps:[null,null,null],sample_age_ms:500});
  await f.advance(500);
  check(f.node("imu-state").includes("Fallo de lectura")&&f.node("imu-accel")==="Sin lectura vigente"&&f.node("imu-error").includes("32")&&f.node("imu-age").includes("500"),"error hides prior values");
  cases.push("IMU read failure exposes HAL/I2C diagnosis and last-success age without showing old readings as valid");
  for(const state of ["initializing","ready","init_error","no_data"]){
    const received=state!=="no_data";
    f.setData({...good(),received,valid:false,stale:true,connected:received,state,hal_status:!received?null:state==="init_error"?1:0,i2c_error:received?0:null,
      sample_age_ms:null,response_age_ms:received?0:null,accel_g:[null,null,null],gyro_dps:[null,null,null]});
    await f.advance(500);check(f.node("imu-accel")==="Sin lectura vigente"&&f.node("imu-age")==="Sin lectura exitosa","initial states never fabricate zero measurements");
  }
  cases.push("IMU startup, absent sensor, initializing and first-sample wait remain explicit");
  f.setData(good());await f.advance(500);api.setStatus({connected:false});await f.advance(100);
  check(f.node("imu-state").includes("STM32 sin respuesta")&&f.node("imu-gyro")==="Sin lectura vigente","STATUS withdrawal before next IMU poll");
  api.setStatus({connected:true});f.setMode("pending");await f.advance(1700);
  check(f.node("imu-accel")==="Sin lectura vigente","elapsed browser time invalidates unanswered telemetry");
  cases.push("IMU loss detected by STATUS or elapsed acquisition age immediately hides measurements");
  f=await fixture();f.setMode("offline");await f.advance(500);check(f.node("imu-state").includes("No se pudo actualizar")&&f.node("imu-gyro")==="Sin lectura vigente","HTTP loss");
  f.setMode("ok");await f.advance(500);check(f.node("imu-state")==="Lectura válida","new successful response recovers display");
  f.setData({...good(),paused:true,valid:false,stale:true,accel_g:[null,null,null],gyro_dps:[null,null,null],sample_age_ms:1600});
  await f.advance(500);check(f.node("imu-state").includes("pausada")&&f.node("imu-state").includes("desactualizada"),"manual pause and stale are both visible");
  cases.push("IMU network recovery needs fresh data; paused manual telemetry retains age and explicit staleness");
  for(const bad of [{},{...good(),state:"unknown"},{...good(),accel_g:[NaN,0,0]},{...good(),accel_g:[3,0,0]},
    {...good(),gyro_dps:[null,null,null]},{...good(),state:"init_error"},{...good(),sample_age_ms:1501},
    {...good(),valid:false,stale:true},{...good(),i2c_error:1}])check(!api.imuValidate(bad),"malformed IMU schema rejected");
  cases.push("IMU schema rejects malformed, incoherent, nonfinite and out-of-range values");
  return cases;
}
