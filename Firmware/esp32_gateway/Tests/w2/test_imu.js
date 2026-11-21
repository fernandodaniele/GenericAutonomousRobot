async function runImuOrientationTests(source) {
  const check=(v,s)=>{if(!v)throw new Error(s);};
  const api=new Function("let lastStatus=null;\n"+source+"\nreturn {imuValidate,createImuPanel,setStatus:s=>{lastStatus=s;}};")();
  const good=()=>({received:true,connected:true,stm32_connected:true,paused:false,valid:true,stale:false,state:"valid",hal_status:0,i2c_error:0,
    response_age_ms:0,sample_age_ms:0,max_age_ms:1500,accel_g:[0,0,1],gyro_dps:[0,0,0],gyro_raw:[0,0,0],gyro_corrected_dps:[0,0,0],
    calibration:{available:true,state:"valid",valid:true,reason:"none",samples:250,target_samples:250,request:7,age_ms:100,offset_dps:[0,0,0],request_phase:"accepted"},
    orientation:{available:true,state:"valid",valid:true,requires_calibration:false,reason:"none",degrees:450,sample_age_ms:0,reference_tick:0,reference_request:7},
    comparison:{state:"valid",valid:true,sample_tick:80,sample_age_ms:20,gyro_delta_deg:88,encoder_delta_deg:87,difference_deg:1,encoder_reference_deg:-30,encoder_calibration:1}});
  const cases=[];
  let time=0,id=0,data=good(),pending=false;const nodes=new Map(),timers=new Map(),requests=[];
  const doc={getElementById(name){if(!nodes.has(name))nodes.set(name,{textContent:"",disabled:false,addEventListener(){}});return nodes.get(name);}};
  class Controller {constructor(){this.signal={};}abort(){this.signal.reject?.(new Error("timeout"));}}
  const fetcher=async(url,options)=>{
    check(url==="/api/imu"&&!options.method&&!options.body,"orientation never commands movement or resets odometry");requests.push(url);
    if(pending)return new Promise((resolve,reject)=>{options.signal.reject=reject;});
    return {ok:true,json:async()=>data};
  };
  const flush=async()=>{for(let i=0;i<16;++i)await Promise.resolve();};
  const panel=api.createImuPanel(doc,fetcher,()=>time,(fn,delay)=>{timers.set(++id,{fn,at:time+delay});return id;},n=>timers.delete(n),Controller);
  const text=name=>doc.getElementById(name).textContent;
  async function advance(ms){const end=time+ms;while(true){const next=[...timers].sort((a,b)=>a[1].at-b[1].at)[0];if(!next||next[1].at>end)break;time=next[1].at;timers.delete(next[0]);next[1].fn();await flush();}time=end;await flush();}
  check(api.imuValidate(data),"IMU-3 valid schema");panel.start();await flush();
  check(text("imu-yaw").includes("450.00°")&&text("imu-reference").includes("tick STM32 0")&&
    text("imu-common-angles").includes("88.00° / Encoders 87.00°")&&text("imu-angle-difference").includes("1.00°")&&text("imu-angle-difference").includes("provisional"),"common reference comparison, not 450 minus encoder origin");
  cases.push("IMU-3 accumulated gyro yaw, legitimate reference tick zero and same-origin deltas render separately with provisional encoder label");
  data=good();data.orientation.degrees=-90;data.comparison={...data.comparison,state:"odometry_unavailable",valid:false,sample_tick:null,sample_age_ms:null,
    gyro_delta_deg:null,encoder_delta_deg:null,difference_deg:null,encoder_reference_deg:null,encoder_calibration:0};
  await advance(500);check(text("imu-yaw").includes("-90.00°")&&text("imu-common-angles").includes("Sin pose")&&text("imu-angle-difference")==="Sin comparación vigente","negative yaw without encoder pose");
  cases.push("IMU-3 unavailable encoder association does not suppress useful gyro yaw or fabricate an angular error");
  data=good();data.orientation={...data.orientation,state:"invalid",valid:false,requires_calibration:true,reason:"timing",degrees:null,sample_age_ms:500};
  data.comparison={...data.comparison,state:"yaw_unavailable",valid:false,sample_tick:null,sample_age_ms:null,gyro_delta_deg:null,encoder_delta_deg:null,difference_deg:null,encoder_reference_deg:null,encoder_calibration:0};
  await advance(500);check(text("imu-yaw").includes("calibrar nuevamente")&&!text("imu-yaw").includes("450.00"),"invalid timing needs explicit calibration");
  data=good();data.orientation={...data.orientation,degrees:0,reference_tick:5000};await advance(500);
  check(text("imu-yaw").includes("0.00°")&&text("imu-reference").includes("5000"),"new calibration reference shown");
  cases.push("IMU-3 invalid acquisition continuity hides previous yaw and successful recalibration visibly changes its reference");
  pending=true;await advance(1700);check(!text("imu-yaw").includes("0.00°")&&text("imu-angle-difference")==="Sin comparación vigente","browser aging hides stale angles");
  pending=false;await advance(1000);api.setStatus({connected:false});await advance(100);
  check(!text("imu-yaw").includes("0.00°"),"STATUS disconnect invalidates orientation immediately");
  api.setStatus(null);cases.push("IMU-3 paused/unanswered telemetry and STATUS loss cannot retain apparently current angles or resume commands");
  for(const change of [s=>{s.orientation.degrees=NaN;},s=>{s.orientation.reference_request=8;},s=>{s.orientation.valid=false;},
    s=>{s.comparison.encoder_calibration=0;},s=>{s.comparison.difference_deg=181;},s=>{s.comparison.sample_tick=null;},
    s=>{s.orientation.requires_calibration=true;},s=>{s.orientation.reference_tick=null;},s=>{delete s.calibration;}]) {
    const s=good();change(s);check(!api.imuValidate(s),"inconsistent orientation schema rejected");
  }
  check(requests.length>1,"automatic polling retained");
  cases.push("IMU-3 web schema rejects nonfinite angles, unrelated references, impossible validity/calibration and out-of-range comparisons");
  return cases;
}

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

async function runImuCalibrationTests(source) {
  const check=(v,s)=>{if(!v)throw new Error(s);};
  const api=new Function("let lastStatus=null;\n"+source+"\nreturn {imuValidate,createImuPanel,setStatus:s=>{lastStatus=s;}};")();
  const base=()=>({received:true,connected:true,stm32_connected:true,paused:false,valid:true,stale:false,state:"valid",hal_status:0,i2c_error:0,
    response_age_ms:0,sample_age_ms:0,max_age_ms:1500,accel_g:[0,0,1],gyro_dps:[1,-2,0.5],gyro_raw:[131,-262,66],gyro_corrected_dps:[null,null,null],
    calibration:{available:true,state:"pending",valid:false,reason:"none",samples:0,target_samples:250,request:0,age_ms:null,offset_dps:[null,null,null],request_phase:"idle"}});
  const cases=[];
  async function fixture() {
    let time=0,id=0,data=base(),postMode="ok";const nodes=new Map(),timers=new Map(),requests=[];
    const doc={getElementById(name){if(!nodes.has(name))nodes.set(name,{textContent:"",disabled:false,events:{},addEventListener(n,fn){this.events[n]=fn;}});return nodes.get(name);}};
    class Controller{constructor(){this.signal={};}abort(){this.signal.reject?.(new Error("timeout"));}}
    const fetcher=async(url,options)=>{
      requests.push({url,options});
      if(url==="/api/imu/calibrate") {
        check(options.method==="POST"&&options.body==="CALIBRATE"&&options.headers["X-Request-ID"].length===32,"closed calibration request only");
        if(postMode==="pending")return new Promise((resolve,reject)=>{options.signal.reject=reject;});
        return {ok:postMode==="ok",json:async()=>({accepted:postMode==="ok",phase:"queued",error:"unsafe"})};
      }
      check(url==="/api/imu"&&!options.method&&!options.body,"only monitoring GET otherwise");
      return {ok:true,json:async()=>data};
    };
    const flush=async()=>{for(let i=0;i<16;++i)await Promise.resolve();};
    const panel=api.createImuPanel(doc,fetcher,()=>time,(fn,delay)=>{timers.set(++id,{fn,at:time+delay});return id;},n=>timers.delete(n),Controller,()=>"1234567890abcdef1234567890abcdef");
    panel.start();await flush();
    async function advance(ms){const end=time+ms;while(true){const next=[...timers].sort((a,b)=>a[1].at-b[1].at)[0];if(!next||next[1].at>end)break;time=next[1].at;timers.delete(next[0]);next[1].fn();await flush();}time=end;await flush();}
    return {node:name=>doc.getElementById(name),requests,advance,setData:v=>{data=v;},setMode:m=>{postMode=m;},click(){return doc.getElementById("imu-calibrate").events.click();}};
  }
  api.setStatus({valid:true,connected:true,state:3,age_ms:0});let f=await fixture();
  check(!f.node("imu-calibrate").disabled&&f.node("imu-calibration").textContent.includes("Pendiente")&&f.requests.length===1,"explicit start only");
  const first=f.click();await f.click();await first;
  check(f.requests.filter(r=>r.options.method==="POST").length===1&&f.node("imu-calibration-message").textContent.includes("no confirma"),"one accepted POST is not success");
  cases.push("IMU-2 button starts one explicit deduplicated request, never calibrates on load or declares success from HTTP acceptance");
  let d=base();d.calibration={...d.calibration,state:"running",samples:125,request:7,request_phase:"accepted"};f.setData(d);await f.advance(500);
  check(f.node("imu-calibrate").disabled&&f.node("imu-calibration").textContent.includes("125/250")&&f.node("imu-gyro-corrected").textContent==="Sin calibración vigente","progress only");
  d=base();d.gyro_corrected_dps=[0,0,0];d.calibration={...d.calibration,state:"valid",valid:true,samples:250,request:7,age_ms:0,offset_dps:[1,-2,0.5],request_phase:"accepted"};
  f.setData(d);await f.advance(500);
  check(f.node("imu-calibration").textContent.includes("Válida")&&f.node("imu-offset").textContent.includes("-2.0000")&&f.node("imu-gyro-corrected").textContent==="0.000 / 0.000 / 0.000"&&f.node("imu-gyro").textContent.includes("1.000")&&f.node("imu-gyro-raw").textContent==="131 / -262 / 66","raw and corrected independent");
  cases.push("IMU-2 shows sample progress and measured offsets, preserves raw angular velocity and adds corrected velocity only after real valid calibration");
  d=base();d.calibration={...d.calibration,state:"failed",reason:"motion",samples:100,request:7,request_phase:"accepted"};f.setData(d);await f.advance(500);
  check(f.node("imu-calibration").textContent.includes("Movimiento")&&!f.node("imu-calibrate").disabled&&f.node("imu-gyro-corrected").textContent==="Sin calibración vigente","failed motion");
  d={...d,state:"read_error",valid:false,stale:true,hal_status:3,i2c_error:32,accel_g:[null,null,null],gyro_dps:[null,null,null],gyro_raw:[null,null,null],calibration:{...d.calibration,reason:"i2c"}};
  f.setData(d);await f.advance(500);check(f.node("imu-calibrate").disabled&&f.node("imu-calibration").textContent.includes("Falla I²C"),"bus failure disables start");
  d=base();d.calibration={...d.calibration,state:"failed",reason:"i2c",samples:100,request:7,request_phase:"accepted"};f.setData(d);await f.advance(500);
  check(!f.node("imu-calibrate").disabled&&f.requests.filter(r=>r.options.method==="POST").length===1,"recovery is explicit");
  cases.push("IMU-2 motion and I2C failures remain visible, recovery never reuses offsets or automatically restarts calibration");
  for(const mode of [1,2]) {api.setStatus({valid:true,connected:true,state:mode,age_ms:0});await f.advance(100);await f.click();check(f.node("imu-calibrate").disabled,"operating mode refused");}
  api.setStatus({valid:true,connected:false,state:3,age_ms:3000});await f.advance(100);check(f.node("imu-calibrate").disabled,"link lost");
  cases.push("IMU-2 mode and freshness controls disable calibration without changing MANUAL or AUTO controls");
  api.setStatus({valid:true,connected:true,state:0,age_ms:0});f=await fixture();f.setMode("pending");const post=f.click();await f.advance(1600);await post;
  check(f.requests.filter(r=>r.options.method==="POST").length===1&&f.node("imu-calibration-message").textContent.includes("No se pudo confirmar"),"timeout no retry");
  cases.push("IMU-2 network timeout does not retry, issue motion, or turn an unconfirmed request into successful calibration");
  const good=base();
  for(const c of [{...good.calibration,state:"valid",samples:249},{...good.calibration,state:"failed"},{...good.calibration,valid:true},
    {...good.calibration,target_samples:0},{...good.calibration,request_phase:"unknown"}])check(!api.imuValidate({...good,calibration:c}),"bad calibration contract rejected");
  check(!api.imuValidate({...good,gyro_raw:[0.5,0,0]}),"raw integer check");
  check(!api.imuValidate({...good,gyro_corrected_dps:[1,2,3]}),"uncalibrated corrected data rejected");
  cases.push("IMU-2 schema rejects inconsistent progress, invalid correction, unknown phases and noninteger raw axes");
  return cases;
}
