/* Fuentes reales, Canvas/HTTP/reloj sustituidos. Ninguna lectura fisica. */
async function runMapTests(source) {
  const assert=(value,message)=>{if(!value)throw new Error(message);};
  const {mapValidate,mapToCanvas,createMapPanel}=new Function(source+"\nreturn {mapValidate,mapToCanvas,createMapPanel};")();
  const cases=[],nodes=new Map(),timers=new Map(),requests=[],drawing=[];
  let now=0,id=0,mode="ok",pendingReject;
  const context=new Proxy({}, {get(object,key){if(!(key in object))object[key]=(...args)=>drawing.push({key,args});return object[key];}});
  const node=(name)=>{if(!nodes.has(name))nodes.set(name,{textContent:"",width:640,height:640,getContext:()=>context});return nodes.get(name);};
  const blank=()=>({connected:false,calibrated:false,calibration:"incomplete",epoch:0,pose:{valid:false,state:"no_sample",stale:true,tick:null,age_ms:null,left_counts:null,right_counts:null,
    x_mm:null,y_mm:null,theta_mrad:null,increment_mm:null,increment_mrad:null},rays:[{state:0,age_ms:0},{state:0,age_ms:0}],
    grid:{width:8,height:8,resolution_mm:50,origin_x_mm:-200,origin_y_mm:-200,revision:0,cells:"0".repeat(64)},trajectory_mm:[]});
  let data=blank();
  class Controller {constructor(){this.signal={};}abort(){if(this.signal.reject)this.signal.reject(new Error("timeout"));}}
  async function fetchMock(url,options){
    assert(url==="/api/map"&&options.cache==="no-store"&&!options.method,"read-only local mapping request");requests.push({url,options});
    if(mode==="network")throw new Error("offline");
    if(mode==="pending")return new Promise((resolve,reject)=>{pendingReject=reject;options.signal.reject=reject;});
    return {ok:mode!=="http",json:async()=>mode==="malformed"?{}:data};
  }
  const panel=createMapPanel({getElementById:node},fetchMock,()=>now,(fn,delay)=>{const n=++id;timers.set(n,{fn,delay});return n;},n=>timers.delete(n),Controller);
  const flush=async()=>{for(let i=0;i<12;i++)await Promise.resolve();};
  const fire=(delay)=>{const match=[...timers].filter(([,t])=>t.delay===delay);assert(match.length===1,"one timer per purpose "+delay);const [n,t]=match[0];timers.delete(n);return t.fn();};
  const cycle=async()=>{now+=500;fire(500);await flush();};
  assert(mapValidate(data),"unknown schema");panel.start();panel.start();await flush();
  assert(requests.length===1&&node("map-state").textContent==="Sin muestra"&&!drawing.some(c=>c.key==="translate"),"no simulated robot on load");
  cases.push("W7 no sample renders unknown occupancy and no robot/path; single startup GET and no commands");
  data={...blank(),connected:true,epoch:7,pose:{...blank().pose,state:"uncalibrated",tick:0,age_ms:0,left_counts:"123",right_counts:"-456"}};
  await cycle();assert(node("map-state").textContent.includes("No calibrada")&&node("map-counts").textContent.includes("-456")&&node("map-pose").textContent.includes("no disponible"),"uncalibrated honest display");
  cases.push("W7 actual signed count strings remain visible without inventing meters or calibration");
  data={...data,calibrated:true,calibration:"calibrated",pose:{...data.pose,valid:true,state:"valid",stale:false,x_mm:50,y_mm:100,theta_mrad:1571,increment_mm:10,increment_mrad:0},
    rays:[{state:3,age_ms:10},{state:1,age_ms:20}],trajectory_mm:[[0,0],[50,100]]};
  assert(mapValidate(data),"valid calibrated schema");drawing.length=0;await cycle();
  assert(node("map-pose").textContent.includes("0.050 m")&&node("map-pose").textContent.includes("1.571 rad"),"metric units");
  assert(drawing.some(c=>c.key==="translate"&&c.args[0]===400&&c.args[1]===160),"world to canvas including inverted Y");
  assert(drawing.some(c=>c.key==="rotate"&&Math.abs(c.args[0]+1.571)<1e-9),"CCW robot heading on canvas");
  assert(node("map-rays").textContent.includes("Adquisición fallida"),"invalid attempt not free space");
  cases.push("W7 real calibrated pose and trajectory project in meters with correct Y/heading and independent echo validity");
  now+=501;panel.render();assert(node("map-state").textContent.includes("histórica")&&node("map-age").textContent.includes("501 ms"),"local aging");
  data={...data,connected:false,pose:{...data.pose,valid:false,stale:true}};await cycle();
  assert(node("map-state").textContent.includes("histórica")&&node("map-pose").textContent.includes("0.050 m"),"history retained on disconnect");
  cases.push("W7 pose ages locally beyond 500 ms; disconnected source keeps explicitly historical map and pose");
  mode="network";await cycle();assert(node("map-age").textContent.includes("Sin vigencia"),"network state");
  const before=requests.length;mode="ok";data={...data,connected:true,pose:{...data.pose,valid:true,stale:false,age_ms:0}};await cycle();
  assert(requests.length===before+1&&node("map-state").textContent.includes("actualizada"),"only monitoring recovers");
  cases.push("W7 HTTP loss and recovery never issue motion or manufacture new measurements");
  const invalid=[null,{}, {...data,calibrated:false}, {...data,pose:{...data.pose,x_mm:null}},
    {...data,grid:{...data.grid,cells:"9".repeat(64)}},{...data,grid:{...data.grid,cells:"0"}},
    {...data,grid:{...data.grid,width:10001}},{...data,trajectory_mm:new Array(129).fill([0,0])},
    {...data,pose:{...data.pose,theta_mrad:4000}},{...data,pose:{...data.pose,left_counts:100}},
    {...data,grid:{...data.grid,resolution_mm:0}},{...data,pose:{...data.pose,state:"uncalibrated"}}];
  for(const value of invalid)assert(!mapValidate(value),"reject malformed/inconsistent map");
  mode="malformed";await cycle();assert(node("map-pose").textContent.includes("0.050 m"),"bad schema preserves prior data");
  cases.push("W7 bounded dimensions, cells, coordinates, counts, calibration and path schema reject malformed payloads");
  mode="pending";now+=500;fire(500);await flush();const waiting=requests.length;
  assert(![...timers.values()].some(t=>t.delay===500),"one inflight");
  fire(1500);await flush();assert(requests.length===waiting&&typeof pendingReject==="function","timeout no duplicate request");
  cases.push("W7 1500-ms HTTP timeout with one inflight and delayed polling, no catch-up or automatic commands");
  mode="ok";data=blank();await cycle();assert(node("map-pose").textContent.includes("no disponible")&&node("map-counts").textContent.includes("Sin conteos"),"ESP32 restart clears old state");
  cases.push("W7 source restart clears old pose instead of replaying a fictitious relative origin");
  assert(JSON.stringify(mapToCanvas(data.grid,0,0,640,640))==="[320,320]","origin");
  assert(mapToCanvas(data.grid,-200,-200,640,640)[1]===640,"lower edge");
  cases.push("W7 world origin and both canvas axes use explicit grid origin and resolution");
  data={...blank(),connected:true,calibrated:true,calibration:"calibrated",epoch:8,
    pose:{valid:true,state:"valid",stale:false,tick:0,age_ms:0,left_counts:"10",right_counts:"10",x_mm:0,y_mm:0,theta_mrad:0,increment_mm:0,increment_mrad:0},
    grid:{width:16,height:8,resolution_mm:50,origin_x_mm:-400,origin_y_mm:-200,revision:0,cells:"0".repeat(128)}};
  await cycle();assert(node("map-canvas").height===320,"rectangular cells preserve aspect ratio");
  cases.push("W7 rectangular configurable grids preserve metric scale on both canvas axes");
  data={...data,calibrated:false,calibration:"incomplete",pose:{...data.pose,valid:false,state:"invalid",x_mm:null,y_mm:null,theta_mrad:null,increment_mm:null,increment_mrad:null}};
  drawing.length=0;await cycle();assert(node("map-state").textContent.includes("inválida")&&!drawing.some(c=>c.key==="translate"),"invalid is not hidden by missing calibration");
  cases.push("W7 encoder discontinuity is explicitly invalid, independently of calibration, and removes the current pose marker");
  data={...data,calibrated:false,calibration:"provisional",pose:{...data.pose,valid:true,state:"valid",stale:false,
    age_ms:0,x_mm:50,y_mm:100,theta_mrad:0,increment_mm:5,increment_mrad:0}};
  assert(mapValidate(data),"provisional complete metric schema");await cycle();
  assert(node("map-calibration").textContent.includes("provisional")&&node("map-calibration").textContent.includes("no calibrada físicamente"),"no false definitive calibration");
  assert(node("map-state").textContent.includes("aproximada")&&node("map-pose").textContent.includes("0.050 m"),"usable approximate pose");
  cases.push("W7.4 complete provisional geometry displays approximate metric pose distinctly from physical calibration");
  now+=501;panel.render();
  assert(node("map-state").textContent.includes("histórica")&&node("map-state").textContent.includes("aproximada"),"stale approximate data stays historical");
  cases.push("W7.4 provisional labels retain age and historical warnings");
  assert(!mapValidate({...data,calibrated:true}),"boolean cannot promote provisional geometry");
  assert(!mapValidate({...data,calibration:"unknown"}),"closed calibration enum");
  assert(!mapValidate({...data,calibration:"incomplete"}),"missing parameters cannot contain metric pose");
  data={...data,calibrated:true,calibration:"calibrated"};await cycle();
  assert(node("map-calibration").textContent==="Parámetros físicamente calibrados"&&!node("map-state").textContent.includes("aproximada"),"only class 2 physical calibration");
  cases.push("W7.4 calibrated geometry requires coherent explicit metadata; incomplete, unknown and contradictory states rejected");
  return cases;
}
if(typeof module!=="undefined"&&require.main===module){const fs=require("node:fs"),path=require("node:path");
  runMapTests(fs.readFileSync(path.join(__dirname,"../../web/map.js"),"utf8")).then(cases=>console.log(`RESULT ${cases.length} W7 JavaScript groups passed`)).catch(e=>{console.error(e);process.exitCode=1;});}
