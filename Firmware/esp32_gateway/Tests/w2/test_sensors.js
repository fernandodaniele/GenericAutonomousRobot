async function runSensorTests(source) {
  const assert = (v, reason) => { if (!v) throw new Error(reason); };
  const result = [];
  function payload(available = true) {
    const u = () => ({ state: available ? "valid" : "no_sample", valid: available, failed: false, stale: !available,
      distance_mm: available ? 345 : null, age_ms: available ? 0 : null });
    return { valid: available, connected: available, stm32_connected: true, paused: false, age_ms: available ? 0 : null,
      snapshot_available: available, snapshot_age_ms: available ? 0 : null,
      limits_ms: { snapshot: 1500, ultrasound: 1500, ir: 1500 }, front: u(), rear: { ...u(), distance_mm: available ? 330 : null },
      ir: { available, stale: !available, age_ms: available ? 0 : null, floor: [0,1,2,3].map(() => available ? true : null), cliff: available ? false : null } };
  }
  async function fixture(initial = payload()) {
    let now = 0, serial = 1, data = initial, mode = "ok";
    const nodes = new Map(), timers = new Map(), requests = [];
    function node(id) { if (!nodes.has(id)) nodes.set(id,{textContent:"",dataset:{}});return nodes.get(id); }
    class Controller { constructor() { this.signal = {}; } abort() { this.signal.abort?.(); } }
    async function fetchMock(url, options) {
      assert(url === "/api/sensors" && !options.method && !options.body, "only read-only sensor GET");
      requests.push({ url, now });
      if (mode === "offline") throw new Error("offline");
      if (mode === "pending") return new Promise((resolve,reject) => { options.signal.abort=()=>reject(new Error("timeout")); });
      return { ok: mode !== "http-error", json: async () => JSON.parse(JSON.stringify(data)) };
    }
    const updateStatus = new Function("document","fetch","AbortController","performance","setTimeout","clearTimeout",
      "let lastStatus = null;\n" + source + "\nreturn value => { lastStatus = value; };")(
      { getElementById:node },fetchMock,Controller,{now:()=>now},
      (fn,delay)=>{const id=serial++;timers.set(id,{fn,at:now+delay});return id;}, id=>timers.delete(id));
    async function flush() { for(let i=0;i<16;++i)await Promise.resolve(); }
    async function advance(ms) {
      const target=now+ms;
      for(let i=0;i<1000;++i) {
        const next=[...timers].filter(([,t])=>t.at<=target).sort((a,b)=>a[1].at-b[1].at)[0];
        if(!next){now=target;await flush();return;}
        const [id,t]=next;timers.delete(id);now=t.at;t.fn();await flush();
      }
      throw new Error("timer burst");
    }
    await flush();return {node,advance,requests,flush,set(v){data=v;},mode(v){mode=v;},status:updateStatus};
  }
  let f=await fixture(payload(false));
  assert(f.node("sensor-front-distance").textContent==="—" && f.node("sensor-front-detail").textContent.includes("Sin muestra"),"unknown is not zero");
  assert(f.node("sensor-ir-front-left").textContent==="Sin muestra", "IR unknown");
  result.push("no telemetry or source sample uses explicit unknowns and never zero distances");
  f=await fixture();assert(f.node("sensor-front-distance").textContent==="345 mm" && f.node("sensor-rear-distance").textContent==="330 mm", "independent real distances");
  assert(f.node("sensor-front-detail").textContent.includes("Medición válida") && f.node("sensor-front-detail").textContent.includes("0 ms"),"zero age legitimate");
  result.push("valid independent ultrasound measurements and legitimate zero ages render in mm");
  let s=payload();s.front={state:"invalid",valid:false,failed:true,stale:false,distance_mm:null,age_ms:50};
  f=await fixture(s);assert(f.node("sensor-front-distance").textContent==="—" && f.node("sensor-front-detail").textContent.includes("Adquisición fallida"),"invalid differs from absent");
  assert(f.node("sensor-rear-distance").textContent==="330 mm", "rear unaffected");f.set(payload());await f.advance(500);
  assert(f.node("sensor-front-distance").textContent==="345 mm" && !f.node("sensor-front-detail").textContent.includes("fallida"), "recovery");
  result.push("failed attempt distinguished from no sample; front recovery preserves independent rear");
  for(let mask=0;mask<16;++mask) {
    s=payload();s.ir.floor=[1,2,4,8].map(bit=>Boolean(mask&bit));s.ir.cliff=mask!==15;f=await fixture(s);
    ["front-left","front-right","rear-left","rear-right"].forEach((name,i)=>assert(f.node("sensor-ir-"+name).textContent===(s.ir.floor[i]?"Detecta piso (1)":"No detecta piso (0)"),"IR polarity and placement"));
  }
  result.push("all 16 four-IR combinations preserve 1=detects floor and aggregate cliff meaning");
  for (const age of [300, 600, 900]) {
    s=payload();s.snapshot_age_ms=s.front.age_ms=s.rear.age_ms=s.ir.age_ms=age;f=await fixture(s);
    await f.advance(200);
    assert(!f.node("sensor-front-detail").textContent.includes("desactualizada") &&
      !f.node("sensor-ir-detail").textContent.includes("desactualizada") &&
      f.node("sensor-front-detail").textContent.includes(`${age+200} ms`),"normal readings and local age stay visible");
  }
  s=payload();s.front.age_ms=1501;s.rear.age_ms=30;s.snapshot_age_ms=0;f=await fixture(s);
  assert(f.node("sensor-front-distance").dataset.stale==="true" && f.node("sensor-rear-distance").dataset.stale==="false", "individual ages");
  result.push("snapshot and individual ages grow locally; stale values remain explicitly historical");
  s=payload();s.snapshot_age_ms=1500;f=await fixture(s);
  assert(f.node("sensor-front-distance").dataset.stale==="false", "source snapshot age exactly 1500 allowed while link recent");
  await f.advance(100);assert(f.node("sensor-front-distance").dataset.stale==="true", "snapshot becomes historical above 1500");
  s=payload();s.front.age_ms=1500;s.rear.age_ms=1501;s.ir.age_ms=1500;f=await fixture(s);
  assert(f.node("sensor-front-distance").dataset.stale==="false" && f.node("sensor-rear-distance").dataset.stale==="true" && f.node("sensor-ir-front-left").dataset.stale==="false", "individual exact boundaries");
  await f.advance(100);assert(f.node("sensor-front-distance").dataset.stale==="true" && f.node("sensor-ir-front-left").dataset.stale==="true", "individual ages keep increasing");
  result.push("1500/1501-ms source and individual monitor boundaries preserve real historical warnings");
  s=payload();s.stm32_connected=false;f=await fixture(s);assert(f.node("sensors-connection").textContent.includes("STM32 sin respuesta"),"STM32 link distinct from HTTP");
  assert(f.node("sensor-front-distance").textContent==="345 mm" && f.node("sensor-front-distance").dataset.stale==="true", "retained stale");
  s=payload();s.paused=true;f=await fixture(s);assert(f.node("sensors-connection").textContent.includes("pausada"),"manual suspension visible");
  result.push("STM32 communication loss and intentional manual-session pause are distinct from acquisition failure");
  s=payload();s.paused=true;f=await fixture(s);f.mode("pending");f.status({valid:true,connected:false});
  await f.advance(100);assert(f.requests.length===1 && f.node("sensors-connection").textContent.includes("STM32 sin respuesta") && f.node("sensors-status").dataset.state==="error", "latest STATUS wins stale sensor HTTP copy and pause indicator before next poll");
  assert(f.node("sensor-front-distance").dataset.stale==="true" && f.node("sensor-front-distance").textContent==="345 mm", "STATUS loss preserves historical values");
  f.status({valid:true,connected:true});await f.advance(100);
  assert(f.requests.length===1 && f.node("sensors-connection").textContent.includes("pausada") && f.node("sensors-status").dataset.state==="pending", "STATUS recovery restores pause description without new sensor request");
  result.push("latest W2 STATUS loss/recovery reflected on next 100-ms render without waiting for sensor HTTP, even while paused");
  s=payload();s.paused=true;f=await fixture(s);f.mode("pending");await f.advance(1500);
  assert(f.node("sensors-connection").textContent.includes("pausada") && f.node("sensor-front-distance").dataset.stale==="true", "intentional pause still ages samples");
  s=payload();f=await fixture(s);f.mode("pending");await f.advance(1499);
  assert(f.node("sensor-front-distance").dataset.stale==="false", "link recent below 1500-ms local reception age");
  await f.advance(1);assert(f.node("sensor-front-distance").dataset.stale==="true" && f.requests.length===2, "existing link deadline applies locally at 1500 before HTTP timeout");
  result.push("local telemetry age expires at unchanged 1500-ms link timeout; pause never freezes ages");
  f=await fixture();f.mode("offline");await f.advance(600);
  assert(f.node("sensors-connection").textContent.includes("No se pudo actualizar") && f.node("sensor-front-distance").textContent==="345 mm", "network fail preserves history");
  f.mode("ok");await f.advance(500);assert(f.requests.length===3 && f.node("sensors-connection").textContent.includes("reciente"), "read-only recovery without bursts");
  result.push("HTTP loss retains old readings, increments age, and recovers only monitoring");
  const mutations=[s=>{s.front.distance_mm=0;},s=>{s.rear.state="other";},s=>{s.front.failed=true;},s=>{s.ir.floor=[true];},
    s=>{s.ir.cliff=true;},s=>{s.front.age_ms=null;},s=>{s.snapshot_age_ms=-1;},s=>{s.limits_ms.ir=0;},s=>{s.valid=false;},s=>{s.ir.floor[0]=1;}];
  for(const mutate of mutations) {f=await fixture();s=payload();mutate(s);f.set(s);await f.advance(500);assert(f.node("sensors-connection").textContent.includes("No se pudo actualizar") && f.node("sensor-front-distance").textContent==="345 mm","malformed schema cannot replace last sample");}
  result.push("ten malformed or inconsistent schemas rejected without replacing the last sample");
  f=await fixture();f.mode("pending");await f.advance(500);assert(f.requests.length===2,"one pending");await f.advance(1499);assert(f.requests.length===2,"no overlapping requests");
  await f.advance(1);assert(f.node("sensors-connection").textContent.includes("No se pudo actualizar"),"timeout1500");
  await f.advance(499);assert(f.requests.length===2,"no immediate retry");f.mode("ok");await f.advance(1);assert(f.requests.length===3,"one delayed next poll");
  result.push("1500-ms HTTP timeout, single-flight 500-ms polls, no operational endpoints or catch-up requests");
  return result;
}
if(typeof module!=="undefined" && require.main===module) {
  const fs=require("node:fs"),path=require("node:path");
  runSensorTests(fs.readFileSync(path.join(__dirname,"../../web/sensors.js"),"utf8"))
    .then(r=>{r.forEach((s,i)=>console.log(`PASS ${i+1} ${s}`));console.log(`RESULT ${r.length} W5 JavaScript groups passed`);})
    .catch(e=>{console.error(e);process.exitCode=1;});
}
