"use strict";

function imuFusionValidate(s) {
  const f=s.fusion;
  if(f===undefined)return true; // IMU-1/2/3 HTTP compatibility.
  const number=n=>Number.isInteger(n)&&n>=0&&n<=4294967295;
  const states=["unavailable","pending","valid","degraded","invalid"];
  const reasons=["none","imu","encoder","timing","numeric","reference"];
  if(!f||f.experimental!==true||f.encoder_only!==false||!states.includes(f.state)||!reasons.includes(f.reason)||
    !["received","connected","valid","requires_calibration"].every(k=>typeof f[k]==="boolean")||
    f.received!==(f.state!=="unavailable")||f.requires_calibration!==["degraded","invalid"].includes(f.state)||
    (["degraded","invalid"].includes(f.state))!==(f.reason!=="none")||
    (f.state==="degraded"&&f.reason!=="encoder")||(f.connected&&(!f.received||!s.stm32_connected)))return false;
  if(![f.reference_request,f.reference_tick,f.sample_tick,f.sample_age_ms,f.tau_ms].every(n=>n===null||number(n))||
    f.received!==(f.tau_ms!==null)||(f.tau_ms!==null&&f.tau_ms===0)||
    !number(f.encoder_calibration)||f.encoder_calibration>2)return false;
  const reference=f.reference_request!==null;
  if([f.reference_tick,f.sample_tick,f.sample_age_ms].some(n=>(n!==null)!==reference)||
    (reference&&(f.reference_request===0||f.encoder_calibration===0||typeof f.alpha!=="number"||!Number.isFinite(f.alpha)||f.alpha<=0||f.alpha>=1))||
    (!reference&&(f.alpha!==null||f.encoder_calibration!==0))||(!f.received&&reference))return false;
  if(f.valid&&(!f.connected||f.state!=="valid"||!s.valid||!s.calibration?.valid||!s.orientation?.valid||!reference||
    f.reference_request!==s.orientation.reference_request||f.reference_tick!==s.orientation.reference_tick||f.sample_age_ms>s.max_age_ms))return false;
  return [f.degrees,f.gyro_degrees,f.encoder_degrees,f.difference_gyro_deg,f.difference_encoder_deg].every((n,i)=>
    f.valid?typeof n==="number"&&Number.isFinite(n)&&Math.abs(n)<=(i<3?2147483.647:180):n===null);
}

function imuValidate(s) {
  const states = ["no_data", "initializing", "ready", "valid", "init_error", "read_error"];
  const number = n => Number.isInteger(n) && n >= 0 && n <= 4294967295;
  if (!s || !states.includes(s.state) || !["received", "connected", "stm32_connected", "paused", "valid", "stale"].every(k => typeof s[k] === "boolean") ||
      !number(s.max_age_ms) || s.max_age_ms === 0 ||
      ![s.response_age_ms, s.sample_age_ms, s.hal_status, s.i2c_error].every(n => n === null || number(n))) return false;
  if(!imuFusionValidate(s))return false;
  if (s.received !== (s.state !== "no_data") || (s.received && (s.hal_status === null || s.i2c_error === null || s.response_age_ms === null)) ||
      (!s.received && (s.sample_age_ms !== null || s.hal_status !== null || s.i2c_error !== null || s.response_age_ms !== null)) ||
      (s.hal_status !== null && s.hal_status > 3) || s.valid === s.stale) return false;
  if ((s.connected && (!s.received || !s.stm32_connected)) ||
      (["init_error", "read_error"].includes(s.state) && s.hal_status === 0) ||
      (["initializing", "ready", "valid"].includes(s.state) && (s.hal_status !== 0 || s.i2c_error !== 0))) return false;
  if (s.valid && (!s.connected || !s.stm32_connected || s.state !== "valid" || s.hal_status !== 0 || s.i2c_error !== 0 ||
      s.sample_age_ms === null || s.sample_age_ms > s.max_age_ms)) return false;
  if (![[s.accel_g, 2.001], [s.gyro_dps, 250.2]].every(([v, bound]) =>
    Array.isArray(v) && v.length === 3 && v.every(n => s.valid ? typeof n === "number" && Number.isFinite(n) && Math.abs(n) <= bound : n === null))) return false;
  if (s.calibration === undefined) return s.orientation === undefined && s.comparison === undefined; // IMU-1 compatibility.
  const c=s.calibration, phases=["idle","queued","sent","accepted","rejected","tx_error","expired"];
  const triple=(v,usable,bound)=>Array.isArray(v)&&v.length===3&&v.every(n=>usable?typeof n==="number"&&Number.isFinite(n)&&Math.abs(n)<=bound:n===null);
  if(!c || typeof c.available!=="boolean" || typeof c.valid!=="boolean" ||
    !["unavailable","pending","running","valid","failed"].includes(c.state)||
    !["none","motion","i2c","timing","mode","insufficient"].includes(c.reason)||
    !phases.includes(c.request_phase)||!number(c.samples)||c.samples>250||c.target_samples!==250||
    !number(c.request)||(c.age_ms!==null&&!number(c.age_ms))||c.available!==(c.state!=="unavailable"))return false;
  if(c.valid&&(!s.valid||c.state!=="valid"||c.samples!==250||c.reason!=="none"||c.age_ms===null))return false;
  if((c.state==="failed")!==(c.reason!=="none")||(c.state==="pending"&&c.samples!==0)||
    (c.state==="running"&&c.samples>=250)||(c.state==="valid"&&c.samples!==250))return false;
  if(!(triple(s.gyro_raw,s.valid,32768)&&s.gyro_raw.every(n=>n===null||(Number.isInteger(n)&&n<=32767))&&
    triple(s.gyro_corrected_dps,c.valid,500.3)&&triple(c.offset_dps,c.valid,250.2)))return false;
  if(s.orientation===undefined && s.comparison===undefined)return true; // IMU-2 compatibility.
  const o=s.orientation,p=s.comparison;
  const angle=(v,usable,bound)=>usable?typeof v==="number"&&Number.isFinite(v)&&Math.abs(v)<=bound:v===null;
  if(!o||!p||typeof o.available!=="boolean"||typeof o.valid!=="boolean"||typeof o.requires_calibration!=="boolean"||
    !["unavailable","pending","valid","invalid"].includes(o.state)||
    !["none","calibration","i2c","timing","numeric","reinitialized"].includes(o.reason)||
    o.available!==(o.state!=="unavailable")||o.requires_calibration!==(o.available&&o.state!=="valid")||
    (o.state==="invalid")!==(o.reason!=="none")||
    ![o.sample_age_ms,o.reference_tick,o.reference_request].every(n=>n===null||number(n))||
    ((o.reference_tick===null)!==(o.reference_request===null))||((o.reference_tick===null)!==(o.sample_age_ms===null))||
    (o.reference_request!==null&&o.reference_request===0)||
    (o.valid&&(!c.valid||o.state!=="valid"||o.reference_tick===null||o.reference_request!==c.request||o.sample_age_ms>s.max_age_ms))||
    !angle(o.degrees,o.valid,2147483.647))return false;
  if(typeof p.valid!=="boolean"||!["unavailable","waiting_reference","valid","odometry_unavailable","yaw_unavailable"].includes(p.state)||
    ![p.sample_tick,p.sample_age_ms].every(n=>n===null||number(n))||!number(p.encoder_calibration)||p.encoder_calibration>2||
    ((p.sample_tick===null)!==(p.sample_age_ms===null))||
    (p.valid&&(!o.valid||p.state!=="valid"||p.sample_tick===null||p.sample_age_ms>s.max_age_ms||p.encoder_calibration===0)))return false;
  return [p.gyro_delta_deg,p.encoder_delta_deg,p.difference_deg,p.encoder_reference_deg].every(v=>angle(v,p.valid,180));
}

function createImuPanel(doc, fetcher, now, schedule, clear, Controller,
  makeId=()=>Array.from(crypto.getRandomValues(new Uint8Array(16)),b=>b.toString(16).padStart(2,"0")).join("")) {
  let sample = null, receivedAt = 0, reachable = false, started = false;
  let posting=false,waitingUntil=0;
  const button=doc.getElementById("imu-calibrate");
  const text = (id, value) => { const node = doc.getElementById(id); if (node.textContent !== value) node.textContent = value; };
  function render() {
    if (!sample) {
      if(button)button.disabled=true;
      text("imu-state", reachable ? "Sin telemetría IMU" : "No se pudo obtener telemetría IMU");
      return;
    }
    const elapsed = Math.max(0, Math.floor(now() - receivedAt));
    const age = sample.sample_age_ms === null ? null : sample.sample_age_ms + elapsed;
    const stm32 = sample.stm32_connected && (!lastKnownStatus() || lastKnownStatus().connected);
    const valid = reachable && stm32 && sample.valid && age !== null && age <= sample.max_age_ms &&
      sample.response_age_ms + elapsed < 1500;
    const names = { no_data: "Sin telemetría desde el arranque", initializing: "Inicializando IMU", ready: "Inicializada · esperando lectura",
      valid: "Lectura válida", init_error: "Error de inicialización / comunicación I²C", read_error: "Fallo de lectura I²C" };
    text("imu-state", !reachable ? "No se pudo actualizar la telemetría IMU" : !stm32 ? "STM32 sin respuesta STATUS reciente" :
      names[sample.state] + (sample.paused ? " · Consulta pausada durante control manual u operativo" : "") +
      (sample.state === "valid" && !valid ? " · Información desactualizada" : ""));
    text("imu-age", age === null ? "Sin lectura exitosa" : `${age} ms desde la última lectura exitosa`);
    text("imu-error", sample.hal_status === null ? "Sin información" :
      `HAL=${sample.hal_status} · error I²C=${sample.i2c_error} (máscara)`);
    text("imu-accel", valid ? sample.accel_g.map(n => n.toFixed(3)).join(" / ") : "Sin lectura vigente");
    text("imu-gyro", valid ? sample.gyro_dps.map(n => n.toFixed(3)).join(" / ") : "Sin lectura vigente");
    text("imu-gyro-raw",valid&&Array.isArray(sample.gyro_raw)?sample.gyro_raw.join(" / "):"Sin lectura RAW vigente");
    const c=sample.calibration;
    const calibrated=valid&&c?.valid===true;
    const reasons={motion:"Movimiento o lecturas incompatibles con reposo",i2c:"Falla I²C; se requiere una nueva calibración",timing:"Muestreo discontinuo o demasiado rápido",mode:"Cambio a MANUAL/AUTO durante la ventana",insufficient:"Muestras insuficientes dentro del plazo"};
    text("imu-calibration",!c?.available?"Calibración no disponible en esta versión":
      c.state==="pending"?"Pendiente · iniciar con el robot inmóvil":c.state==="running"?`En curso · ${c.samples}/250 muestras`:
      c.state==="failed"?`Fallida · ${reasons[c.reason]}`:calibrated?`Válida · offsets medidos en RAM · hace ${c.age_ms+elapsed} ms`:"Calibración registrada · información desactualizada");
    text("imu-offset",calibrated?c.offset_dps.map(n=>n.toFixed(4)).join(" / "):"Sin offsets vigentes");
    text("imu-gyro-corrected",calibrated?sample.gyro_corrected_dps.map(n=>n.toFixed(3)).join(" / "):"Sin calibración vigente");
    const o=sample.orientation,p=sample.comparison;
    const yawValid=valid&&o?.valid===true&&o.sample_age_ms+elapsed<=sample.max_age_ms;
    const comparisonValid=yawValid&&p?.valid===true&&p.sample_age_ms+elapsed<=sample.max_age_ms;
    const yawReasons={calibration:"Calibración perdida",i2c:"Falla I²C",timing:"Intervalo de adquisición discontinuo",numeric:"Límite numérico",reinitialized:"IMU reinicializada"};
    text("imu-yaw",yawValid?`${o.degrees.toFixed(2)}° · relativa acumulada`:!o?.available?"No disponible en esta versión":
      o.requires_calibration?`${yawReasons[o.reason]||"Sin referencia"} · calibrar nuevamente`:"Estimación sin información vigente");
    text("imu-reference",o?.reference_tick!==null&&o?.reference_tick!==undefined?
      `Referencia ${o.reference_request} · tick STM32 ${o.reference_tick} · muestra utilizada hace ${o.sample_age_ms+elapsed} ms`:"Sin referencia angular");
    text("imu-common-angles",comparisonValid?
      `Giroscopio ${p.gyro_delta_deg.toFixed(2)}° / Encoders ${p.encoder_delta_deg.toFixed(2)}°`:
      p?.state==="odometry_unavailable"?"Sin pose de encoders asociable al mismo instante":"Sin comparación vigente");
    text("imu-angle-difference",comparisonValid?
      `${p.difference_deg.toFixed(2)}° · muestra común hace ${p.sample_age_ms+elapsed} ms · odometría ${p.encoder_calibration===2?"calibrada":"provisional"}`:"Sin comparación vigente");
    const f=sample.fusion;
    const fusionValid=valid&&f?.valid===true&&f.sample_age_ms+elapsed<=sample.max_age_ms;
    const fusionReasons={imu:"Orientación IMU o calibración no válida",encoder:"Pose de encoders no disponible a tiempo",timing:"Intervalo de fusión discontinuo",
      numeric:"Límite numérico",reference:"Referencia común no disponible o incompatible"};
    text("imu-fusion-state",!f?.received?"Fusión experimental no disponible":
      f.requires_calibration?`${f.state==="degraded"?"Degradada":"Inválida"} · ${fusionReasons[f.reason]} · recalibrar inmóvil`:
      f.state==="pending"?"Esperando referencias y muestras sincronizadas":fusionValid?"Fusión experimental válida · solo diagnóstico":"Fusión sin información vigente");
    text("imu-fusion-angle",fusionValid?`${f.degrees.toFixed(2)}° · relativa acumulada`:"Sin estimación fusionada vigente");
    text("imu-fusion-sources",fusionValid?`Encoders ${f.encoder_degrees.toFixed(2)}° / Giroscopio ${f.gyro_degrees.toFixed(2)}°`:"Sin fuentes comunes vigentes");
    text("imu-fusion-differences",fusionValid?`Con giroscopio ${f.difference_gyro_deg.toFixed(2)}° / Con encoders ${f.difference_encoder_deg.toFixed(2)}°`:"Sin diferencias vigentes");
    text("imu-fusion-parameters",f?.tau_ms?`τ=${f.tau_ms} ms · α=${f.alpha===null?"sin actualización":f.alpha.toFixed(6)} (peso de predicción)`:
      "Sin parámetros recibidos de STM32");
    text("imu-fusion-age",f?.reference_request?`Referencia ${f.reference_request} · tick ${f.reference_tick} · muestra común hace ${f.sample_age_ms+elapsed} ms`:"Sin referencia común de fusión");
    const robot=lastKnownStatus();
    if(button)button.disabled=posting||now()<waitingUntil||!valid||!c?.available||c.state==="running"||
      ["queued","sent"].includes(c.request_phase)||!robot?.valid||!robot.connected||![0,3].includes(robot.state)||robot.age_ms>=2000;
    if(c&&!posting&&now()>=waitingUntil&&["rejected","tx_error","expired"].includes(c.request_phase))
      text("imu-calibration-message","La solicitud no se completó: "+c.request_phase+". No se reintenta automáticamente.");
  }
  /* STATUS is owned by W2. Reading it cannot authorize or emit movement. */
  function lastKnownStatus() { return typeof lastStatus === "undefined" ? null : lastStatus; }
  button?.addEventListener?.("click",async()=>{
    if(button.disabled||posting)return;
    posting=true;render();
    const controller=new Controller(),timeout=schedule(()=>controller.abort(),1500);
    try {
      const response=await fetcher("/api/imu/calibrate",{method:"POST",cache:"no-store",signal:controller.signal,
        headers:{"Content-Type":"text/plain","X-Request-ID":makeId()},body:"CALIBRATE"});
      const result=await response.json();
      if(!response.ok||result.accepted!==true)throw new Error(result.error||"request");
      waitingUntil=now()+2000;
      text("imu-calibration-message","Solicitud encolada; no confirma calibración válida. Mantené el robot inmóvil unos 5 segundos.");
    }catch{
      text("imu-calibration-message","No se pudo confirmar la solicitud. No se reintenta automáticamente; verificá el estado antes de repetir.");
    }finally{clear(timeout);posting=false;render();}
  });
  async function poll() {
    const controller = new Controller();
    const timeout = schedule(() => controller.abort(), 1500);
    try {
      const response = await fetcher("/api/imu", { cache: "no-store", signal: controller.signal });
      if (!response.ok) throw new Error("HTTP");
      const value = await response.json();
      if (!imuValidate(value)) throw new Error("schema");
      sample = value; receivedAt = now(); reachable = true;
      if(sample.calibration&&sample.calibration.request_phase!=="idle")waitingUntil=0;
    } catch { reachable = false; }
    finally { clear(timeout); render(); schedule(poll, 500); }
  }
  function refresh() { render(); schedule(refresh, 100); }
  return { start() { if (!started) { started = true; void poll(); refresh(); } } };
}
if (typeof document !== "undefined") createImuPanel(document, fetch, () => performance.now(), setTimeout, clearTimeout, AbortController).start();
