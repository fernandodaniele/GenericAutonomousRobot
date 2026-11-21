"use strict";

function imuValidate(s) {
  const states = ["no_data", "initializing", "ready", "valid", "init_error", "read_error"];
  const number = n => Number.isInteger(n) && n >= 0 && n <= 4294967295;
  if (!s || !states.includes(s.state) || !["received", "connected", "stm32_connected", "paused", "valid", "stale"].every(k => typeof s[k] === "boolean") ||
      !number(s.max_age_ms) || s.max_age_ms === 0 ||
      ![s.response_age_ms, s.sample_age_ms, s.hal_status, s.i2c_error].every(n => n === null || number(n))) return false;
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
  if (s.calibration === undefined) return true; // IMU-1 HTTP consumers remain compatible.
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
  return triple(s.gyro_raw,s.valid,32768)&&s.gyro_raw.every(n=>n===null||(Number.isInteger(n)&&n<=32767))&&
    triple(s.gyro_corrected_dps,c.valid,500.3)&&triple(c.offset_dps,c.valid,250.2);
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
