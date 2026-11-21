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
  return [[s.accel_g, 2.001], [s.gyro_dps, 250.2]].every(([v, bound]) =>
    Array.isArray(v) && v.length === 3 && v.every(n => s.valid ? typeof n === "number" && Number.isFinite(n) && Math.abs(n) <= bound : n === null));
}

function createImuPanel(doc, fetcher, now, schedule, clear, Controller) {
  let sample = null, receivedAt = 0, reachable = false, started = false;
  const text = (id, value) => { const node = doc.getElementById(id); if (node.textContent !== value) node.textContent = value; };
  function render() {
    if (!sample) {
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
  }
  /* STATUS is owned by W2. Reading it cannot authorize or emit movement. */
  function lastKnownStatus() { return typeof lastStatus === "undefined" ? null : lastStatus; }
  async function poll() {
    const controller = new Controller();
    const timeout = schedule(() => controller.abort(), 1500);
    try {
      const response = await fetcher("/api/imu", { cache: "no-store", signal: controller.signal });
      if (!response.ok) throw new Error("HTTP");
      const value = await response.json();
      if (!imuValidate(value)) throw new Error("schema");
      sample = value; receivedAt = now(); reachable = true;
    } catch { reachable = false; }
    finally { clear(timeout); render(); schedule(poll, 500); }
  }
  function refresh() { render(); schedule(refresh, 100); }
  return { start() { if (!started) { started = true; void poll(); refresh(); } } };
}
if (typeof document !== "undefined") createImuPanel(document, fetch, () => performance.now(), setTimeout, clearTimeout, AbortController).start();
