"use strict";

// Monitoreo independiente: no modifica controles, sesiones ni manda operaciones.
(() => {
  const TELEMETRY_LINK_MS = 1500; // Timeout de enlace existente, sin cambios.
  let sample = null, receivedAt = 0, reachable = false;
  const element = id => document.getElementById(id);
  function text(id, value) { const node = element(id); if (node.textContent !== value) node.textContent = value; }
  const age = n => Number.isInteger(n) && n >= 0 && n <= 4294967295;
  function valid(s) {
    if (!s || ![s.valid, s.connected, s.stm32_connected, s.paused, s.snapshot_available].every(v => typeof v === "boolean")) return false;
    if ((s.valid ? !age(s.age_ms) : s.age_ms !== null) || s.connected && !s.valid || s.snapshot_available && !s.valid) return false;
    if (s.snapshot_available ? !age(s.snapshot_age_ms) : s.snapshot_age_ms !== null) return false;
    if (!s.limits_ms || ![s.limits_ms.snapshot, s.limits_ms.ultrasound, s.limits_ms.ir].every(v => age(v) && v > 0)) return false;
    for (const u of [s.front, s.rear]) {
      if (!u || !["no_sample", "valid", "invalid"].includes(u.state) ||
          typeof u.stale !== "boolean" || typeof u.valid !== "boolean" || typeof u.failed !== "boolean" ||
          u.valid !== (u.state === "valid") || u.failed !== (u.state === "invalid")) return false;
      if (u.state === "valid" ? !age(u.distance_mm) || u.distance_mm < 20 || u.distance_mm > 4000 : u.distance_mm !== null) return false;
      if (u.state === "no_sample" ? u.age_ms !== null : !age(u.age_ms)) return false;
      if (!s.snapshot_available && u.state !== "no_sample") return false;
    }
    const ir = s.ir;
    if (!ir || ir.available !== s.snapshot_available || typeof ir.stale !== "boolean" ||
        !Array.isArray(ir.floor) || ir.floor.length !== 4) return false;
    return ir.available ? age(ir.age_ms) && ir.floor.every(v => typeof v === "boolean") &&
      typeof ir.cliff === "boolean" && ir.cliff === ir.floor.some(v => !v) :
      ir.age_ms === null && ir.cliff === null && ir.floor.every(v => v === null);
  }
  function render() {
    if (!sample) {
      element("sensors-status").dataset.state = reachable ? "unknown" : "error";
      text("sensors-connection", reachable ? "Sin telemetría desde el arranque" : "Telemetría sin respuesta");
      return;
    }
    const elapsed = Math.max(0, Math.floor(performance.now() - receivedAt));
    // STATUS W2 puede detectar la perdida antes del siguiente GET de sensores.
    // Solo leer su ultima observacion; no alterar W2 ni los controles W4.
    const status = typeof lastStatus !== "undefined" ? lastStatus : null;
    const stm32Connected = status ? status.valid && status.connected : sample.stm32_connected;
    const telemetryConnected = sample.connected && sample.age_ms !== null && sample.age_ms + elapsed < TELEMETRY_LINK_MS;
    const snapshotOld = !reachable || !telemetryConnected || !stm32Connected ||
      !sample.snapshot_available || sample.snapshot_age_ms + elapsed > sample.limits_ms.snapshot;
    element("sensors-status").dataset.state = !reachable || !stm32Connected ? "error" :
      sample.paused ? "pending" : snapshotOld ? "error" : "ok";
    text("sensors-connection", !reachable ? "No se pudo actualizar la telemetría" :
      !stm32Connected ? "STM32 sin respuesta STATUS reciente" :
      sample.paused ? "Consulta de sensores pausada durante control manual u operativo" :
      !sample.valid ? "Sin telemetría desde el arranque" :
      !sample.snapshot_available ? "STM32 todavía sin instantánea de sensores" :
      snapshotOld ? "Última información recibida · histórica" : "Instantánea reciente de sensores");
    text("sensors-age", sample.valid ? `${sample.age_ms + elapsed} ms desde la respuesta de telemetría` : "Sin respuesta SENSORS válida");
    for (const name of ["front", "rear"]) {
      const u = sample[name], old = snapshotOld || u.stale || (u.age_ms !== null && u.age_ms + elapsed > sample.limits_ms.ultrasound);
      text(`sensor-${name}-distance`, u.distance_mm === null ? "—" : `${u.distance_mm} mm`);
      text(`sensor-${name}-detail`, u.state === "no_sample" ? "Sin muestra: ningún intento resuelto disponible" :
        `${u.failed ? "Adquisición fallida · lectura inválida" : "Medición válida"}${old ? " · Información desactualizada" : ""} · ${u.age_ms + elapsed} ms desde el disparo del intento`);
      element(`sensor-${name}-distance`).dataset.stale = String(old);
    }
    const ir = sample.ir, irOld = snapshotOld || ir.stale || (ir.available && ir.age_ms + elapsed > sample.limits_ms.ir);
    ["front-left", "front-right", "rear-left", "rear-right"].forEach((name, index) => {
      text(`sensor-ir-${name}`, ir.floor[index] === null ? "Sin muestra" : ir.floor[index] ? "Detecta piso (1)" : "No detecta piso (0)");
      element(`sensor-ir-${name}`).dataset.stale = String(irOld);
    });
    text("sensor-ir-detail", !ir.available ? "Sin lectura IR disponible" :
      `${ir.cliff ? "Posible caída detectada" : "Los cuatro sensores detectan piso"}${irOld ? " · Información desactualizada" : ""} · ${ir.age_ms + elapsed} ms desde lectura GPIO`);
  }
  async function poll() {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 1500);
    try {
      const response = await fetch("/api/sensors", { cache: "no-store", signal: controller.signal });
      if (!response.ok) throw new Error("HTTP");
      const value = await response.json();
      if (!valid(value)) throw new Error("schema");
      sample = value; receivedAt = performance.now(); reachable = true;
    } catch { reachable = false; }
    finally { clearTimeout(timeout); render(); setTimeout(poll, 500); }
  }
  function refreshAge() { render(); setTimeout(refreshAge, 100); }
  poll(); refreshAge();
})();
