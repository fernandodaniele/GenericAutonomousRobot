"use strict";
(() => {
  const node = id => document.getElementById(id);
  const start = node("explore-start"), end = node("explore-end");
  if (!start || !end) return;
  let busy = false, requestedAt = null, sample = null;
  const text = message => { node("explore-message").textContent = message; };
  const actions = ["ninguna", "avanzar 50 mm", "barrer a izquierda hasta 10 grados", "orientar a izquierda hasta 10 grados", "orientar a derecha hasta 10 grados", "retroceder hasta 20 mm"];
  function safeIdle() {
    return typeof lastStatus !== "undefined" && lastStatus?.valid && lastStatus.connected && lastStatus.state === 0 &&
      lastStatus.faults === 0 && lastStatus.stop === 0 && lastStatus.age_ms + Math.max(0, performance.now() - lastStatusAt) < 2000;
  }
  function randomId() {
    if (!globalThis.crypto?.getRandomValues) throw new Error("Generador seguro no disponible");
    return Array.from(crypto.getRandomValues(new Uint8Array(16)), n => n.toString(16).padStart(2, "0")).join("");
  }
  async function request(path, options = {}, timeout = 800) {
    const controller = new AbortController(), deadline = setTimeout(() => controller.abort(), timeout);
    try { const r = await fetch(path, { cache: "no-store", ...options, signal: controller.signal });
      if (!r.ok) throw new Error(`Solicitud rechazada (${r.status})`); return await r.json();
    } finally { clearTimeout(deadline); }
  }
  start.addEventListener("click", async () => {
    if (busy || requestedAt !== null || !safeIdle() || !node("explore-supervised").checked) return;
    busy = true; requestedAt = performance.now(); node("explore-supervised").checked = false;
    try {
      const r = await request("/api/command", { method: "POST", headers: {
        "Content-Type": "text/plain", "X-Request-ID": randomId(), "X-Explore-Owner": randomId()
      }, body: "AUTO_EXPLORE" });
      if (!r.accepted) throw new Error("Inicio no aceptado");
      text("Solicitud aceptada por ESP32; esperando confirmacion de la sesion en STM32. No confirma movimiento.");
    } catch (e) { text(`${e.message}. Resultado no confirmado: consultar el estado o usar STOP. Sin reintento automatico.`); }
    finally { busy = false; }
  });
  end.addEventListener("click", async () => {
    if (busy) return;
    busy = true;
    try {
      const r = await request("/api/command", { method: "POST", headers: {
        "Content-Type": "text/plain", "X-Request-ID": randomId()
      }, body: "STOP" });
      if (!r.accepted) throw new Error("STOP no aceptado");
      text("STOP solicitado; comprobar detencion en el estado STM32. Sin reanudacion automatica.");
    } catch (e) { text(`${e.message}. STOP remoto no confirmado; utilizar el corte fisico de alimentacion si es necesario.`); }
    finally { busy = false; }
  });
  /* No WATCH or visibility handlers. ESP32 owns continuity after confirmation;
   * closing this page or losing Wi-Fi neither renews nor cancels the session. */
  function valid(s) {
    return s && [s.session_pending,s.session_active,s.session_started,s.session_cancelled,s.stop_received,s.link_connected,s.link_confirmed,s.link_monitoring,s.renewals_required,s.control_ready,s.received, s.connected, s.available, s.historical, s.sectors_available].every(v => typeof v === "boolean") &&
      [s.age_ms,s.session_id,s.started_tick,s.state_code,s.action_code,s.challenge,s.segments,s.turns,s.retreats,s.selected_sector,s.candidates,s.repeated,s.lease_ms,s.sequence,s.critical_faults,s.stabilization_state,s.stabilization_observations,s.stabilization_ms].every(v => Number.isSafeInteger(v) && v >= 0) &&
      s.authorization_mode === "session" && s.renewals_required === false && s.renewal_source === "none" &&
      (s.link_age_ms === null || Number.isSafeInteger(s.link_age_ms) && s.link_age_ms >= 0) && s.link_watchdog_ms === 3000 &&
      Number.isSafeInteger(s.terminal_reason_code) && s.terminal_reason_code >= 0 && s.terminal_reason_code <= 16 && typeof s.terminal_reason === "string" &&
      s.state_code <= 6 && s.action_code <= 5 && s.lease_ms <= 1500 && s.selected_sector <= 36 && s.candidates <= 36 &&
      Number.isFinite(s.scan_deg) && s.scan_deg >= 0 && s.scan_deg <= 200 && typeof s.block_reason === "string" &&
      s.sector_step_deg === 10 && typeof s.sectors === "string" && (s.sectors_available ? /^[0-3]{36}$/.test(s.sectors) : s.sectors === "") &&
      Array.isArray(s.range_mm) && s.range_mm.length === 36 && s.range_mm.every(v => v === null || Number.isInteger(v) && v >= 20 && v <= 4000);
  }
  async function poll() {
    try {
      const s = await request("/api/exploration", {}, 1200); if (!valid(s)) throw new Error("Telemetria incompatible");
      const previous = sample; sample = s;
      if (s.session_active && s.connected) {
        requestedAt = null;
        if (!previous?.session_active) text("Sesion confirmada por STM32, sin renovaciones de autorizacion. El enlace interno se vigila por consultas diagnosticas; puede cerrar la pagina. Supervisar presencialmente: STOP remoto requiere Wi-Fi y existe corte fisico de alimentacion.");
      } else if (s.session_active) text("Estado de sesion sin confirmacion reciente. No se inicia ni reactiva: recuperar la comunicacion para consultar el estado real o solicitar STOP.");
      else if (s.session_pending) text("ESP32 espera confirmacion de la nueva sesion en STM32; todavia no hay continuidad activa.");
      else if (previous?.session_active) text("La sesion STM32 termino. Consultar la causa; no se reanudara automaticamente.");
      node("explore-diagnostics").textContent = !s.available ? "Sin sesion de exploracion recibida." :
        `${s.connected ? "Actual" : "Historico/desactualizado"}: enlace interno ${s.link_connected ? "responde" : "sin respuesta reciente"}, confirmacion STM32 ${s.link_confirmed ? "vigente" : s.link_monitoring ? "pendiente" : "historica/sesion detenida"} (${s.link_age_ms === null ? "sin dato" : s.link_age_ms + " ms"}); STOP ${s.stop_received ? "recibido" : "no registrado"}; causa terminal ${s.terminal_reason}; sesion ${s.session_id}, fase ${s.state_code}; ${actions[s.action_code]}. Motivo ${s.block_reason}. Segmentos ${s.segments}, giros ${s.turns}, retiradas ${s.retreats}, barrido ${s.scan_deg.toFixed(1)} grados. Candidatos ${s.candidates}, orientacion elegida ${s.selected_sector ? (s.selected_sector - 1) * 10 + " grados en el marco de odometria" : "ninguna"}, repetidos ${s.repeated}. Edad ${s.age_ms} ms. Estabilizacion ${s.stabilization_state}: ${s.stabilization_observations} observaciones, ${s.stabilization_ms} ms. Sectores: ${s.sectors || "pendientes"} (0 desconocido, 1 insuficiente, 2 ocupado, 3 eco despejado puntual).`;
      const labels = ["desconocido", "evidencia insuficiente", "ocupado", "eco puntual despejado"];
      node("explore-sectors").textContent = !s.sectors_available ? "Sin instantanea angular compatible." :
        Array.from(s.sectors, (v,i) => `${i * 10} grados: ${labels[Number(v)]}${s.range_mm[i] === null ? "" : " (" + s.range_mm[i] + " mm)"}`).join("; ");
    } catch (e) { sample = null; node("explore-diagnostics").textContent = "Telemetria de exploracion no disponible. La sesion puede continuar: recuperar Wi-Fi para STOP remoto o utilizar el corte fisico."; }
    setTimeout(poll, 500);
  }
  function render() {
    if (requestedAt !== null && performance.now() - requestedAt >= 2000) requestedAt = null;
    start.disabled = busy || requestedAt !== null || !!sample?.session_pending || !!sample?.session_active || !safeIdle();
    end.disabled = busy;
    setTimeout(render, 100);
  }
  render(); poll(); // GET only. Reconnection never starts or renews a session.
})();
