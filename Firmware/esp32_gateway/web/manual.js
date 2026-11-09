"use strict";

(() => {
  const directions = {
    AVANZAR: document.getElementById("manual-forward"),
    RETROCEDER: document.getElementById("manual-reverse"),
    GIRAR_IZQUIERDA: document.getElementById("manual-left"),
    GIRAR_DERECHA: document.getElementById("manual-right")
  };
  const halt = document.getElementById("manual-halt");
  let gesture = null;

  function safe() {
    return lastStatus && lastStatus.valid && lastStatus.connected && lastStatus.state === 2 &&
      lastStatus.faults === 0 && lastStatus.stop === 0 &&
      lastStatus.age_ms + Math.max(0, performance.now() - lastStatusAt) < 2000 && !activeCommand;
  }
  function render() {
    for (const [name, button] of Object.entries(directions))
      button.disabled = !safe() || Boolean(gesture && gesture.direction !== name);
    halt.disabled = false; // Una parada explicita no depende del STATUS mostrado.
    setText("manual-session", gesture?.token ? "Autorización manual activa" :
      gesture ? "Solicitando autorización…" : "Sesión manual inactiva");
  }
  function id() {
    return Array.from(crypto.getRandomValues(new Uint8Array(16)), b => b.toString(16).padStart(2, "0")).join("");
  }
  async function post(path, body, headers = {}, controller = new AbortController()) {
    let timedOut = false;
    const timeout = setTimeout(() => { timedOut = true; controller.abort(); }, 250);
    try {
      const response = await fetch(path, { method: "POST", cache: "no-store", signal: controller.signal,
        headers: { "Content-Type": "text/plain", "X-Request-ID": id(), ...headers }, body });
      const result = await response.json();
      if (!response.ok || result.accepted !== true) throw new Error(result.error || "request");
      return result;
    } catch (error) {
      if (timedOut) throw new Error("network_timeout");
      throw error;
    } finally { clearTimeout(timeout); }
  }
  function failureMessage(error) {
    const reasons = {
      busy: "Sesión ocupada por otro cliente o solicitud operativa.",
      unsafe: "Estado del robot inseguro o desactualizado.",
      expired: "Autorización vencida o sesión cerrada por la ESP32.",
      network_timeout: "Timeout de red: la respuesta HTTP superó 250 ms."
    };
    return (reasons[error.message] || "Sin respuesta válida o solicitud rechazada por la ESP32.") +
      " Se cerró la sesión y se solicitó detener. Se requiere una nueva pulsación.";
  }
  function end(token) {
    // Mejor esfuerzo, una sola solicitud. La ESP32 vence aun sin este paquete.
    void post("/api/manual/end", "END", { "X-Session-Token": token }).catch(() => {});
  }
  function cancel(message, sendEnd = true) {
    const old = gesture;
    gesture = null;
    if (old) {
      old.held = false;
      clearTimeout(old.renewTimer); clearTimeout(old.leaseTimer);
      old.controller?.abort();
      if (sendEnd && old.token) end(old.token);
    }
    if (message) setText("manual-message", message);
    render();
  }
  function deadline(op, sentAt) {
    clearTimeout(op.leaseTimer);
    op.expiresAt = sentAt + 300;
    const remaining = op.expiresAt - performance.now();
    if (remaining <= 0) { cancel("La autorización venció. Soltá y volvé a presionar para iniciar otra sesión."); return false; }
    op.leaseTimer = setTimeout(() => {
      if (gesture === op) cancel("Autorización vencida; se solicitó detener. No se reanudará al reconectar.");
    }, remaining);
    return true;
  }
  async function renew(op) {
    if (gesture !== op || !op.held) return;
    if (!safe()) { cancel("Movimiento no autorizado por el estado disponible. Se solicitó detener."); return; }
    if (op.sequence >= 4294967295) { cancel("Secuencia agotada; se cerró la sesión."); return; }
    op.controller = new AbortController();
    const sentAt = performance.now();
    if (sentAt >= op.expiresAt) { cancel("Autorización vencida; se solicitó detener. Se requiere una nueva pulsación."); return; }
    // Vigilar esta solicitud mientras espera confirmacion (HTTP sigue limitado a
    // 250 ms). La ESP32 valida la vigencia real; enviar no confirma una renovacion.
    if (!deadline(op, sentAt)) return;
    try {
      const result = await post("/api/manual/renew", op.direction,
        { "X-Session-Token": op.token, "X-Sequence": String(++op.sequence) }, op.controller);
      if (gesture !== op || !op.held) return;
      if (result.lease_ms !== 300) throw new Error("invalid lease");
      if (!deadline(op, sentAt)) return;
      setText("manual-message", "Autorización renovada mientras mantenés presionado. No confirma desplazamiento físico.");
      // Cadencia desde el envio, sin sumar otra espera completa a la latencia.
      // Si la respuesta consumio el periodo, solo se agenda una nueva solicitud;
      // no hay cola, rafagas de recuperacion ni peticiones simultaneas.
      op.renewTimer = setTimeout(() => renew(op), Math.max(0, 200 - (performance.now() - sentAt)));
    } catch (error) {
      if (gesture === op) cancel(failureMessage(error));
    }
  }
  async function press(direction, event) {
    if (gesture || !safe() || event.isPrimary === false || event.button !== 0) return;
    event.preventDefault();
    const op = { direction, pointer: event.pointerId, held: true, sequence: 0, controller: new AbortController() };
    gesture = op; render();
    try { directions[direction].setPointerCapture(event.pointerId); } catch { /* Los eventos globales tambien liberan. */ }
    const sentAt = performance.now();
    try {
      const result = await post("/api/manual/start", "OPEN", {}, op.controller);
      if (typeof result.token !== "string" || !/^[a-f0-9]{32}$/.test(result.token) || result.lease_ms !== 300)
        throw new Error("invalid token");
      if (gesture !== op || !op.held) { end(result.token); return; }
      op.token = result.token;
      if (!safe() || !deadline(op, sentAt)) { if (gesture === op) cancel("Autorización no vigente; se solicitó detener."); return; }
      render();
      await renew(op); // Primera direccion solo tras recibir el token y seguir presionado.
    } catch (error) {
      if (gesture === op) cancel(failureMessage(error));
    }
  }
  function release(event, message = "Pulsación liberada; se solicitó detener el movimiento.") {
    if (gesture && event.pointerId === gesture.pointer)
      cancel(message);
  }
  for (const [direction, button] of Object.entries(directions)) {
    button.addEventListener("pointerdown", event => { void press(direction, event); });
    button.addEventListener("lostpointercapture", event => release(event, "Se perdió la captura del puntero; se solicitó detener. Se requiere una nueva pulsación."));
    button.addEventListener("contextmenu", event => event.preventDefault());
  }
  document.addEventListener("pointerup", release);
  document.addEventListener("pointercancel", event => release(event, "El navegador canceló el puntero; se solicitó detener. Se requiere una nueva pulsación."));
  window.addEventListener("blur", () => cancel("Se perdió el foco; se solicitó detener."));
  window.addEventListener("pagehide", () => cancel("Se cerró la página; se solicitó detener."));
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) cancel("Página oculta; se solicitó detener.");
  });
  let haltBusy = false;
  halt.addEventListener("click", async () => {
    if (haltBusy) return;
    haltBusy = true;
    cancel("Solicitando DETENER movimiento…", false);
    try { await post("/api/manual/halt", "DETENER"); setText("manual-message", "Solicitud de detención aceptada por ESP32; no confirma parada física."); }
    catch { setText("manual-message", "No se pudo confirmar la solicitud de detención. La autorización temporal no se renueva; STOP global sigue disponible."); }
    finally { haltBusy = false; }
  });
  manualControl = { cancel, onStatus() {
    if (gesture && !safe()) cancel("Estado no habilitado o desactualizado; se cerró la sesión.");
    render();
  } };
  render();
  setText("manual-message", "Elegí modo manual y verificá el estado. Se requiere una nueva pulsación para cada sesión.");
})();
