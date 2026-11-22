"use strict";
let manualControl = null; // manual.js: controles aislados del flujo puntual W3.

const statusBox = document.getElementById("server-status");
const statusText = document.getElementById("status-text");
const statusDetail = document.getElementById("status-detail");
const checkButton = document.getElementById("check-health");

async function checkHealth() {
  checkButton.disabled = true;
  statusBox.dataset.state = "pending";
  statusText.textContent = "Comprobando…";
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 4000);

  try {
    const response = await fetch("/health", { cache: "no-store", signal: controller.signal });
    if (!response.ok) throw new Error("HTTP no disponible");
    const health = await response.json();
    if (health.status !== "ok" || health.service !== "esp32-web") {
      throw new Error("Respuesta inesperada");
    }
    statusBox.dataset.state = "ok";
    statusText.textContent = "Servidor disponible";
    statusDetail.textContent = "La ESP32 respondió correctamente a /health. Esto verifica únicamente su servidor web.";
  } catch {
    statusBox.dataset.state = "error";
    statusText.textContent = "No se pudo verificar";
    statusDetail.textContent = "Revisá la conexión a la red Wi-Fi del robot y volvé a comprobar. La disponibilidad STM32 se muestra por separado.";
  } finally {
    clearTimeout(timeout);
    checkButton.disabled = false;
  }
}

checkButton.addEventListener("click", checkHealth);
checkHealth();

const connectionBox = document.getElementById("stm32-status");
const readings = document.getElementById("robot-values");
const stateNames = [
  "En espera",
  "Modo autónomo",
  "Modo manual",
  "Detenido por seguridad"
];
// Correspondencia con task_safety.h; los bits representan fallas enclavadas.
const faultNames = [
  [1, "Caída detectada"],
  [2, "Obstáculo detectado"],
  [4, "Sensores sin datos válidos"],
  [8, "Emergencia informada"]
];
let lastStatus = null;
let lastStatusAt = 0;
let statusAvailable = false;

function setText(id, value) {
  const element = document.getElementById(id);
  if (element.textContent !== value) element.textContent = value;
}

function validStatus(value) {
  if (!value || typeof value.valid !== "boolean" || typeof value.connected !== "boolean") return false;
  if (!value.valid) return !value.connected &&
    [value.state, value.faults, value.stop, value.age_ms].every(field => field === null);
  return Number.isInteger(value.state) && value.state >= 0 && value.state <= 3 &&
    Number.isInteger(value.faults) && value.faults >= 0 && value.faults <= 4294967295 &&
    (value.stop === 0 || value.stop === 1) && Number.isSafeInteger(value.age_ms) && value.age_ms >= 0;
}

function describeFaults(mask) {
  if (mask === 0) return "Sin fallas informadas";
  const labels = faultNames.filter(([bit]) => Math.floor(mask / bit) % 2 === 1).map(([, label]) => label);
  const unknown = mask - mask % 16;
  if (unknown) labels.push(`Código de falla sin descripción: ${unknown}`);
  return labels.join(" | ");
}

function renderStatus(status) {
  statusAvailable = true;
  lastStatus = status;
  lastStatusAt = performance.now();
  connectionBox.dataset.state = !status.valid ? "unknown" : status.connected ? "ok" : "error";
  readings.dataset.stale = String(!status.connected);
  setText("stm32-connection", !status.valid ? "Sin datos desde el arranque" :
    status.connected ? "Respuesta reciente del robot" : "Comunicación desactualizada");
  setText("stm32-detail", !status.valid ? "El robot todavía no envió una respuesta de estado válida." :
    status.connected ? "Última respuesta recibida; no implica autorización de movimiento." :
      "No hay una respuesta válida reciente. Se conservan los últimos valores informados; no representan el estado actual.");
  setText("robot-state", status.valid ? stateNames[status.state] : "Desconocido");
  setText("robot-faults", status.valid ? describeFaults(status.faults) : "Desconocidas");
  setText("robot-stop", status.valid ? status.stop === 1 ? "Bloqueo activo" : "Sin bloqueo informado" : "Desconocido");
  setText("robot-age", status.valid ? `${status.age_ms} ms` : "Sin datos");
  updateCommandButtons();
}

async function pollStatus() {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 2500);
  try {
    const response = await fetch("/api/status", { cache: "no-store", signal: controller.signal });
    if (!response.ok) throw new Error("HTTP no disponible");
    const status = await response.json();
    if (!validStatus(status)) throw new Error("STATUS inválido");
    renderStatus(status);
    manualControl?.onStatus(status);
    observeCommand(status);
  } catch {
    connectionBox.dataset.state = "error";
    statusAvailable = false;
    updateCommandButtons();
    manualControl?.cancel("Monitoreo no disponible; se cerró la autorización manual.");
    readings.dataset.stale = "true";
    setText("stm32-connection", "Monitoreo sin respuesta");
    setText("stm32-detail", "No se pudo consultar la ESP32. El enlace STM32 y el estado actual del robot no pueden verificarse.");
    if (lastStatus && lastStatus.valid) {
      const age = lastStatus.age_ms + Math.max(0, Math.floor(performance.now() - lastStatusAt));
      setText("robot-age", `${age} ms aprox. · Sin actualizar`);
    }
  } finally {
    clearTimeout(timeout);
    // Una consulta a la vez, sin intervalos ni rafagas al recuperarse la red.
    setTimeout(pollStatus, 1000);
  }
}

pollStatus();

// Lista cerrada tambien en el servidor. No se generan ordenes al cargar.
const commandButtons = {
  STOP: document.getElementById("command-stop"),
  RESET: document.getElementById("command-reset"),
  MANUAL: document.getElementById("command-manual"),
  AUTO: document.getElementById("command-auto"),
  AUTO_TURN: document.getElementById("command-auto-turn"),
  AUTO_EVADE: document.getElementById("command-auto-evade")
};
let activeCommand = null;
const lastCommandAt = { STOP: -Infinity, RESET: -Infinity, MANUAL: -Infinity, AUTO: -Infinity, AUTO_TURN: -Infinity, AUTO_EVADE: -Infinity };

function canRequestAuto() {
  return statusAvailable && lastStatus?.valid && lastStatus.connected && lastStatus.state === 0 &&
    lastStatus.faults === 0 && lastStatus.stop === 0 &&
    lastStatus.age_ms + Math.max(0, performance.now() - lastStatusAt) < 2000;
}

function updateCommandButtons() {
  for (const [command, button] of Object.entries(commandButtons)) {
    button.disabled = (Boolean(activeCommand) && !(command === "STOP" && activeCommand.name !== "STOP")) ||
      (command.startsWith("AUTO") && !canRequestAuto());
  }
}

function finishCommand(operation, message) {
  if (activeCommand !== operation) return;
  clearTimeout(operation.timer);
  activeCommand = null;
  setText("command-message", message);
  updateCommandButtons();
}

function observeCommand(status) {
  const operation = activeCommand;
  if (!operation || !operation.accepted) return;
  const receipt = status.command;
  if (!receipt || receipt.id !== operation.id || receipt.name !== operation.name) return;
  if (receipt.phase === "failed" || receipt.phase === "cancelled") {
    finishCommand(operation, receipt.phase === "failed" ? "Falló la entrega a UART. No se reintentará automáticamente." :
      "Solicitud cancelada antes de transmitirse. No se reintentará automáticamente.");
  } else if (receipt.phase === "transmitted") {
    setText("command-message", "Solicitud aceptada y trama entregada a UART. Esperando observar el resultado en el robot.");
    if (status.valid && status.connected && receipt.status_after_send === true) {
      const matches = operation.name === "STOP" ? status.state === 0 || status.state === 3 :
        operation.name === "RESET" ? status.state === 0 && status.faults === 0 && status.stop === 0 :
          status.state === (operation.name.startsWith("AUTO") ? 1 : 2) && status.faults === 0 && status.stop === 0;
      if (matches) finishCommand(operation, `Estado posterior observado: ${stateNames[status.state]}. Revisá las protecciones y el bloqueo. Es una observación de estado, no un acuse correlacionado del comando.`);
    }
  }
}

async function sendCommand(name) {
  if (!Object.prototype.hasOwnProperty.call(commandButtons, name)) return;
  if (name.startsWith("AUTO") && !canRequestAuto()) {
    setText("command-message", "AUTO requiere un estado reciente En espera, sin fallas ni bloqueo.");
    updateCommandButtons();
    return;
  }
  if (name === "AUTO_TURN" || name === "AUTO_EVADE") {
    const consent = document.getElementById("auto3-supervised");
    if (consent?.checked !== true) { setText("command-message", "Confirmá supervisión y espacio lateral despejado para cada ensayo experimental."); return; }
    consent.checked = false;
  }
  if (activeCommand && !(name === "STOP" && activeCommand.name !== "STOP")) return;
  manualControl?.cancel("Solicitud operativa: se cerró la autorización manual.");
  const requestedAt = performance.now();
  if (requestedAt - lastCommandAt[name] < 1000) return; // Tambien cubre una confirmacion muy rapida entre dos pulsaciones.
  if (typeof crypto === "undefined" || typeof crypto.getRandomValues !== "function") {
    setText("command-message", "Este navegador no permite generar el identificador de solicitud. Usá la consola existente.");
    return;
  }
  if (activeCommand) {
    clearTimeout(activeCommand.timer);
    activeCommand.controller.abort(); // Mejor esfuerzo; no deshace una solicitud ya aceptada por la ESP32.
  }
  lastCommandAt[name] = requestedAt;
  const id = Array.from(crypto.getRandomValues(new Uint8Array(16)), byte => byte.toString(16).padStart(2, "0")).join("");
  const controller = new AbortController();
  const operation = { id, name, accepted: false, controller };
  activeCommand = operation;
  updateCommandButtons();
  setText("command-message", "Enviando solicitud a la ESP32… Todavía no hay resultado confirmado.");
  operation.timer = setTimeout(() => finishCommand(operation,
    "No fue posible confirmar el resultado con un estado posterior reciente. Revisá el panel antes de repetir; no habrá reintento automático."), 6000);
  const timeout = setTimeout(() => controller.abort(), 3500);
  try {
    const response = await fetch("/api/command", {
      method: "POST", cache: "no-store", signal: controller.signal,
      headers: { "Content-Type": "text/plain", "X-Request-ID": id }, body: name
    });
    const result = await response.json();
    if (!response.ok) {
      finishCommand(operation, "La API rechazó la solicitud. No se enviará de nuevo automáticamente; revisá el estado e intentá más tarde.");
      return;
    }
    if (result.accepted !== true || result.id !== id || result.command !== name || result.confirmed !== false ||
        !["queued", "sending", "transmitted", "failed", "cancelled"].includes(result.phase)) throw new Error("Respuesta no válida");
    if (activeCommand !== operation) return;
    operation.accepted = true;
    setText("command-message", "Solicitud aceptada por la API ESP32. Pendiente de verificar transmisión y estado del robot.");
  } catch {
    finishCommand(operation, "No se pudo conocer si la solicitud fue aceptada o transmitida. No se reintentará automáticamente. Revisá el estado antes de repetir.");
  } finally {
    clearTimeout(timeout);
  }
}

for (const [name, button] of Object.entries(commandButtons)) button.addEventListener("click", () => sendCommand(name));
updateCommandButtons();
setText("command-message", "Sin solicitudes pendientes. Elegí una acción; el estado observado se muestra por separado.");


function createNavigationPanel(doc, fetcher, schedule, clear, Controller) {
  let started=false;
  const text=(id,value)=>{const e=doc.getElementById(id);if(e&&e.textContent!==value)e.textContent=value;};
  const profiles={AUTO_2:"AUTO-2F operativo sin giro",AUTO_1:"AUTO-1",B7_LEGACY:"B7 historico",
    AUTO_3_TURN:"AUTO-3 experimental: giro aislado supervisado",AUTO_3_EVASION:"AUTO-3 experimental: secuencia supervisada"};
  const number=n=>Number.isFinite(n)?n.toFixed(2):"sin muestra";
  function render(s) {
    if(!s||typeof s.received!=="boolean"||typeof s.available!=="boolean"||
      (s.available&&(!Object.prototype.hasOwnProperty.call(profiles,s.profile)||!Number.isInteger(s.session_id))))throw new Error("schema");
    text("navigation-status",!s.received?"Sin telemetria de navegacion":
      !s.connected?"Telemetria desactualizada o enlace perdido: datos historicos":
      s.paused?"Consultas pausadas":s.historical?"Resultado historico retenido":"Telemetria actualizada");
    text("navigation-profile",s.available?`${profiles[s.profile]}: sesion ${s.session_id}, antiguedad ${s.age_ms} ms`:"Sin sesion registrada");
    text("navigation-phase",s.available?`Fase: ${s.state}, ultima: ${s.last_phase}, ${s.elapsed_ms} ms`:"Sin fase disponible");
    text("navigation-progress",s.available?`Avance ${number(s.advance_progress_mm)} mm, retroceso ${number(s.reverse?.progress_mm)} mm`:"Sin progreso disponible");
    const t=s.turn;
    text("navigation-turn",t?.available?`Giro ${t.direction==="left"?"izquierdo":"derecho"}: ${number(t.progress_deg)} / ${number(t.target_deg)} grados, ${t.elapsed_ms} ms, dato hace ${t.age_ms ?? "?"} ms${t.historical ? " (historico)" : ""}; solicitado ${t.requested}, publicado ${t.published}, aplicado ${t.applied}, completado ${t.completed}`:"Sin diagnostico de giro recibido");
    const wait=(w)=>w?.available?`${w.state}, ${w.observations} muestras, ventana ${w.window_ms} ms, rango ${w.left_span_counts}/${w.right_span_counts} cuentas`:"sin informacion";
    text("navigation-stabilization",`Antes de retroceso: ${wait(s.stabilization)}. Antes de giro: ${wait(s.turn_stabilization)}.`);
    text("navigation-result",s.available?`Finalizacion: ${s.finish_reason}, odometria: ${s.odometry_error}`:"Sin resultado disponible");
  }
  async function poll() {
    const c=new Controller(),timer=schedule(()=>c.abort(),1500);
    try { const response=await fetcher("/api/navigation",{cache:"no-store",signal:c.signal});
      if(!response.ok)throw new Error("HTTP");render(await response.json());
    } catch {text("navigation-status","No se pudo actualizar navegacion: conservar resultados como historicos");}
    finally {clear(timer);schedule(poll,1000);}
  }
  return {render,start(){if(!started){started=true;void poll();}}};
}
if(document.getElementById("navigation-status")?.dataset.navigationPanel==="true")
  createNavigationPanel(document,fetch,setTimeout,clearTimeout,AbortController).start();
