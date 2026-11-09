"use strict";

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
    observeCommand(status);
  } catch {
    connectionBox.dataset.state = "error";
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
  MANUAL: document.getElementById("command-manual")
};
let activeCommand = null;
const lastCommandAt = { STOP: -Infinity, RESET: -Infinity, MANUAL: -Infinity };

function updateCommandButtons() {
  for (const [command, button] of Object.entries(commandButtons)) {
    button.disabled = Boolean(activeCommand) && !(command === "STOP" && activeCommand.name !== "STOP");
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
          status.state === 2 && status.faults === 0 && status.stop === 0;
      if (matches) finishCommand(operation, `Estado posterior observado: ${stateNames[status.state]}. Revisá las protecciones y el bloqueo. Es una observación de estado, no un acuse correlacionado del comando.`);
    }
  }
}

async function sendCommand(name) {
  if (!Object.prototype.hasOwnProperty.call(commandButtons, name)) return;
  if (activeCommand && !(name === "STOP" && activeCommand.name !== "STOP")) return;
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
