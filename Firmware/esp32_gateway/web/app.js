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
  "ROBOT_STATE_IDLE · Reposo",
  "ROBOT_STATE_AUTONOMOUS · Autónomo",
  "ROBOT_STATE_MANUAL · Manual",
  "ROBOT_STATE_SAFETY_STOP · Parada de seguridad"
];
// Correspondencia con task_safety.h; los bits representan fallas enclavadas.
const faultNames = [
  [1, "SAFETY_FAULT_CLIFF · Caída"],
  [2, "SAFETY_FAULT_OBSTACLE · Obstáculo"],
  [4, "SAFETY_FAULT_SENSOR · Sensores"],
  [8, "SAFETY_FAULT_EMERGENCY · Emergencia"]
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
  if (mask === 0) return "0 · Sin fallas informadas";
  const labels = faultNames.filter(([bit]) => Math.floor(mask / bit) % 2 === 1).map(([, label]) => label);
  const unknown = mask - mask % 16;
  if (unknown) labels.push(`Bits sin definición local: ${unknown}`);
  return `${mask} · ${labels.join(" | ")}`;
}

function renderStatus(status) {
  lastStatus = status;
  lastStatusAt = performance.now();
  connectionBox.dataset.state = !status.valid ? "unknown" : status.connected ? "ok" : "error";
  readings.dataset.stale = String(!status.connected);
  setText("stm32-connection", !status.valid ? "Sin datos desde el arranque" :
    status.connected ? "STATUS reciente" : "Comunicación desactualizada");
  setText("stm32-detail", !status.valid ? "Todavía no se recibió una respuesta STATUS válida." :
    status.connected ? "Última respuesta UART válida; no implica autorización de movimiento." :
      "No hay STATUS válido reciente. Se conservan los últimos valores informados; no representan el estado actual.");
  setText("robot-state", status.valid ? `${status.state} · ${stateNames[status.state]}` : "Desconocido");
  setText("robot-faults", status.valid ? describeFaults(status.faults) : "Desconocidas");
  setText("robot-stop", status.valid ? status.stop === 1 ? "1 · Bloqueo activo" : "0 · Sin bloqueo informado" : "Desconocido");
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
