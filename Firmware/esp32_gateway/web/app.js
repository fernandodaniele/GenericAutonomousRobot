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
    statusDetail.textContent = "Revisá la conexión a la red Wi-Fi del robot y volvé a comprobar. El enlace STM32 no se evalúa desde esta página.";
  } finally {
    clearTimeout(timeout);
    checkButton.disabled = false;
  }
}

checkButton.addEventListener("click", checkHealth);
checkHealth();
