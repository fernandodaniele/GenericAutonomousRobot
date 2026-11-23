# Panel web y vista de mapa

Base: `G1-improve/InterfazWeb`, `5470271`. Una sola interfaz offline en ESP32.
No se modifica STM32, UART, contratos HTTP ni configuración del servidor.

## Organización

- Encabezado fijo con STOP y enlace al mapa; estado de STM32, Safety y bloqueo
  separados de la comprobación del servidor ESP32.
- AUTO_EXPLORE como inicio habitual. Conserva la confirmación explícita de
  supervisión, la aceptación de API separada de la confirmación STM32 y la
  continuidad de sesión independiente del navegador. Sin inicio automático.
- Mapa destacado, distancias frontal/trasera y resumen de cuatro IR; una
  adquisición inválida o antigua sigue identificándose como tal.
- MPU6050: estado y calibración manual únicamente. Se conservan todos los
  requisitos de calibración y la telemetría avanzada en `/api/imu`.
- Control manual desplegable con la cruceta existente, pulsación sostenida y
  cancelación al soltar, perder interacción o cerrar el panel. Abrir el panel
  no solicita MANUAL ni mueve el robot; su botón de modo sigue siendo explícito.
- Ensayos históricos y diagnóstico en una sección cerrada por defecto. Las
  casillas de autorización se inician desmarcadas; sus comandos siguen intactos.

No se agregan frameworks, fuentes, imágenes remotas, servicios ni dependencias.
Los ocho recursos existentes siguen embebidos por el mismo CMake/PlatformIO.

## Transformación visual del mapa

Se conserva `/api/map` y la grilla de 80 × 80 celdas de 50 mm (4 × 4 m).
`mapValidate` y los datos recibidos no cambian. Un viewport independiente
define el centro y zoom 1×/2×/4×/8×. La proyección original sigue siendo:

`pixelX = (x - origenX) / anchoMundo * anchoCanvas`

`pixelY = altoCanvas - (y - origenY) / altoMundo * altoCanvas`

El viewport transforma las celdas, la trayectoria y la pose por igual. Centrar
usa la última pose métrica recibida; no sigue ni extrapola movimientos.
Mapa completo restaura la extensión original. Un nuevo epoch/configuración
restaura la vista, sin alterar coordenadas ni reiniciar el estimador.

La escala se calcula con el ancho visible real. El marcador mantiene tamaño
legible en píxeles CSS: es un símbolo, no la huella medida del chasis. La flecha
se orienta con `theta_mrad`, X positivo a derecha y Y positivo hacia arriba.

Los estados desconocido, libre observado y ocupación posible conservan sus
valores y se distinguen por color y leyenda. Las guías métricas no significan
espacio libre. La vista histórica reduce opacidad y mantiene sus avisos y edad.
No se pinta un robot cuando la pose es inválida o inexistente. Los cambios de
vista no generan HTTP adicional, POST ni órdenes; solo continúan los GET
existentes. La edad se actualiza sin repintar toda la grilla a cada tick visual.

## Verificación de software

- JavaScript real, DOM/HTTP/reloj sustituidos: **185 grupos aprobados** con
  `Tests/w2/run_js.js` en V8. Incluye las regresiones de W1–W7, IMU, comandos,
  exploración y control manual; zoom/centrado/escala, límites, epoch, inmutabilidad,
  histórico, símbolo móvil y cancelación al cerrar el panel.
- Pruebas nativas ESP32: **217 grupos aprobados**, con los recursos finales
  embebidos. No se atribuyen ensayos de protocolo físico a los mocks.
- Chrome headless: **130 comprobaciones aprobadas** en ocho escenarios, con
  viewports CSS de 320, 375, 390, 768 y 1280 px. Incluye IDLE, MANUAL, AUTO,
  Safety, ausencia de muestra y pérdida de enlace; DOM sin IDs duplicados ni
  excepciones, ningún POST al cargar, cruceta, superficies táctiles y STOP
  visible al desplazar la página. La API de prueba solo escucha en localhost;
  bloquea POST y marca las capturas como **datos sintéticos**. No se embebe.
- Compilación PlatformIO satisfactoria, sin errores ni advertencias nuevas.

| Recurso | Base | Final |
| --- | ---: | ---: |
| Flash reportada por PlatformIO | 968161 | 973393 bytes (92,8 %) |
| RAM estática reportada | 80368 | 80368 bytes (24,5 %) |
| BIN de aplicación | 968560 | 973792 bytes |
| Margen real de BIN hasta 1 MiB | 80016 | 74784 bytes |

El incremento es de 5232 bytes. El resto de RAM no equivale a heap libre ni
describe la memoria del navegador. Las cadenas HTTP analizadas del ELF
suman 3088 y 2048 bytes sobre 4096: cotas parciales, sin medición física de
high-water mark. No se cambia el stack HTTP ni se agregan tareas.

Comandos reproducibles desde la raíz:

```powershell
pio run -d Firmware/esp32_gateway -e esp32
powershell -ExecutionPolicy Bypass -File Firmware/esp32_gateway/Tests/w2/run.ps1
# Si Node está disponible; la misma función del runner se ejecutó en V8:
node Firmware/esp32_gateway/Tests/w2/run_js.js
# Chrome local, Python estándar, sin paquetes adicionales ni acceso al robot:
python Firmware/esp32_gateway/Tests/w2/test_dashboard_browser.py
git diff --check
```

Logs, resultados JSON, capturas con fixtures y diff completo se conservan en
`Firmware/esp32_gateway/.pio/build/ui_redesign/` (ignorado por Git).

## Validación posterior en hardware

Después de revisión y programación autorizada de **ESP32 únicamente**:

1. Conectar PC/iPhone a GenericRobot, abrir la interfaz sin Internet y recargar
   los recursos. Verificar servidor y enlace STM32 por separado. Safari real
   queda pendiente: Chrome headless no lo sustituye.
2. Sin solicitar movimiento, comprobar estados y antigüedad contra `/api/status`,
   `/api/sensors`, `/api/imu`, `/api/map` y `/api/exploration`. Comprobar que un
   enlace perdido o un dato antiguo nunca se muestre como actual.
3. Probar zoom, centrar y mapa completo: coordenadas, celdas y trayectoria
   deben permanecer iguales en la API. Conservar advertencia provisional.
4. Inmóvil y en estado permitido, calibrar el giroscopio y observar progreso,
   aceptación y resultado real. El botón no debe habilitarse durante movimiento.
5. En el escenario previamente autorizado, comprobar inicio explícito de
   exploración, STOP y rearme. Desplazar la página y comprobar acceso a STOP.
   El STOP remoto sigue dependiendo de la comunicación; mantener corte físico.
6. Verificar la cruceta y cancelar al soltar, cerrar el panel, perder foco u
   ocultar la página. Reabrir/reconectar nunca debe reanudar una pulsación.

Sin validación física en esta revisión. Se conservan deriva, incertidumbre de
HC-SR04, cobertura lateral incompleta y dependencia de calibración: el mapa
continúa siendo diagnóstico y no una autoridad de seguridad.
