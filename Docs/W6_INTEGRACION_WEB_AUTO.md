# W6 — Integración de navegación autónoma y web

## Alcance

Se integra `G1-add/NavegacionAutonoma` con W1–W5. No se inicia AUTO al
arrancar, cargar la página, reconectar Wi-Fi ni recuperar STATUS. No se
incorporan nuevas tareas, colas, protocolos de movimiento ni cambios de Safety.
La integración por software no autoriza movimiento físico.

## Solicitud y confirmación

`POST /api/command` admite los cuerpos exactos `STOP`, `RESET`, `MANUAL` y
`AUTO`, con las validaciones existentes de Origin/Host, Content-Type
`text/plain`, longitud e identificador hexadecimal `X-Request-ID` de 32 caracteres.
No admite instrucciones UART arbitrarias. Se conserva la misma ruta HTTP y
el historial acotado de idempotencia; no hay reintentos automáticos.

AUTO exige STATUS válido de IDLE (0), fallas=0, bloqueo=0 y edad estrictamente
inferior a 2000 ms. Se exige una respuesta posterior, no del mismo instante,
a las barreras locales de STOP/RESET/cambio de modo/detención. Un rechazo de
condiciones devuelve HTTP 412 `auto_requires_safe_idle`; otra solicitud operativa
pendiente devuelve 409. Un ID ya conocido conserva su resultado, sin repetirlo.

HTTP 202 significa solicitud encolada en ESP32. La entrega completa de `AUTO\n`
a UART queda en `phase=transmitted`. `AUTO_QUEUED` de STM32 tampoco demuestra
una transición. El navegador informa AUTO solamente al observar STATUS posterior
con STATE=1, FAULTS=0 y STOP=0. Es una observación, no un acuse correlacionado
del protocolo. La FSM puede seguir en AUTO aunque la navegación esté DETENIDA;
STATUS no publica sus subestados.

La tecla `n` de consola comparte la cola y las mismas validaciones; ya no
escribe directamente a UART. STOP/RESET/MANUAL y DETENER invalidan solicitudes
AUTO pendientes mediante una época. La antigüedad máxima de una solicitud
ESP32 es 500 ms; estado, época y edad se revalidan después de esperar el turno
TX y antes de escribir, con permiso de un solo uso. La STM32 conserva además
sus propias barreras, época y antigüedad de 500 ms desde el primer byte UART.
Los sucesos ambiguos del mismo tick STM32 se rechazan conservadoramente.

Se revoca cualquier sesión manual al solicitar AUTO. El controlador manual
requiere siempre STATE=2 y no puede renovar durante AUTO. Si corresponde,
DETENER se entrega antes de AUTO. Una detención concurrente puede cancelar
la petición, que requiere luego una nueva acción explícita. STOP sustituye la
solicitud aún pendiente y sigue a una trama cuya escritura ya se comprometió;
ninguna capa promete retirar bytes ya entregados al periférico.

## Navegación conservada

`TaskControlRobot` procesa eventos y llama `Navigation_Step()` cada 20 ms.
`mid_navigation` usa instantáneas de sensores, `TaskSafety_EvaluateSnapshot`,
`mid_motion` y la vía exclusiva del controlador de motores. Se preservan la
identidad de tarea, sesión, fase, permiso de emisión, generación y fechas de
origen tanto al emitir como al consumir consignas.

| Fase | Comportamiento |
| --- | --- |
| ESPERAR_DATOS | Cero hasta disponer de información segura; obstáculo inicial no provoca retroceso. |
| AVANZAR | +1600/+1600 cps mientras frontal >300 mm y todas las condiciones siguen vigentes. |
| RETROCEDER_ACOTADO | Tras detener el avance por obstáculo, −1600/−1600 cps, frontal >200 mm y trasero >300 mm, como máximo 300 ms. |
| GIRAR_ACOTADO | Solo experimental, después de retroceso completado, hasta 250 ms, inicialmente −1200/+1200 cps. |
| DETENIDO | Terminal: cero, sin renovación ni reanudación automática. |

`NAV_EXPERIMENTAL_TURN_ENABLED` vale **0 por defecto**. La compilación operativa
de W6 no habilita el giro; los dos sentidos se verifican solo en pruebas
nativas con el flag de compilación. Se conserva el límite global de 2000 ms,
incluida la espera y las maniobras; el giro no comienza sin margen para su
intervalo completo. Parámetros centralizados en `mid_navigation.h`.

Safety conserva 200 mm, validez de ambos ultrasónicos, snapshot hasta 100 ms,
capturas hasta 200 ms y prioridad de caída IR. No cambian enclavamiento ni
rearme. El límite PWM sigue en 10 %, PI y encoders sin cambios. Al detener se
invalidan objetivos, permisos e integrales; el timeout permanente sigue en
500 ms. STOP vuelve a IDLE salvo falla Safety enclavada. RESET no restaura AUTO.

## Resoluciones de merge

- `task_control_robot.c`: se conservan `mid_log` y `mid_navigation`, mensajes
  de eventos fechados y barreras AUTO; STOP registra 300 una sola vez y RESET
  no se convierte en STOP. Las transiciones efectivas conservan el evento 110.
  La fecha/época AUTO se comprueba dentro de la sección crítica de transición,
  pero la publicación al logger se ejecuta fuera de ella.
- `test_safety.c`: se conservan los ensayos de productores del datalogger y
  telemetría junto con B7.2/B7.3/B7.5/B7.7. Se agregan regresiones W6 sobre
  las rutas reales combinadas.
- En el buzón UART STM32 se clasifica AUTO como orden de modo, junto a MANUAL
  y RESET: STOP > DETENER > movimiento/modo > STATUS > SENSORS. No se cambia
  hardware ni el formato de STATUS/SENSORS.
- Los módulos de sensores, logger, Safety, motores y navegación integrados
  automáticamente conservan sus funciones: el logger sigue fuera de las
  secciones críticas, y la API común de evaluación reutiliza las mismas reglas.

## Comunicaciones y límites

ESP32 mantiene un lector UART y el mutex TX existente; los handlers HTTP no
transmiten. STATUS sigue cada segundo y SENSORS cada 500 ms cuando el arbitraje
lo permite. Se conservan la pausa durante sesión manual, los paneles W1–W5,
la cruceta, los registros UART W4.9 y `lru_purge_enable=true`.

La pérdida de Wi-Fi/UART no equivale a un STOP recibido. No se añade un nuevo
watchdog de comunicación: una sesión AUTO ya aceptada puede continuar hasta
su límite de 2 s si los sensores siguen siendo seguros. El plazo de 500 ms es
de consignas de motor, que navegación renueva durante su sesión autorizada.
Los 2 s no garantizan parada con CPU/scheduler detenido. STOP web depende del
enlace y no constituye una parada de emergencia física certificada.

Los ultrasonidos no cubren los laterales ni garantizan frenado a 300 mm.
No se deducen desplazamientos a partir de cps o duración. W7, el mapa 2D,
permanece pendiente: ver `W7_REQUISITOS_MAPEO.md`.

## Verificación reproducible

- STM32: crear el directorio ignorado `build/w6`, luego ejecutar desde
  `Firmware/stm32_control`:
  `mingw32-make -B -o build/w6 -j4 BUILD_DIR=build/w6`.
- ESP32: definir `PLATFORMIO_BUILD_DIR` con la ruta absoluta a
  `Firmware/esp32_gateway/.pio/w6-build`, luego
  `pio run -d Firmware/esp32_gateway -e esp32`.
- STM32: `Tests/safety_b11/run.bat` (normal y giro experimental izquierdo/derecho),
  `Tests/ultrasound/run.bat`, `Tests/sensors_w5/run.ps1`.
- ESP32: `Tests/w2/run.ps1` compila módulos reales contra mocks SDK;
  `node Tests/w2/run_js.js` ejecuta JavaScript sin dependencias (también se puede
  usar el mismo runner en V8 suministrando sus fuentes).

Los builds separados evitan sobrescribir ELF previos. No usar un ELF antiguo
para validar W6. Los mocks no prueban UART físico, tiempos reales, frenado,
SD ni espacio barrido por el chasis.

### Resultados de software W6

- STM32: enlace completo correcto, sin advertencias; `text=81692`, `data=924`,
  `bss=32988` bytes. RAM estática `data+bss=33912` bytes; el heap FreeRTOS ya
  forma parte de BSS, y su disponibilidad efectiva y margen de pila requieren
  observación en placa. La cola FSM ahora almacena mensajes fechados, no enums.
- ESP32: compilación correcta, sin advertencias, aplicación 881605/1048576 bytes
  (84,1 %), RAM estática 34160/327680 bytes (10,4 %); no es heap libre en ejecución.
- Pruebas STM32: 406 casos con giro deshabilitado, 55 de giro experimental
  izquierdo y 55 derecho, 19 del driver ultrasónico y 7 de adquisición W5.
- ESP32: 76 grupos nativos (63 previos y 13 W6) y 82 grupos JavaScript
  (11 W2, 17 comandos W3/W6, 21 W4, 12 W5 y 21 W4 con sensores indisponibles).
  El runner JavaScript real se ejecutó en V8; Node no está instalado en el entorno.
- ELF separados: `Firmware/stm32_control/build/w6/stm32_control.elf` y
  `Firmware/esp32_gateway/.pio/w6-build/esp32/firmware.elf`.

## Validación física posterior, exclusivamente sin potencia

Tras revisión y autorización para cargar firmware: verificar arranque IDLE y
objetivos/CCR cero; consultar STATUS/SENSORS; rearmar con capturas nuevas seguras;
solicitar AUTO explícitamente desde IDLE y observar STATE=1; comprobar rechazo
de dirección manual, parada preventiva/retroceso acotado y fin a cero en 2 s.
Comprobar STOP durante avance y retroceso, caída IR, sensores inválidos y rearme
sin reactivación. Revisar EVENTOS.CSV para 110/300 y fallas, sin duplicados.
El giro debe seguir deshabilitado. Verificar desconexión y recuperación de web
sin nueva solicitud AUTO. La potencia de motores permanece desconectada.

El merge queda abierto: los marcadores se resuelven por edición, pero Git
mantiene ambos archivos como `UU` hasta que el usuario revise y actualice el
índice. No se finaliza, cancela ni registra el merge desde esta etapa.
