# B6 - Comandos funcionales de movimiento

Base: feda552. Implementacion de desarrollo, pendiente de validacion fisica B6.
No incorpora navegacion autonoma ni autoriza alimentar motores.

## Arquitectura

Consola ESP32 -> UART2 -> DrvUart_RxCallback -> TaskComms (10 ms) ->
Motion_Execute -> TaskMotorControl_TrySetTargetSpeedsAt -> cola existente de un
elemento -> TaskMotorControl (10 ms) -> PI existente -> driver motor con techo
permanente provisional del 10%. No hay tarea nueva ni escritura directa de CCR.

Core/Inc/mid_motion.h y Core/Src/mid_motion.c contienen las cinco traducciones
de alto nivel. No se reutiliza mid_kinematics: ese modulo es un esqueleto sin
implementacion que expresa salidas PWM, no la interfaz de velocidad requerida.

Motion_Execute(command, required_mode, issued_tick) permite reutilizacion futura
desde tarea. El llamador debe proporcionar la fecha real de origen y el modo
que exige, sin renovar una solicitud retenida. UART exige MANUAL; no se agrega
un productor AUTONOMOUS. Un futuro navegador debe validar ademas sus datos y
politicas antes de cada emision. Esta funcion no cambia modos ni rearma Safety.

Se conserva TaskMotorControl_SetTargetSpeeds(left_cps, right_cps), con su firma
y semantica de emision desde tarea. Ambas interfaces comparten la misma ruta de
publicacion, generacion, consumo, PI y proteccion PWM. La variante fechada es
necesaria para no rejuvenecer una orden retenida en la recepcion UART y para
informar admision/rechazo sin cambiar la API void existente.

## Comandos

Tramas ASCII exactas, mayusculas y terminadas en LF; tambien se acepta CRLF.
La tecla se escribe en la consola existente del ESP32. No se aceptan numeros
de velocidad ni variantes de nombres. Esperar la respuesta antes de otra orden.

| Tecla | Trama STM32 | Objetivo izquierdo / derecho (cps) | Resultado |
|---|---|---|---|
| p | PING | Sin cambio | PONG |
| s | STATUS | Sin cambio | STATE, FAULTS, STOP existentes |
| r | RESET | Sin emision | RESET_QUEUED / RESET_QUEUE_ERROR |
| m | MANUAL | Sin emision | MANUAL_QUEUED / MANUAL_QUEUE_ERROR |
| t | STOP | Anulacion inmediata al procesar solicitud | STOP_QUEUED / STOP_QUEUE_ERROR |
| w | AVANZAR | +1600 / +1600 | MOVE_ACCEPTED / MOVE_REJECTED |
| x | RETROCEDER | -1600 / -1600 | MOVE_ACCEPTED / MOVE_REJECTED |
| a | GIRAR_IZQUIERDA | -1200 / +1200 | MOVE_ACCEPTED / MOVE_REJECTED |
| d | GIRAR_DERECHA | +1200 / -1200 | MOVE_ACCEPTED / MOVE_REJECTED |
| e | DETENER | 0 / 0, invalida objetivos y PI | MOVE_ACCEPTED / MOVE_REJECTED |

MOVE_ACCEPTED confirma admision/publicacion, no movimiento fisico. STOP/Safety
pueden invalidar la generacion antes del consumo. Una trama desconocida responde
ERR_UNKNOWN; una trama mal formada se descarta. MANUAL_QUEUED no confirma el
cambio de estado: consultar STATUS hasta STATE=2, FAULTS=0, STOP=0 antes de mover.

Los valores estan centralizados en MOTION_DRIVE_CPS y MOTION_TURN_CPS.
1600 cps genera, con encoder inicialmente quieto, termino proporcional de 800
cuentas (Kp=0,50), cercano pero inferior al techo fisicamente probado de 840.
Se eligio por debajo de los 1700 cps diagnosticos, evitando volver al caso de
100 cps que solo produjo unas 54 cuentas y no arranco el motor. Para el giro
se usa 1200 cps, termino proporcional inicial de 600, por debajo del rectilineo.
Son consignas provisionales, no velocidades calibradas en m/s ni garantia de
arranque. Integral, carga y feedback pueden llevar ambos casos hasta el techo
de 840. No se modifica Kp/Ki ni se incrementa automaticamente la consigna.

La tabla de giro supone ruedas en disposicion convencional. Verificar avance,
reversa y sentidos izquierdo/derecho sobre el chasis; el signo electrico y el
encoder coherentes no demuestran por si solos esa correspondencia mecanica.

## Frescura, STOP y concurrencia

- La ISR registra el tick del primer byte de cada trama. No ejecuta movimientos
  ni accede a colas. USART2 mantiene prioridad 5, compatible con la API ISR de
  FreeRTOS existente.
- Una orden de movimiento solo se admite en MANUAL, Safety sin bloqueo y sin
  STOP pendiente. Se exige edad <500 ms tanto al publicar como al consumir.
  Vence a edad >=500 ms desde el primer byte, no desde el procesamiento.
- No existe renovacion automatica, temporizador de movimiento ni reenvio ESP32.
  Cada nueva tecla/trama explicita puede emitir otra consigna acotada. Evitar
  mantener teclas pulsadas: la repeticion del terminal genera nuevas ordenes.
- El origen debe ser estrictamente posterior a la ultima parada y a la entrada
  efectiva al modo. Igual tick se rechaza conservadoramente; enviar una orden
  nueva mas tarde. Se usan restas de TickType_t, incluso al cruzar el wrap;
  como en B1, se presupone ejecucion del scheduler, sin retener datos un modulo
  completo del contador de ticks.
- STOP marca pendiente, invalida generacion, limpia objetivos/integrales y
  escribe PWM cero antes del encolado de su evento. Si la cola FSM esta llena,
  permanece la parada y la FSM completa IDLE en su siguiente procesamiento.
  Los modos y publicaciones se bloquean mientras STOP esta pendiente.
- El encolado STOP es no bloqueante, fuera de la seccion critica, con suspension
  breve del scheduler para ordenar marca y evento. Las IRQ siguen activas;
  Safety que se haga lista ejecuta su bloqueo al reanudarse el scheduler.
- DETENER usa la misma anulacion de objetivos y PI, pero permanece en MANUAL.
  No permite ejecutar despues un movimiento UART anterior a esa parada.
- La cola conserva generaciones B1: STOP/Safety concurrentes entre admision y
  encolado invalidan la orden tardia. El consumo y las escrituras PWM mantienen
  sus secciones criticas existentes. RESET/MANUAL no emiten consignas.
- Recepcion acotada: un mensaje pendiente y otro buffer de ensamblado. STOP
  reemplaza cualquier pendiente, DETENER reemplaza trafico ordinario, y ninguna
  orden ordinaria reemplaza una parada pendiente. Las rafagas ordinarias pueden
  perderse; no existe backlog de movimientos. Se descarta overflow hasta LF,
  evitando ejecutar sufijos; NUL embebido/control/CR mal formado tambien se
  descartan. Se conserva tolerancia al NUL inicial de arranque.

STOP UART requiere recepcion valida y ejecucion de TaskComms (base 10 ms mas
jitter); no es un corte electrico ni una accion inmediata desde la IRQ. Las
respuestas siguen usando la transmision HAL existente, acotada a 50 ms por
llamada, fuera de secciones criticas. No se garantizan entrega sin errores,
autenticacion, recuperacion de errores UART ni deteccion de una corrupcion que
coincida con otra trama valida. Ante perdida de comunicacion, la consigna vence.

Safety conserva arranque bloqueado, enclavamiento, rearme explicito, ambas
distancias >0,20 m, IR sin caida, snapshot <=100 ms y mediciones <=200 ms.
Los 200 ms siguen siendo provisionales; no constituyen homologacion de frenado
ni autorizacion de movimiento autonomo. PWM cero requiere CPU y scheduler
funcionando; no pausar Debug con potencia conectada suponiendo que vencera.

## Compilacion y pruebas

Desde la raiz, con GNU Arm Embedded y MinGW make en PATH:

```powershell
Push-Location Firmware/stm32_control
New-Item -ItemType Directory -Force -Path build/b6/firmware | Out-Null
mingw32-make -B -o build/b6/firmware -j4 BUILD_DIR=build/b6/firmware
Pop-Location
pio run -d Firmware/esp32_gateway -e esp32
& Firmware/stm32_control/Tests/safety_b11/run.bat
& Firmware/stm32_control/Tests/ultrasound/run.bat
```

ELF STM32: Firmware/stm32_control/build/b6/firmware/stm32_control.elf.
ELF ESP32: Firmware/esp32_gateway/.pio/build/esp32/firmware.elf.
No ejecutar upload. En CubeIDE reconstruir desde los fuentes actuales; comprobar
ruta y fecha del ELF antes de una futura sesion autorizada, sin editar launch.

Validacion local B6: 165/165 casos (123 anteriores +42 nuevos) y 19/19
ultrasonicos. Se ejercitan los fuentes reales de Motion, UART y TaskComms junto
con FSM/Safety/MotorControl/PI/driver existentes. HAL/RTOS/sensores son sustitutos
deterministas; no se afirma validacion fisica. Cobertura B6: mapeos y signos,
modos, Safety, STOP con cola llena, mismo tick, rearme, carreras al encolar,
500 ms, wrap/tick cero, prioridad de recepcion y tramas invalidas.

STM32 compila completo sin advertencias. ESP32 compila con advertencia de
configuracion local: placa espera 4 MB, sdkconfig indica 2 MB; no se cambia la
configuracion. Verificar capacidad real/configuracion antes de programar ESP32.

## Primera validacion fisica propuesta, posterior a revision

1. Sin potencia de motores, cargar ambos firmwares solamente tras autorizacion.
   Verificar PING/STATUS, arranque bloqueado y rechazo de w/x/a/d/e fuera de
   MANUAL. Preparar sensores seguros, RESET, MANUAL y confirmar STATE=2 sin
   fallas. Ninguna de estas operaciones debe producir PWM por si sola.
2. Enviar w una sola vez, sin repeticion de teclado. Observar objetivos +1600
   y ambos CCR <=840 con ARR=8399. Despues de 500 ms mas un ciclo/jitter,
   ambos objetivos/integrales/PWM deben quedar cero sin cambiar de modo.
3. Repetir x/a/d individualmente, con pausa entre pruebas; verificar signos.
   Probar e durante una orden: cero sin salir de MANUAL. Probar t: cero e IDLE.
   Intentar otra orden despues de t y antes de nuevo MANUAL: debe rechazarse.
4. Con una orden vigente, provocar una condicion sensorial insegura supervisada:
   PWM cero y bloqueo enclavado. Retirar causa y RESET/MANUAL sin otra orden:
   PWM debe permanecer cero. Probar tambien perdida de comunicacion y comandos
   desconocidos. No detener la CPU durante un ensayo con potencia.
5. Solo con autorizacion adicional: ruedas elevadas, corte de potencia accesible,
   alimentacion de ensayo de 6 V y limite de corriente supervisado. Probar una
   orden cada vez y confirmar giro/montaje y retorno a cero. B6 acciona ambos
   motores; el consumo conjunto y las inversiones aun requieren validacion.
   No aumentar tension, corriente, ganancias ni techo PWM para lograr arranque.

No se calibra velocidad mecanica, frenado ni perfiles de aceleracion. Quedan
pendientes dinamica al invertir, corriente conjunta, duracion real bajo carga,
latencias/jitter/pila y seguridad independiente ante CPU detenida.
