# AUTO-3: giro odometrico experimental supervisado

Implementacion sobre `G1-add/InterfazWeb`, HEAD base `0dd462c`.
Validacion de esta entrega: software exclusivamente. No se grabaron placas ni
se realizaron movimientos fisicos. AUTO-2F sigue siendo el perfil normal.

## Arquitectura y seleccion

Se mantiene una unica maquina de navegacion en `mid_navigation`, ejecutada
por TaskControlRobot (20 ms), y la ruta Motion -> MotorControl existente.
No hay nuevas tareas, colas ni controladores. Safety, PI/PWM, drivers,
odometria global, geometria, CPR, IMU/fusion y mapa permanecen sin cambios.

| Solicitud UART exacta | Perfil NAV | Accion |
|---|---|---|
| `AUTO\n` | 2 / `AUTO_2` | AUTO-2F normal: avance 100 mm, posible retroceso 20 mm, sin giro. |
| `AUTO_TURN\n` | 3 / `AUTO_3_TURN` | Giro aislado, previo chequeo de sensores y estabilizacion. |
| `AUTO_EVADE\n` | 4 / `AUTO_3_EVASION` | Secuencia finita de avance, retroceso y giro supervisado. |

Las solicitudes reutilizan START_AUTO y responden `AUTO_QUEUED` o
`AUTO_REJECTED`. Encolado no confirma transicion ni movimiento. Se admiten
solo desde IDLE sin Safety ni STOP pendiente y con las barreras temporales
y generaciones originales; se revalidan al consumir el evento. El perfil
viaja por valor dentro del evento, nunca en una seleccion global pendiente.
La cola existente conserva ocho entradas; el mensaje incorpora un uint8_t
(con alineacion, cuatro bytes adicionales por entrada en STM32).

Los dos comandos nuevos estan en la lista cerrada de `POST /api/command`,
Content-Type `text/plain`, cuerpo exacto `AUTO_TURN` o `AUTO_EVADE` y
X-Request-ID hexadecimal de 32 caracteres. Se conservan Origin/Host,
idempotencia, prioridad STOP, vencimiento de solicitud y validacion STATUS
IDLE reciente (<2000 ms). HTTP no transmite UART: lo hace el bucle existente.
El permiso de envio tambien se vincula al comando exacto del recibo pendiente.

El dashboard exige marcar supervision y espacio lateral despejado para cada
ensayo; consume la marca al solicitarlo. Consultar, recargar, reconectar,
RESET o finalizar una fase no inicia otra maniobra. La consola `n` y AUTO
normal no seleccionan perfiles experimentales. Los defines historicos
NAV_EXPERIMENTAL_TURN_ENABLED/NAV_TURN_MAX_MS se conservan para regresiones;
no se habilitan en la compilacion operativa normal.

## Secuencias y limites

- Aislado: ESPERAR_DATOS -> DETENER_Y_ESPERAR -> GIRAR_ACOTADO -> DETENIDO.
- Integrado: ESPERAR_DATOS -> AVANZAR -> DETENER_Y_ESPERAR ->
  RETROCEDER_ACOTADO -> DETENER_Y_ESPERAR -> GIRAR_ACOTADO -> DETENIDO.
- Sin obstaculos, el avance integrado conserva el objetivo de 100 mm y finaliza
  detenido al alcanzarlo; no busca obstaculos indefinidamente.
- Obstaculo ya presente al entrar: no inicia avance ni retroceso/giro.

| Parametro | AUTO-2F | AUTO-3 |
|---|---|---|
| Avance | +1600/+1600 cps, 100 mm, hasta 2000 ms | Igual en integrado; ausente en aislado |
| Frontal preventivo | 275 mm | Sin cambios |
| Trasera para retroceso/giro | >250 mm | Sin cambios |
| Safety critico | 200 mm frontal y trasero | Sin cambios |
| Retroceso | -1600/-1600 cps, 20 mm o 800 ms | Igual |
| Giro | Deshabilitado | Izquierda, -1200/+1200 cps, 10 grados o 800 ms |
| Global | 3000 ms | Aislado 1400 ms; integrado 4200 ms |
| Discrepancia de recorridos | 10 mm | Solo en giro AUTO-3: 3 mm |

El global integrado reserva 2000+200+800+200+800=4000 ms y 200 ms para
transiciones/referencias. Los 1400 ms del aislado incluyen espera de datos,
estabilizacion y giro (presupuesto orientativo 200+200+800+200).
Ningun cambio de fase reinicia el inicio global. Si no queda presupuesto
para estabilizacion y giro completos, se rechaza esa continuacion.
Los tiempos individuales incluyen la espera de referencia posterior a cada
transicion. El tiempo de estabilizacion no consume el plazo propio del giro.

## Estabilizacion y referencia

Se reutiliza la misma fase de espera con un destino interno (retroceso/giro).
En toda la espera los objetivos y PWM permanecen en cero. Se exigen cuatro
observaciones realmente nuevas, una ventana >=60 ms y rango maximo de una
cuenta por rueda. Si una rueda sigue moviendose se reinicia la ventana;
no se renueva el timeout de 200 ms. Muestras repetidas no aumentan el contador.

Tras confirmar estabilidad se cambia de fase sin emitir movimiento; una
muestra odometrica estrictamente posterior establece la nueva referencia de
conteos. No se reinicia la odometria global. Primera y segunda estabilizacion
se almacenan independientemente, incluyendo sus referencias.

## Control angular y cancelacion

Con dL=deltaL*pi*DL/CPRL y dR=deltaR*pi*DR/CPRR, el giro izquierdo exige
dL<=0 y dR>=0. El incremento relativo es (dR-dL)/base. Se usa el promedio
de recorridos proyectados al signo esperado y se compara con base*angulo/2.
Con base=149,2 mm, 10 grados corresponden a 13,02 mm por rueda. La referencia
por cuentas evita restar directamente orientaciones globales envueltas.

La guarda de 3 mm limita a 1,5 mm el desplazamiento ideal del centro debido
al desequilibrio entre ruedas. Es experimental: no detecta deslizamientos y
no corrige trayectorias. Se conserva la vigilancia de 1 mm/200 ms (equivale
aproximadamente a 0,768 grados de giro por ventana), signos, overflow,
coherencia de timestamps y odometria valida con antiguedad <=50 ms.

Cada transicion revoca consignas y permisos de la fase anterior mediante
TaskMotorControl_Stop(). MotorControl comprueba tarea de origen, modo,
generacion, sesion, fase, vencimiento y permiso de un solo uso. Al alcanzar
el objetivo cancela la actuacion antes del siguiente ciclo de navegacion.
Aplicar una consigna no demuestra movimiento: este se verifica por encoders.

El giro requiere frontal >275 mm y trasera >250 mm, tambien durante su
estabilizacion. Retroceder 20 mm no garantiza disponer de esa nueva medicion
frontal: si no se confirma, termina EVASION_NOT_AUTHORIZED.
Sensores invalidos/antiguos, IR, Safety, STOP, error odometrico, falta de
progreso y vencimientos siempre terminan detenidos. Se conserva la primera
causa terminal; no hay recuperacion ni rearme automaticos.
Aislado completado: TARGET; integrado con giro completado: EVASION_COMPLETED.
La FSM puede permanecer en AUTO despues de finalizar. Para otro ensayo:
STOP -> IDLE confirmado -> nueva solicitud explicita.

## Telemetria compatible

STATUS y SENSORS permanecen intactos. `NAV Q=N` conserva formato y campos,
agregando perfiles 3 y 4 a los existentes. `NAV Q=N S=1` conserva NAVW para
la primera estabilizacion. La parte nueva `NAV Q=N S=2` devuelve:

```text
NAVT,1,Q,SESSION,START,FLAGS,PROGRESS_MDEG,ELAPSED_MS,WAIT_STATE,WAIT_MS,OBS,WINDOW_OBS,WINDOW_MS,LSPAN,RSPAN,REF_TICK,REF_L,REF_R,ENTRY_TICK,SNAPSHOT_AGE_MS,TARGET_MDEG,LEFT
```

FLAGS: solicitado=1, publicado=2, aplicado=4, completado=8, progreso medido=16.
LEFT=1 izquierda, 0 derecha. WAIT_STATE: 0 no solicitada, 1 esperando,
2 confirmada, 3 timeout, 4 cancelada. Referencia/entrada ausentes: `-`;
conteos int64 con signo; timestamp cero es valido. Cada linea <=255 bytes.
El angulo se publica en miligrados, redondeado; procede de encoders.

NAV, NAVW y NAVT comparten la copia/Q de TaskComms, vigente durante <1000 ms.
ESP32 mantiene un solo lector. Consulta base a 1 Hz y partes complementarias
sin acumular solicitudes; NAVT solo se consulta para perfiles AUTO-3. La
telemetria sigue debajo de controles, STATUS, sensores e IMU en el arbitraje.
No se garantiza observar en vivo cada fase de pocos cientos de ms.

`GET /api/navigation` conserva sus campos e incorpora:
- profile: AUTO_3_TURN / AUTO_3_EVASION cuando corresponde.
- turn: available, age_ms, historical, requested, published, applied,
  completed, target_deg, progress_deg, elapsed_ms, direction.
- turn_stabilization: available, state, elapsed_ms, observations,
  window_observations, window_ms, left_span_counts, right_span_counts,
  snapshot_age_ms, turn_entry_tick, reference_tick y cuentas int64 como strings.
- stabilization sigue describiendo exclusivamente la espera previa al retroceso.

Los metadatos se retienen tras STOP/IDLE sin rejuvenecer sus edades. Una
sesion nueva limpia los anteriores; partes ausentes no se inventan.
La respuesta HTTP usa buffer estatico de 4096 bytes, fuera del stack.
Todos los snprintf comprueban capacidad y fallan sin enviar JSON truncado.
El mutex protege una copia consistente, no la transmision HTTP.

## Verificacion de software

| Suite | Resultado |
|---|---|
| STM32 AUTO-2F + AUTO-3 (opciones predeterminadas) | 508/508 |
| AUTO-1 | 363/363 |
| AUTO-2/3, variante historica experimental izquierda | 534/534 |
| AUTO-2/3, variante historica experimental derecha | 534/534 |
| B7 sin odometria | 436/436 |
| Giros B7 izquierda / derecha | 55/55 cada uno |
| Odometria / adquisicion sensores / IMU / ultrasonidos | 28 / 8 / 48 / 19 aprobados |
| ESP32 nativo, HTTP/UART/concurrencia incluidos | 190/190 |
| JavaScript W1-W7 e IMU, incluidos panel y controles AUTO-3 | 162 grupos aprobados |

Los tests usan conteos y relojes sinteticos solo en arneses. Incluyen bucle
real MotorControl sin reinicios entre fases, residuos negativos al terminar
retroceso, referencias independientes, limites 3 mm/800 ms/1400 ms/4200 ms,
reordenamientos, STOP/Safety, falta de progreso, tick wrap, retencion,
concurrencia de snapshots y respuestas del formateador STM32 procesadas por
el parser/HTTP reales de ESP32 para ambos perfiles.
Las expectativas antiguas de comando/parte/perfil desconocido se trasladan
a valores realmente desconocidos; los limites JSON siguen verificandose
contra su nueva capacidad. No se eliminaron regresiones existentes.

Compilacion STM32 completa: text=105576, data=948, bss=36760 bytes;
RAM estatica=37708 bytes. ELF normal: `Firmware/stm32_control/build/stm32_control.elf`.
ESP32: Flash=946485/1048576 bytes (90,3%), RAM estatica=77040/327680 (23,5%).
Margen de particion de aplicacion=102091 bytes; no equivale a heap libre.
Ambas compilaciones finalizaron sin errores ni advertencias.
Stack HTTP: cadena analizada en ELF=2848/4096 bytes, diferencia 1248;
frame del handler=64 bytes. Es una cota de frames, no un high-water mark ni
una certificacion del stack completo del SDK. No se aumento el stack.

Comandos: `Tests/safety_b11/run.bat`; scripts run.ps1 de odometria, sensores,
IMU; `Tests/ultrasound/run.bat`; ESP32 `Tests/w2/run.ps1`.
JavaScript se ejecuto con el runner existente en V8, sin Node instalado.
Resumen: `.pio/build/auto3_javascript_results.json`.
Build STM32: `mingw32-make -B -o build -j4 "CC=arm-none-eabi-gcc -fstack-usage"`.
Build ESP32: `pio run -d Firmware/esp32_gateway -e esp32`.
Logs STM32: `build/auto3_*.log`; ESP32: `.pio/build/auto3_*.log`.

## Archivos y alcance

STM32: mid_navigation.c/.h (perfiles, espera reutilizada, control y diagnostico),
mid_motion.c/.h y task_motor_control.c (compilar la ruta existente de giro,
con barreras de autorizacion), task_control_robot.c/.h (perfil en evento),
task_comms.c (dos comandos explicitos y consulta NAVT).
ESP32: uart_comm.c/.h, web_commands.c/.h (lista cerrada/arbitraje),
stm32_navigation.c/.h (tercera parte, retencion y JSON), web_server.c
(capacidad estatica/lista de comandos), app.js e index.html (ensayos y panel).
Se agregan test_navigation_auto3.c y test_auto3.c; se amplian los arneses y
regresiones existentes. No se modificaron main, drivers, Safety ni configuraciones.

## Programacion y validacion posteriores: requieren revision previa

No se ejecutaron estos pasos. Tras aprobar la entrega:
1. Desconectar bateria; retirar ambas placas de la PCB antes de conectar USB,
   siguiendo el montaje usado en las etapas IMU. El MPU6050 no estara alimentado.
2. STM32CubeProgrammer/ST-LINK o el flujo CubeIDE habitual: seleccionar
   `Firmware/stm32_control/build/stm32_control.elf`, descargar y verificar.
   El ELF contiene las direcciones; el binario alternativo comienza en 0x08000000.
3. ESP32 por USB, en su puerto real: ejecutar manualmente
   `pio run -d Firmware/esp32_gateway -e esp32 -t upload --upload-port COMx`.
   Sustituir COMx por el puerto comprobado. Este flujo usa bootloader y tabla
   de particiones existentes; no copiar firmware.bin a una direccion arbitraria.
4. Desconectar USB, reinstalar ambas placas con bateria desconectada y luego
   alimentar por bateria. Comprobar Wi-Fi, STATUS y sensores desde la web.
5. Preparar suelo continuo para IR, frontal/trasera con ecos validos claramente
   por encima de los umbrales (por ejemplo, reflectores aproximadamente a
   0,5-1 m), y espacio amplio lateral verificado visualmente. Abrir un area
   alrededor del chasis y sus salientes; no confiar solo en distancias axiales.
6. Primero giro aislado: RESET si corresponde, confirmar IDLE/faults=0/STOP=0,
   marcar supervision y solicitar una sola vez AUTO-3 giro aislado. Comprobar
   giro pequeno izquierdo y detencion, motivo TARGET, progreso cercano a 10
   grados, aplicado/completado y estabilizacion confirmada. No exigir exactitud
   angular absoluta. STOP -> IDLE; comprobar retencion de /api/navigation.
7. Solo despues, secuencia integrada: obstaculo inicialmente fuera de 275 mm,
   trasera despejada y laterales comprobados. Observar avance, primera parada,
   retroceso y segunda parada antes del giro. Si el frontal no queda >275 mm,
   esperar rechazo seguro del giro; no forzarlo ni rearmar automaticamente.
8. En ensayos separados, verificar STOP durante espera y durante giro, siempre
   con zona despejada. Confirmar ausencia de reanudacion y regreso a IDLE cuando
   no haya una falla Safety enclavada. Una nueva ejecucion exige nueva solicitud.

Limitaciones: no hay sensores laterales, deteccion de deslizamiento ni garantia
de precision angular/frenado. La envolvente nominal del chasis cuadrado de
170 mm tiene radio de unos 120 mm desde su centro, sin contar salientes ni
error de montaje. La ausencia de eco puede enclavar Safety incluso en un area
abierta. Los tiempos no garantizan parada con CPU/scheduler detenidos.
La perdida de Wi-Fi no envia automaticamente STOP a STM32: una sesion AUTO
en curso puede continuar hasta su objetivo, limite temporal o parada Safety.
STOP web depende del enlace y no es una parada de emergencia fisica certificada.
No usar IMU fusionada ni mapa para autorizar estas maniobras. No hay avance
posterior al giro, alternancia de sentido, reintentos ni navegacion continua.
