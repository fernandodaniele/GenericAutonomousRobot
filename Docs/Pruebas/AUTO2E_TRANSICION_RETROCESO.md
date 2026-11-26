# AUTO-2E: detencion y estabilizacion antes del retroceso

AUTO-2F refina parametros y retencion de telemetria sin cambiar esta espera.
Valores vigentes y resultados: [AUTO2F_REFINAMIENTO.md](AUTO2F_REFINAMIENTO.md).
Este documento conserva el punto de control AUTO-2E de 300 ms y 300/>300 mm.

Implementado sobre AUTO-2/AUTO-2D/AUTO-2D.1 pendientes, rama
`G1-add/InterfazWeb`, HEAD `b0967cd`. No se graban placas ni se realizan
movimientos fisicos durante esta etapa.

## Causa y alcance

El usuario confirmo fisicamente ODOMETRY/wrong_direction con retroceso
autorizado/publicado/procesado, deltas +3/+2, progreso cero y fase de 38 ms;
frontal 295 mm y trasera 487 mm validos. La referencia anterior se tomaba
antes de estabilizar las ruedas. La correccion agrega una espera supervisada,
sin tolerar conteos contrarios una vez iniciado el retroceso.

Se conserva la misma maquina en Navigation_Step (20 ms), MotorControl (10 ms),
Motion_NavigationReverse y todas las barreras de tarea, modo, generacion,
sesion, timestamp y permiso de emision de un solo uso. No hay nueva tarea,
cola, driver, filtro, rearme ni ruta de control.

## Secuencia y parametros

```text
ESPERAR_DATOS -> AVANZAR -> DETENER_Y_ESPERAR -> RETROCEDER_ACOTADO -> DETENIDO
```

Se conserva la prioridad de finalizacion por objetivo de avance cuando alcanza
100 mm, y la prohibicion de retroceder por un obstaculo presente al iniciar AUTO.

| Parametro | Valor |
|---|---:|
| Objetivo de avance / consigna | 100 mm / +1600 cps por rueda |
| Plazo de espera inicial y avance | 2000 ms desde Begin |
| Preventivo frontal | 300 mm |
| Safety frontal/trasero | 200 mm, sin cambios |
| Guardia trasera, admision y mantenimiento | estrictamente >300 mm |
| Ventana minima de estabilizacion | 60 ms |
| Observaciones nuevas minimas | 4 |
| Rango maximo por rueda en la ventana | 1 cuenta |
| Plazo propio de DETENER_Y_ESPERAR | 200 ms |
| Objetivo / consigna de retroceso | 20 mm / -1600 cps por rueda |
| Plazo propio de retroceso | 300 ms |
| Vigilancia de progreso | 1 mm nuevo en 200 ms |
| Plazo global AUTO-2 | 3000 ms desde Begin, nunca renovado |
| Giro experimental | deshabilitado por defecto |

Los cuatro parametros de estabilizacion estan centralizados en mid_navigation.h:
NAV_SETTLE_WINDOW_MS, NAV_SETTLE_MIN_OBSERVATIONS, NAV_SETTLE_MAX_SPAN_COUNTS
y NAV_SETTLE_TIMEOUT_MS. No se cambian PI, PWM, geometria, CPR ni odometria.

## Criterio de estabilizacion

Navigation_StartPhase(NAV_DETENER_Y_ESPERAR) llama a TaskMotorControl_Stop:
revoca objetivos, sesion/generacion anterior y permisos, limpia integrales y
pone ambos PWM a cero. La espera no emite consignas ni habilita movimiento;
el backend sigue rechazando mensajes de avance o retroceso de otra fase.

Navigation_SettleStep usa solo snapshots ya adquiridos. Exige muestras frescas,
estrictamente posteriores a la orden de cero. Las observaciones posteriores
deben tener timestamps nuevos y ordenados. Un timestamp repetido no suma;
conteos diferentes en un mismo tick o timestamps atrasados cancelan por error
odometrico. Tick cero y wrap son validos, con resta unsigned de TickType_t.

Para CADA rueda conserva minimo y maximo de conteos acumulados. Si el rango
maximo-minimo excede una cuenta, comienza una ventana nueva en esa muestra.
No basta una diferencia neta cero, una sola muestra inmovil, cuatro lecturas
de la misma muestra ni cuatro muestras muy proximas. Se requieren a la vez
cuatro observaciones nuevas y al menos 60 ms entre primera y ultima muestra.

Con cadencia nominal de 20 ms, las observaciones tipicas son +20/+40/+60/+80 ms
tras ordenar cero: la confirmacion mas temprana habitual ocurre a +80 ms.
No se programa un retardo fijo de 80 ms. El criterio tambien admite intervalos
irregulares cuando ambas condiciones se cumplen antes del timeout de 200 ms.

Una cuenta equivale aproximadamente a 0,056 mm con los CPR actuales. El criterio
estima reposo; no garantiza ausencia absoluta de movimiento, ruido, alias o
fallas mecanicas. No constituye deteccion de motor bloqueado ni de deslizamiento.

## Referencia y tiempos de retroceso

Al confirmar estabilidad se revalidaron, en el mismo ciclo, AUTO, STOP, Safety,
validez/antiguedad frontal y trasera, ausencia de caida y distancia trasera.
Navigation_StartPhase revoca de nuevo la fase anterior y habilita exclusivamente
la fase de retroceso, sin emitir aun. Otra iteracion requiere una muestra
estrictamente posterior a esa confirmacion para Navigation_RecordReference.

Se registran los conteos absolutos nuevos de ambas ruedas; el progreso se mide
desde ellos, sin reiniciar la odometria global ni reutilizar el origen de avance.
Entonces se puede emitir -1600/-1600 cps. La regla original de signo se conserva:
un conteo positivo posterior a la nueva referencia sigue provocando cancelacion.

Los 300 ms comienzan al entrar en RETROCEDER_ACOTADO, despues de la espera.
Incluyen la siguiente iteracion de referencia/emision, como en AUTO-2 anterior.
La vigilancia de progreso de 200 ms parte de esa nueva muestra de referencia.
La espera no consume ninguno de esos relojes, pero SI consume el plazo global.
No se aumentan los limites: las pruebas de inversion y recorrido acotado pasan
con los valores actuales. Su suficiencia fisica se verifica posteriormente.

## Cancelacion y finalizacion

- STOP y Safety mantienen la transaccion existente de revocacion y cero.
- Una trasera <=300 mm termina por EVASION_NOT_AUTHORIZED; lecturas invalidas,
  antiguas, caida o datos sensoriales insuficientes terminan sin retroceder.
  Safety conserva su enclavamiento independiente y su autoridad final.
- Odometria invalida/antigua/incoherente u overflow del rango cancela la espera.
- A los 200 ms de espera, sin confirmacion, termina TIMEOUT con ultima fase
  DETENER_Y_ESPERAR. La comprobacion del plazo precede a una nueva autorizacion.
- El plazo global puede terminar la espera antes; no se renueva al cambiar de fase.
- Despues de finalizar permanece detenido en AUTO; requiere STOP -> IDLE y una
  nueva orden AUTO. RESET no restaura movimientos ni borra la primera causa.
- Completar 20 mm conserva completed=true; el giro operativo sigue prohibido,
  de modo que la causa EVASION_NOT_AUTHORIZED posterior puede indicar el giro
  rechazado, no un fallo del retroceso. Consultar tambien reverse.completed.

## Telemetria compatible

Se conservan todos los campos anteriores de GET /api/navigation y se agrega
el subestado interno numero **5 = DETENER_Y_ESPERAR**, sin renumerar 0..4.
No es un nuevo estado de la FSM principal. La correccion de stack AUTO-2D.1
sigue vigente: buffer HTTP estatico de 1792 bytes y error JSON HTTP 500.

Se agrega este objeto (ejemplo SINTETICO del contrato, no datos nuevos del robot):

```json
{
  "stabilization": {
    "available": true,
    "state": "confirmed",
    "elapsed_ms": 80,
    "observations": 4,
    "window_observations": 4,
    "window_ms": 60,
    "left_span_counts": 0,
    "right_span_counts": 0,
    "snapshot_age_ms": 100,
    "reverse_entry_tick": 1080,
    "reference_tick": 1100,
    "reference_left_counts": "3",
    "reference_right_counts": "2"
  }
}
```

- available indica que existe una sesion y llego la parte complementaria de esa misma copia NAV.
  Si falta, los campos son null, sin inferir estabilidad ni referencia cero.
- state: not_requested=0, waiting=1, confirmed=2, timeout=3 (plazo de espera),
  cancelled=4 (STOP/Safety/sensores/odometria/modo o plazo global anterior).
- elapsed_ms se congela al confirmar o cancelar; no suma el tiempo de retroceso.
  observations cuenta muestras nuevas aceptadas en toda la espera;
  window_observations cuenta solo la ultima ventana sin rango excesivo.
- window_ms y los rangos describen la ventana utilizada, no velocidades medidas.
- reference_* son la referencia posterior a la confirmacion. Cero es valido;
  los conteos int64 se publican como strings, como los deltas anteriores.
- reverse_entry_tick indica entrada/autorizacion de fase, no movimiento fisico.
  Los flags requested/published/applied siguen diferenciados. applied no acredita
  desplazamiento; progress_mm y completed siguen siendo estimaciones odometricas.
- Los resultados se retienen tras STOP/RESET y cambian al comenzar una nueva AUTO.
  Un registro terminal es historico aunque la comunicacion este actualizada.

### Parte complementaria UART, sin agrandar buffers de drivers

`NAV Q=N` conserva su respuesta NAV v1 de 24 campos, hasta 255 bytes con LF.
Al formatearla, TaskComms conserva una sola copia estatica de diagnostico y Q.
`NAV Q=N S=1` devuelve la parte complementaria de ESA copia, no otra lectura de
sensores/odometria. Se rechaza si Q no coincide o la copia tiene >=1000 ms.

```text
NAVW,1,Q,session_id,started_tick,state,elapsed_ms,observations,window_observations,window_ms,left_span,right_span,reference_tick,reference_left,reference_right,reverse_entry_tick,snapshot_age_ms
```

Tras NAVW son 16 campos (version/Q/ID/tick de sesion y 12 valores). Los cuatro
campos de referencia/entrada pueden ser '-' si aun no existen. Los tres de
referencia se presentan juntos; entrada puede existir antes de la referencia.
Los dos conteos son int64 firmados; el resto son decimales sin signo. Una linea
tambien cabe en 255 bytes. No hay emision espontanea ni dos respuestas juntas.

ESP32 conserva la consulta base cada 1000 ms. Tras aceptar NAV, intenta UNA
consulta S=1, como pronto 100 ms despues y antes del siguiente periodo base.
Cada respuesta tiene su plazo de 200 ms. Se reutilizan el gate TX, las prioridades
y la pausa por control manual; sin rafagas, reintentos continuos o nuevas tareas.
Si no hay turno antes de la consulta base siguiente, se omite la parte.

El parser valida ambas lineas, Q, ID y tick de sesion; combina la parte solo
con la instantanea base correspondiente bajo el mismo lock. Una base nueva
no hereda detalles de la anterior. snapshot_age_ms crece desde la captura
STM32, usando el reloj ESP32 y RTT conservador, sin restar relojes de dos placas.
Un cliente HTTP no transmite consultas ni comandos UART.

ESP32 acepta tambien NAV previo sin NAVW: conserva todos los campos anteriores
y muestra stabilization.available=false. Para observar el nuevo subestado y
la estabilizacion deben actualizarse ambos firmwares AUTO-2E.

## Verificacion de software

Pruebas sinteticas, sin movimientos fisicos:

- Caso +3/+2, una rueda detenida antes que otra, rango oscilante de neto cero,
  cuantizacion de una cuenta, cuatro muestras/60 ms, intervalos irregulares,
  muestras repetidas/desordenadas, overflow, tick y referencia cero/wrap.
- Ausencia de emision en espera; referencia posterior; progreso negativo;
  fin de 20 mm; presupuesto de retroceso separado aun esperando 180 ms.
- Timeout de espera/global, STOP, Safety, invalidacion trasera y obstaculo
  trasero, RESET/salida de modo, diagnostico retenido y sin reactivacion.
- El bucle real continuo de MotorControl recibe residuales positivos durante
  espera, mantiene PWM cero, y despues completa retroceso. Se conserva el test
  que rechaza +1 posterior a la referencia; no se relaja la proteccion de signo.
- Tramas emparejadas reales del arnes STM32 llegan al parser/HTTP ESP32;
  duplicados, partes atrasadas/malformadas, Q/ID incorrectos y limites numericos
  se rechazan; pausa manual/STOP tambien prevalecen sobre S=1.

Resultados locales finales (los perfiles comparten casos; no sumarlos como
pruebas independientes):

| Conjunto | Resultado |
|---|---:|
| AUTO-2 operativo, Safety/control/navegacion | 419/419 |
| AUTO-1 conservado | 363/363 |
| AUTO-2 giro experimental izquierdo/derecho, solo tests | 445/445 cada variante |
| B7 anterior sin giro | 436/436 |
| B7 giro izquierdo/derecho | 55/55 cada variante |
| Odometria STM32 | 28/28 |
| Adquisicion sensores / IMU / driver ultrasonico | 8/8, 48/48, 19/19 |
| Nativas ESP32 | 171/171 grupos |
| JavaScript dashboard/control/telemetria | 160/160 grupos |

Los helpers y expectativas de transicion inmediata AVANZAR->RETROCEDER se
adaptaron a atravesar la espera REAL. Ningun conjunto anterior se deshabilito;
se conservan guardias y limites. El caso anterior de tick cero se cubre ahora
tanto al entrar en espera como al entrar en retroceso y tomar su referencia.

Compilaciones satisfactorias, sin errores ni advertencias del compilador:

```text
cd Firmware/stm32_control
mingw32-make -B -o build -j4 "CC=arm-none-eabi-gcc -fstack-usage"

pio run -d Firmware/esp32_gateway -e esp32
```

- STM32: text=103640, data=940, bss=36616 bytes; RAM estatica 37556 bytes,
  +192 frente al estado inicial AUTO-2D.1. ELF en
  `Firmware/stm32_control/build/stm32_control.elf`.
- ESP32: Flash 938301/1048576 bytes (89,5%), RAM estatica 74624/327680 (22,8%).
  Incrementos de 2024 y 96 bytes respectivamente. ELF/BIN en
  `Firmware/esp32_gateway/.pio/build/esp32/firmware.elf` y `firmware.bin`.
- El caso limite con todas las referencias int64/edades grandes genera JSON
  valido de 1435 bytes, dentro de 1792. El test de frames Xtensa incluye ahora
  Navigation_WaitJson y pasa: 2752 bytes de cadena analizada frente a 4096 de
  stack HTTP. No mide el high-water mark real de SDK/interrupciones.
- GCC ARM -fstack-usage: Navigation_SettleStep=80, Navigation_Step=128,
  Navigation_FormatDiagnostics=248 y Navigation_FormatStabilization=200 bytes
  de frame local. No se cambian stacks, heap ni prioridades.

Comandos de pruebas: `Tests/safety_b11/run.bat`,
`Tests/odometry_w7/run.ps1`, `Tests/sensors_w5/run.ps1`, `Tests/imu_1/run.ps1`
y `Tests/ultrasound/run.bat` desde STM32; `Tests/w2/run.ps1` y
`Tests/w2/check_navigation_stack.py` desde ESP32/raiz segun su invocacion.
JavaScript usa el runner V8 existente Tests/w2/run_js.js sin instalar dependencias.
Los logs y el diff exclusivo de AUTO-2E quedan en build/ y .pio/, ignorados por Git.

Archivos funcionales: mid_navigation.c/.h (espera y metadatos), task_comms.c
(parte UART complementaria), stm32_navigation.c/.h y uart_comm.c ESP32
(parser/copia/JSON y planificador de consulta). Se amplian tests y documentacion.
Se verificaron hashes contra la entrada de AUTO-2E: MotorControl, FSM, Safety,
todos los drivers, sensores, geometria/CPR, odometria, IMU y mapeo no cambiaron.

## Validacion fisica posterior

1. Revisar la entrega antes de autorizar grabacion. Actualizar ambas placas con
   el procedimiento habitual fuera de la PCB; luego reinstalar y usar bateria
   y Wi-Fi. No se requiere USB/ST-LINK ni UART externa durante el ensayo.
2. Sin ordenar movimiento, comprobar STATUS/SENSORS y /api/navigation. Confirmar
   perfil AUTO_2 y disponibilidad de la parte stabilization cuando llegue NAVW.
3. En superficie plana y zona despejada lateralmente, con ambos ultrasonicos
   validos y espacio trasero holgado >300 mm durante todo el retroceso, disponer
   un obstaculo frontal aproximadamente como en el ensayo de 350 mm. STOP debe
   permanecer accesible. No se habilita el giro ni se cambian umbrales.
4. Solicitar AUTO UNA vez. Debe parar, esperar y solo despues retroceder hasta
   el criterio de distancia o un limite. La espera breve puede no distinguirse
   visualmente: guardar el JSON terminal con confirmed, ventana/observaciones,
   referencia y los flags/progreso del retroceso.
5. STOP -> confirmar IDLE -> volver a consultar: se conserva el mismo diagnostico.
   Si aparece timeout/cancelled o nuevo wrong_direction, conservar JSON antes
   de apagar. No aumentar tiempos, ignorar fallas ni repetir automaticamente.

No exigir precision milimetrica. Los HC-SR04 no verifican cobertura lateral;
la ausencia de eco conserva la parada enclavada. La CPU/scheduler detenidos
no garantizan parada por software. IMU-4 sigue siendo exclusivamente diagnostica.
