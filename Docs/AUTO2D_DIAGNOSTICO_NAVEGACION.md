# AUTO-2D: diagnostico retenido de navegacion

AUTO-2F corrige el borrado transitorio de NAVW al recibir otra NAV de la misma
sesion inmutable, conservando su edad. Contrato vigente y parametros:
[AUTO2F_REFINAMIENTO.md](AUTO2F_REFINAMIENTO.md).

Actualizacion AUTO-2E: se implemento la espera supervisada que este documento
proponia; los apartados anteriores se conservan como historial. No se aplico
la reduccion de umbrales. Contrato adicional y comportamiento vigente:
[AUTO2E_TRANSICION_RETROCESO.md](AUTO2E_TRANSICION_RETROCESO.md).

Sobre AUTO-2 pendiente de commit, HEAD `b0967cd`. Solo incorpora observacion,
consulta UART y API. No implementa pausa de estabilizacion ni modifica umbrales,
tiempos, PI/PWM, odometria, sensores, Safety, rearme, IMU o mapa. El giro sigue
deshabilitado. Las propuestas AUTO-2E del final NO estan implementadas.

AUTO-2D.1 corrige exclusivamente el almacenamiento del buffer HTTP (estatico)
y la respuesta ante error de serializacion; ver la seccion final. No cambia
el esquema NAV/JSON ni las decisiones de movimiento.

## Diagnostico del retroceso

La prioridad actual en `Navigation_Auto2StepLocked()` es Safety/STOP/plazos,
validez sensorial, odometria/progreso/objetivo y finalmente obstaculo/evasion.
Desde 400 mm iniciales, el avance de 100 mm puede finalizar por TARGET antes
de iniciar evasion a 300 mm. Desde 350 mm, una lectura de 298 mm es compatible
con prevencion, pero la lectura web no demuestra la causa interna.

Ante obstaculo durante AVANZAR, solo autoriza retroceso con ambos ultrasonicos
validos/recientes, sin caida, frontal >200 mm, trasera >300 mm, odometria valida
y ausencia de STOP/Safety. Revoca objetivos, limpia PI, escribe PWM cero e
invalida generacion/sesion de fase. En otra iteracion toma una referencia
odometrica posterior y emite `Motion_NavigationReverse()` (-1600/-1600 cps).
MotorControl aun debe admitir el mensaje y revalidarlo antes de PI/driver.

**No existe confirmacion de ruedas detenidas.** La separacion entre ciclos y
la referencia nueva no equivalen a estabilizacion mecanica. Los plazos actuales
son 300 ms desde entrar en retroceso y 1 mm nuevo en 200 ms desde la referencia.
A la velocidad nominal (~89,4 mm/s), recorrer 20 mm requiere unos 224 ms ideales,
sin incluir la transicion ni aceleracion. Hay poco margen dentro de los 300 ms.

`Navigation_OdometryOutcome()` rechaza inmediatamente conteos que aumenten en
retroceso. Una sola cuenta positiva residual (~0,056 mm) tras su referencia
produce ODOMETRY, aunque todavia no se aplico la primera consigna negativa o
solo se proceso durante un ciclo. Las pruebas nuevas reproducen ambos casos
con el bucle real de MotorControl. No demuestran que esta sea la causa fisica
de los ensayos: esa conclusion requiere leer el nuevo diagnostico en la PCB.

## Instantanea implementada

`NavigationDiagnostics_t` reside en `mid_navigation.c`. Se actualiza bajo las
secciones criticas existentes usando datos ya adquiridos. No hay nueva lectura
de sensores/encoders, E/S, cola, tarea ni historial. Ningun campo diagnostico
se consulta para autorizar o cancelar movimiento.

`Navigation_GetDiagnostics()` copia coherentemente el registro. El consumidor
UART formatea fuera del lock. La sesion diagnostica tiene ID RAM propio que
no cambia con cada fase: se incrementa en `Navigation_Begin()`. Puede repetirse
tras reinicio; no es un identificador persistente global. Tick cero y wrap son
validos. `started_tick` proviene de FreeRTOS, actualmente 1000 Hz.

Se conserva la primera causa, ultima fase, progreso aceptado y tiempos terminales
despues de STOP/IDLE/RESET. Una nueva entrada efectiva a AUTO limpia los datos
y cambia el ID. Una publicacion ya en curso puede confirmar tarde su entrega
a la cola de la misma sesion; no cambia la causa, no prueba aplicacion y no
puede contaminar una nueva sesion. Las barreras originales siguen rechazando
esa consigna si fue revocada.

### Estados y causas

Estados de navegacion (distintos del STATE de la FSM):

| Codigo | Nombre |
|---|---|
| 0 | ESPERAR_DATOS |
| 1 | AVANZAR |
| 2 | DETENIDO |
| 3 | RETROCEDER_ACOTADO |
| 4 | GIRAR_ACOTADO, solo compilacion experimental |

| Causa | Nombre API | Significado |
|---|---|---|
| 0 | NONE | Aun sin causa terminal. |
| 1 | TARGET | Objetivo de avance alcanzado. |
| 2 | CANCELLED | RESET/salida de modo/revocacion, incluido timeout individual de consigna. |
| 3 | SAFETY | Safety cancelo la fase activa. |
| 4 | TIMEOUT | Vencio plazo global, avance o fase; consultar last_phase y tiempos. |
| 5 | OBSTACLE | Obstaculo presente en espera inicial; no retrocede. |
| 6 | SENSORS | Datos sensoriales inseguros o ausentes. |
| 7 | ODOMETRY | Odometria rechazada; consultar detalle y deltas. |
| 8 | EMISSION | Publicacion rechazada. |
| 9 | EVASION_COMPLETED | Retroceso y giro experimental completados. |
| 10 | EVASION_NOT_AUTHORIZED | Evasion rechazada; tambien giro bloqueado tras completar la separacion. |
| 11 | OPERATOR_STOP | Solicitud prioritaria de STOP. |
| 12 | NO_PROGRESS | Progreso insuficiente durante la ventana vigente. |

Detalle odometrico: 0 `none`; 1 `unavailable` (referencia/estado/geometria no
utilizables); 2 `stale_or_future`; 3 `timestamp` incoherente o conteos distintos
en el mismo tick; 4 `wrong_direction`; 5 `overflow`; 6 `nonfinite` o conversion
invalida; 7 `wheel_disagreement`; 8 `numeric_range`. Es observacion del rechazo
original, no un codigo nuevo de Safety o del datalogger.

## Protocolo UART

Consulta ASCII de solo lectura: `NAV Q=N\n`, con N decimal, 1..4294967295.
Se utiliza la misma recepcion UART y TaskComms; no modifica STATUS/SENSORS ni
las respuestas anteriores. En el buzon STM32, NAV es telemetria subordinada a
STATUS, SENSORS, IMU y MAPPING, y siempre a movimiento, DETENER y STOP.
Solo se amplia la clasificacion del nuevo comando; puerto, pines y velocidad
permanecen intactos. Consultas invalidas no actuan motores.

Respuesta unica `NAV,` seguida de 24 campos delimitados por comas y LF:

| Indice tras NAV | Campo / unidad |
|---|---|
| 0 | Version =1. |
| 1 | Q original. |
| 2 | Perfil: 0 B7_LEGACY, 1 AUTO_1, 2 AUTO_2; proviene del STM32 compilado. |
| 3,4 | ID de sesion diagnostica, tick de entrada en AUTO. |
| 5,6,7 | Estado actual, ultima fase autorizada/evaluada, causa. |
| 8 | Flags de observacion, mascara descrita abajo. |
| 9,10 | Tiempo de sesion y de ultima fase [ms]; congelados al cancelar. |
| 11,12 | Progreso de avance y retroceso [micrometros derivados de conteos]; consultar flags de disponibilidad. |
| 13,14 | Frontal/trasero [mm redondeados]; cero de transporte solo si no disponibles. La API los convierte a null. |
| 15 | Flags sensoriales: snapshot disponible=1, frontal valido=2, trasero valido=4, caida=8. |
| 16,17,18 | Edades del snapshot y capturas validas en la decision [ms]. No son edades actuales. |
| 19 | Detalle odometrico 0..8. |
| 20,21 | Deltas int64 firmados respecto de la referencia de fase, o ambos `-` si no calculables sin overflow. |
| 22,23 | Edad de la decision sensorial y del ultimo evento observado al responder [ms]. |

Mascara del campo 8: sesion disponible=1, obstaculo detectado=2, retroceso
considerado=4, autorizado=8, consigna solicitada=16, publicacion exitosa=32,
acuse de MotorControl=64, retroceso completado=128, progreso avance disponible=256,
progreso retroceso disponible=512. Una sesion con ID cero despues de wrap sigue
siendo valida si tiene el bit 1. Ausencia de sesion no se representa con una
distancia inventada.

La linea esta acotada a 255 bytes incluyendo LF; se comprueba truncamiento.
Los deltas int64 se convierten sin depender de `%lld` de newlib-nano. TaskComms
reutiliza la transmision existente con timeout de 50 ms, fuera de locks; no se
agregan esperas a tareas criticas. No es una garantia de latencia UART cero.

ESP32 consulta una vez por segundo, despues de STATUS, sensores e IMU, sin
acumular solicitudes ni reintentar fallos. Pausa la consulta durante la sesion
manual o controles pendientes y revalida despues de adquirir el turno TX.
Solo app_main alimenta el parser desde el receptor UART existente. Se exige Q
pendiente coincidente y respuesta antes de 200 ms; duplicados o respuestas
viejas no rejuvenecen el cache. El parser valida version, campos/rangos,
fragmentacion, LF/CRLF, NUL, longitud y limites enteros.

## GET /api/navigation

No envia UART ni inicia movimiento. Devuelve una copia JSON, `no-store`.
Se conserva LRU; solo se agrega una ruta GET. No hay pantalla ni controles
nuevos. Abrir `http://192.168.4.1/api/navigation` desde el dispositivo conectado
al AP, sin monitor UART externo ni USB.

Ejemplo **sintetico del test nativo**, no una medicion del robot:

```json
{
  "received": true,
  "connected": true,
  "stm32_connected": true,
  "paused": false,
  "stale": false,
  "age_ms": 0,
  "available": true,
  "historical": true,
  "profile": "AUTO_2",
  "session_id": 7,
  "started_tick": 1000,
  "state": "DETENIDO",
  "state_code": 2,
  "last_phase": "RETROCEDER_ACOTADO",
  "finish_reason": "ODOMETRY",
  "finish_code": 7,
  "terminal": true,
  "active_reported": false,
  "obstacle_detected": true,
  "reverse": {
    "considered": true,
    "authorized": true,
    "requested": true,
    "published": true,
    "applied": false,
    "completed": false,
    "progress_mm": null
  },
  "advance_progress_mm": 50.0,
  "elapsed_ms": 150,
  "phase_elapsed_ms": 30,
  "event_age_ms": 0,
  "decision": {
    "age_ms": 10,
    "snapshot_available": true,
    "snapshot_age_at_decision_ms": 0,
    "front_mm": 298,
    "front_valid": true,
    "front_age_at_decision_ms": 20,
    "rear_mm": 450,
    "rear_valid": true,
    "rear_age_at_decision_ms": 90,
    "cliff": false
  },
  "odometry_error": "wrong_direction",
  "left_delta_counts": "1",
  "right_delta_counts": "0"
}
```

Interpretacion y limites:

- `received`: existe una trama NAV completa aceptada. `connected`: NAV reciente
  (<3000 ms) y STATUS reciente. La perdida detectada por STATUS marca stale
  inmediatamente; `paused` distingue consultas omitidas por control manual.
  `age_ms` usa solo reloj monotono ESP32 de 64 bits. No se restan relojes MCU.
- `available`: STM32 tiene una sesion actual o ultima retenida. Sin respuesta,
  perfil/estados/lecturas/flags de sesion son null; antes del primer AUTO el
  perfil puede conocerse, pero la sesion sigue ausente.
- `historical` es true si la sesion termino o si el enlace esta desactualizado.
  Un registro terminal puede estar actualizado como telemetria e historico
  como maniobra simultaneamente. No se borran evidencias al perder enlace.
- `active_reported` y `state` describen la ultima instantanea recibida, no una
  confirmacion actual de motores en marcha. El STATE principal sigue en STATUS.
- `last_phase` significa fase autorizada/evaluada, no movimiento demostrado.
  `requested` es llamada a la API; `published` es entrega exitosa a la cola,
  incluso si ya habia sido revocada. `applied` es el acuse existente despues
  de procesar ambos objetivos por MotorControl/PI/driver. **No garantiza PWM
  no nulo ni desplazamiento fisico.** `completed` acredita el criterio de
  distancia de retroceso observado, no una evasion lateral segura.
- Progresos son valores calculados aceptados. Pueden quedar en cero/null si
  falla la primera muestra. Deltas se conservan al rechazar: por ejemplo +1/0
  con `wrong_direction` demuestra el rechazo por signo en esa referencia.
  Los int64 viajan como strings JSON para evitar perdida de precision en JS.
- `decision` es la ultima copia usada por navegacion. Sus edades *_at_decision
  quedan congeladas; `decision.age_ms` crece desde esa copia. No se presentan
  como lecturas actuales. Si no hubo snapshot, lecturas/edades/validez son null.
  Safety puede intervenir con una copia posterior: usar tambien sus fallas
  actuales de STATUS. Un `cliff=false` historico no contradice un Safety nuevo.
- Milimetros/decimales son conversion odometrica, no precision fisica calibrada.
  Tampoco los mm ultrasonicos redondeados reproducen todas las fracciones usadas
  en el comparador original. Los ticks STM32 pueden desbordarse; las edades
  locales se calculan con resta unsigned, dentro de su rango de representacion.

## Procedimiento fisico posterior

Despues de revisar/autorizar y programar con las placas retiradas, reinstalarlas
en la PCB, conectar bateria y utilizar Wi-Fi. No se requiere USB/ST-LINK ni
monitor externo con los motores alimentados.

1. Leer STATUS, SENSORS y /api/navigation. Confirmar perfil AUTO_2. Anotar ID;
   antes de la primera sesion debe decir available=false.
2. Rearmar con sensores seguros y entrar en IDLE. Disponer un obstaculo fijo
   frontal alrededor de 350 mm y trasera holgada (>300 mm durante toda la
   maniobra); superficie plana sin caidas. Mantener STOP accesible. Calibrar IMU
   no es requisito del retroceso ni debe repetirse para este diagnostico.
3. Solicitar AUTO una vez. Despues de detenerse, consultar /api/navigation y
   guardar el JSON. STOP -> confirmar IDLE -> volver a consultar: la causa,
   sesion y deltas anteriores deben conservarse.
4. TARGET indica avance terminado sin evasion. EVASION_NOT_AUTHORIZED sin
   authorized indica rechazo de condiciones; completed=true distingue un
   retroceso terminado seguido de giro bloqueado. ODOMETRY/wrong_direction
   con deltas positivos muestra el problema de signo residual; applied=false
   o true distingue si el controlador llego a procesar la consigna. TIMEOUT
   y NO_PROGRESS orientan a tiempos/progreso; SENSORS/SAFETY exigen revisar
   mediciones/rearme sin ignorar ausencias de eco.
5. No encadenar intentos automaticamente. Una nueva AUTO tras STOP/IDLE debe
   cambiar session_id y limpiar los flags. No habilitar giro ni cambiar los
   umbrales como parte de esta prueba.

## Propuesta AUTO-2E: detencion y estabilizacion (NO implementada)

Hoy solo se ordena cero, se revocan objetivos y se espera otra iteracion. No
existe `DETENER_Y_ESPERAR` ni confirmacion mecanica. Se propone incorporarla
en la misma Navigation_Step, sin nuevas tareas ni delay bloqueante:

- Mantener cero y permisos de avance revocados. Supervisar Safety, STOP,
  sensores y odometria en cada ciclo. No iniciar la espera con falla activa.
- Candidato inicial: ventana de 60 ms, al menos cuatro observaciones con
  timestamps nuevos y odometria fresca. En CADA rueda, maximo-menos-minimo de
  conteos <=1 cuenta (~0,056 mm), no solo diferencia neta o una velocidad cero.
  Equivale aproximadamente a <1 mm/s observable durante la ventana. Son
  parametros experimentales, no una demostracion absoluta de reposo: existen
  cuantizacion, ruido, alias y posibles fallas del encoder.
- Timeout de espera propuesto: 200 ms. Si no confirma estabilidad, terminar
  detenido, con causa/fase diferenciables. No continuar invirtiendo a ciegas.
- Al estabilizar, revalidar sensores y tomar referencia nueva antes de publicar
  negativo. Conservar comprobaciones de signo despues de la autorizacion;
  cualquier tolerancia posterior requiere evidencia y revision separadas.
- La espera debe tener reloj propio y consumir el plazo GLOBAL original, no
  el tiempo reservado a marcha de retroceso. Iniciar vigilancia de progreso
  de marcha desde la aplicacion verificable, sin dejar emisiones pendientes
  fuera de los timeouts existentes. Nunca reiniciar started_tick global.

Alternativa de retardo fijo acotado (p.ej. 60 ms): mas simple, pero no comprueba
si una rueda sigue girando ni detecta cambios de carga. Se recomienda la
confirmacion por encoders con timeout; no una unica muestra cero.

Mantener inicialmente 20 mm y analizar el resultado. Si el motivo observado
es TIMEOUT con progreso correcto, evaluar 500 ms en otra revision. 2000 ms de
avance +200 de espera +500 de retroceso caben en 3000 ms con giro deshabilitado;
si se considerara el giro futuro, debe reservarse su tiempo o rechazarlo, no
ampliar indefinidamente la sesion. Subir tiempos no resuelve el rechazo +1.

Tests futuros: ambas ruedas aun girando, solo una con residual, estabilidad
en cuatro timestamps, movimiento oscilante de neto cero, ruido de una cuenta,
odometria vieja/duplicada, espera vencida, STOP/Safety durante espera, sensor
trasero invalido, nuevo origen al estabilizar y ausencia de consignas residuales.
Un estado nuevo requerira actualizar conjuntamente protocolo/parser y tests;
no confundirlo con los cinco estados actuales.

## Propuesta de umbrales (NO implementada)

Permanecen 300 mm frontal, 200 mm Safety y >300 mm trasera. No se cambia el
umbral global de Safety ni los requisitos de rearme (capturas seguras nuevas).

Ubicacion de las reglas vigentes, sin modificaciones:

- `mid_navigation.h`: `NAV_FRONT_STOP_MM=300U`,
  `NAV_REAR_REVERSE_MIN_MM=300U`. `Navigation_Auto2StepLocked()` los aplica a
  deteccion y mantenimiento del retroceso; AUTO-1/B7/giro tambien los usan.
- `task_safety.c`: `TASK_SAFETY_OBSTACLE_DISTANCE_M=0.20f`.
  `TaskSafety_SnapshotFaults()` exige ambas distancias estrictamente mayores;
  esta misma evaluacion se reutiliza en `TaskSafety_RequestReset()`. No existe
  una distancia de rearme independiente ni histeresis adicional.
- `task_safety.h`: `TASK_SAFETY_SENSOR_TIMEOUT_MS=100U` para el snapshot y
  `TASK_SAFETY_ULTRASOUND_MAX_AGE_MS=200U` por captura; validez, caida y ausencia
  de eco conservan sus reglas. El rearme exige capturas posteriores a la falla.
- `test_navigation_auto1.c`, `test_navigation_auto2.c`, los casos B7 y los
  casos de Safety en `test_safety.c` cubren limites, rearme y prioridad. No se
  sustituyen pruebas de Safety por umbrales preventivos de navegacion.

| Parametro | Candidato estudiado | Recomendacion para primer ensayo posterior |
|---|---|---|
| Preventivo frontal | 250 mm | Preferir 275 mm inicialmente por falta de medida de frenado/latencia; evaluar 250 despues. |
| Safety frontal/trasero | 200 mm | Conservar; afecta MANUAL/AUTO/IDLE y rearme. Cualquier reduccion es otra modificacion independiente. |
| Guardia trasera de retroceso | >250 mm | Razonable como candidato con retroceso <=20 mm, pero mantener >300 mientras se identifica la causa. Para completar 20 mm se necesita espacio adicional. |

A 1600 cps, ruedas de 25,4 mm y CPR vigentes, la velocidad nominal es unos
89,4 mm/s: ~12,5 mm por ciclo ultrasonico de 140 ms. Un presupuesto orientativo
de edad valida 200 ms +navegacion 20 ms +motor 10 ms supone ~20,6 mm antes de
sumar frenado, jitter, sobrevelocidad e incertidumbre de ecos. No constituye
una cota medida. El margen 250-200=50 mm podria consumirse; no se garantiza
observar siempre una muestra entre ambos umbrales. 275-200=75 mm conserva mas
reserva para el primer ensayo. La detencion rapida observada no cuantifica
esa reserva ni justifica cambiar simultaneamente Safety.

El umbral trasero es de admision Y mantenimiento: desde 260 mm con guardia
250 mm puede detenerse despues de solo ~10 mm. Para los 20 mm completos,
necesita >270 mm iniciales mas incertidumbre/sobrepaso. Durante el retroceso
una muestra puede tener hasta 200 ms, y un obstaculo puede acercarse; no basta
restar unicamente los 20 mm del valor inicial. IR detecta falta de piso, no
reemplaza una distancia a obstaculos.

Los extrinsecos nominales +/-85 mm y chasis de 170 mm suponen sensor en el
borde; no demuestran el margen real sensor-extremo de la PCB/chasis. El alcance
util observado de ~2 m y los ecos ausentes/reflejados siguen siendo limitaciones.
Ausencia de eco no es espacio libre ni se corrige reduciendo el umbral.

Los umbrales actuales son compartidos por AUTO-1/B7 y validacion de giro.
Una reduccion futura de AUTO-2 debe separarse para no modificar silenciosamente
esos perfiles. Tests: 249/250/251 (o 274/275/276), Safety 199/200/201 sin cambios,
trasera en frontera durante marcha, sensores invalidos/antiguos, STOP, rearme,
timeout/distancia y AUTO-1. No realizar pruebas de contacto ni exigir mediciones
milimetricas. La parada depende de CPU/scheduler; no es una emergencia fisica.

## Verificacion de software y alcance final

Resultados locales de AUTO-2D, 2026-11-22. Son pruebas con mocks/datos sinteticos,
no resultados de desplazamiento ni de comunicacion sobre las placas:

| Conjunto | Resultado |
|---|---|
| Safety/control/navegacion AUTO-2 operativo | 393/393 |
| Perfil AUTO-1 conservado | 363/363 |
| AUTO-2 con giro habilitado solo en tests, izquierda/derecha | 419/419 cada variante |
| Perfil B7 anterior, giro deshabilitado | 436/436 |
| Giro B7 anterior, izquierda/derecha | 55/55 cada variante |
| Odometria/protocolo STM32 | 28/28 |
| Adquisicion sensorial STM32 | 8/8 |
| Driver/adquisicion/calibracion IMU STM32 | 48/48 |
| Driver ultrasonico | 19/19 |
| ESP32 nativas W1-W7/IMU/AUTO-2D | 158/158 grupos |
| JavaScript W2-W7/IMU y convivencia de paneles | 160/160 grupos |

Los siete conjuntos de navegacion comparten regresiones; no se suman como
casos independientes. Se compararon sus etiquetas PASS con la salida previa
a AUTO-2D: siguen presentes y aprobados todos los casos anteriores (363, 353,
388, 388, 428, 55, 55 respectivamente). Solo se agregan observaciones/tests;
no se relajan expectativas de movimiento/Safety para aceptar el residual.

Las pruebas nuevas incluyen objetivo, obstaculo, rechazo/autorizacion, fallo
y exito real de publicacion en cola, acuse del backend, rechazo residual antes
y despues del acuse, retroceso completado, timeout de cada fase, falta de
progreso, ocho clases odometricas, STOP/Safety y retencion despues de STOP y
RESET. Tambien formato/truncamiento/prioridades UART, fragmentacion, Q/tiempos,
datos desconocidos/antiguos, JSON real, errores HTTP, pausa manual y 10000
publicaciones simultaneas con copias HTTP. El planificador app_main se prueba
con esperas UART de 100 ms, consultas anteriores y sin una nueva tarea.

Se alimenta el parser/endpoint ESP32 con la trama producida por el caso residual
STM32 real del arnes. Su resultado ODOMETRY/wrong_direction, delta +1/0,
published=true/applied=false, tampoco es una observacion fisica. La API permite
distinguirla de un error de sensores o un retroceso simplemente no autorizado.

Compilaciones existentes, sin cambiar configuracion ni dependencias:

```text
cd Firmware/stm32_control
mingw32-make -B -o build -j4 "CC=arm-none-eabi-gcc -fstack-usage"

pio run -d Firmware/esp32_gateway -e esp32
```

Ambas compilan/enlazan sin errores ni advertencias del compilador. Artefactos:
`Firmware/stm32_control/build/stm32_control.elf` y
`Firmware/esp32_gateway/.pio/build/esp32/firmware.elf` (tambien firmware.bin).

- STM32: text=102224, data=940, bss=36424 bytes. RAM estatica data+bss=37364
  bytes; +424 bytes respecto de AUTO-2 previo (36940). El registro diagnostico
  mide 80 bytes; la respuesta y copia de TaskComms son estaticas. El enlace
  usa las regiones existentes, sin cambiar heap, prioridades ni stacks.
- ESP32: Flash 936193/1048576 bytes (89,3%; margen 112383), RAM estatica
  72736/327680 bytes (22,2%). Esto no mide heap/stack libres en ejecucion.
  Se agrega una ruta HTTP y se conserva LRU, sockets y timeouts.
- GCC -fstack-usage: Navigation_FormatDiagnostics=248 bytes de frame local,
  Navigation_Step=128; la funcion preexistente TaskComms_SendSensors=264.
  Estos frames no son una medicion del maximo de toda la cadena. En AUTO-2D
  inicial el handler HTTP tenia un buffer local de 1792 bytes; AUTO-2D.1 lo
  retira del stack, como se detalla abajo. Queda pendiente verificar margenes
  reales de stack/heap en la futura prueba, sin afirmar que el enlace los mida.

Comandos de pruebas desde STM32: `Tests/safety_b11/run.bat`,
`Tests/odometry_w7/run.ps1`, `Tests/sensors_w5/run.ps1`, `Tests/imu_1/run.ps1`
y `Tests/ultrasound/run.bat`; en ESP32, `Tests/w2/run.ps1`. JavaScript usa
el runner V8 existente `Tests/w2/run_js.js`, sin instalar Node ni dependencias.
Los logs se conservan en build/ y .pio/, ignorados por Git.

Cambios de AUTO-2D:

- STM32: mid_navigation.c/.h (instantanea/observacion/formato), task_comms.c
  (consulta), drv_uart.c (clasificar exclusivamente NAV como subordinado),
  test_navigation_diagnostics.c, integracion minima en test_safety.c y README.
- ESP32: stm32_navigation.c/.h nuevos; integracion en uart_comm.c/.h,
  main.c, CMakeLists.txt y web_server.c; test_navigation.c e integracion en
  test_status.c. No cambia la web ni el control manual para este diagnostico.
- Documentacion: este contrato y enlace desde AUTO2_REACCION_OBSTACULOS.md.

Los cambios pendientes previos de AUTO-2 se conservan, incluidos el perfil,
su suite, run.bat, task_motor_control.c, index.html y documentacion AUTO-1/2.
Se verificaron hashes contra el estado de entrada de AUTO-2D: Safety, FSM,
MotorControl, drivers de motor/encoder/ultrasonido, adquisicion, geometria,
odometria, IMU y mapa permanecen iguales. La navegacion solo recibe observacion;
las constantes/guardias/acciones anteriores se conservan.

No hay grabacion, ensayo fisico ni commit. El siguiente paso es revisar esta
telemetria y obtener la causa real; AUTO-2E, la espera y los umbrales menores
requieren aprobacion posterior y sus pruebas especificas.

## AUTO-2D.1: correccion del stack del endpoint HTTP

### Evidencia y alcance del diagnostico

El ensayo fisico termino con Safari indicando conexion interrumpida al abrir
`/api/navigation`; el dashboard seguia accesible. No se recupero la sesion
antes de apagar el robot. No se dispone de traza de panic ni identificacion
del binario fisico que permita demostrar el mecanismo exacto de ese cierre.

En el ELF ESP32 local anterior a la correccion se demuestra un defecto de
memoria independiente: los frames activos de una cadena real exceden los
4096 bytes de la tarea HTTP. Se inspeccionan instrucciones Xtensa `entry a1,N`
del ejecutable enlazado, incluidas las funciones de newlib, no frames de GCC
nativo Windows. La ruta del SDK invoca el handler de manera sincrona:

```text
httpd_thread -> httpd_server -> httpd_process_session -> httpd_sess_process
 -> httpd_req_new -> httpd_parse_req -> httpd_uri -> WebServer_Navigation
 -> Stm32Navigation_Json -> snprintf -> _svfprintf_r
```

| Frames de la cadena | Antes | Despues |
|---|---:|---:|
| SDK hasta el handler (32+80+32+32+32+144+64) | 416 | 416 |
| WebServer_Navigation | 1856 | 64 |
| Stm32Navigation_Json | 976 | 976 |
| snprintf | 192 | 192 |
| _svfprintf_r | 800 | 800 |
| Suma de esos frames | **4240** | **2448** |
| Stack HTTP configurado | 4096 | 4096 |

4240 ya supera el stack, aun sin llamadas auxiliares, wrapper de tarea o
contexto adicional. El defecto esta demostrado en el ELF local; su relacion
con el error exacto del ensayo es probable, no una traza fisica confirmada.
Que los 158 tests Windows anteriores pasaran no cubria el stack Xtensa.

No se encontro desbordamiento del JSON: los casos numericos/cadenas limites
del arnes generan hasta 1052 bytes, dentro de 1792 (int64 extremos, nombres
largos y edad monotona de 64 bits). Los campos sin datos tambien
generan JSON completo. La ruta esta registrada y su buffer anterior tenia
vida util suficiente durante el envio sincrono; el problema era alojarlo en
stack junto con la cadena de formateo.

### Correccion minima

Solo cambia el firmware `src/web_server.c`:

- `response[1792]` pasa a almacenamiento estatico privado, siguiendo el patron
  de `/api/imu` y `/api/map`. No se reduce capacidad ni se aumenta stack.
- Un unico servidor/tarea HTTP ejecuta los handlers en serie. El envio
  `httpd_resp_send()` es sincrono y termina antes de reutilizar este buffer.
  El lector UART no lo utiliza; copia la instantanea mediante su lock anterior.
- Si el generador falla o informa longitud fuera de rango, se responde
  `500 Internal Server Error`, MIME JSON y `Cache-Control: no-store`, con
  `{"error":"navigation_unavailable"}`. No se envia JSON parcial ni se cierra
  deliberadamente sin respuesta por un error de formato.
- Los errores de red/SDK siguen propagandose: no se garantiza entregar una
  respuesta a un cliente desconectado o cuando falla el propio envio.

La implementacion real de ESP-IDF `httpd_resp_send()` escribe Content-Length
y envia exactamente la longitud recibida, sin requerir chunk final ni LF
despues del JSON. No se requiere cambiar su API ni forzar Connection: close.

### UART y compatibilidad

NAV conserva la linea de hasta 255 bytes con LF, parser de 256 bytes, un solo
lector, Q correlacionada, limite de respuesta 200 ms y consulta de 1 Hz. Los
mensajes incompletos/malformados se descartan; ninguna respuesta NAV es
necesaria para generar el JSON de campos null. Un error de datos no justifica
interrumpir TCP. No hay asignacion dinamica propia ni espera de UART desde
este handler. No se cambiaron prioridades, LRU, sockets, timeouts ni tareas.

### Pruebas y compilacion

Se agrega `Tests/w2/check_navigation_stack.py`, que inspecciona el ELF Xtensa
y lee el stack predeterminado del header SDK. Sobre el ELF anterior FALLA
como regresion esperada (4240>4096); sobre el nuevo PASA (2448<4096 y frame
del handler <=128). No certifica el high-water mark de toda la cadena SDK;
el margen real aun debe comprobarse en ejecucion.

Invocacion, desde la raiz del repositorio (rutas toolchain/SDK instaladas):

```powershell
python Firmware/esp32_gateway/Tests/w2/check_navigation_stack.py `
  Firmware/esp32_gateway/.pio/build/esp32/firmware.elf `
  C:/Users/leogo/.platformio/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp32-elf-objdump.exe `
  C:/Users/leogo/.platformio/packages/framework-espidf/components/esp_http_server/include/esp_http_server.h
```

Pruebas nativas: **163/163** grupos, conservando los 158 anteriores. Se cubren
los escenarios AUTO-2D y se agregan sin sesion previa, sesion activa/terminada,
retencion ante STATUS posterior a STOP/RESET y nueva sesion, MIME/longitud y
cuerpo completo, error de formato HTTP 500 y 600 respuestas intercaladas de
health/status/sensors/map/imu/navigation. Se simula un fallo del generador
solo desde el arnes, sin interfaces de prueba en produccion. Se mantienen los
tests de fragmentacion, maximos, datos obsoletos y concurrencia UART/HTTP.

Regresiones STM32 sin modificar archivos: **393/363/419/419/436/55/55** en los
siete perfiles de navegacion/Safety/control. JavaScript: **160/160** grupos.
Los resultados con mocks no reproducen una conexion TCP Safari fisica.

`pio run -d Firmware/esp32_gateway -e esp32`: compilacion/enlace satisfactorios,
sin errores ni advertencias del compilador. Flash **936277/1048576** bytes
(+84), RAM estatica **74528/327680** (+1792). Es el traslado previsto del buffer
de stack a BSS, sin nuevas asignaciones de heap ni cambios de configuracion.
ELF: `Firmware/esp32_gateway/.pio/build/esp32/firmware.elf`; BIN en el mismo
directorio. STM32 permanece byte a byte igual en fuentes a la entrada de
AUTO-2D.1: no requiere nueva compilacion ni grabacion para esta correccion.

### Validacion fisica posterior, inicialmente inmovil

1. Tras revision y autorizacion, actualizar solo ESP32 con el BIN nuevo mediante
   el procedimiento habitual con la placa retirada. No se graba durante este
   encargo. Reinstalar y usar bateria/Wi-Fi; no hace falta USB ni UART externa.
2. Sin solicitar MANUAL/AUTO ni movimientos, abrir `/health`, `/api/status`,
   `/api/imu` y `/api/navigation` desde Safari. Antes de AUTO: received=true
   cuando llegue NAV y available=false; si no hay UART, received=false con
   campos null. Ambas situaciones deben devolver HTTP/JSON completos.
3. Repetir /api/navigation y alternar con dashboard/endpoints anteriores;
   comprobar que siguen accesibles y que las edades/comunicacion son coherentes.
   No interpretar available=false como fallo HTTP.
4. Solo en una validacion de movimiento posterior autorizada, consultar la
   causa retenida despues de AUTO y de STOP. Si reaparece conexion interrumpida,
   anotar si /health y STATUS siguen respondiendo y si se reinicio ESP32, antes
   de apagar. No atribuir ese posible fallo automaticamente al JSON o a UART.

La falta de retroceso y su posible conteo residual siguen pendientes de
confirmacion fisica. AUTO-2E, DETENER_Y_ESPERAR y umbrales menores no se implementan.
