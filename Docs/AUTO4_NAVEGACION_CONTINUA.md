# AUTO-4: navegación continua acotada y supervisada

Implementación de software sobre `G1-add/InterfazWeb`, base `79ddac6`.
No se grabaron placas ni se hicieron pruebas físicas durante esta etapa.

## 1. Política adoptada y alcance

AUTO-4 es un perfil experimental explícito (`NAV_PROFILE_CONTINUOUS = 5`).
No sustituye AUTO-2F. Reutiliza TaskControlRobot, Navigation, Motion,
TaskMotorControl, Safety, odometría y adquisición de sensores existentes.
No añade tareas, colas, drivers ni otro controlador de motores.

| Solicitud | Comportamiento |
| --- | --- |
| `AUTO` | AUTO-2F: avance, retirada autorizada y detención; sin giro operativo |
| `AUTO_TURN` | AUTO-3: giro aislado supervisado |
| `AUTO_EVADE` | AUTO-3: una evasión integrada supervisada |
| `AUTO_CONTINUOUS` | AUTO-4: segmentos y evasiones dentro de límites finitos |

MANUAL conserva sus protecciones y temporización. AUTO-1 y los perfiles
históricos siguen como configuraciones explícitas de regresión.

**Safety no se modifica.** Ambos ultrasonidos conservan el umbral crítico
de 200 mm y las fallas enclavadas. Caída IR, emergencia y pérdida de datos
sensoriales conservan su prioridad. No hay rearme automático.

La proximidad recuperable es una medida frontal válida y reciente
**mayor de 200 mm y menor o igual a 275 mm**, detectada durante avance.
Provoca cero motores y evaluación de retirada, sin inventar una falla.
Con distancia frontal o trasera de 200 mm o menos se aplica Safety;
AUTO-4 no intenta retirar al robot bajo ese enclavamiento. No existe
evidencia física suficiente para autorizar esa excepción.

La ausencia de eco nunca se convierte en espacio libre. No se añade una
tolerancia a errores de adquisición ni reintentos de movimiento. Se esperan
ecos posteriores a la detención solamente mientras las mediciones anteriores
siguen siendo válidas y recientes. Un fallo sensorial real sigue la política
conservadora existente, incluso si se produce durante una espera.

## 2. Secuencia y límites

Camino despejado:

`ESPERAR_DATOS → AVANZAR → DETENER_Y_ESPERAR → AVANZAR ... → DETENIDO`

Ante proximidad preventiva durante avance:

`AVANZAR → DETENER_Y_ESPERAR → RETROCEDER_ACOTADO → DETENER_Y_ESPERAR → GIRAR_ACOTADO → DETENER_Y_ESPERAR → AVANZAR`

Cada cambio de fase revoca permisos y consignas anteriores. Las esperas no
emiten movimiento. Antes de abrir la siguiente fase se exige estabilización
y una nueva referencia odométrica; no se reinicia la odometría global.
Después del giro se reevalúan los sensores antes de avanzar.

| Parámetro | Valor | Motivo |
| --- | --- | --- |
| Sesión AUTO-4 | 12000 ms desde la entrada; nunca se renueva | Presupuesto finito para hasta cuatro segmentos y dos recuperaciones |
| Segmentos iniciados | Máximo 4, incluidos los interrumpidos por obstáculo | Evita que la evasión habilite segmentos adicionales indefinidamente |
| Segmento de avance | 100 mm, +1600 cps por rueda, máximo 3000 ms | Plazo exclusivo AUTO-4 ajustado tras el primer ensayo; los perfiles anteriores conservan 2000 ms |
| Recuperaciones consideradas | Máximo 2 | Incluye un intento aunque luego sea rechazado por espacio o presupuesto |
| Evasiones completadas | Máximo 2 | Se cuentan al completar el giro |
| Recorrido acumulado | Máximo 500 mm estimados | Cuenta avance, retroceso, giro y movimiento residual observado |
| Prevención frontal | 275 mm | Conserva AUTO-2F; margen nominal de 75 mm sobre Safety |
| Autorización trasera | Estrictamente mayor de 250 mm | Conserva AUTO-2F para retirada de hasta 20 mm |
| Safety crítico | 200 mm, frontal y trasero | Sin reducción ni cambios de política |
| Retroceso | 20 mm, −1600 cps por rueda, máximo 800 ms | Distancia o plazo: lo que ocurra primero; timeout no equivale a evasión completada |
| Giro | 10° a izquierda, −1200/+1200 cps, máximo 800 ms | Experimental supervisado; conserva las guardas angulares de AUTO-3 |
| Estabilización | 4 observaciones nuevas, ventana mínima 60 ms, rango máximo 1 cuenta por rueda, timeout 200 ms | No acepta una sola muestra repetida como detención |
| Vigilancia de progreso | Al menos 1 mm cada 200 ms durante movimiento | Heredada; no se relaja para prolongar maniobras bloqueadas |
| Mejora antes de otro intento | Desplazamiento neto desde origen al menos 25 mm mayor que en el intento anterior | Supera la retirada de 20 mm; no confunde recorrido acumulado con salida del bloqueo |
| Repetición de pose de intento | Distancia ≤30 mm y diferencia angular módulo 360° ≤15° | Rechaza volver a una de las dos poses de recuperación conservadas |

Constantes en `Core/Inc/mid_navigation.h`. Son límites experimentales,
no una distancia de frenado garantizada ni una prueba de espacio lateral.
El límite global domina: no se promete ejecutar cuatro segmentos y dos
evasiones si cada fase consume su plazo máximo. La suma de esos peores
casos puede superar 12 s; se cancela por tiempo antes de completar todo,
en lugar de extender la sesión para terminar la secuencia.
Se conservan los límites de actualidad: instantánea de sensores 100 ms,
ultrasonidos 200 ms y odometría según `ODOM_MAX_INTERVAL_MS` (50 ms).

AUTO-4 exige además, en cada estabilización, que **ambos timestamps de
adquisición ultrasónica sean posteriores al inicio de esa detención**.
Esto evita reanudar usando únicamente ecos tomados durante la fase anterior.
No dispara adquisiciones; respeta la alternancia existente. Si no llegan
ecos nuevos y estabilización dentro de 200 ms, termina detenido. Se comprueba
con adquisiciones alternadas de 140 ms por sensor, separación de 70 ms y
latencia de publicación en las pruebas; el margen físico debe verificarse.

El giro exige frontal >275 mm y trasera >250 mm, incluso después de la
retirada. Una retirada correcta puede terminar sin giro si no recupera ese
espacio. No se elige otra dirección ni se repite a ciegas.

## 3. Barreras de ejecución y terminación

`Navigation_ObserveMotionSensors()` se ejecuta desde TaskMotorControl antes
de su sección de PI/PWM, sin adquirir sensores ni esperar. AUTO-4 comprueba
la última instantánea y establece una inhibición persistente `motion_hold`
si la dirección activa pierde autorización. Una lectura que vuelve a ser
despejada no revive esa consigna: Navigation debe resolver la transición.
Las fallas críticas se comunican mediante `TaskSafety_ReportFault()` fuera
del lock externo, conservando los bits y el enclavamiento originales.

Las validaciones de tarea, origen, sesión, generación, timestamps y permisos
de emisión de un solo uso permanecen en la ruta Motion/MotorControl.
También se comprueban progreso, vencimientos y límite acumulado antes de
aplicar una consigna pendiente. No hay escritura directa de PWM desde Navigation.

`Navigation_ContinuousObserve()` toma como máximo una observación por tick
del productor, comprueba orden temporal, finitud, rangos y restas int64
antes de acumular desplazamientos. La conversión usa diámetro y CPR vigentes
de cada rueda. El recorrido es la suma del promedio de los módulos de ambos
desplazamientos; no se suma `increment_m` como recorrido entre consultas.

El campo `useful_mm` es la distancia radial entre pose actual y origen de
sesión. Es una guarda sencilla contra repeticiones, **no** progreso hacia
una meta, detección completa de ciclos ni planificación de trayectoria.
Puede rechazar una ruta físicamente posible; se prefiere detenerse.

Nuevos motivos terminales:

- `BLOCKED_NO_SAFE_PATH` (13): obstáculo inicial, retirada o giro no
  autorizados, ausencia de mejora, repetición o exceso de recuperaciones.
- `LIMIT_REACHED` (14): límite de segmentos o recorrido.

Se conservan los motivos anteriores para STOP, cancelación, Safety,
timeout, sensores, odometría, falta de progreso y errores de emisión.
Un bloqueo de ruta sin falla crítica no inventa una falla de hardware ni
exige RESET. La navegación queda terminal y ambos motores con objetivo cero.
La FSM puede permanecer en AUTO: se requiere **STOP → IDLE y nueva solicitud**.
RESET no restaura consignas ni reinicia una navegación terminada.

El cuarto segmento no permite iniciar un quinto aunque termine con evasión.
Un obstáculo preventivo tiene prioridad sobre un objetivo de avance alcanzado
simultáneamente en AUTO-4. STOP y Safety prevalecen sobre toda transición.

## 4. Comunicación, supervisión y limitaciones

El dashboard incorpora una solicitud experimental explícita. Consume la
confirmación de supervisión y espacio lateral de ese ensayo, igual que
AUTO-3; no se inicia por polling, recarga o reconexión. El botón AUTO normal
continúa enviando AUTO-2F. HTTP utiliza la lista cerrada existente:
`POST /api/command`, cuerpo `AUTO_CONTINUOUS`; UART: `AUTO_CONTINUOUS\n`.
La aceptación de API/UART no demuestra ejecución; STATUS confirma el estado
principal y `/api/navigation` describe las fases y desplazamientos estimados.

No se añade un heartbeat de operador: STATUS UART emitido por ESP32 tampoco
demostraría que sigue conectado el usuario por Wi-Fi. **Una pérdida de Wi-Fi
puede dejar continuar la sesión hasta sus límites STM32 o una cancelación
de Safety.** La sesión no supera sus límites por falta de dashboard ni se
reanuda al volver la conexión. El timeout existente de consignas de 500 ms
protege frente a falta de renovación interna; no sustituye los límites de
la navegación ni garantiza detención ante cualquier bloqueo total de CPU.
STOP web depende del enlace; no se presenta como parada de emergencia física.

Sin sensores laterales, ni los ecos ni la confirmación humana miden el
espacio barrido por el chasis. AUTO-4 se limita a un entorno controlado,
despejado lateralmente, con supervisión continua y dos giros pequeños como
máximo. No garantiza encontrar una salida ni autoriza explorar ambientes
desconocidos. No se añadieron umbrales inferiores a 200 mm.

## 5. Telemetría coherente e histórica

Se conservan NAV, NAVW y NAVT. Para perfil 5 se consulta además
`NAV Q=N S=3\n`, respuesta acotada a 255 bytes:

```text
NAVC,1,Q,SESSION,START,SEGMENTS,COMPLETED_SEGMENTS,ATTEMPTS,EVASIONS,PATH_UM,USEFUL_UM,FORWARD_UM,BLOCK,PERMISSIONS,RESUME_STATE,RESUME_MS,OBS,WINDOW_OBS,WINDOW_MS,LSPAN,RSPAN,CRITICAL_FAULTS,SNAPSHOT_AGE_MS,SESSION_MAX_MS,SEGMENT_LIMIT,EVASION_LIMIT,PATH_LIMIT_MM,ECHO_REFRESH_PENDING
```

Son 27 campos numéricos después del prefijo NAVC, incluida la versión.
Distancias en micrómetros enteros sin signo salvo `PATH_LIMIT_MM` en mm;
tiempos de espera y edad en ms; START identifica el tick de inicio.
La instantánea congelada se vincula por Q, sesión e inicio y vence a 1000 ms.
El parser estricto rechaza partes incompatibles, fuera de rango o mezcladas.
Los límites JSON proceden de STM32; no se duplica una configuración distinta
en ESP32. No hay sesión ni comandos nuevos creados por esta consulta.

`BLOCK`: 0 ninguno; 1 prevención frontal; 2 espacio trasero; 3 autorización
de giro; 4 ausencia de mejora neta; 5 pose repetida; 6 límite de intentos;
7 obstáculo al iniciar. Es diagnóstico, no un bit de falla Safety.

`PERMISSIONS`: bit 1 avance, bit 2 retirada, bit 4 elegibilidad sensorial de
giro supervisado. **No es una autorización independiente de MotorControl**
ni demuestra cobertura lateral. Una sesión terminal publica máscara cero.
Los resultados de estabilización conservan la semántica existente
(0 ausente, 1 en curso, 2 confirmada, 3 timeout, 4 cancelada).

`GET /api/navigation` añade `continuous` con:

- `available`, `age_ms`, `historical`, `echo_refresh_pending`.
- `segments`, `completed_segments`, `attempts`, `evasions`.
- `path_mm`, `forward_mm`, `useful_mm`, `block_reason`.
- `forward_eligible`, `reverse_eligible`, `supervised_turn_eligible`.
- `critical_faults`, `limits` y `resume_stabilization`.

Si falta la parte complementaria, sus valores no se inventan: se indica
indisponibilidad y campos nulos. Se mantienen los diagnósticos anteriores de
avance, retirada, giro y estabilizaciones previas. Entre repeticiones se
conservan los contadores acumulados y la última maniobra de cada tipo, no
un historial completo de todas sus muestras. Después de STOP se retiene
la última sesión; una nueva sesión no hereda sus autorizaciones.

Se descubrió mediante prueba del `app_main()` real que añadir una cuarta
parte sin ordenar las consultas podía dejar NAVT sin servicio. AUTO-4
solicita ahora NAVW, NAVT y NAVC en orden dentro del ciclo congelado.
El único receptor UART usa espera de 20 ms cuando hay diagnóstico AUTO-4
reciente activo o partes pendientes, incluido el cierre histórico; conserva
100 ms en el resto de casos. No cambia el baudrate, las tramas, tareas ni
prioridades de STOP/DETENER y movimiento sobre consultas. No hay ráfagas
para recuperar consultas atrasadas. Las pruebas verifican convivencia con
STATUS y con las consultas de sensores, pose y ecos.

## 6. Mapa real y recursos

Se reutiliza sin alterar el mapa existente: encoders/POSE, RAY asociados
temporalmente, grilla 80×80 y trayectoria acotada a 128 puntos. AUTO-4 no
modifica la pose global. Sus desplazamientos y ecos pueden actualizar esa
representación diagnóstica real; no se insertan obstáculos simulados.
Las celdas desconocidas siguen siendo desconocidas y nunca autorizan giros.

El panel de navegación muestra fases, evasiones y bloqueos junto al mapa.
No se dibujan marcas precisas de cada detención: no existe un registro
completo de eventos de navegación asociado a poses para garantizar esa
atribución. La frecuencia de telemetría y el historial limitado pueden
omitir detalles; el mapa no es una autoridad de seguridad ni un SLAM.

Compilaciones finales:

| Firmware | Memoria verificada |
| --- | --- |
| STM32 | text 109240 B, data 948 B, bss 37000 B; data+bss 37948 B |
| ESP32 | Flash 951893/1048576 B (90,8%); RAM estática 77208/327680 B (23,6%) |

Estas cifras no representan heap libre ni stack libre medidos en ejecución.
El JSON de navegación utiliza el buffer estático existente de 4096 bytes.
La inspección del ELF encuentra 3088 bytes en la cadena de frames analizada
de HTTP, frente a stack configurado de 4096 (diferencia 1008). Es una
estimación estática parcial, no una garantía de high-water mark del SDK;
la medición real sigue pendiente. No se amplió el stack HTTP.

## 7. Archivos y validación de software

STM32: `mid_navigation.c/.h` (perfil, límites, contadores, guardas, NAVC),
`task_control_robot.c` (selección cerrada), `task_comms.c` (comando y consulta),
`task_motor_control.c` (observación preventiva fuera de la sección PI).
Pruebas en `Tests/safety_b11/`, incluido `test_navigation_auto4.c`.

ESP32: `stm32_navigation.c/.h` (parser, instantáneas, JSON y planificación),
`uart_comm.c/.h` (comando y único receptor), `web_commands.c`, `web_server.c`,
`web/app.js`, `web/index.html` y pruebas `Tests/w2/`, incluido `test_auto4.c`.
No se modifican Safety, drivers, adquisición, PI/PWM, CPR, geometría,
odometría, IMU/fusión ni algoritmos del mapa.

Resultados con módulos reales y mocks; datos sintéticos exclusivamente en tests:

| Grupo | Resultado |
| --- | --- |
| Safety/control/navegación, configuración operativa | 607 aprobados |
| AUTO-1 | 363 aprobados |
| Configuración histórica experimental izquierda/derecha | 633 + 633 aprobados |
| Perfil B7 y giros históricos izquierda/derecha | 436 + 55 + 55 aprobados |
| Odometría/protocolo | 28 aprobados |
| Adquisición de sensores | 8 aprobados |
| IMU/adquisición/calibración | 48 aprobados |
| Driver ultrasónico | 19 aprobados |
| ESP32 nativo, incluidas 5 interoperabilidades con tramas reales del formatter STM32 | 203 grupos aprobados |
| JavaScript W2–W7/IMU y pérdida de endpoints, runtime V8 | 164 grupos aprobados |

Incluyen secuencias de cero, una y dos evasiones; rechazo de una tercera;
prohibición del quinto segmento; revisita y falta de mejora; nuevo eco tras
detención; errores de odometría/valores no finitos; límites de recorrido,
progreso y tiempo; todos los umbrales; Safety/STOP en seis fases; críticas
sin rearme automático; permisos antiguos; retención histórica; parser y
JSON; concurrencia HTTP; mapa real durante polling; ausencia de comandos
automáticos al reiniciar/reconectar; y regresiones anteriores.

Comandos de compilación y ejecución desde la raíz, salvo donde se indica:

```powershell
# En Firmware/stm32_control; -fstack-usage solo añade informes de compilación.
mingw32-make -B -o build -j4 "CC=arm-none-eabi-gcc -fstack-usage"
# Desde la raíz:
pio run -d Firmware/esp32_gateway -e esp32
cmd /c Firmware\stm32_control\Tests\safety_b11\run.bat
powershell -ExecutionPolicy Bypass -File Firmware/esp32_gateway/Tests/w2/run.ps1
git diff --check
```

Las cinco pruebas cruzadas nativas leen variables `GAR_AUTO3_BASE_0/1`,
`GAR_AUTO3_TURN_0/1`, `GAR_AUTO4_BASE_0/1/2` y `GAR_AUTO4_DETAIL_0/1/2`,
extraídas de las líneas `AUTO3_*_WIRE` y `AUTO4_*_WIRE` del log STM32.
Sin esas variables el runner sigue pasando, pero no ejecuta esos cinco casos.
JavaScript usa `Tests/w2/run_js.js` con sus fuentes reales en V8; no requiere
dependencias nuevas. No se confunden estas simulaciones con validación física.

Logs y resultados locales ignorados por Git:
`Firmware/stm32_control/build/auto4_*.log`,
`Firmware/esp32_gateway/.pio/build/auto4_*.log` y
`auto4_javascript_results.json`. Los builds enlazaron correctamente,
sin errores ni advertencias de compilación detectadas.

## 8. Validación física posterior, solamente tras revisión

1. Revisar el diff y el perfil seleccionado. Con batería desconectada,
   programar las placas retiradas de la PCB mediante el procedimiento USB
   habitual, usando `build/stm32_control.elf` y el entorno PlatformIO `esp32`.
   No confundir un build correcto con una grabación ya realizada.
2. Reinstalar ambas placas y alimentar desde batería. Entrar por Wi-Fi;
   comprobar STATUS, SENSORS, odometría y navegación. Preparar un área de
   ensayo estable, sin caídas y despejada alrededor del chasis. Mantener
   referencias frontal/trasera que den ecos válidos dentro del alcance útil
   observado (por ejemplo alrededor de 0,5–1 m), sin interferir en el recorrido.
3. Desde IDLE seguro, seleccionar explícitamente AUTO-4 y confirmar supervisión.
   Primero probar segmentos sin obstáculo: detenciones intermedias, máximo
   cuatro inicios y terminal LIMIT_REACHED, sin quinto segmento ni reinicio.
   Tener accesible STOP y la desconexión de alimentación. No depender de una
   detención remota durante pérdida de Wi-Fi.
4. Con el área lateral controlada y espacio trasero, probar una detección
   preventiva. Observar cero, estabilización/eco nuevo, retirada hasta 20 mm,
   segunda estabilización, giro pequeño, tercera estabilización y evaluación
   antes de avanzar. No exigir precisión milimétrica ni instrumentos nuevos.
5. Probar un obstáculo persistente o falta de espacio trasero sin acercar el
   robot intencionadamente a una colisión: debe terminar BLOCKED sin rearmarse
   ni reiniciar. Si el frontal no queda >275 mm después de retroceder, el giro
   debe rechazarse. Un giro de 10° no garantiza evitar el objeto.
6. Comprobar STOP en distintas fases y conservación del diagnóstico tras IDLE.
   Una falla crítica real conserva Safety y requiere rearme explícito; no se
   ensaya anulando IR, eco o enclavamientos. Comparar mapa y recorrido visible
   como diagnóstico aproximado, sin utilizar el mapa para autorizar la maniobra.

Pendientes físicos: márgenes reales de frenado y ecos durante movimiento,
estabilización dentro de 200 ms, comportamiento de dos recuperaciones,
deriva/deslizamiento de odometría, latencia HTTP/UART y stack/heap de ejecución.
La memoria estática, los límites y las pruebas de software no garantizan
ausencia de colisión, detección de todo obstáculo ni recuperación de toda ruta.

## 9. Ajuste del plazo de segmentos tras el primer ensayo

Resultado físico aportado por el usuario: primer segmento de 92,953 mm,
2020 ms, `TIMEOUT`, cero segmentos completados, sin obstáculo ni error
odométrico, con detención correcta. No se realizaron ensayos físicos nuevos
durante esta corrección.

El código confirma una causa suficiente: `Navigation_DeadlineExpired()`
aplicaba `NAV_AUTO2_ADVANCE_MAX_MS = 2000` también a AUTO-4. Comprueba el
vencimiento antes de evaluar el objetivo; lo utilizan Navigation y la
barrera de aplicación de consignas de MotorControl. Una distancia inferior
a 100 mm al vencer el plazo no completa el segmento. Los 2020 ms observados
son compatibles con la evaluación cíclica y el registro de la terminación;
no demuestran por sí solos el instante exacto en que el PWM llegó a cero.

Se introduce `NAV_AUTO4_ADVANCE_MAX_MS = 3000` y se selecciona solamente
para perfil `NAV_PROFILE_CONTINUOUS`. Conserva 100 mm, velocidades, STOP,
Safety, watchdog de progreso, espera, recorrido, cantidad de segmentos y
sesión global de 12000 ms. AUTO-2F y AUTO-3 mantienen 2000 ms de avance.
No hay cambios de ESP32, protocolo o dashboard por este ajuste.

Como referencia aproximada, 92,953 mm / 2,020 s da 46 mm/s; a ese promedio
100 mm requerirían unos 2,17 s. Cuatro segmentos representarían unos 8,7 s,
más hasta 0,6 s de tres estabilizaciones intermedias y las transiciones.
Esto deja un margen razonable bajo 12 s sin evasiones, pero no garantiza
la misma velocidad futura ni cuatro segmentos con maniobras de recuperación.
Cuatro plazos completos de 3000 ms ya consumen 12 s: prevalece el límite
global y no se amplía para completar una estabilización o segmento pendiente.

La prueba nueva con conteos sintéticos, conversión geométrica real y muestras
de 20 ms reprodujo el fallo antes de corregir: a 2000 ms la navegación ya
no seguía en AVANZAR, sin alcanzar 100 mm. Después del cambio comprueba:

- Objetivo alcanzado después de 2000 ms y antes de 3000 ms, con cero motores
  al completar el segmento y contador incrementado una sola vez.
- Cuatro segmentos lentos, con sus estabilizaciones, completados dentro de
  los 12000 ms y sin habilitar un quinto.
- Consigna admisible a 2999 ms; detención a 3000 ms o en el primer ciclo
  posterior (3020 ms), tanto por Navigation como por autorización del motor.
- AUTO-2F y AUTO-3 siguen venciendo a los 2000 ms originales.
- El global de 12000 ms cancela un segmento posterior aunque aún tenga
  tiempo individual disponible; no hay reactivación tras terminar.

Regresiones completas: 607/363/633/633/436/55/55 pruebas STM32 según
configuración; 203 grupos nativos ESP32 con tramas cruzadas; 164 grupos
JavaScript. Ambas compilaciones correctas, sin advertencias detectadas.
Evidencias de este ajuste en el directorio ignorado
`Firmware/stm32_control/build/auto4_segment_review/`, incluido
`before_fix_tests.log` con la reproducción previa y `after_fix_tests.log`
con las regresiones posteriores. La validación física del nuevo plazo
queda pendiente de revisión y grabación por parte del usuario.
