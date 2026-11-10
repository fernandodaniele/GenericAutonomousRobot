# W7 — Localización relativa y mapa de ocupación 2D

Actualización W7.4: este documento conserva el punto de control W7 original.
La geometría nominal, las tres clases de calibración y el protocolo POSE v2
actuales se describen en [W7_4_GEOMETRIA_CALIBRACION.md](W7_4_GEOMETRIA_CALIBRACION.md).
Los CPR continúan pendientes; no se habilitan métricas con CPR desconocido.

## Estado y alcance

Implementación de software sobre `e2ca82a`, rama `G1-add/InterfazWeb`.
No se grabaron placas ni se realizaron movimientos o validaciones físicas.
El mapa es informativo y no interviene en Safety, control manual, AUTO ni
planificación de rutas. No hay SLAM, cierre de bucle ni referencia absoluta.

**La configuración de producción está NO CALIBRADA.** Se publican conteos
reales, pero posición métrica, proyección y ocupación permanecen no disponibles
hasta incorporar medidas físicas. La grilla inicial desconocida no es un mapa
simulado; no se dibuja un robot en un origen ficticio. La calibración sintética
de las pruebas no se utiliza en el firmware.

## Datos físicos encontrados y pendientes

| Parámetro | Evidencia y decisión |
| --- | --- |
| Encoders | TIM1 izquierdo de 16 bits, TIM2 derecho de 32 bits, cuadratura HAL; lectura diferencial existente cada 10 ms. Se conservan sus signos y cableado validados. |
| Resolución efectiva | No hay un CPR/PPR físico documentado. Hay que medir cuentas de cuadratura por vuelta de **rueda** o proporcionar resolución del encoder, multiplicador y reducción conocidos. |
| Relación de transmisión | No está documentada. No se deduce de las consignas ni del PWM. No es indispensable conocerla por separado si se mide directamente el CPR de rueda efectivo. |
| Radios | No disponibles; medir radio/diámetro efectivo de cada rueda bajo carga. |
| Separación | `mid_kinematics.h` declara 200 mm, sin evidencia de calibración; W7 no usa ese valor como medida real. Medir separación entre centros/contactos efectivos de ruedas. |
| Extrínsecos HC-SR04 | Medir offsets X/Y y yaw de cada sensor respecto al centro entre ruedas. Frontal=0 y trasero=π son solamente orientaciones nominales, no una validación del montaje. |
| Haz | Configurar una semiamplitud conservadora de incertidumbre angular a partir de información verificable y ensayo posterior. No se inventa una apertura calibrada. |
| MPU6050 | Existe `TaskIMU` con cola y lecturas cada 20 ms, pero sin timestamp de pose ni rumbo absoluto validado. No se incorpora a la odometría W7 ni se afirma que esté calibrado. |

Completar exclusivamente los valores medidos de `Core/Inc/odometry_config.h`:
`ODOM_LEFT_RADIUS_M`, `ODOM_RIGHT_RADIUS_M`, `ODOM_LEFT_WHEEL_CPR`,
`ODOM_RIGHT_WHEEL_CPR`, `ODOM_TRACK_M`. Están en cero y
`ODOM_CALIBRATION_CONFIRMED=0`. Habilitar la confirmación solo tras revisar las
medidas. Para proyección completar offsets/yaws, `ODOM_BEAM_HALF_ANGLE_RAD` y
`ODOM_SENSOR_GEOMETRY_CONFIRMED`; esta confirmación es independiente de la
calibración de ruedas. No hay configuración remota ni un botón web que eluda
estas condiciones. Aplicar valores requiere reconstruir y, con autorización
posterior, programar firmware.

## Modelo de odometría STM32

`mid_odometry` recibe exactamente los deltas ya leídos por MotorControl; no
consulta los timers ni consume nuevamente `Encoder_ReadDelta()`. Su actualización
ocurre después de la transacción PI/PWM, fuera de su sección crítica. Se cuentan
también movimientos residuales o manuales de las ruedas con Safety bloqueado.
STOP/RESET no reinician la pose ni sus conteos.

Con `dl=delta_l*2π*radio_l/CPR_l`, `dr=delta_r*2π*radio_r/CPR_r`:

```
ds = (dr + dl)/2
dtheta = (dr - dl)/separacion
x += ds*sinc(dtheta/2)*cos(theta+dtheta/2)
y += ds*sinc(dtheta/2)*sin(theta+dtheta/2)
theta = normalizar(theta+dtheta) a [-pi, pi]
```

Se conserva el límite estable de `sinc` cerca de cero. X/Y son metros, theta
radianes; incrementos firmados por intervalo. El origen relativo es el inicio
de MotorControl, +X avance inicial y +Y izquierda, giro positivo antihorario.
Los acumulados son `int64_t`, con comprobaciones antes de sumar. No dependen
del estado principal ni constituyen órdenes de movimiento.

Estados: 0 sin baseline, 1 sin calibrar, 2 pose válida, 3 discontinuidad/error
numérico. Un intervalo nulo o mayor de 50 ms, overflow, paso >0,25 m,
incremento angular >π o coordenada fuera de ±100 m invalida la métrica hasta
reiniciar la adquisición de odometría al arrancar nuevamente. Son guardas
numéricas/de calidad, no límites de actuación ni garantías físicas.

El delta modular del driver conserva el desbordamiento de TIM1/TIM2. Como en
el driver existente, no se puede recuperar un movimiento superior a media
escala de contador entre lecturas. Debe comprobarse esa cota con CPR y velocidad
reales. La guarda temporal no detecta por sí sola todo alias de encoder.

## Asociación temporal de ecos

Historial estático de 64 poses: nominalmente 630 ms entre extremo más antiguo
y más nuevo con muestras cada 10 ms. `Odometry_PoseAt()` busca dos muestras
que encierren el tick del disparo; interpola X/Y y el arco angular corto.
No extrapola, no sustituye por la pose actual y rechaza historia insuficiente
o poses inválidas. Los cálculos de edad usan restas de `TickType_t`, incluido
timestamp cero y wrap.

`TaskSensores_SetUltrasoundResult()` notifica un observador después de resolver
cada intento. No cambia `SensoresDatos_t`, validez, fechas ni cadencia de
adquisición. Cada sensor tiene secuencia independiente y registra intentos
fallidos. Se usan la distancia y el tick originales del intento, no una lectura
antigua conservada tras timeout.

Con pose `(x,y,theta)` asociada y extrínsecos `(sx,sy,yaw)`:
`sensor_world=(x+cos(theta)*sx-sin(theta)*sy, y+sin(theta)*sx+cos(theta)*sy)`;
la dirección mundial es `theta+yaw`. Es el origen del sensor, no el centro
del robot. El tick existente representa la solicitud software de disparo antes
del armado/TRIG, no una marca hardware exacta del eco. Hay cuantización de
ticks, latencia de armado y tiempo de vuelo; el modelo no elimina ese error.
La interpolación presupone movimiento aproximadamente uniforme entre muestras.

## Responsabilidades, memoria y concurrencia

- STM32: acumulados, integración, historial y dos ecos asociados. Publicación
  y copias breves bajo las secciones críticas FreeRTOS existentes; búsqueda
  acotada a 64 puntos. Trigonometría, formato y UART quedan fuera del bloqueo.
- ESP32: un único lector UART alimenta los parsers de STATUS, SENSORS y mapeo.
  Solo ese contexto modifica la grilla. Publica una copia bajo `portMUX`; HTTP
  copia y formatea fuera del bloqueo. No hay E/S ni construcción del mapa
  dentro del bloqueo. No se agregan tareas ni colas FreeRTOS ni heap del modelo.
- Se mantienen buffers estáticos para evitar poner la grilla o JSON en los
  stacks existentes. Hay tres copias acotadas de caché: trabajo, publicación y
  copia de HTTP. No se usa un mutex de red para retener el productor UART.

La configuración inicial es 80×80 celdas de 50 mm: **4×4 m**, origen inferior
izquierdo (-2000,-2000) mm. Cada copia de la grilla usa 6400 bytes más una
trayectoria circular de 128 pares `int32_t` (1024 bytes) y metadatos. Dimensiones
y resolución se centralizan en `mapping_grid.h`; validaciones de compilación
limitan dimensiones a 128 y producto a 10000 celdas, resolución 20–200 mm.
El buffer JSON depende de esos límites. No hay crecimiento ilimitado.

La reserva estática total verificable pasó de 33912 a 35532 bytes en STM32
(+1620 B) y de 34160 a 69712 bytes en ESP32 (+35552 B). El heap FreeRTOS STM32
sigue siendo el existente, incluido en BSS; no son cifras de heap libre en
ejecución. Los stacks y prioridades no se cambian. `-fstack-usage` reporta,
por función, 96 B para Update, 104 B para PoseAt, 96 B para Observe y 272 B
para formato de telemetría; no incluyen todas las llamadas de biblioteca ni
el contexto de interrupción. Medir high-water marks y jitter en hardware
antes de cualquier autorización de movimiento.

## Protocolo UART de solo lectura

Solicitud ASCII exacta: `MAPPING Q=N S=P\n`, N decimal no nulo uint32,
P=0 pose, 1 frontal, 2 trasero. Se rechazan overflow y sufijos. Una solicitud
produce una línea de menos de 160 bytes, dentro del buffer ESP32 de 192 bytes
del parser y las lecturas UART de 255 bytes. Los comandos RX STM32 siguen
cabiendo en 64 bytes. No se dispara ningún sensor.

```
POSE,1,Q,E,T,ESTADO,CAL,L,R,X,Y,A,DS,DA,EDAD
RAY,1,Q,E,T,SENSOR,SEQ,ESTADO,EDAD,D,X,Y,A,HAZ
```

- `1`: versión; `Q`: eco exacto de consulta; `E`: primera Q recibida por
  STM32 desde su arranque. Una secuencia ESP32 no nula permite detectar un
  nuevo arranque STM32 y descartar el mapa/pose anteriores. No es un ID
  persistente ni criptográfico. Requiere que no se reutilice esa primera Q;
  las colisiones teóricas de 32 bits no son una garantía absoluta de identidad.
- `T`: tick STM32 de la muestra/solicitud de disparo; `EDAD`: edad en ms al
  construir la respuesta. ESP32 añade el RTT completo, conservadoramente, y
  después tiempo monotónico local. No compara relojes de las placas.
- `CAL`: 0/1, calibración de ruedas confirmada. Estado POSE: 0 sin muestra,
  1 no calibrada, 2 válida, 3 inválida. `L/R`: acumulados firmados decimales
  de 64 bits, convertidos sin depender de `%lld` de newlib nano.
- `X/Y/DS`: enteros mm; `A/DA/HAZ`: enteros mrad. Los valores métricos POSE
  solo aparecen en estado 2; de otro modo son `-`, nunca ceros sustitutos.
- `SENSOR`: 0 frontal, 1 trasero. `SEQ`: una secuencia por intento resuelto,
  no por consulta; cero solo sin muestra. Estado RAY: 0 sin muestra,
  1 adquisición fallida, 2 eco sin asociación métrica, 3 eco asociado.
- `D`: distancia mm, solo para RAY 2/3. `X/Y/A/HAZ` existen solo para RAY 3.
  Cada tupla se valida completa; un código o formato desconocido no refresca
  la caché. Secuencias/ticks anteriores se rechazan con comparación modular.

Consultas opcionales cada 100 ms, alternando POSE/frontal/trasero; nominalmente
300 ms por tipo, sin recuperar slots atrasados. Una sola consulta pendiente;
respuesta válida antes de 200 ms y correlacionada por Q/parte. No hay ráfagas
ni reintentos inmediatos. Pueden perderse capturas intermedias; no se simulan.
La continuidad del mapa se conserva, pero no se garantiza observar todo eco.

Prioridad de despacho ESP32: STOP/DETENER, control manual/modos, STATUS,
SENSORS y finalmente MAPPING. El buzón STM32 también permite que SENSORS
desplace MAPPING, sin cambiar las prioridades relativas operativas anteriores.
Durante MANUAL se consulta solo sin dirección/parada/control pendiente y con
**más de 150 ms** de margen en la lease de 300 ms. Se revalida después del
turno TX. MAPPING no renueva permisos. Normalmente una respuesta máxima tarda
menos de 14 ms de línea a 115200 bps; el timeout HAL preexistente ante error
no se modifica. Safety/MotorControl tienen mayor prioridad que Comms.

## Modelo de ocupación ESP32

Estados de celda: 0 desconocida, 1 libre observado, 2 hipótesis conservadora de
ocupación. Solo se insertan ecos localizados, nuevos, de 20–4000 mm, con
edad total hasta 300 ms, calibración métrica y pose disponible. Una lectura
inválida o sin pose no vacía el mapa.

Un eco es **una observación con ambigüedad angular**. Aporta evidencia libre
solo a una línea central hasta antes de su margen de alcance; un arco de
incertidumbre aporta ocupación posible alrededor del rango. La semiamplitud
angular viene de la configuración confirmada. Se utiliza un margen inicial
de rango de 100 mm, provisional del modelo, no una precisión del HC-SR04.
Las celdas sobre el arco no son retornos angulares independientes. No se
vacía todo un cono ni se declara espacio lateral libre.

La ocupación prevalece sobre evidencia libre posterior: no se eliminan
automáticamente hipótesis previas. Esto es conservador pero deja obstáculos
fantasma y no sirve todavía para entornos dinámicos. Cada proyección recorre
un número acotado de pasos y segmentos; se rechaza fuera de límites sin marcar
un falso obstáculo sobre el borde. No hay expansión de mapa. La pose fuera
de la grilla sigue disponible numéricamente, con advertencia visual.

El mapa y trayectoria persisten entre consultas, STOP y cambios MANUAL/AUTO.
Se reinician al reiniciar ESP32 o detectar un nuevo E de STM32. No se guardan
en SD/flash y no se mezclan marcos relativos de arranques distintos.

## HTTP e interfaz offline

`GET /api/map` es solo lectura. Incluye `connected`, `calibrated`, `epoch`,
`pose` (estado, validez, stale, tick, edad, conteos como **strings** para
conservar 64 bits, coordenadas/incrementos mm/mrad o null), estado/edad de
dos ecos, `grid` (dimensiones, resolución, origen, revisión y string de celdas
0/1/2 en filas de Y creciente) y `trajectory_mm` (máximo 128 pares).
`connected` requiere STATUS reciente y POSE con edad menor a 1500 ms; una
pose solo es vigente hasta 500 ms. Datos antiguos conservan sus coordenadas,
pero se marcan históricos. No son permiso de movimiento.

`/map.js` se embebe en flash como los demás recursos. El panel consulta una
vez cada 500 ms después de terminar la anterior, timeout HTTP de 1500 ms,
sin POST ni generación de movimiento. Muestra calibración, conteos, edad,
pose, orientación y trayectoria; los datos ausentes no producen una flecha
en (0,0). El origen inicial se ve en el centro de la grilla; +X a la derecha,
+Y arriba en pantalla. Canvas respeta el aspecto de grillas rectangulares.
PC y teléfono usan el ancho disponible sin alterar la cruceta ni handlers W4.
No hay librerías, fuentes remotas, mapas ficticios ni dependencia de Internet.
LRU, STATUS, SENSORS, comandos y endpoints anteriores se conservan.

## Archivos

STM32 nuevos: `Core/Inc/odometry_config.h`, `mid_odometry.h`, `mid_mapping.h`,
`Core/Src/mid_odometry.c`, `mid_mapping.c`, `Tests/odometry_w7/test_odometry.c`
y `run.ps1`. Se conectan en `task_motor_control.c`, `task_sensores.c`,
`task_comms.c`, clasificación `drv_uart.c`, Makefile y pruebas existentes.
No se modifican drivers de encoder/ultrasonido/motor, PI, Safety ni FSM.

ESP32 nuevos: `include/mapping_grid.h`, `stm32_mapping.h`,
`src/mapping_grid.c`, `stm32_mapping.c`, `web/map.js`,
`Tests/w2/test_mapping.c`, `test_map.js`. Se integran en `uart_comm.c/.h`,
arbitraje de consulta `web_commands.c/.h`, bucle `main.c`, `web_server.c`,
CMake, listado de recursos PlatformIO, HTML/CSS y arneses de pruebas.
No se cambia framework, particiones, configuración de flash, Wi-Fi ni STM32
generado por CubeMX.

## Validación de software y límites físicos

Los tests usan módulos reales con mocks HAL/FreeRTOS/UART/HTTP; únicamente
sus valores de ruedas, posiciones y ecos son sintéticos. Cubren cinemática,
avance/reversa/giro, signos, wrap, error acumulado, límites, transformación y
asociación temporal, ocupación, parsing, edad y disponibilidad, prioridades,
concurrencia, web y regresiones W1–W6.

Reproducir: `Tests/odometry_w7/run.ps1`, `Tests/safety_b11/run.bat`,
`Tests/sensors_w5/run.ps1`, `Tests/ultrasound/run.bat` en STM32;
`Tests/w2/run.ps1` y `node Tests/w2/run_js.js` en ESP32. El runner JavaScript
también funciona en V8 suministrando las fuentes, sin instalar dependencias.
Construir STM32 en `build/w7` y ESP32 con `PLATFORMIO_BUILD_DIR` apuntando a
`.pio/w7-build`, sin sobrescribir ELF anteriores. No ejecutar targets upload.

Los mocks no validan mecánica, exactitud de pose, ángulo del sonar, jitter,
stack real ni coexistencia de tráfico físico. El mapa deriva por error de
radios, separación, deslizamiento y reflexiones; no puede corregirse por sí
solo ni confirmar cobertura lateral. No se usa como protección ni para elegir
rutas. Los timeouts de Safety/consignas y límites de sesión AUTO siguen intactos.

## Resultado de las verificaciones locales

- Compilación completa STM32 y verificación final correctas, sin advertencias.
  `text=90172`, `data=924`, `bss=34608`: carga Flash 91096 B, RAM estática
  35532 B. ELF: `Firmware/stm32_control/build/w7/stm32_control.elf`.
- ESP32 correcto, sin advertencias: 898673/1048576 B de aplicación (85,7 %),
  RAM estática 69712/327680 B (21,3 %). ELF:
  `Firmware/esp32_gateway/.pio/w7-build/esp32/firmware.elf`.
- STM32: 411 regresiones normales; 55 con giro experimental izquierdo y 55
  derecho; 22 grupos de odometría/protocolo; 8 de adquisición y 19 del driver
  ultrasónico. En el firmware normal el giro experimental sigue deshabilitado.
- ESP32: 98 grupos nativos; JavaScript 114 grupos, incluidos 11 del mapa y 21
  de convivencia con W4/W5 y endpoint de mapa indisponible. Se ejecutó el runner
  JavaScript real en V8, sin instalar Node ni dependencias.
- No hay cambios en Safety, FSM, PI/PWM, drivers de encoder/ultrasonido/motor,
  configuración de periféricos ni `sdkconfig.esp32`. Ninguna prueba anterior
  se eliminó. No hay productores de datos simulados en el firmware.

## Calibración física futura (requiere autorización)

1. Con potencia de motores desconectada, registrar conteos antes/después de
   un número conocido de vueltas de cada rueda, cuando el mecanismo permita
   giro manual sin forzarlo. Promediar varias vueltas y sentidos para obtener
   CPR efectivo. Si no se puede girar la rueda a mano, no forzar la reductora:
   identificar encoder y transmisión y acordar un ensayo supervisado.
2. Medir cada diámetro/radio efectivo y separación entre centros de ruedas.
   Medir los offsets/yaws de los HC-SR04 respecto al mismo marco. Documentar
   incertidumbres y determinar una semiamplitud angular conservadora.
3. Completar la configuración y revisar los flags de confirmación; reconstruir.
   La carga de firmware y cualquier ensayo físico quedan a cargo de una etapa
   autorizada posterior. No invertir por software el signo del encoder derecho.
4. Primero sin potencia: verificar conteos, pose relativa bajo desplazamiento
   manual si es viable, asociación de cada eco y reinicios del marco; STOP,
   Safety y el control manual deben seguir funcionando. Medir pila y jitter.
5. Solo con autorización adicional y medidas de contención, comparar recorridos
   y giros reales con referencias externas para estimar escala, base efectiva
   y deriva. Las ruedas elevadas sirven para signos/actuación, no para calibrar
   deslizamiento en suelo. No autoriza navegación libre ni planificación.
