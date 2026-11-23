# Exploración local supervisada: perfil LOCAL_EXPLORE

Base: `G1-add/InterfazWeb`, HEAD `79ddac6`, sobre los cambios pendientes
de AUTO-4, incluido el plazo de 3000 ms por segmento. Esta etapa implementa
y verifica software; no graba placas ni realiza ensayos físicos.

## Alcance y fundamentos

Perfil nuevo 6, solicitado mediante `AUTO_EXPLORE`. AUTO normal conserva
AUTO-2F; AUTO_TURN, AUTO_EVADE y AUTO_CONTINUOUS conservan sus decisiones,
plazos y protecciones. LOCAL_EXPLORE no tiene máximo global de segmentos,
evasiones, recorrido o tiempo de sesión. Cada movimiento sigue acotado y
la autorización se concede al iniciar; STM32 vigila por separado el enlace interno.

La inspiración académica es el histograma polar de obstáculos y la selección
de valles/candidatos de VFH: Borenstein y Koren, *The Vector Field Histogram—
Fast Obstacle Avoidance for Mobile Robots*, IEEE Transactions on Robotics
and Automation, 7(3), 1991, pp. 278–288,
[DOI 10.1109/70.88137](https://doi.org/10.1109/70.88137).
Se adopta la organización angular y el costo local de orientación, no una
implementación completa de VFH ni sus prestaciones experimentales. Aquí hay
dos sonares que se orientan con el cuerpo, no un barrido instantáneo denso.
No hay objetivo global, ROS, SLAM, planificación completa ni garantía de
convergencia; un valle de mediciones no demuestra por sí solo un paso libre.

## Arquitectura

- STM32 mantiene adquisición, pose/historial odométrico, evidencia angular,
  selección, estabilizaciones y ejecución mediante Navigation/Motion/MotorControl.
- `mid_local_exploration.c/.h` contiene solamente evidencia y selección puras:
  no adquiere sensores, crea tareas, espera, escribe PWM ni mantiene otra FSM.
- `mid_navigation` amplía su FSM interna con `NAV_EXPLORAR` (6). Reutiliza
  AVANZAR, RETROCEDER_ACOTADO, GIRAR_ACOTADO y DETENER_Y_ESPERAR.
- ESP32 conserva un solo receptor UART, arbitraje TX y servidor con LRU.
  El módulo `stm32_exploration` confirma la sesión, consulta y confirma respuestas diagnósticas
  por UART y expone diagnóstico; no decide movimientos ni depende del navegador.
- No se modifican Safety, drivers, PI/PWM, geometría/CPR, odometría global,
  IMU, fusión ni el algoritmo del mapa. No hay tareas ni colas nuevas.

## Evidencia y selección

Se reutilizan `Odometry_GetRay()` y las asociaciones temporales que ya
produce la adquisición. Cada eco conserva el origen y yaw del sensor en
el instante de disparo, incluidos los extrínsecos frontal/trasero. No se
asigna una pose actual a una medición antigua ni se disparan sonares desde
Navigation o HTTP. La alternancia y los períodos originales no cambian.

Hay 36 sectores fijos de 10°, en el marco global de odometría: 0° apunta
al X inicial y el sentido positivo es izquierdo. El frontal y el trasero
aportan observaciones a sus propios rumbos. Estados:

| Valor | Interpretación |
| --- | --- |
| 0 | Desconocido, no observado o evidencia vencida |
| 1 | Medido, evidencia insuficiente |
| 2 | Posible obstáculo, con inflación conservadora |
| 3 | Eco despejado en el rumbo medido; no certifica todo un cono libre |

Se rechazan rayos inválidos, sin pose, no finitos, fuera de 20–4000 mm,
de más de 200 ms, con secuencia repetida o anterior. Se soporta wrap de
secuencia y ticks. La evidencia vence a los 4000 ms y se descarta al cambiar
el origen local más de 30 mm. Esa ventana no cambia los plazos de Safety.

El umbral de planificación es 350 mm: prevención existente de 275 mm +
segmento de 50 mm + reserva experimental de 25 mm. No es un umbral crítico
ni una medida de frenado. Un retorno más próximo se infla angularmente con
el radio circunscrito nominal del chasis, 30 mm de margen y la incertidumbre
acústica provisional de ±45°. Un eco distante no borra una ocupación
contradictoria todavía reciente.

Un candidato requiere sectores contiguos medidos despejados. La anchura
depende del menor alcance local y del radio nominal del cuerpo: alrededor
de 800 mm bastan tres sectores para la heurística, mientras que a 400 mm
se requieren cinco. Se usa la discretización sectorial de 10°; esto no
convierte los puntos medidos en una garantía física de cobertura lateral.
El costo favorece menor giro y mayor alcance, y excluye poses/rumbos ya
visitados próximos. Los desconocidos nunca se convierten en libres.

La selección está acotada: 36 sectores, 32 visitas recientes, hasta 32
comprobaciones espaciales y cinco vecinos angulares por visita. No busca
sobre una grilla completa mientras MotorControl necesita ejecutar su ciclo.

## Secuencia y recuperaciones

1. Desde IDLE seguro se recibe una solicitud explícita AUTO_EXPLORE con nonce
   nuevo. El operador autoriza una sesión completa en el escenario preparado.
   La FSM y Safety validan el inicio; la autorización no tiene renovaciones.
2. STM32 se detiene y estabiliza. Antes del primer movimiento exige comprobar
   el enlace interno con una ida y vuelta diagnóstica nueva. ESP32 confirma el
   inicio al recibir una instantánea completa de esa sesión. Una respuesta de
   otra sesión, una consulta antigua o una reconexión no conceden autorización.
3. Con dos ecos frontales distintos, recientes, coherentes y asociados a la
   pose, prioriza el avance de 50 mm si no hay inhibición ni ciclo.
   No exige barrer el entorno antes de continuar en una dirección ya viable.
4. Si el frontal no es viable, emplea el histograma y los giros acotados para
   buscar y alinear alternativas. Las retiradas sucesivas conservan la guarda
   direccional y el historial de visitas. No existen aprobaciones intermedias.
5. Cada movimiento obtiene una nueva referencia y las autorizaciones internas
   de un solo uso; se revalidan sensores en Navigation y en MotorControl.
6. Al completar un segmento, detiene, estabiliza y reevalúa. Puede reutilizar
   esa misma ventana de detención para el avance inmediato si sigue reciente
   y ninguna rueda cambió más de una cuenta. No elimina la estabilización:
   evita repetirla dos veces entre segmentos. Tras esperas largas o movimiento
   residual debe estabilizar de nuevo.
7. El barrido y las retiradas permiten volver a buscar desde otra pose. Al
   agotar alternativas espera información nueva con motores detenidos.
   No hay cuota global de segmentos, retiradas, tiempo o evasiones.

| Límite por maniobra | Valor |
| --- | --- |
| Avance | 50 mm, +1600/+1600 cps, máximo 3000 ms |
| Retirada | 20 mm, −1600/−1600 cps, máximo 800 ms |
| Giro | Hasta 10°, ±1200 cps, máximo 800 ms |
| Barrido local acumulado | Hasta 180° estimados, incluyendo alineaciones |
| Estabilización | Cuatro muestras nuevas, ≥60 ms, rango ≤1 cuenta por rueda, máximo 200 ms |
| Falta de progreso | Guarda heredada: ≥1 mm equivalente por 200 ms |
| Referencia para repetición | Posición a menos de 50 mm y rumbo a menos de 20° |
| Autorización de navegación | Una sola vez al iniciar; sin renovaciones ni vencimiento periódico |
| Enlace interno | Consultas cada 500 ms; watchdog único STM32 de 3000 ms sin ida y vuelta nueva |
| Confirmación de respuesta | A la siguiente consulta; respuesta previa de menos de 1000 ms |
| Permiso de emisión | Interno y de un solo uso; no es una autorización humana por maniobra |

El último giro puede reducir su objetivo para respetar el presupuesto
restante, dejando una reserva angular de 2°. MotorControl comprueba además
el progreso angular observado antes de aplicar otra consigna. La cuantización
y el frenado impiden prometer exactamente 180° físicos; no se amplía el giro
para terminar una vuelta. Se conserva la odometría real como única fuente de
progreso, sin integrar consignas ni utilizar IMU-4 para controlar.

El timeout o la falta de progreso de una maniobra revocan sus consignas y
devuelven al robot detenido a la evaluación. Un error odométrico, una falla
crítica o la pérdida comprobada del enlace interno terminan la sesión; no se ocultan como
un intento de búsqueda normal. Nunca se renuevan automáticamente consignas
viejas después de un cambio de fase.

Un timeout o falta de progreso no habilitan repetir inmediatamente el mismo
intento. Se inhibe esa dirección en la pose actual; puede intentar el giro
opuesto y una retirada verificada a otra posición. Las inhibiciones se
reconsideran con al menos 15 mm de cambio de posición, 20° de orientación
medidos o un cambio sensorial confirmado, manteniendo el historial de visitas.
Si tampoco hay progreso en esas alternativas, espera detenido. Esta guarda
evita el ciclo que antes quedaba oculto por la pausa de confirmación humana.

## Bloqueo, espera y ciclos

Se registran poses de inicio de avance y retirada, no simples observaciones
de un barrido. Si ya no hay candidato ni nueva maniobra autorizable, queda
en EXPLORAR con motores cero y motivo `alternatives_exhausted` o `cycle`.
Esto no inventa una falla de hardware ni exige RESET. No repite el mismo
barrido por el simple vencimiento de un temporizador.

La reconsideración requiere cambios de al menos 50 mm respecto a las
distancias del bloqueo, confirmados por tres pares de adquisiciones nuevas
de ambos sonares, coherentes entre sí dentro de 25 mm. Muestras repetidas o
lecturas fluctuantes no reinician la búsqueda. Se reconsideran solamente
las visitas cercanas a la posición actual; se conserva el resto del historial.
Durante la confirmación se informa `environment_change_pending`, distinto
de `alternatives_exhausted` o `cycle`. Ante nueva evidencia confirmada puede
reevaluar y ejecutar alternativas automáticamente si la sesión sigue activa y el enlace interno vigente. La espera no equivale a una falla crítica ni requiere RESET.

La memoria es finita: 32 visitas recientes no prueban ausencia de todos los
ciclos posibles en recorridos largos. La espera significa agotamiento de
las alternativas actualmente autorizables, no demostración de que el entorno
completo esté cerrado. El histograma de 4 s tampoco equivale a observar
simultáneamente 360° durante un barrido lento de hasta 180°.

## Seguridad y comunicaciones

Se mantienen frontal/trasera críticos de 200 mm, prevención de 275 mm y
retirada solamente con trasera >250 mm. Una retirada de hasta 20 mm conserva
un margen nominal de 30 mm antes de los 200 mm, sin garantizar frenado físico.
Las latencias ultrasónicas y el frenado bajo carga todavía requieren ensayo.

**No se implementa la excepción direccional bajo 200 mm.** Safety actualmente
enclava la proximidad de cualquiera de los dos sonares para todos los modos.
Sin evidencia física suficiente de frenado y márgenes no se puede justificar
una modificación de esa política. Por debajo de ese umbral, caída, emergencia,
ausencia de eco o datos inválidos/antiguos se conserva la detención crítica
y el rearme explícito, aun cuando una retirada pudiera parecer conveniente.

Los sonares no certifican el lateral ni la envolvente completa del cuerpo.
El chasis nominal de 170×170 mm implica radio circunscrito de unos 120 mm,
más margen; los salientes reales y la geometría provisional impiden tomar
esa cifra como una envolvente calibrada. La autorización inicial exige un
escenario de ensayo conocido y preparado para las maniobras automáticas,
con supervisión continua del operador. No es una medición del lateral ni
una prueba de seguridad de cada giro. No hay exploración irrestricta,
giros amplios desatendidos ni autorización basada en celdas desconocidas.

La autorización de navegación reside en STM32 y se concede al aceptar
AUTO_EXPLORE. No existe una licencia de 1500 ms, un permiso cada 300 ms ni
un timeout de confirmaciones de 1200 ms en ESP32. El navegador no participa
en su vigencia. No se agregan cuotas globales de tiempo, segmentos o evasiones.

La comunicación se vigila de forma independiente. Se reutilizan las consultas
EXPLORATION de 500 ms, con la identidad de sesión y el número de la última
respuesta completa realmente recibida. STM32 actualiza la evidencia de enlace
solo si la consulta es nueva, pertenece a la sesión activa, confirma su respuesta
anterior de menos de 1000 ms y fue recibida recientemente. La primera consulta
no prueba el retorno: el primer movimiento espera la confirmación siguiente.
Una respuesta perdida puede recuperarse con una ida y vuelta nueva; no se
consideran vigentes consultas repetidas o respuestas antiguas.

Hay un solo watchdog de enlace, en STM32: 3000 ms (seis períodos nominales de
consulta), incluido el plazo inicial para obtener comunicación bidireccional.
Si faltan las solicitudes, o si las respuestas dejan de llegar a ESP32 y por
lo tanto dejan de ser confirmadas, Navigation y MotorControl cancelan con
INTERNAL_LINK_LOST (16). Se invalidan consignas y se detienen ambos motores.
No se inventa una falla de hardware ni se rearma Safety. La FSM puede seguir
en AUTO detenido hasta STOP → IDLE. Un paquete tardío no recupera el watchdog
vencido, aunque aún no haya transcurrido el siguiente ciclo de Navigation.

SUPERVISION_LOST (15) queda reservado para interpretar registros históricos;
no lo genera la implementación actual de LOCAL_EXPLORE. El watchdog de enlace
no cambia la autorización de navegación ni puede iniciar una sesión.
La frescura de STATUS/HTTP sigue siendo diagnóstica y no renueva permisos.

Cerrar el navegador, bloquear el celular o perder Wi-Fi no cancela la sesión.
Tampoco autoriza sesiones nuevas. STOP conserva prioridad y revoca de inmediato
el contexto ESP32 al aceptarse; al llegar a STM32 cancela por las barreras
existentes. Una falla crítica o ausencia de piso mantiene sus paradas enclavadas.
La vigilancia de comunicación no detecta presencia humana: se exige supervisión
presencial y corte físico. STOP remoto puede estar indisponible sin Wi-Fi.

Tras reiniciar ESP32 se puede consultar el histórico, pero no se reconstruye
la identidad de control a partir de la telemetría. Las consultas de ese nuevo
proceso no prolongan la sesión anterior: STM32 la cancela al vencer su watchdog.
Para iniciar otra se exige STOP → IDLE y una solicitud explícita nueva.
No se promete protección frente a todo bloqueo de CPU, alimentación o hardware.

## Protocolo y web

Comandos UART nuevos, ASCII y acotados:

```text
AUTO_EXPLORE K=nonce
EXPLORATION Q=consulta S=0 K=nonce A=respuesta_anterior_recibida
EXPLORATION Q=consulta S=1
```

El inicio utiliza la cola FSM existente, incluyendo nonce, epoch y timestamp.
Las consultas son de solo lectura: A confirma recepción de una respuesta,
no concede permiso de movimiento ni reinicia el plazo de ninguna maniobra.
Un reinicio ESP32 consulta con K=0/A=0, sin autoridad para reconstruir una sesión.
La identidad solo se conserva desde el inicio explícito que envió ese proceso.
El comando antiguo EXPLORE K=... N=... P=... se rechaza siempre; nunca renueva
navegación ni el enlace. CHALLENGE conserva su significado diagnóstico sin autoridad.
S=0 congela una instantánea durante un máximo de 1000 ms; S=1 debe corresponder
a la misma consulta, sesión e inicio:

```text
EXPL,1,Q,SESSION,START,STATE,ACTION,CHALLENGE,BLOCK,SEGMENTS,TURNS,RETREATS,SCAN_MDEG,SELECTED_PLUS_ONE,CANDIDATES,REPEATED,LEASE_MS,SEQUENCE,FRONT_MM,REAR_MM,AVAILABLE,CRITICAL_FAULTS,SETTLE_STATE,SETTLE_OBSERVATIONS,SETTLE_MS,SESSION_NONCE,LINK_AGE_MS,LINK_ACK_QUERY,LINK_READY,FINISH_REASON,STOP_RECEIVED
EXPS,1,Q,SESSION,START,STATES_36,RANGES_HEX_108
```

Son 30 campos numéricos después de EXPL. Se mantienen los 25 anteriores;
LEASE_MS y SEQUENCE valen cero: son campos históricos de la autorización
eliminada. Los cinco finales informan edad de enlace, consulta confirmada,
primera ida y vuelta comprobada, causa y STOP retenido. El parser admite los
formatos anteriores de 24/25 campos para diagnóstico, sin reconstruir el enlace.
El nonce queda en el enlace interno, sin publicarse en JSON ni recibos HTTP.
En EXPS hay 36 caracteres 0–3 y
36 alcances de tres dígitos hexadecimales, en centímetros, con cero para
desconocido. La API devuelve mm cuantizados a 10 mm o `null`; esa cuantización
diagnóstica no participa en Safety ni Navigation. Ambos paquetes caben en
255 bytes y conservan el framing LF/CRLF existente.

Acciones: 0 ninguna, 1 avance, 2 barrido izquierdo, 3 alineación izquierda,
4 alineación derecha, 5 retirada. Selección: 0 ninguna; 1–36 representan
0°–350° en odometría. BLOCK: 0 ninguno, 1 evidencia insuficiente, 2 frontal
inhibido, 3 trasera inhibida, 4 evaluación de maniobra,
5 presupuesto angular, 6 alternativas agotadas, 7 repetición, 8 cambio sensorial pendiente de confirmar,
9 timeout de maniobra, 10 falta de progreso, 11 supervisión perdida histórica, 12 enlace interno perdido.
Los contadores de maniobra son intentos/inicios, no una prueba de movimiento:
NAV/NAVT conservan publicación, aplicación y progreso por encoders por separado.

- `POST /api/command`, cuerpo `AUTO_EXPLORE`: usa la validación Origin/Host,
  Content-Type y X-Request-ID existente, más X-Explore-Owner privado.
- `POST /api/exploration`, cuerpo heredado `WATCH secuencia permiso`: conserva
  las validaciones, pero responde 412 sin renovar ni restaurar una sesión.
  Se mantiene cerrado para que páginas antiguas no controlen la continuidad.
- `GET /api/exploration`: estados, sectores, candidatos, repetición, barrido,
  maniobras y estabilización; activa consultas diagnósticas de descubrimiento,
  nunca adopta una identidad ni inicia o reactiva movimiento. Se agregan
  authorization_mode="session", renewals_required=false, renewal_source="none",
  session_started, session_active, session_cancelled, stop_received,
  link_connected, link_confirmed, link_monitoring, link_age_ms, link_sequence, link_watchdog_ms y terminal_reason.
  link_age_ms es null sin el formato nuevo. Edad y conectividad acompañan
  siempre los valores históricos: una consulta antigua no es estado actual.
  link_connected indica respuestas UART recientes en ESP32; link_confirmed
  indica prueba bidireccional vigente en STM32 para la sesión monitorizada.
  El enlace puede volver a responder después de una cancelación sin que
  session_cancelled ni la causa histórica desaparezcan, ni se reanude el robot.
- `/api/navigation` reconoce LOCAL_EXPLORE, EXPLORAR, INTERNAL_LINK_LOST
  y el SUPERVISION_LOST histórico,
  manteniendo los campos anteriores y la retención después de STOP.
- `/exploration.js` y el panel nuevo son locales, responsive, sin dependencias.
  Conservan la confirmación inicial y un botón de detención que usa STOP
  por la ruta existente, accesible también al reconectar. No solicitan
  aprobaciones por maniobra ni modifican la cruceta.

ESP32 publica atómicamente los dos paquetes. Conserva la última instantánea
completa mientras recibe la siguiente, evitando huecos periódicos de sectores
o mezclar datos de consultas distintas. Su edad sigue creciendo; no se
rejuvenece por una parte inválida. STM32 aplica el único watchdog independiente cuando no hay nuevas
confirmaciones de su enlace interno. HTTP conserva el stack de 4096 bytes y
utiliza un buffer estático de 2048 bytes para este endpoint.

STOP/DETENER y movimiento manual conservan prioridad. No existe un servicio
de renovaciones. Las consultas de exploración se procesan desde app_main; sus consultas son de 500 ms con dos
partes, sin recuperación de retrasos. Se mantiene un lector UART, con espera
corta durante el seguimiento de la sesión, y siguen funcionando STATUS, SENSORS, IMU y mapa.

## Mapa y límites físicos pendientes

Se conservan POSE/RAY, mapa 80×80, trayectoria y asociaciones existentes.
La evidencia y las visitas usan esa misma odometría; no se fuerza la pose,
no se simulan obstáculos y no se usa la grilla incompleta de ESP32 como
autoridad de seguridad en STM32. Posición y orientación se consultan en
`/api/map`; los sectores usan su marco, con antigüedad propia. No se dibujan
marcas de navegación en una pose supuesta cuando no existe una asociación
temporal garantizada entre el evento y la consulta web del mapa.

No quedan resueltos por software: espacio lateral no medido, falta de ecos
del HC-SR04, objetos finos o acústicamente difíciles, deriva/deslizamiento,
precisión de la geometría provisional, frenado real, cambios rápidos del
entorno y ciclos mayores que el historial local. Estas limitaciones impiden
presentar el perfil como exploración irrestricta o navegación segura certificada.

## Ensayos posteriores, solamente después de revisión

1. Revisar el diff y compilar/grabar por el procedimiento habitual, con
   placas retiradas de la PCB y alimentación desconectada. Esta entrega
   no realizó esa grabación. Reinstalar, alimentar con batería y acceder por Wi-Fi.
2. Preparar un sector académico conocido, piso continuo, espacio alrededor
   del cuerpo y ecos dentro del alcance útil observado. No hace falta
   instrumentación angular adicional; no se exige precisión milimétrica.
3. Recargar la interfaz después de actualizar ambas placas. Desde IDLE seguro
   confirmar una sola vez el escenario y seleccionar LOCAL_EXPLORE. El robot
   podrá moverse automáticamente cuando haya supervisión y datos válidos:
   verificar estabilización, avance prioritario con frontal despejado y decisiones sin pulsar
   autorizaciones intermedias. Mantener el área de ensayo controlada.
4. Con varios rumbos válidamente medidos, verificar selección, alineación
   y segmentos cortos, manteniendo STOP accesible. Comparar contadores con
   publicación/aplicación y desplazamiento real; revisar sectores y mapa.
5. Ensayar un obstáculo persistente y una retirada autorizada; después,
   otras alternativas supervisadas. Al repetir poses o agotar alternativas
   debe esperar detenido sin inventar una falla. Un cambio sostenido del
   entorno puede reabrir la evaluación; no un eco aislado fluctuante.
6. Con el corte físico accesible y espacio controlado, bloquear el celular o
   cerrar la página: la sesión debe continuar. Reconectar y comprobar la misma
   sesión sin iniciar otra; solicitar STOP y verificar detención, retención
   histórica y ausencia de reinicio automático. No desconectar cables internos
   con batería ni inducir caídas; el watchdog UART se verifica aquí por mocks.

Los casos críticos conservan Safety y rearme explícito. No se autorizan
excepciones bajo 200 mm ni se realizan ensayos anulando sensores o protecciones.

## Verificación de software y recursos

| Grupo | Resultado |
| --- | --- |
| Safety/control/motores/navegación operativa | 694 pruebas aprobadas |
| AUTO-1 | 363 aprobadas |
| Variantes experimentales izquierda/derecha | 720 + 720 aprobadas |
| B7 y giros históricos izquierda/derecha | 436 + 55 + 55 aprobadas |
| Odometría / adquisición / IMU / ultrasónicos | 28 / 8 / 48 / 19 aprobadas |
| ESP32 nativo, incluidos siete cruces con tramas generadas por STM32 | 224 grupos aprobados |
| JavaScript real en V8, DOM/fetch/reloj sustituidos | 176 grupos aprobados |

Se verificaron barrido con frontal/trasero, valles estrechos y anchos,
selección y alineación en ambos sentidos, referencias independientes,
giro acumulado, retiradas adicionales tras búsquedas fallidas y detección de
la repetición resultante. También espera sin rearme, cambio de ambiente
confirmado frente a fluctuación, más de doce segmentos sin los topes de
AUTO-4, vencimiento por movimiento y pérdida de enlace, STOP/caída/fallas en fases
distintas, rearme sin reactivación, secuencias antiguas, wrap, paquetes
fragmentados o inválidos y conservación de una instantánea completa entre
consultas. La concurrencia incluye 1000 publicaciones con 10000 lecturas
JSON. No se eliminaron las regresiones anteriores.

Las compilaciones STM32 y ESP32 enlazaron correctamente, sin advertencias
detectadas en los logs finales. Se conservan framework y particiones:

| Recurso | Resultado |
| --- | --- |
| STM32 text / data / bss | 119672 / 948 / 38432 bytes |
| STM32 data+bss | 39380 bytes; no equivale a heap/stack libre medido |
| ESP32 Flash de aplicación | 968161 / 1048576 bytes (92,3%) |
| ESP32 RAM estática | 80368 / 327680 bytes (24,5%) |
| Cadena HTTP de navegación analizada | 3088 / 4096 bytes |
| Cadena HTTP de exploración analizada | 2048 / 4096 bytes |

Los análisis de stack son cotas estáticas parciales del ELF real: no miden
el high-water mark del SDK ni certifican todas las rutas de ejecución.
No se amplió el stack HTTP. Se conservan informes `-fstack-usage` en el
build STM32; la duración de las secciones críticas, heap y stack en ejecución
continúan pendientes de medición física.

Comandos y evidencias reproducibles:

```powershell
# Raíz del repositorio:
cmd /c Firmware\stm32_control\Tests\safety_b11\run.bat
powershell -ExecutionPolicy Bypass -File Firmware/stm32_control/Tests/odometry_w7/run.ps1
powershell -ExecutionPolicy Bypass -File Firmware/stm32_control/Tests/sensors_w5/run.ps1
powershell -ExecutionPolicy Bypass -File Firmware/stm32_control/Tests/imu_1/run.ps1
cmd /c Firmware\stm32_control\Tests\ultrasound\run.bat
powershell -ExecutionPolicy Bypass -File Firmware/esp32_gateway/Tests/w2/run.ps1
pio run -d Firmware/esp32_gateway -e esp32
git diff --check
# En Firmware/stm32_control:
mingw32-make -B -o build -j4 "CC=arm-none-eabi-gcc -fstack-usage"
```

El runner JavaScript `Tests/w2/run_js.js` funciona con Node o con las mismas
fuentes cargadas en V8, sin instalar dependencias. Esta verificación utilizó
V8. Las pruebas nativas cruzadas leen `GAR_AUTO3_BASE_0/1`,
`GAR_AUTO3_TURN_0/1`, `GAR_AUTO4_BASE_0/1/2`, `GAR_AUTO4_DETAIL_0/1/2`,
`GAR_LOCAL_BASE`, `GAR_LOCAL_SECTORS`, `GAR_LOCAL_LINK_BASE` y `GAR_LOCAL_LINK_DETAIL`, extraídos de las líneas `*_WIRE`
del harness STM32. Sin esas variables se omiten siete grupos de interoperabilidad.

Los logs finales, resultados JavaScript, manifiesto y diff de esta intervención
quedan en `Firmware/stm32_control/build/local_explore_review/`, ignorado por
Git. Son pruebas con mocks, no evidencia de escritura UART física ni de
movimientos reales. Esta entrega termina antes de grabar o ensayar hardware.

## Archivos y responsabilidades

STM32:

- `Core/Inc/mid_local_exploration.h` y `Core/Src/mid_local_exploration.c`:
  constantes, evidencia sectorial, inflación, candidatos y visitas.
- `Core/Inc/mid_navigation.h` y `Core/Src/mid_navigation.c`: perfil, estado,
  autorización con vencimiento, selección y ejecución por las fases existentes,
  objetivos angulares, diagnóstico y paquetes EXPL/EXPS.
- `Core/Inc/task_control_robot.h` y `Core/Src/task_control_robot.c`: solicitud
  explícita con nonce transportado por la cola FSM existente.
- `Core/Src/task_comms.c`: gramática cerrada de inicio y consultas diagnósticas con confirmación; rechazo de renovaciones antiguas.
- `Core/Src/mid_motion.c` y `Core/Src/task_motor_control.c`: vector de giro
  elegido por la fase vigente, con las barreras originales intactas.
- `Makefile`: registro del módulo puro nuevo.
- `Tests/safety_b11/test_local_exploration.c` y `test_safety.c`: nuevas
  pruebas sobre los módulos reales y ejecución junto a los perfiles existentes.

ESP32:

- `include/stm32_exploration.h` y `src/stm32_exploration.c`: parser, staging
  coherente, estado compartido y correlación diagnóstica sin permisos periódicos.
- `include/uart_comm.h` y `src/uart_comm.c`: comando, arbitraje y único receptor.
- `include/web_commands.h` y `src/web_commands.c`: comando cerrado y capacidad
  privada, sin exponerla en los recibos públicos.
- `src/main.c`: servicio desde el bucle existente, sin nueva tarea.
- `src/web_server.c`: dos rutas del endpoint y recurso JS local, conservando LRU.
- `src/stm32_navigation.c`: compatibilidad de perfil, estado, motivo y giro parcial.
- `src/CMakeLists.txt` y `platformio.ini`: fuentes y archivos embebidos nuevos;
  no cambian framework, dependencias ni particiones.
- `web/index.html`, `web/app.js` y `web/exploration.js`: panel explícito,
  supervisión, confirmación inicial de sesión y diagnóstico legible de sectores.
- `Tests/w2/test_exploration.c/.js`, `test_status.c`, `test_commands.js`,
  `test_navigation.c`, `run.ps1`, `run_js.js` y `check_navigation_stack.py`:
  protocolo, HTTP, concurrencia, navegador y stack. Las expectativas de valores
  desconocidos se desplazan al siguiente código libre, manteniendo el rechazo.

Todos los cambios previos permanecen sin commit; el diff incremental de esta
etapa se distingue del diff completo acumulado desde HEAD.

## Sesión autónoma y conservación del avance prioritario

Se elimina el permiso periódico, no las barreras de seguridad. La última
prueba física informó cuatro segmentos, sin giros innecesarios ni errores,
con finalización SUPERVISION_LOST a los 5480 ms. Sin una traza UART completa
no se atribuye ese instante a una falla física específica; la dependencia de
renovaciones y sus timeouts redundantes ya no existe en este diseño.

La admisión frontal permanece intacta: dos capturas distintas, secuencias
y timestamps nuevos separados por no más de 200 ms, última captura de no
más de 200 ms, rumbo a menos de 5 grados del frontal y variación de alcance
no mayor a 25 mm. Ambos alcances deben ser al menos 350 mm: 275 mm preventivos
+ 50 mm de segmento + 25 mm de reserva. Siguen vigentes Safety, odometría,
estabilización reutilizable solo con ruedas estables, ciclos y retiradas
sucesivas. No se borra evidencia ocupada ni se finge un barrido lateral.

Los ensayos sobre el bucle real de MotorControl mantienen 24 segundos de
avance con comunicación diagnóstica cada 500 ms y sonares alternados 140/70 ms,
sin una sola renovación de navegación. STOP, caída y pérdida de enlace se
prueban durante avance, giro y retirada, verificando objetivos/PWM cero y
rechazo de consignas viejas. Se comprueban también prioridad simultánea de
STOP/caída y timeout, recepciones duplicadas, eco tardío, reinicio ESP32,
wrap de tick/consulta, bloqueo por alternativas agotadas y avance prioritario.

El cruce del formateador real STM32 con el parser ESP32 detectó un límite
obsoleto de 1000 ms para el tiempo de estabilización retenido al terminar.
Se permite el uint32 histórico cuando la navegación ya está detenida,
sin cambiar la ventana real ni su timeout de 200 ms. El resultado cruza
ambos firmwares y HTTP conservando INTERNAL_LINK_LOST y STOP recibido.

No se crean tareas, colas ni asignaciones dinámicas. El enlace se verifica
desde TaskComms, Navigation/MotorControl y el bucle ESP32 existentes. No hay
watchdog redundante de autorización en ESP32 ni control desde handlers HTTP.
Las pruebas con mocks no demuestran prestaciones temporales físicas del UART,
ni el espacio barrido del cuerpo, ni el margen real de heap/stack en ejecución.

Evidencias, manifiesto y diff incremental de esta revisión:
`Firmware/stm32_control/build/local_session_review/` (ignorado por Git).
No se grabaron placas ni se realizaron pruebas físicas. Después de revisión,
actualizar ambos firmwares, comprobar frontal despejado y avance, cerrar o
bloquear el celular, reconectar sin iniciar otra sesión y solicitar STOP.
No desenchufar cableado interno con batería: la pérdida UART se prueba aquí
por software. Mantener supervisión presencial, zona preparada y corte físico.

## Corrección de retiradas sucesivas

El historial general conserva el radio de 50 mm y la tolerancia angular de
20 grados. `Local_Visited()` y `Local_Visit()` siguen impidiendo repetir los
mismos rumbos de avance. Ese criterio no distingue por sí solo una retirada
útil de 20 mm de un ciclo; anteriormente se aplicaba tanto al elegir una
retirada como al registrar su inicio, bloqueando la siguiente posición
antes de poder reevaluarla.

Se marcan ahora los orígenes de retirada en el mismo historial acotado de
32 poses. Para planificar otra retirada, `Local_RetreatAllowed()` exige que
el desplazamiento desde cada origen de retirada situado a menos de 50 mm
tenga al menos 10 mm de componente hacia atrás según el rumbo actual.
`Local_RetreatVisit()` registra el origen al autorizar la fase. Los 10 mm son
un criterio de progreso neto entre orígenes (la mitad de la maniobra de
20 mm), no una reducción del radio general ni un nuevo objetivo de recorrido.
El progreso procede de la pose odométrica; nunca de consignas publicadas.

Esto permite retirar el robot varias veces en la misma dirección aunque
los orígenes sucesivos estén dentro del radio general. No permite repetir
una retirada en el mismo origen, confundir movimiento lateral con progreso
hacia atrás ni girar para regresar hacia un origen cercano ya recorrido.
Un barrido de medio giro tras una retirada puede quedar bloqueado antes
de autorizar un retroceso que desharía el desplazamiento anterior. Las
visitas generales no vetan la primera retirada desde una posición de avance
ya registrada, pero siguen penalizando los candidatos de avance.

Cada retirada conserva objetivo de 20 mm, consigna de -1600 cps, timeout
800 ms, vigilancia de progreso, estabilización y referencia nuevas, ecos
recientes y trasera estrictamente mayor que 250 mm. STOP, Safety y el
watchdog independiente del enlace permanecen activos. Los intentos fallidos siguen
inhibidos en su pose; no se reintentan simplemente por recibir otra consulta diagnóstica.
Después de cada retirada completada se renueva la evidencia y se reabre
el presupuesto de barrido existente para evaluar orientaciones alternativas.

La prueba nueva de cuatro retiradas consecutivas reprodujo el defecto con
el código anterior. Tras la corrección pasa con más de 79 mm de
retroceso odométrico sintético y permite un nuevo giro con evidencia
actualizada. Se verifican también los límites traseros de 249/250/251 mm,
STOP, caída, datos traseros inválidos o antiguos durante una retirada
posterior, falta de progreso, retorno cíclico y memoria acotada durante
200 retiradas simuladas. El antiguo ensayo de dos barridos de medio giro
se actualiza para exigir rechazo de la retirada que volvería hacia su
origen; se conserva la prueba de espera y recuperación por cambio ambiental.

La corrección aumenta la RAM estática STM32 en 128 bytes para identificar
las visitas de retirada. No cambia ESP32 ni el contrato de telemetría.
Las evidencias y el diff incremental de este ajuste quedan en
`Firmware/stm32_control/build/local_retreat_review/`, ignorado por Git.
El historial de 32 poses es local y finito: no garantiza reconocer todos
los circuitos posibles en recorridos arbitrariamente largos. El criterio
direccional puede rechazar retiradas cercanas tras cambios de rumbo; ese
rechazo conservador exige nueva evaluación, no una excepción de seguridad.

Validación física pendiente: en el escenario ya preparado, con espacio
trasero y lateral inspeccionado, iniciar una única sesión supervisada y
comprobar tres o más retiradas breves cuando el frontal siga inhibido.
Observar conteos, pose y número de retiradas; comprobar una detención por
STOP. No inducir caídas ni usar las lecturas ultrasónicas como certificado
de espacio lateral libre. No se efectuó ese ensayo en esta revisión.

## Evidencia final de recursos de sesión y enlace

STM32: text/data/bss = 119752/948/38432 bytes; data+bss = 39380 bytes.
ESP32: Flash reportada 968161/1048576 bytes (92,3%), RAM estática
80368/327680 bytes (24,5%). El binario de aplicación mide 968560 bytes,
con 80016 bytes de margen hasta 1 MiB (ver manifiesto para el valor exacto).
No se debe interpretar la diferencia de RAM como heap libre efectivo.
La cadena HTTP analizada de exploración suma 2048/4096 bytes y la de
navegación 3088/4096; son cotas estáticas parciales del ELF, sin medición
de high-water mark ni garantía de todas las rutas del SDK.

## Asimetría odométrica tolerada durante giros LOCAL

La guarda compartida por Navigation y la autorización de MotorControl
admite hasta 4 mm de diferencia acumulada entre ruedas solo durante un
giro LOCAL y después de que ambas hayan progresado al menos 3 mm en el
sentido esperado. El arranque conserva el límite de 3 mm; los giros de
AUTO-3/AUTO-4 también conservan 3 mm, y avance/retroceso mantienen 10 mm.

Los 4 mm corresponden a un presupuesto experimental de 2 mm de recorrido
longitudinal neto del centro en el modelo ideal. No certifican adherencia,
encoders sanos ni espacio lateral. No se cambian referencias, objetivo,
timeout, consignas ni estados para recuperarse de un rechazo. Una diferencia
mayor, datos inválidos o dirección incorrecta siguen deteniendo la sesión.
Ver `LOCAL_EXPLORE_DIAGNOSTICO_GIRO.md` para cálculos, pruebas y limitaciones.
