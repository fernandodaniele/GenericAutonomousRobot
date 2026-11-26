# AUTO-2F: retroceso, distancias y retencion de estabilizacion

Rama G1-add/InterfazWeb, HEAD b0967cd. Se conservan AUTO-2/D/D.1/E pendientes.
Etapa de implementacion/pruebas por software; no se graba ni mueve el robot.

## Valores implementados

| Parametro | AUTO-2E | AUTO-2F operativo |
|---|---:|---:|
| Objetivo de retroceso | 20 mm | 20 mm |
| Consigna de retroceso | -1600 cps/rueda | -1600 cps/rueda |
| Timeout propio de retroceso | 300 ms | **800 ms** |
| Falta de progreso | 1 mm nuevo/200 ms | sin cambios |
| Avance | 100 mm / +1600 cps | sin cambios |
| Timeout de avance desde Begin | 2000 ms | sin cambios |
| Estabilizacion | 4 muestras nuevas, 60 ms, rango <=1 cuenta, maximo 200 ms | sin cambios |
| Timeout global AUTO-2 | 3000 ms | **3000 ms, sin renovacion** |
| Preventivo frontal AUTO-2 | 300 mm | **275 mm** |
| Guardia trasera AUTO-2, admision y mantenimiento | >300 mm | **>250 mm** |
| Safety frontal/trasero, todos los modos y rearme | >200 mm para permitir | **sin cambios** |
| Giro experimental | deshabilitado | sin cambios |

Los cambios de parametros estan centralizados en mid_navigation.h y se aplican
solo con NAV_AUTO2_ENABLED=1. AUTO-1 y B7 mantienen 300 mm, >300 mm y los plazos
anteriores; sus regresiones se siguen compilando por separado. No hay seleccion
arbitraria de distancias/velocidades por UART o HTTP.

La detencion ocurre al alcanzar primero 20 mm estimados o un limite: 800 ms,
falta de progreso, global, STOP, Safety, invalidez sensorial/odometrica u otra
cancelacion. No se prolonga el movimiento hasta 800 ms si ya alcanzo 20 mm.
La espera sigue excluida del reloj propio de retroceso y de la vigilancia de
progreso, pero incluida en el global. La referencia sigue siendo posterior
a estabilizar; no se relaja el signo de los encoders.

## Justificacion del plazo

El usuario reporto 11,759 mm en una fase limitada a 300 ms, sin error odometrico.
La media incluyendo transitorios es aproximadamente 39,2 mm/s. A esa media,
20 mm requeririan unos 510 ms. No es una velocidad calibrada ni una prediccion
garantizada, pero justifica ensayar 800 ms conservando el limite de distancia.

El presupuesto nominal 2000+200+800=3000 ms cabe en el global actual. La espera
debe confirmar antes de los 200 ms y el avance no puede seguir hasta 2000 ms
sin detenerse. No hace falta aumentar el global. El jitter puede hacer que el
global termine una fase antes; no se compensa ampliandolo ni renovandolo. El
giro permanece deshabilitado y no se le reserva tiempo operativo adicional.
El timeout permanente de consignas del backend sigue siendo 500 ms y requiere
renovaciones validas de navegacion; el nuevo limite no permite eludirlo.

## Evaluacion de 230 / 170 / >230 mm

Se descarta por ahora la combinacion propuesta y se usan alternativas mas
conservadoras. Motivos basados en el codigo y los datos disponibles:

- Diametros vigentes 25,4 mm; CPR 1435,2/1421,6; 1600 cps equivalen nominalmente
  a unos 89,4 mm/s de media. La media observada de retroceso no es una cota
  superior para avance ni para todas las condiciones de bateria/carga.
- HC-SR04 alternados: separacion minima 70 ms, periodo nominal 140 ms por sensor,
  timeout 50 ms. Safety conserva edad maxima 200 ms por captura y 100 ms para
  el snapshot. Una publicacion nueva no rejuvenece los dos ecos.
- A velocidad nominal se recorren ~12,5 mm en 140 ms. Un presupuesto orientativo
  de 200 ms de edad aceptada +20 de navegacion +10 de motor supone ~20,6 mm,
  ANTES de sumar frenado, jitter, sobrevelocidad o error de eco. No es una
  latencia medida ni una cota certificada.
- Con frontal 230 y Safety 200 quedarían solo 30 mm (unos 9,4 mm tras ese
  presupuesto nominal). No hay una medida de frenado que justifique ese resto.
  Bajar Safety a 170 recuperaria un salto entre umbrales de 60 mm, pero tambien
  reduciria la proteccion absoluta de MANUAL, AUTO, IDLE y del rearme.
- En odometry_config.h, chasis de 170 mm y centros acusticos nominales +/-85 mm
  suponen sensores en el borde. ODOM_SENSOR_GEOMETRY_CONFIRMED=0: no demuestran
  la distancia real al extremo de PCB/chasis ni el volumen completo del robot.
  La detencion rapida observada tampoco cuantifica un margen absoluto de 170 mm.

**Safety se mantiene a 200 mm.** Se recomienda revisarlo independientemente
solo despues de conocer mejor frenado y montaje. No se cambian TaskSafety,
fallas enclavadas, politica de rearme, ni requisitos de datos validos/frescos.

275 mm frontales conservan un salto preventivo de 75 mm sobre Safety, frente
a 30 mm si se bajara directamente a 230 sin tocar Safety. Tras el presupuesto
nominal quedarían ~54,4 mm para incertidumbres no medidas. Sigue siendo una
configuracion experimental, no una distancia de frenado validada.

En retroceso, >250 mm deja 50 mm sobre Safety (unos 29,4 mm tras el presupuesto
anterior). Es mas prudente que >230, cuyo resto nominal seria ~9,4 mm. Ademas,
el recorrido esta limitado a 20 mm. La guardia se comprueba continuamente:
desde 251 mm puede cancelar tras ~1 mm por caer a 250. Para completar 20 mm
se requiere distancia inicial >270 mm mas incertidumbre y sobrepaso; el primer
ensayo debe tener bastante mas espacio. No basta restar 20 mm al valor inicial:
un obstaculo puede acercarse y una medicion puede estar retrasada.

IR conserva su significado de falta de piso, no sustituye la distancia a objetos.
No obtener eco NO significa espacio libre: mantiene la parada enclavada conocida.

### Dependencias Safety comprobadas

task_safety.c define TASK_SAFETY_OBSTACLE_DISTANCE_M=0.20f.
TaskSafety_SnapshotFaults evalua ambos sensores sin discriminar modo ni sentido.
TaskSafety_Run, TaskSafety_EvaluateSnapshot y TaskSafety_RequestReset reutilizan
esta evaluacion. Rearmar requiere ambos sensores seguros, validos, recientes,
capturas posteriores a la falla y ausencia de nuevas fallas/emergencia.
No existe un umbral separado de rearme que deba bajarse para AUTO-2F.

## Defecto de stabilization.available y correccion

Se reprodujo ANTES de corregir con el parser/endpoint reales del arnes:

1. NAV valida de una sesion terminal.
2. NAVW valida: stabilization.available=true.
3. Otra NAV de la MISMA sesion terminal: el inicializador local dejaba vacia
   la estructura anidada y la asignacion global borraba los detalles anteriores.
4. HTTP consultado antes de la proxima NAVW: available=false aunque existia
   diagnostico valido y el retroceso se habia ejecutado.

La ventana entre ambas consultas se repite cada periodo; un cliente HTTP
alineado con ella podia observar false repetidamente. Esto demuestra un
defecto del cache compatible con el ensayo; sin su traza UART no demuestra
que cada false fisico se debiera exclusivamente a ese mecanismo.

Se corrige SOLO Navigation_Accept en stm32_navigation.c. Al actualizar NAV,
conserva NAVW previamente recibida de la misma sesion, tick inicial y perfil,
unicamente cuando sus datos ya son inmutables:

- Estabilizacion confirmada CON referencia completa, mientras la base sigue
  en retroceso/giro o termina desde esas fases con autorizacion registrada.
- O un registro terminal que conserva causa, ultima fase y tiempos terminales.

No se heredan ventanas activas/incompletas ni otra sesion; tampoco una referencia
ante un retroceso del tiempo transcurrido o una reactivacion aparente de un
registro terminal. La antiguedad propia
de NAVW sigue creciendo: una NAV nueva no cambia su received_us ni source_age.
Si nunca llego NAVW, available sigue false; no se inventa una confirmacion a
partir de reverse.applied. Una NAVW nueva correlacionada puede actualizar los
detalles normalmente. Los datos terminales siguen etiquetados historicos.

No cambian captura/control STM32, formato UART, frecuencia, prioridades,
consultas S=1, estructura JSON ni su longitud maxima. Se mantiene el unico
lector y la correccion de stack HTTP AUTO-2D.1, con buffer estatico de 1792 bytes.

## Pruebas y compilaciones

Resultados locales con mocks/datos sinteticos; no son movimientos fisicos:

| Conjunto | Resultado |
|---|---:|
| AUTO-2F operativo, Safety/control/navegacion | 434/434 |
| AUTO-1 | 363/363 |
| AUTO-2 experimental izquierdo/derecho, solo tests | 460/460 cada uno |
| B7 sin giro | 436/436 |
| B7 giro izquierdo/derecho | 55/55 cada uno |
| Odometria / adquisicion / IMU / ultrasonicos | 28/28, 8/8, 48/48, 19/19 |
| Nativas ESP32 | 176/176 grupos |
| JavaScript | 160/160 grupos |

Los perfiles comparten casos; no sumar como pruebas independientes. No se
deshabilitan regresiones: se ajustan solo limites/fixtures AUTO-2 a 275/250/800.
El fixture de pruebas B7.2 de APIs sin movimiento coloca explicitamente el
obstaculo en el umbral de su perfil, pues 300 mm ahora es camino despejado
para AUTO-2; conserva sus asserts de cero PWM/PI y rechazo de APIs manuales.

Tests nuevos: bucle real con retroceso lento que completa 20 mm despues de
300 y antes de 800; movimiento aun mas lento que vence exactamente en 800;
cero progreso que detiene antes; obstaculo tardio y compatibilidad global;
STOP/Safety despues de 500 ms; critico 170/200/201 en ambos sensores y rearme;
datos invalidos/antiguos/ausentes. Se mantienen estabilizacion y signo estricto.
ESP32 comprueba retencion, edad, cambio de sesion, datos ausentes e integracion
del app_main real con respuestas UART de 100 ms y consultas consecutivas.

Compilaciones, sin errores ni advertencias del compilador:

```text
cd Firmware/stm32_control
mingw32-make -B -o build -j4 "CC=arm-none-eabi-gcc -fstack-usage"

pio run -d Firmware/esp32_gateway -e esp32
```

STM32: text=103640, data=940, bss=36616 bytes; RAM estatica 37556, sin incremento.
ELF: Firmware/stm32_control/build/stm32_control.elf.
ESP32: Flash 938637/1048576 bytes, RAM estatica 74624/327680; +336 de Flash y
sin incremento de RAM frente a AUTO-2E. ELF/BIN en .pio/build/esp32 de ESP32.
El test de frames del ELF HTTP pasa: 2752 bytes de cadena analizada frente a
4096 configurados; no es una medicion del high-water mark real en hardware.

Archivos funcionales: mid_navigation.h (parametros por perfil) y
stm32_navigation.c (retencion de metadatos). Se amplian tests y documentacion.
Los demas modulos de produccion y la modificacion IDE preexistente se conservan.
Logs y diff exclusivo AUTO-2F en build/auto2f y .pio/, ignorados por Git.

## Validacion fisica posterior, sujeta a aprobacion

1. Revisar la entrega antes de grabar. Actualizar STM32 y ESP32 con los nuevos
   artefactos mediante el procedimiento habitual fuera de la PCB. Reinstalar
   y usar bateria/Wi-Fi; no hace falta depuracion USB/UART externa en el ensayo.
2. Inmovil, comprobar STATUS/SENSORS y /api/navigation. Tras existir un diagnostico
   y llegar NAVW, varias consultas terminales deben conservar available=true.
   Antes de la primera NAVW/sesion, false con campos null es correcto.
3. Superficie plana, IR y ambos ecos validos, trasera inicialmente holgada
   (p.ej. ~400-500 mm). Frontal aproximadamente 320-330 mm para que la prevencion
   pueda aparecer antes del objetivo de avance de 100 mm. STOP accesible.
4. Solicitar UNA AUTO. Verificar avance, detencion/espera y retroceso acotado.
   Guardar JSON: stabilization.state="confirmed", reverse.completed, reverse.progress_mm y
   phase_elapsed_ms. Puede finalizar por otro limite antes de 20 mm; no ignorarlo.
   completed=true con EVASION_NOT_AUTHORIZED indica separacion completada y
   giro bloqueado, no habilitacion de giro.
5. STOP -> IDLE -> consultar otra vez: causa y referencia se conservan. Si la
   estabilizacion sigue sin estar disponible despues de varias consultas,
   conservar el JSON antes de apagar: la ausencia de toda NAVW no se oculta.
6. Solo despues de confirmar la separacion, comprobar preventivo 275 y rechazo
   trasero cerca de 250, inicialmente inmovil, sin buscar deliberadamente
   contacto ni acercarse al limite critico. No exigir precision milimetrica.

Los margenes son experimentales. La odometria estima recorrido de ruedas,
no garantiza desplazamiento real ni ausencia de deslizamiento. No hay cobertura
lateral, giro automatico ni nueva politica de recuperacion. El global no es
watchdog de Wi-Fi y la CPU/scheduler detenidos no garantizan parada por software.
