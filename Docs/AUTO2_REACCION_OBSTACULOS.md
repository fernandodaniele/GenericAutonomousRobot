# AUTO-2: reaccion preventiva y separacion acotada

AUTO-2F vigente: retroceso 20 mm/800 ms, preventivo 275 mm, trasera >250 mm;
Safety conserva 200 mm y el global 3000 ms. Correccion de retencion NAVW y
justificacion: [AUTO2F_REFINAMIENTO.md](AUTO2F_REFINAMIENTO.md).
Los apartados AUTO-2E/D siguientes se conservan como historial.

AUTO-2E incorpora DETENER_Y_ESPERAR antes del retroceso, con referencia nueva
posterior a confirmar estabilidad. Los umbrales y plazos anteriores se conservan.
Ver [AUTO2E_TRANSICION_RETROCESO.md](AUTO2E_TRANSICION_RETROCESO.md).
El texto siguiente conserva el contexto de AUTO-2/AUTO-2D anteriores.

AUTO-2D agrega exclusivamente diagnostico retenido y `GET /api/navigation`.
Contrato, interpretacion, pruebas y propuestas NO implementadas:
[AUTO2D_DIAGNOSTICO_NAVEGACION.md](AUTO2D_DIAGNOSTICO_NAVEGACION.md).
La secuencia y los parametros siguientes permanecen sin cambios.

Implementacion sobre `b0967cd`, rama `G1-add/InterfazWeb`. Esta etapa integra
AUTO-2A/B/C por software. No graba placas ni valida desplazamientos fisicos.
La configuracion operativa permite avance y retroceso limitado. **El giro
permanece deshabilitado por defecto**: los HC-SR04 no verifican el lateral.

## Arquitectura y perfiles

Se amplia la misma maquina interna de `mid_navigation`, ejecutada por
`TaskControlRobot` cada 20 ms. No hay nuevas tareas, colas ni otra FSM principal.
Se reutilizan `Navigation_Begin/Step/Cancel`, `Odometry_GetSnapshot`,
`TaskSafety_EvaluateSnapshot`, `Motion_NavigationAdvance/Reverse/Turn` y las
barreras del controlador de motores (identidad de tarea, origen, generacion,
sesion/fase, timestamp y permiso de emision de un uso).

En `mid_navigation.h`:

| Compilacion | Seleccion |
|---|---|
| Operativa AUTO-2 | `NAV_AUTO1_ODOMETRY=1`, `NAV_AUTO2_ENABLED=1`, `NAV_EXPERIMENTAL_TURN_ENABLED=0` (valores predeterminados). |
| AUTO-1 conservada | `NAV_AUTO2_ENABLED=0`; conserva 100 mm, +1600 cps, 2000 ms, sin retroceso/giro ni nueva guarda de falta de progreso. |
| B7 anterior, solo regresion | `NAV_AUTO1_ODOMETRY=0` (implica AUTO-2 deshabilitada). |
| AUTO-2 con giro experimental, solo pruebas autorizadas | AUTO-2 habilitada y `NAV_EXPERIMENTAL_TURN_ENABLED=1`. `NAV_TURN_LEFT=1` por defecto; 0 selecciona derecha al compilar. |

No hay seleccion de perfil, direccion o velocidades arbitrarias por UART/web.
AUTO sigue siendo la orden existente. El texto de la web describe el firmware
operativo AUTO-2; no es una deteccion remota del perfil compilado. Si se prepara
un ELF alternativo AUTO-1, identificarlo y no confundirlo con el operativo.

Ejemplo de compilacion alternativa desde `Firmware/stm32_control`, sin cambiar
la configuracion versionada (utilizar otro directorio de salida para no
reemplazar el ELF operativo):

```powershell
New-Item -ItemType Directory -Force build/auto1_regression | Out-Null
mingw32-make -B -o build/auto1_regression -j4 BUILD_DIR=build/auto1_regression 'C_DEFS=-DUSE_HAL_DRIVER -DSTM32F407xx -DNAV_AUTO2_ENABLED=0'
```

El comando normal `mingw32-make -B -o build -j4` genera
`build/stm32_control.elf` con AUTO-2 y giro deshabilitado. No usar macros de los
ensayos nativos como autorizacion para una prueba fisica con giro.

## Secuencia finita

| Estado | Accion / siguiente estado |
|---|---|
| `NAV_ESPERAR_DATOS` | Salidas cero. Espera sensores seguros y referencia odometrica posterior a AUTO. Un obstaculo frontal ya presente finaliza detenido, sin retroceso automatico. |
| `NAV_AVANZAR` | +1600/+1600 cps. Finaliza a 100 mm. Ante frontal <=300 mm, primero revoca el avance y escribe cero; solo pasa a retroceso con sensores/odometria seguros y trasera >300 mm. |
| `NAV_RETROCEDER_ACOTADO` | Tras una nueva referencia, -1600/-1600 cps. Objetivo 20 mm, actuacion maxima 300 ms. Reevalua sensores, progreso y permisos. |
| `NAV_GIRAR_ACOTADO` | Solo tras completar realmente el retroceso, y con habilitacion experimental explicita. Izquierda -1200/+1200 cps, objetivo 10 grados, actuacion maxima 250 ms. |
| `NAV_DETENIDO` | Terminal: objetivos/integrales/PWM cero; no renueva ni reanuda. Nueva maniobra requiere STOP -> IDLE confirmado -> AUTO. |

No se vuelve de retroceso/giro a avance, no se alterna el sentido, no hay
segundos intentos ni exploracion continua. La FSM principal conserva AUTO al
finalizar normalmente. Safety puede llevarla a SAFETY_STOP; su politica de
enclavamiento y rearme permanece intacta. RESET no restaura la navegacion.

En cada cambio de fase se utiliza `TaskMotorControl_Stop()`: invalida la
generacion y la sesion anteriores, revoca permisos, limpia PI y escribe cero.
La fase siguiente no emite en ese ciclo: necesita una muestra odometrica
estrictamente posterior a la parada y otra evaluacion. No se reutiliza una
consigna de avance para retroceso ni una de retroceso para giro.

El backend tambien comprueba el progreso y los plazos antes de aplicar una
orden pendiente. Si alcanza el objetivo de retroceso/giro, retira el permiso
y limpia las salidas; solo la tarea de navegacion decide la siguiente fase.
La finalizacion ya detectada puede evaluarse en el siguiente ciclo de 20 ms,
con motores detenidos, sin prolongar el tiempo de actuacion de esa fase.
STOP, Safety y el plazo global conservan prioridad durante esa evaluacion.

Se ampliaron exclusivamente tres guardas de compilacion en MotorControl para
reutilizar `Navigation_CommandApplied/CompleteCommand` tambien en AUTO-2. No se
cambiaron su PI, PWM, encoders, periodos, serializacion ni timeout de consignas.

## Parametros y justificacion

Todos los parametros de navegacion estan centralizados en `mid_navigation.h`:

| Parametro | Valor / motivo |
|---|---|
| Avance | 100 mm, +1600 cps: conserva AUTO-1. |
| Preventivo frontal | 300 mm: conserva el margen existente sobre Safety; no es una distancia de frenado validada. |
| Safety | 200 mm: no se modifica. Un frontal <=200 mm impide autorizar retroceso incluso antes de que Safety procese el snapshot. |
| Trasera para retroceso | >300 mm: conserva el margen anterior, se comprueba antes de cada emision. |
| Retroceso | 20 mm / 300 ms, -1600 cps. El limite temporal ya existia; la distancia es una separacion inicial pequena, no una evasion completa. A la consigna nominal, 300 ms representan unos 27 mm teoricos; el objetivo menor deja margen, sin asumir velocidad fisica alcanzada. |
| Giro experimental | 10 grados / 250 ms, +/-1200 cps. Conserva velocidad y tiempo anteriores; el objetivo pequeno requiere unos 13 mm por rueda con la base vigente. No se modifica el sentido validado de motores/encoders. |
| Falta de progreso | Menos de 1 mm nuevo en 200 ms: 20 ciclos de motor / 10 de navegacion; unos 18 conteos por rueda con la geometria vigente, por encima de un conteo aislado. Es una guarda provisional de esta maniobra, no un diagnostico permanente de motor bloqueado. |
| Discrepancia de ruedas | >10 mm: se conserva la guarda de consistencia AUTO-1, aplicada a recorridos proyectados segun el signo de la fase. |
| Odometria | Estado 2, geometria provisional completa o calibrada y edad <=50 ms; limites existentes de odometria. |
| Avance/espera inicial | Maximo 2000 ms desde AUTO: no se amplia la maniobra recta validada. |
| Secuencia global | Maximo 3000 ms desde AUTO: cubre 2000 + 300 + 250 ms y cambios de fase, con margen acotado. Ninguna transicion reinicia este plazo. |
| Consigna individual | 500 ms desde emision, sin cambios. Si navegacion deja de emitir, no se mantiene indefinidamente el ultimo objetivo aunque los encoders sigan progresando. |

Los plazos de 300/250 ms comienzan al entrar en la fase e incluyen la espera de
su referencia nueva. Llegar al timeout sin detectar antes la distancia/angulo
objetivo es TIMEOUT, no una maniobra completada. Sin progreso se cancela antes.
No se compensan retrasos enviando rafagas ni se espera bloqueando tareas.

## Odometria y autorizacion

Por fase se conservan ambos conteos acumulados de referencia y los factores
independientes `pi * diametro * 1000 / CPR` existentes. El progreso es el
promedio de los desplazamientos de ambas ruedas, proyectados al signo esperado.
No se reinicia la odometria global ni se suma solo el ultimo `increment_m`.

Para el giro, la orientacion incremental proviene de
`delta_theta = (delta_right_m - delta_left_m) / ODOM_TRACK_M`. La comprobacion
equivalente usa el recorrido medio absoluto de las dos ruedas con signos
opuestos, limitado a `angulo_objetivo_rad * base_mm / 2`. Asi no hay saltos
por cruces de +/-pi ni se mezclan referencias angulares. No usa IMU-4 ni hace
correcciones de trayectoria o fusion.

Se conservan resta int64 comprobada, valores finitos, limites numericos,
coherencia de timestamps/conteos y wrap de ticks. Un incremento contrario al
signo esperado o una discrepancia excesiva finaliza la sesion; no se invierten
signos automaticamente. Se acepta geometria provisional como estimacion,
sin declararla una medicion exacta del desplazamiento fisico.

Ambos HC-SR04 deben ser validos y recientes segun los criterios existentes:
snapshot <=100 ms, cada captura <=200 ms, sin IR de caida ni falla Safety.
No se inicia adquisicion desde navegacion. Ausencia de eco nunca equivale a
camino libre: puede producir SAFETY_FAULT_SENSOR y exigir rearme explicito.

El retroceso solo se evalua al detectar un obstaculo durante el avance, con
frontal aun >200 mm, trasera >300 mm y todas las condiciones validas. La
incertidumbre o una trasera demasiado proxima impiden o cancelan la maniobra.

El giro NO esta autorizado en el firmware operativo aunque los dos HC-SR04
esten despejados. No existe informacion del espacio lateral barrido. En una
compilacion experimental previamente autorizada se exige ademas retroceso
aplicado y completado, ambos extremos >300 mm, sensores/odometria vigentes,
sesion vigente, ningun STOP/Safety, un solo giro y presupuesto para los 250 ms
completos mas una iteracion. Estas comprobaciones no demuestran seguridad
lateral y no sustituyen la supervision de un area fisicamente libre.

## Causas de finalizacion

`Navigation_GetFinishReason()` conserva la primera causa; no es un codigo
Safety ni un evento nuevo del datalogger. No se modifica STATUS/SENSORS ni se
agrega un endpoint. La web muestra el estado principal, no esta causa interna.

| Causa | Interpretacion |
|---|---|
| `NAV_FINISH_TARGET` | Avance de 100 mm completado. |
| `NAV_FINISH_OBSTACLE` | Obstaculo ya presente al iniciar: detencion preventiva, sin retroceso. |
| `NAV_FINISH_EVASION_COMPLETED` | Retroceso y giro experimental completados; terminal. |
| `NAV_FINISH_EVASION_NOT_AUTHORIZED` | Informacion/espacio insuficientes para evasion; tambien el giro bloqueado por defecto despues de la separacion. El indicador interno `reverse_completed` distingue si el retroceso se completo. |
| `NAV_FINISH_OPERATOR_STOP` | STOP prioritario solicitado por el operador. |
| `NAV_FINISH_SAFETY` | Safety cancelo la fase activa. |
| `NAV_FINISH_ODOMETRY` | Odometria invalida, antigua, incoherente o fuera de limites. |
| `NAV_FINISH_SENSORS` | Perdida/invalidez sensorial, incluyendo IR inseguro. Safety sigue siendo responsable del enclavamiento. |
| `NAV_FINISH_TIMEOUT` | Plazo global, avance o fase vencidos. |
| `NAV_FINISH_NO_PROGRESS` | No alcanza el progreso minimo dentro de la ventana. |
| `NAV_FINISH_CANCELLED` / `NAV_FINISH_EMISSION` | RESET/salida de modo/revocacion o timeout permanente de consigna; publicacion rechazada, respectivamente. |

## Pruebas y compilacion

`Tests/safety_b11/run.bat` conserva el conjunto AUTO-1 sin relajar sus
expectativas y los perfiles B7 anteriores; agrega AUTO-2 operativo y variantes
experimentales izquierda/derecha. Usa fuentes reales de navegacion, FSM,
MotorControl, PI, driver y odometria; las entradas sinteticas son solo de tests.
Incluye bucles continuos de MotorControl con navegacion cada 20 ms, sin
reinicializar odometria entre sus iteraciones, y carreras antes del consumo PWM.

Resultados locales:

| Conjunto | Resultado |
|---|---|
| AUTO-2 operativo, regresiones comunes incluidas | 363/363 |
| AUTO-1 conservada | 353/353 |
| AUTO-2 con giro experimental izquierdo / derecho | 388/388 por sentido |
| Perfil B7 completo / giros B7 anteriores | 428/428; 55/55 por sentido |
| Odometria / sensores / IMU / ultrasonidos | 28/28; 8/8; 48/48; 19/19 |
| ESP32 nativo / JavaScript | 143/143; 160/160 grupos |

Compilacion completa STM32 operativa: text=99584, data=940, bss=36000 bytes;
RAM estatica 36940 bytes (32 bytes adicionales respecto de AUTO-1). Sin nuevos
stacks, tareas, colas o asignaciones de heap. Tambien compila AUTO-1 en el
directorio separado indicado: text=98528, data=924, bss=35984 bytes.
ESP32: flash=930681/1048576 bytes, RAM estatica=72320/327680 bytes. Todas estas
compilaciones finalizaron sin errores ni advertencias, sin cambios de Makefile,
framework, particiones ni configuracion del IDE. El ELF operativo sigue en
`Firmware/stm32_control/build/stm32_control.elf`; el de ESP32 en
`Firmware/esp32_gateway/.pio/build/esp32/firmware.elf`.

`-fstack-usage` informa frames locales de 120 bytes para `Navigation_Step`,
32 para su helper AUTO-2 y 88 para el calculo de progreso. No equivalen al
maximo real de pila ni miden el tiempo de las secciones criticas: esas
comprobaciones y la dinamica fisica quedan pendientes.

Las pruebas comprueban umbrales, progreso y falta de progreso, objetivos exactos,
timeout de fase/global, ticks irregulares/wrap/cero, direccion incorrecta,
falta de eco, IR, Safety/STOP/RESET en cada fase, rearme sin reactivacion,
rechazo de objetivos de fases anteriores y PWM/PI cero tras cancelar. Los tests
de giro habilitado no autorizan su activacion fisica ni demuestran cobertura
lateral. Compilar no equivale a ejecutar el software en la PCB.

## Procedimiento fisico posterior (requiere revision y autorizacion)

1. **Avance y parada preventiva:** robot supervisado sobre superficie plana,
   sin bordes ni personas en la trayectoria. Con Wi-Fi operativo, confirmar
   sensores recientes, odometria valida e IDLE sin fallas/bloqueo. Tener STOP
   accesible. Primero probar una sola orden AUTO con camino despejado y
   comprobar avance aproximado de 10 cm y detencion. STOP -> confirmar IDLE.
   Para la parada preventiva, disponer un obstaculo fijo con lectura frontal
   inicialmente algo mayor de 300 mm (aproximadamente 350--400 mm), con espacio
   trasero libre. Evitar introducir objetos con la mano durante el movimiento.
2. **Separacion limitada:** con el perfil operativo de esta etapa, comprobar
   que ante el obstaculo se detiene, solo retrocede si la trasera es >300 mm,
   recorre aproximadamente 2 cm y queda detenido. No debe girar. Si no hay eco,
   Safety se enclava o no existe progreso, detener la validacion y recuperar
   mediante la politica existente, nunca deshabilitando protecciones.
   El giro requiere autorizacion fisica separada, area lateral libre verificada
   por el supervisor y una compilacion experimental identificada. Solo entonces
   observar un giro pequeno seguido de detencion; no exigir exactitud angular.
3. **Secuencia integrada:** una sola orden AUTO por ensayo, sin nuevas ordenes
   automaticas desde la web. Observar detencion final dentro de la ventana
   global de 3 s y ausencia de reinicio al desaparecer el obstaculo. Usar STOP
   y confirmar IDLE. En un ensayo separado autorizado, interrumpir con STOP.
   No realizar desconexiones de sensores ni pruebas deliberadas de bloqueo de
   CPU con potencia conectada. No depender de USB/depuracion durante la marcha.

Sin instrumentos adicionales se evalua coherencia, reaccion y desplazamiento
aproximado. CPR, ruedas, deslizamiento, suelo y frenado afectan las distancias;
puede existir sobrepaso y movimiento residual. Los HC-SR04 tienen incertidumbre
angular y ecos ausentes/reflejados, sin informacion lateral. Safety no garantiza
una parada con CPU/scheduler bloqueados ni ante fallo del puente. No se cambia
la politica conocida de bloqueo por ausencia de eco. No se implementa una
evasion completa segura ni planificacion de caminos.
