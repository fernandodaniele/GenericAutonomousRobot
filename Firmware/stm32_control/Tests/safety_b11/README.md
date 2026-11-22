# Control de motores, seguridad y navegacion B7.7

AUTO-2F: test_navigation_auto2f.c agrega retroceso real simulado entre 300/800 ms,
limite de 800, falta de progreso, STOP/Safety, presupuesto global y rearme con
Safety 200 mm sin cambios. Fixtures AUTO-2 actualizados a 275 mm / >250 mm;
AUTO-1/B7 conservan sus parametros. Ver
[AUTO2F_REFINAMIENTO.md](../../../../Docs/AUTO2F_REFINAMIENTO.md).

AUTO-2E: `test_navigation_settle.c` verifica la espera supervisada y la nueva
referencia. Los helpers AUTO-2 recorren la espera real; no se deshabilitan
regresiones de Safety, permisos, timeout, odometria, retroceso ni giro experimental.
Ver [AUTO2E_TRANSICION_RETROCESO.md](../../../../Docs/AUTO2E_TRANSICION_RETROCESO.md).

AUTO-2D: `test_navigation_diagnostics.c` observa las fuentes reales y reproduce
una cuenta positiva residual antes/despues de procesar retroceso. Comprueba
retencion tras STOP, nueva sesion, publicacion fallida/tardia, clasificacion
odometrica, formato UART y prioridad inferior de NAV. No modifica expectativas
anteriores ni agrega la pausa propuesta. Ver
[contrato AUTO-2D](../../../../Docs/AUTO2D_DIAGNOSTICO_NAVEGACION.md).

## Perfil operativo AUTO-2 (actual)

Ver [AUTO2_REACCION_OBSTACULOS.md](../../../../Docs/AUTO2_REACCION_OBSTACULOS.md).
`NAV_AUTO2_ENABLED=1` es el valor predeterminado: conserva avance 100 mm/2 s,
agrega reaccion preventiva y retroceso 20 mm/300 ms, con ventana global 3 s.
Sin cobertura lateral, el giro sigue deshabilitado por defecto. Las causas
de finalizacion distinguen la separacion completada de un giro no autorizado.

`run.bat` ejecuta AUTO-2 operativo, AUTO-1 explicita (`NAV_AUTO2_ENABLED=0`),
AUTO-2 experimental izquierdo/derecho y los tres conjuntos B7 anteriores.
Los casos AUTO-1 y B7 conservan sus expectativas; no se sustituyen por nuevos
tests ni se relajan las barreras. Cada resultado identifica AUTO2/ODOM/TURN.
El resto del README conserva el historial de las etapas anteriores.

## Perfil conservado AUTO-1

El perfil conservado usa `NAV_AUTO1_ODOMETRY=1` y `NAV_AUTO2_ENABLED=0`: un unico avance de 100 mm
estimados, +1600 cps por rueda, plazo global de 2000 ms y parada terminal sin
retroceso/giro. Ver [AUTO1_AVANCE_ODOMETRIA.md](../../../../Docs/AUTO1_AVANCE_ODOMETRIA.md).
El README que sigue conserva el contexto historico B7; los valores vigentes
de PI/PWM son los del firmware consolidado (Kp=2,20; Ki=0,10; limite 40%).

La variante AUTO-1 de `run.bat` ejecuta `test_navigation_auto1.c`; se conserva el
conjunto anterior completo con `NAV_AUTO1_ODOMETRY=0` y los dos conjuntos de
giro experimental anteriores (55 por sentido). No se eliminan casos ni se
relajan barreras para habilitar la nueva maniobra. Solo esas compilaciones
host de regresion seleccionan el perfil anterior; el ELF normal usa AUTO-1.

---

Base B7.7: rama G1-add/NavegacionAutonoma, HEAD bc00bbd.
Este README describe el comportamiento permanente y las pruebas actuales.
Los bancos de ensayo fueron retirados; sus resultados figuran solo como historia.

## Comportamiento permanente

- TaskMotorControl ejecuta el lazo nominal cada 10 ms. Kp=0,50, Ki=0,10,
  dt=0,010 s, sin nueva sintonizacion ni compensacion de tension.
- TaskMotorControl_SetTargetSpeeds(int32_t left_speed_cps, int32_t right_speed_cps)
  es la API publica, exclusiva de contexto de tarea. Las unidades son cuentas
  de encoder por segundo, firmadas, no RPM ni m/s.
- Cada emision aceptada recibe tick y generacion dentro de la misma seccion
  critica. Una cola de un elemento conserva el ultimo mensaje.
- Edad <500 ms desde emision se admite; edad >=500 ms se rechaza o vence.
  El consumo o reenqueued no rejuvenece el mensaje. El plazo de 200 ms era
  exclusivo de los ensayos retirados: NO es el timeout permanente de consignas.
- Las APIs genericas siguen siendo exclusivas de MANUAL. B7.5 conserva avance
  AUTO y agrega retroceso acotado con sesion/fase, direccion y permiso de un uso.
  Consulta, arquitectura, compilacion y validacion sin potencia:
  [B7_5_RETROCESO.md](../../../../Docs/B7_5_RETROCESO.md).
  Regresion normal actual: 354 casos (350 anteriores +4 de giro deshabilitado),
  mas 19 ultrasonicos. El giro experimental esta deshabilitado por defecto.
  Su cobertura adicional ejecuta 55 casos host con giro izquierdo y otros 55
  con giro derecho, sobre los mismos fuentes de produccion:
  [B7_7_GIRO_EXPERIMENTAL.md](../../../../Docs/B7_7_GIRO_EXPERIMENTAL.md).
  Entrar en un modo no genera por si solo un objetivo ni recupera el anterior.
- No hay productor de ensayos, pulsos directos, tokens, secuencias automaticas
  ni registros historicos de ensayo en RAM.
- B6 agrega ordenes UART manuales fijas mediante mid_motion, sin velocidades
  numericas ni renovacion automatica. La fecha del primer byte se conserva en
  la publicacion y consumo, con barreras de STOP/entrada al modo.
  Protocolo, arquitectura, compilacion B6 y ensayo fisico propuesto:
  [B6_MOVIMIENTO.md](../../../../Docs/B6_MOVIMIENTO.md).

## B7.2 historica: AUTO sin movimiento

El bloqueo total de actuacion AUTO descrito en esta seccion fue reemplazado
por la ruta controlada B7.3. No interpretar n como un ensayo sin movimiento.

La consola ESP32 agrega n, que envia AUTO seguido de LF. La STM32 responde
AUTO_QUEUED si pudo admitir y encolar la solicitud, o AUTO_REJECTED si el modo,
Safety, STOP pendiente, antiguedad o disponibilidad de la cola lo impiden.
La respuesta de encolado no garantiza la transicion: confirmar mediante STATUS.
Solo IDLE permite entrar en AUTONOMOUS (STATE=1). No se cambia directamente
entre MANUAL y AUTO. RESET conserva su semantica: solo rearma SAFETY_STOP hacia
IDLE; en AUTO sano no cambia el modo ni genera objetivos.

| Tecla | Trama | Funcion |
|---|---|---|
| p | PING | PONG existente |
| s | STATUS | STATE, FAULTS y STOP existentes |
| r | RESET | Rearme explicito; nunca activa AUTO |
| m | MANUAL | Solicitud MANUAL desde IDLE |
| t | STOP | Anulacion y salida a IDLE si Safety lo permite |
| n | AUTO | Solicitud AUTO desde IDLE, sin movimiento |
| w | AVANZAR | Movimiento manual; rechazado en AUTO |
| x | RETROCEDER | Movimiento manual; rechazado en AUTO |
| a | GIRAR_IZQUIERDA | Movimiento manual; rechazado en AUTO |
| d | GIRAR_DERECHA | Movimiento manual; rechazado en AUTO |
| e | DETENER | Parada manual; rechazada en AUTO, usar STOP |

Se conserva la misma cola de ocho eventos. Su mensaje privado agrega fecha de
origen y epoca de solicitudes AUTO; no se agrega otra cola ni tarea. La fecha
del primer byte UART no se rejuvenece al consumir. Se exige edad <500 ms y
origen estrictamente posterior a STOP/RESET y entrada al modo actual, usando
resta modular de ticks. Igual tick se rechaza conservadoramente.

STOP/RESET invalidan AUTO al solicitarse y al procesarse, incluso si la cola
esta llena. La epoca tambien protege un encolado tardio despues de la barrera.
La transicion revalida dentro de una seccion critica breve; las operaciones de
cola quedan fuera. RESET/STOP mantienen sus efectos B1 y las IRQ siguen activas.

Entrar en AUTO invalida objetivos anteriores mediante TaskMotorControl_Stop.
El backend admite y ejecuta objetivos solo en MANUAL en esta etapa; por eso ni
la API clasica ni la fechada pueden accionar motores en AUTO. No hay navegador,
mid_navigation, renovacion periodica, cambios de PI, PWM, drivers ni sensores.
La ventana de 500 ms limita la solicitud de entrada, no la permanencia en AUTO.

Regresion actual: 215 casos, base de 172 (B6/B6.5) mas 43 de B7.2. Se ajustaron
dos expectativas antiguas a la autorizacion actual: AUTO no admite movimiento,
y START_AUTO se rechaza durante STOP pendiente. El resto de regresiones se
conserva. Las pruebas ejercitan UART, cola, FSM y backend reales, con HAL/RTOS
sustituidos: mismo tick, wrap, cola llena, eventos antiguos, carreras de
STOP/RESET/Safety, rearme, persistencia sin objetivos/PWM y rechazo manual.
Los 19 casos ultrasonicos permanecen sin cambios.

Compilar STM32 desde Firmware/stm32_control, creando primero build/b72/firmware:
mingw32-make -B -o build/b72/firmware -j4 BUILD_DIR=build/b72/firmware.
Compilar ESP32 desde la raiz: pio run -d Firmware/esp32_gateway -e esp32.
ELF STM32: Firmware/stm32_control/build/b72/firmware/stm32_control.elf.
ELF ESP32: Firmware/esp32_gateway/.pio/build/esp32/firmware.elf.
run.bat ejecuta las 215 pruebas sin variantes ni bancos temporales.

Validacion posterior propuesta, inicialmente sin potencia: RESET y confirmar
STATE=0 FAULTS=0 STOP=0; pulsar n y confirmar STATE=1. Ambos objetivos y CCR2/CCR3
deben permanecer cero, incluso al esperar mas de 500 ms. w/x/a/d/e deben responder
MOVE_REJECTED. Pulsar t: IDLE, sin fallas nuevas. Desde MANUAL, n debe rechazarse.
Con una falla Safety, STOP no debe borrar el enclavamiento; al recuperar sensores
y hacer RESET debe quedar IDLE sin restaurar AUTO. Cualquier programacion o
prueba fisica requiere autorizacion; no pausar CPU con potencia conectada.

## Limite PWM y controlador PI

Unica configuracion: MOTOR_PWM_MAX_PERCENT en Core/Inc/drv_motor.h.
Valor PROVISIONAL conservado: 10. Configurable en compilacion entre 0 y 100,
pero cualquier aumento para hardware requiere revision de especificaciones.

Motor_GetPwmLimit calcula floor((ARR+1)*porcentaje/100) usando ARR real y
aritmetica de 64 bits, acotado por ARR y capacidad del driver/int16. Con la
configuracion actual ARR=8399, el techo es 840 cuentas. No es una especificacion
definitiva del motor ni permite deducir seguridad para bateria de 7,4 V.

Motor_SetSpeed aplica el mismo limite a ambos signos, preservando direccion.
Todas las rutas habituales no nulas encontradas pasan por este driver. Cero,
Motor_Stop y bloqueo Safety llegan a cero sin calcular el techo. current_speed
del handle es PWM aplicado firmado; no es la velocidad del encoder.

El PI usa el mismo techo efectivo para salida, integral y anti-windup. Una
integral previa se recorta si se reduce ese techo. La integracion condicional
no aumenta saturacion y permite recuperarse con error contrario. Objetivo cero
o techo cero anulan integral/salida. No se integra velocidad residual cuando
el motor no tiene objetivo. STOP y Safety limpian tambien los objetivos y PI.

## Encoders y sentido de giro

TIM1 izquierdo: 16 bits, ARR=65535. TIM2 derecho: 32 bits, ARR=UINT32_MAX.
Encoder_ReadDelta lee CNT, resta la referencia y actualiza last_count; NO pone
CNT a cero. Init/Start y el descarte inicial de TaskMotorControl establecen
referencias. La resta modular conserva signo para desplazamientos menores que
medio modulo entre lecturas.

La conversion nominal es delta*100 en ventanas de 10 ms, con resolucion de
100 cps por cuenta observada. Se conserva la proteccion de overflow con int64
y saturacion al rango int32. No hay factor de inversion de signo por software.
Jitter y pausas largas siguen afectando esta estimacion nominal; no hay
conversion mecanica a RPM sin resolucion del encoder/reductora conocida.

PWM positivo mantiene PD0=1/PD1=0 a izquierda y PD3=1/PD2=0 a derecha.
La polaridad validada depende tambien del montaje y cableado fisicos.

**Cableado derecho confirmado por el usuario, conservado sin cambios:**

- C1 (B) -> PA15.
- C2 (A) -> PA1.

No modificar CubeMX, fases ni agregar inversion software para volver al esquema
anterior. Revalidar el signo si se cambia fisicamente el conjunto motor/encoder.

## Protecciones B1 preservadas

- Arranque bloqueado. Safety exige snapshot con edad <=100 ms; ambos
  ultrasonidos validos con edad individual <=200 ms, distancias >0,20 m y sin
  caida IR. Los 200 ms sensoriales siguen siendo provisionales.
- ReportFault activa el bloqueo e invalida objetivos/PI y ambos PWM en la misma
  transaccion critica, sin depender del procesamiento de eventos de la FSM.
- El rearme explicito exige condiciones seguras y capturas nuevas estrictamente
  posteriores a la falla. Igual tick es ambiguo y se rechaza. Revision de fallas
  y segunda copia estable protegen frente a fallas/publicaciones concurrentes.
- Un rearme exitoso vuelve a IDLE y no ordena movimiento. Recuperar sensores
  no rearma automaticamente. EMERGENCY conserva su politica existente.
- STOP invalida generaciones, limpia objetivos/integrales y pone ambos PWM cero
  antes de IDLE, incluso si MANUAL/AUTONOMOUS sigue en el mismo ciclo/tick.
- Mensajes pendientes o encolados tarde anteriores a STOP se descartan por
  generacion. Solo una emision realmente nueva puede aplicar movimiento.
- MotorControl y driver serializan comprobacion de bloqueo y escritura GPIO/CCR
  con la activacion de Stop/Safety. No hay colas, esperas, HAL bloqueante ni
  lectura de encoder dentro de esas secciones criticas.
- Vencimiento de consigna limpia PI/PWM, sin convertirlo en un rearme ni una
  nueva orden. Tick cero es legitimo; las edades se calculan por resta modular.

El guard de signo contrario y cancelacion de pulsos de los ensayos temporales
fueron retirados. No formaban parte de B1. No se incorpora una nueva politica
permanente de falla de encoder ni se cambia Safety.

## Pruebas permanentes reproducibles

Desde la raiz, con GCC nativo/MinGW disponible:

~~~powershell
& .\Firmware\stm32_control\Tests\safety_b11\run.bat 'C:\MinGW\bin\gcc.exe'
& .\Firmware\stm32_control\Tests\ultrasound\run.bat 'C:\MinGW\bin\gcc.exe'
~~~

El ejecutor de seguridad compila con C11, Wall/Wextra/Werror y conteo de
invocaciones PI solo en host. Ejecuta primero todas las 350 regresiones
anteriores y 4 casos nuevos con la configuracion normal. Luego ejecuta 55 casos
experimentales para cada sentido, con NAV_EXPERIMENTAL_TURN_ENABLED=1 y
NAV_TURN_LEFT=1/0. Son variantes nativas de prueba, no nuevos firmware de banco.
Las expectativas historicas de finalizar el retroceso sin giro se comprueban
en la variante normal; no se eliminan ni se relajan para habilitar el giro.
Salidas:

- Firmware/stm32_control/build/safety_tests/test_safety.exe
- Firmware/stm32_control/build/safety_tests/results.log
- Firmware/stm32_control/build/safety_tests/test_turn_left_results.log
- Firmware/stm32_control/build/safety_tests/test_turn_right_results.log
- Firmware/stm32_control/build/ultrasound_tests/results.log

Base B5 de seguridad/control: 123 casos. Se preservaron los 99 de B1/B1.1.1 y
17 permanentes de limite, anti-windup y encoder. Se migraron 7 comprobaciones
utiles de los ensayos a llamadas directas de la API publica en el arnes, sin
productores adicionales en el firmware.

B6 conserva los 123 casos y agrega 42: total 165. Incluye los fuentes reales
mid_motion.c, drv_uart.c y task_comms.c, con HAL UART sustituida. Comprueba
las cinco traducciones, admision por modo/Safety, STOP inmediato aun con cola
llena, DETENER, rearme, mismo tick, generaciones y carreras, vencimiento desde
origen, ausencia de renovacion, wrap/tick cero, prioridad STOP en el mailbox,
CRLF, overflow, NUL embebido y comandos desconocidos. run.bat conserva todo el
conjunto permanente y agrega las pruebas experimentales host descriptas arriba;
no hay productores ni instrumentacion de banco en ARM.

La cobertura incluye arranque, validez/antiguedad individual, obstaculos e IR,
rearme/fallas concurrentes, STOP/consignas pendientes/generaciones/mismo tick,
499/500 ms, wrap, PI sin objetivo con movimiento residual, limite PWM simetrico,
reduccion de techo e integral, feedback firmado, conversion sin overflow,
ambos motores individualmente en ambos sentidos, tick cero consumido/no consumido
y vencimiento que cruza el limite durante el consumo.

El arnes incluye los .c reales de Safety, FSM, MotorControl y driver motor.
Sustituye RTOS/HAL/sensores/deltas para intercalar eventos en rutas reales.
Tambien incluye drv_encoder.c real bajo nombres host distintos para probar
counter/reference/wrap 16/32 bits sin copiar su algoritmo.

Se verifican ausencia de colas/esperas en secciones criticas, balance de
anidamiento/suspension y hasta seis escrituras GPIO/CCR por transaccion.
Esto no mide tiempos Cortex-M ni sustituye hardware. Los 19 casos ultrasonicos
permanentes se mantienen separados y sin cambios.

## Compilacion B5 historica y ELF de produccion

Con GNU Arm Embedded y MinGW make en PATH, desde la raiz:

~~~powershell
Push-Location Firmware/stm32_control
New-Item -ItemType Directory -Force -Path build/b5/firmware | Out-Null
mingw32-make -B -o build/b5/firmware -j4 BUILD_DIR=build/b5/firmware
Pop-Location
~~~

Se usa un directorio separado y -B para reconstruir todos los objetos. -o
excluye solo el directorio ya creado, evitando el mkdir del Makefile en Windows.
No se cambia el Makefile ni se utilizan operaciones Git para limpiar.

ELF historico B5: Firmware/stm32_control/build/b5/firmware/stm32_control.elf.
Para B6 utilizar build/b6/firmware/stm32_control.elf, segun el documento B6.
Log: Firmware/stm32_control/build/b5/firmware_build.log.

Antes de seleccionar un ELF para una futura sesion autorizada, comprobar exito
del build y su ruta/fecha. Los ELF anteriores en directorios de ensayos son
productos obsoletos ignorados por Git; no se enlazan ni se usan en B5.
No reutilizar esos ELF como firmware de produccion. No modificar Debug.launch
automaticamente para seleccionarlos. En CubeIDE, reconstruir la configuracion
normal completa desde el codigo consolidado.

Verificacion B5: compilacion completa sin errores ni advertencias, 123/123
pruebas permanentes y 19/19 ultrasonicas. Sin simbolos temporales en objeto ni
ELF y sin referencias activas en Core, Tests o configuracion de compilacion.
El BIN conserva el SHA256 de la variante normal B4 anterior. Se comprobaron
sin cambios los hashes de once archivos protegidos, incluido Debug.launch.
En B5 la API conservaba declaracion/definicion y simbolo en su objeto; el enlazador
puede descartar la funcion del ELF si ninguna tarea la invoca actualmente.

## Resultados historicos: NO son funciones disponibles actualmente

- B1/B1.1.1: 99 pruebas base de seguridad y limpieza PI.
- B224/B228: diagnosticos de consignas, tokens y registro retenido de PWM.
- B3.2: giro fisico informado de ambos motores con PWM directo de 10%,
  VM=6 V y pulsos nominales de 200 ms, con retorno de CCR a cero.
- B4: 116 pruebas normales, 251 de banco y 19 ultrasonicas antes de limpieza.
  El usuario valido fisicamente los cuatro casos PI 21..24: ambos motores,
  ambos sentidos, realimentacion de signo coherente, maximo 840 con ARR=8399,
  final PWM cero y cancel_reason=0, VM=6 V/corriente limitada a 0,30 A.
  La correccion de fase derecha se hizo fisicamente con el cableado arriba.
- B5: se retiraron todos los bancos, sus estructuras/historiales, productores,
  tokens, estados, hooks y helpers exclusivos. Las pruebas de diagnostico
  retiradas no se cuentan como cobertura de produccion. La cobertura util se
  conserva en el conjunto permanente; no se repitieron pruebas fisicas.

## Riesgos y limites pendientes

Las ganancias, el limite del 10% y las temporizaciones no son una homologacion
mecanica/electrica. Faltan especificaciones definitivas del motor/reductora/CPR,
caracterizacion dinamica, corriente, frenado, tiempo critico/pila y jitter.
No se implementa compensacion para alimentar motores de 6 V desde 7,4 V.

No se agrega deteccion permanente de encoder parado, desconectado, incoherente
o de signo cambiado. La polaridad debe conservar el montaje validado.
El PI puede pedir signo opuesto al objetivo al corregir sobrevelocidad; no se
introduce rampa nueva ni se afirma que esta validada toda la dinamica.

La parada software requiere CPU/scheduler ejecutando: no garantiza parada ante
CPU bloqueada, interrupciones deshabilitadas indefinidamente, perdida de
alimentacion o fallo del puente. Watchdog/corte independiente requieren otra
etapa. No pausar la CPU con potencia conectada suponiendo que el timeout actuara.

No se autoriza movimiento autonomo. Esta limpieza no graba firmware ni opera
hardware; cualquier prueba posterior requiere revision y autorizacion.
