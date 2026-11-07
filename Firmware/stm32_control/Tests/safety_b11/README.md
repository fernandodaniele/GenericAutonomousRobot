# Control de motores y seguridad: version consolidada B5

Estado de referencia: rama G1-add/ControlMotores, HEAD 83330ea.
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
- Solo MANUAL/AUTONOMOUS y Safety sin bloqueo admiten consignas y actuacion.
  Entrar en un modo no genera por si solo un objetivo ni recupera el anterior.
- No hay productor de ensayos, pulsos directos, tokens, secuencias automaticas,
  registros historicos de ensayo en RAM ni comandos UART nuevos de velocidad.
- La aplicacion actual no incorpora un productor permanente de consignas no
  nulas; la API queda disponible para futuras integraciones autorizadas.

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

El ejecutor de seguridad compila UNA variante, sin opciones de banco, con C11,
Wall/Wextra/Werror y conteo de invocaciones PI solo en host. Salidas:

- Firmware/stm32_control/build/safety_tests/test_safety.exe
- Firmware/stm32_control/build/safety_tests/results.log
- Firmware/stm32_control/build/ultrasound_tests/results.log

Conjunto de seguridad/control: 123 casos. Se preservaron los 99 de B1/B1.1.1 y
17 permanentes de limite, anti-windup y encoder. Se migraron 7 comprobaciones
utiles de los ensayos a llamadas directas de la API publica en el arnes, sin
productores adicionales en el firmware.

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

## Compilacion limpia y ELF de produccion

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

ELF actual: Firmware/stm32_control/build/b5/firmware/stm32_control.elf.
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
La API conserva declaracion/definicion y simbolo en su objeto; el enlazador
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
