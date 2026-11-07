# Etapa B1.1 — protecciones y pruebas reproducibles

Base: `c808843`, rama `G1-add/UltrasonidoTrasero`.
La etapa prepara validacion sobre STM32 con potencia de motores desconectada.
No autoriza movimiento, navegacion ni considera 200 ms una garantia de frenado.

## Archivos y alcance

| Archivo | Cambio |
|---|---|
| `Core/Src/task_safety.c` | Evaluacion comun de snapshot, dos ultrasonidos e IR; tick de falla; rearme con copia final estable; parada inmediata de ambos PWM al notificar falla. |
| `Core/Inc/task_safety.h` | Contrato del rearme y contexto de tarea. |
| `Core/Src/task_control_robot.c` | STOP invalida consignas y limpia PI/PWM antes de pasar a IDLE. La entrada efectiva a SAFETY_STOP tambien invalida. |
| `Core/Src/task_motor_control.c` | Metadata de emision y generacion; rechazo de consignas vencidas/anteriores; parada comun; coordinacion atomica de objetivos, PI y salida. |
| `Core/Inc/task_motor_control.h` | Nueva operacion Stop para FSM/Safety; firma SetTargetSpeeds conservada y documentacion corregida del PI activo. |
| `Core/Src/drv_motor.c`, `Core/Inc/drv_motor.h` | Comprobacion y escrituras serializadas, contexto exclusivo de tarea, Stop sin colas ni esperas. |
| `Tests/safety_b11/` | Arnes, cinco headers de sustitucion, ejecutor y este informe. |

No se modificaron TaskSensores, su estructura/cola, drivers ultrasonicos, IR,
perifericos, HAL, CubeMX, navegacion, comunicaciones, limites de PWM ni ganancias
PI. Las tareas mantienen sus periodos. No se agregaron productores de consignas.

## Vigencia y cobertura conservadora

`TaskSafety_SnapshotFaults` es la evaluacion compartida por supervision y rearme:

- Snapshot: edad <=100 ms; limite existente conservado.
- Cada ultrasonido: valido y edad <=200 ms.
- Cada distancia: estrictamente >0.20 m, independientemente de modo/direccion.
- Ninguna caida IR.

Los 200 ms se centralizan exclusivamente en la constante
`TASK_SAFETY_ULTRASOUND_MAX_AGE_MS` de task_safety.c y estan marcados PROVISIONALES,
solo para desarrollo/validacion sin potencia. Valores numericos en pruebas son
expectativas de los requisitos, no otra configuracion del firmware.

Se usa resta modular de TickType_t. El tick cero no invalida una captura. Una
publicacion nueva no cambia la edad de los ticks individuales. Se conserva el
enclavamiento: recuperar datos no rearma automaticamente. NaN no satisface la
comparacion de distancia estrictamente mayor que el umbral.

La supervision sigue siendo periodica (20 ms): el limite de aceptacion no es una
garantia de que el hardware se detenga exactamente al cumplirse 200 ms. La
latencia de observacion, jitter, escrituras PWM y respuesta mecanica se miden
aparte. Las restas suponen lapsos menores a una vuelta completa del contador;
no acreditan recuperacion tras congelar la ejecucion durante una vuelta completa.

## Rearme con capturas posteriores a la falla

Cada ReportFault registra el tick actual, incrementa la revision y bloquea.
La referencia se actualiza con la ultima notificacion, tambien si ya estaba
bloqueado: es conservador y puede exigir otra ronda de capturas.

Con un snapshot vigente, una captura es posterior solo si su edad es estrictamente
menor que la edad de la falla, calculadas ambas desde el mismo `now` modular.
Si comparten tick se rechaza, sin intentar adivinar el orden submilisegundo.
El timestamp sensorial identifica el disparo, no la publicacion ni la finalizacion.
Una captura iniciada antes de la falla y terminada despues no sirve para rearmar.

Antes de la primera falla notificada se exige igualmente que ambos sensores
sean seguros y vigentes; no existe un tick de falla ficticio que descarte por si
solo la primera captura legitima en tick cero. Arranque sigue bloqueado.

Secuencia de RequestReset:

1. Copia la revision bajo seccion critica y hace la comprobacion preliminar.
2. Suspende brevemente el scheduler y obtiene otra copia mediante GetData/peek
   con timeout cero, FUERA de una seccion critica externa.
3. Dentro de una seccion critica breve revalida todo, revision, emergencia y
   disparos posteriores a la ultima falla. Limpia consignas/PI/PWM antes de
   liberar el bloqueo.
4. Reanuda siempre el scheduler; si una tarea de falla pendiente ejecuta al
   reanudar, se comprueba nuevamente el bloqueo antes de devolver exito.

No hay esperas ni colas dentro de las secciones criticas escritas por esta etapa.
La HAL/FreeRTOS real fue inspeccionada: xQueuePeek admite scheduler suspendido
con espera cero (queue.c:1636,1653); la copia interna de la cola mantiene su
proteccion propia habitual. vTaskSuspendAll no deshabilita las IRQ; FreeRTOS
puede diferir contabilizacion de ticks hasta xTaskResumeAll.

Solo TaskSensores publica y lo hace desde tarea. La suspension impide que publique
entre la copia final y la decision. Capturas DMA pueden terminar durante ese
lapso, pero no escriben SensoresDatos_t ni su cola. No fue necesario alterar la
interfaz sensorial. La garantia se refiere al snapshot mas reciente al confirmar
el rearme; no predice cambios fisicos ni publicaciones posteriores a esa decision.

La FSM conserva el retorno a IDLE, vacia sus eventos pendientes y exige una nueva
orden de modo. No se inicia movimiento como efecto del rearme. EMERGENCY conserva
su politica existente de no poder borrarse mediante RequestReset.

## Emision, STOP y consignas

La inspeccion encontro como unico punto de emision SetTargetSpeeds; no hay
productores intermedios ni llamadas productoras actuales fuera de su definicion
en Core. Su firma publica se conserva. Cada llamada aceptada toma tick y nueva
generacion juntos bajo seccion critica antes de encolar. La estructura privada
de cola agrega `issued_tick` y `generation`; el consumo nunca cambia esos campos.

El plazo permanece en 500 ms: edad <500 se admite, edad >=500 se rechaza/vence.
Un reenqueued del mismo mensaje conserva su fecha y no reinicia el plazo.
Reinvocar explicitamente la API constituye una nueva emision; no existe otro
mensaje externo con fecha que deba propagarse en el firmware actual.

Una generacion comun ordena cada emision aceptada y cada parada. La ultima es
la unica admisible, aunque varias ocurran en un tick o los encolados se inviertan
por preempcion. Si un encolado viejo sobrescribe fisicamente uno nuevo, se
rechaza y se detiene en vez de aplicar la orden vieja; puede perderse esa nueva
orden y debera emitirse otra. Se prioriza la seguridad ante ese intercalado.

Stop invalida inmediatamente todas las generaciones anteriores, borra objetivos,
integrales y comandos PWM y escribe cero en ambos canales. No accede a colas:
el descarte de mensajes pendientes es logico por generacion y su retiro ocurre
en el siguiente consumo no bloqueante. Esto tambien cubre encolados anteriores
que terminen despues de STOP y ordenes ya retiradas de la cola pero no aplicadas.

La FSM llama Stop dentro de la transaccion de parada antes de IDLE. STOP y
MANUAL/AUTONOMOUS en una misma iteracion no restauran objetivos. El rearme tambien
invalida la generacion. Una orden de modo no equivale a una consigna nueva.

La generacion es uint32_t; su vuelta no debe abarcar la vida de una consigna
pendiente. El plazo de 500 ms y la frecuencia fisicamente posible de estas
operaciones evitan esa ambiguedad en la plataforma actual. Se prueba el paso
UINT32_MAX -> 0 para una parada y las edades de ticks a ambos lados del wrap.

## Concurrencia y PWM

ReportFault actualiza el bloqueo y llama Stop en una sola seccion critica.
MotorControl serializa aceptacion de consigna, estado PI y ambos outputs con
esa misma proteccion. Recibir de la cola y leer encoders ocurre antes de entrar.
Motor_SetSpeed protege tambien su propia comprobacion y las escrituras.
Motor_Stop protege sus escrituras y admite anidamiento.

Si la salida ya esta en su seccion critica, la tarea que notifica la falla se
ejecuta al salir y escribe cero. Si la falla ya activo el bloqueo, el driver no
puede escribir un PWM no nulo con una comprobacion anterior. Al retornar de
ReportFault, los compare de ambos motores quedaron en cero y las consignas
anteriores invalidadas; no se espera a la cola FSM ni a la siguiente iteracion.

El puerto Cortex-M4F del repositorio usa BASEPRI y anidamiento; no se agrego
deshabilitacion global con PRIMASK. Son APIs de tarea, no FromISR. Se revisaron
los productores: no hay escrituras no nulas a PWM desde ISR. HAL_GPIO_WritePin
solo valida parametros y escribe BSRR, sin espera. En las secciones criticas hay
calculos PI acotados y hasta seis escrituras GPIO/CCR; no hay retardos, HAL
bloqueante, colas, lectura de encoder, mutex ni bucles de espera.

Los seis accesos no son una medicion de duracion. Debe medirse el peor tiempo en
STM32, incluida carga de IRQ de mayor prioridad. Compare cero tampoco equivale
a demostrar frenado fisico: precarga del timer, puente y mecanica requieren
validacion. CPU/scheduler detenidos, perdida de alimentacion, watchdog y corte
independiente siguen fuera de alcance. No se modificaron handlers fatales.

## Ejecucion y resultados

En Windows con MinGW GCC disponible, desde la raiz del repositorio:

```powershell
& .\Firmware\stm32_control\Tests\safety_b11\run.bat 'C:\MinGW\bin\gcc.exe'
```

Puede omitirse el argumento si gcc esta en PATH. Se usa .bat para no requerir
cambiar la politica local de ejecucion de PowerShell.

El arnes incluye directamente los cuatro .c de produccion, sin duplicar sus
algoritmos. Sustituye RTOS, HAL, encoders y publicacion sensorial. Inyecta eventos
en las rutas reales: despues de copiar el flag dentro del driver, entre emision
y encolado, entre consumo y aceptacion, entre lecturas del rearme y al reanudar
el scheduler. Emula el diferimiento de tareas por anidamiento/suspension.

Resultado B1.1: **88 pruebas aprobadas**, GCC nativo 6.3.0, C11, Wall/Wextra/Werror.
El log queda en build/etapa_b11/tests/results.log. Cubre todos los grupos pedidos:

- Arranque, falta de snapshot, limites 100/200, edades independientes, invalidos,
  distancias <=0.20, IR, tick cero y desbordamiento.
- Capturas anteriores/posteriores/igual tick de falla, publicaciones peligrosas
  durante el rearme, fallas concurrentes, emergencia y retorno a IDLE sin movimiento.
- Emision y plazo 500, reenqueued, colas pendientes, STOP/MANUAL/AUTO, integrales,
  generaciones y emisiones simultaneas o tardias.
- Preempcion en driver/control reales, bloqueo con ambos signos de PWM, parada
  con cola FSM llena, secciones balanceadas y ausencia de colas/esperas dentro.

Compilacion del firmware completo: Makefile existente y GNU Arm GCC 10.3.1,
directorio de objetos separado build/etapa_b11/firmware; salida 0 y sin errores
ni advertencias en build/etapa_b11/firmware_build.log. Se generaron ELF/HEX/BIN.
ELF: text=71916, data=900, bss=29684 bytes. No se uso STM32CubeIDE para compilar,
no se grabo firmware y no se opero hardware.

Los dobles no emulan completamente FreeRTOS, acustica, DMA, tiempos reales de IRQ,
timer PWM ni planta mecanica. Las pruebas no acreditan seguridad fisica.

## B1.1.1 — PI durante parada

Verificado en G2-fix/SeguridadControl, HEAD c808843, sobre los cambios B1.1 pendientes.

TaskMotorControl_Run solo llama MotorPI_Calculate para el motor cuyo objetivo
es distinto de cero. Para objetivo cero limpia explicitamente integral/comando
PWM y llama Motor_Stop. La lectura de encoders sigue ejecutandose para ambos.
No cambian ganancias, periodos, consignas, generaciones ni secciones criticas.
MotorPI_Calculate conserva su retorno defensivo previo para objetivo cero;
ese retorno ya evitaba integrar en el codigo inspeccionado.

La bateria completa ahora tiene **99 casos aprobados**: los 88 originales y 11
con movimiento residual de +200/-300 cps. Cubren falta de consigna, bloqueo,
IDLE, STOP, timeout de orden activa o pendiente, objetivos cero y parada
individual izquierda/derecha. Tambien comprueban que una nueva consigna parte
de integral cero y que parar un motor no limpia el PI del otro motor activo.

El ejecutor agrega -finstrument-functions UNICAMENTE al arnes nativo. Los hooks
cuentan entradas a la funcion PI real: cero durante parada y una por motor
activo/ciclo. No hay instrumentacion agregada al firmware STM32. El log de las
99 pruebas sigue en build/etapa_b11/tests/results.log. La compilacion completa
B1.1.1 utiliza build/etapa_b111/firmware; salida 0, sin errores ni advertencias.
Su log esta en build/etapa_b111/firmware_build.log.

## Validacion fisica propuesta — no realizada

1. Con potencia de motores desconectada, verificar arranque bloqueado y compare
   cero; rechazar rearme con uno o ambos sensores ausentes/invalidos.
2. En ejecucion libre, registrar edades de snapshot, ambos disparos y tick de
   falla. Verificar igualdad 200 frente a >200 y snapshot >100. Usar un mecanismo
   de inyeccion revisado, sin forzar estados DMA mediante escrituras arbitrarias.
3. Desconectar ECHO de cada sensor por separado, verificar falla y cero de ambos
   canales. Reconectar y comprobar que hacen falta las dos capturas posteriores
   a la ultima notificacion; recuperar datos no debe rearmar por si solo.
4. Probar obstaculos frontal/trasero <=0.20 e IR sin piso, rearme y publicacion
   nueva peligrosa durante el rearme. No deducir tiempos de reaccion con F6 o
   breakpoints: alteran el scheduler y los timers pueden continuar funcionando.
5. Con un procedimiento de emision de consignas autorizado para banco y sin
   potencia, verificar 499/500 ms, STOP seguido de MANUAL/AUTO dentro de un ciclo,
   ausencia de PWM hasta nueva consigna y limpieza de PI. Esta etapa no agrega
   un productor de pruebas al firmware ni habilita velocidades por UART.
6. Medir compare/salida PWM y duracion maxima de secciones criticas/suspension
   bajo carga con instrumentacion aprobada. Verificar cero en ambos canales al
   notificar falla, con independencia del estado/ocupacion de la cola FSM.
7. Repetir recuperacion trasera con obstaculo fijo y correlacionar ECHO/TRIG y
   datos frente a la anomalia de 180.54 mm respecto de ~280-294 mm. No se considera
   resuelta; con la politica actual, 180.54 mm debe bloquear por obstaculo.

No conectar potencia ni autorizar movimiento por estos resultados. Las pruebas
con ruedas elevadas requieren una autorizacion posterior; los 200 ms siguen
siendo provisionales y los parametros PI no constituyen calibracion mecanica.

No se realizaron operaciones Git de escritura ni commits. Los archivos nuevos
de pruebas permanecen sin seguimiento hasta que el supervisor autorice el paso
correspondiente. Los productos de compilacion/logs estan ignorados por Git.
