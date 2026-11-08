# B7.7 - Giro autonomo acotado experimental

Base: bc00bbd, rama G1-add/NavegacionAutonoma. Sin programacion ni pruebas
fisicas. El giro queda deshabilitado en el firmware operativo normal.

## Configuracion y parametros

Core/Inc/mid_navigation.h centraliza:

```c
#ifndef NAV_EXPERIMENTAL_TURN_ENABLED
#define NAV_EXPERIMENTAL_TURN_ENABLED 0
#endif
#ifndef NAV_TURN_LEFT
#define NAV_TURN_LEFT 1
#endif
#define NAV_TURN_MAX_MS 250U
```

No hay habilitacion UART ni variable Debug para activar el giro. Una futura
compilacion habilitada requiere definir NAV_EXPERIMENTAL_TURN_ENABLED=1 de
forma coherente para todos los fuentes y reconstruirlos completos. Requiere
revision y autorizacion fisica especifica. No se prepara ni se graba ese
firmware en esta etapa. NAV_TURN_LEFT=1 fija izquierda, -1200/+1200 cps;
NAV_TURN_LEFT=0 fija derecha, +1200/-1200 cps. La magnitud reutiliza la constante
MOTION_TURN_CPS de mid_motion. No alterna sentidos entre sesiones.

Se conservan: frontal preventivo 300 mm, Safety >200 mm en ambos sensores,
trasero >300 mm para retroceso, retroceso 300 ms, sesion 2000 ms, consignas
500 ms y PWM maximo 10%. No hay cambios de PI, drivers, sensores o perifericos.

## Transiciones

| Subestado | Salida o transicion |
|---|---|
| NAV_ESPERAR_DATOS = 0 | Cero hasta datos seguros. Obstaculo inicial: DETENIDO. |
| NAV_AVANZAR = 1 | +1600/+1600; obstaculo frontal <=300 mm cancela primero el avance y evalua retroceso. |
| NAV_RETROCEDER_ACOTADO = 3 | -1600/-1600; maximo 300 ms, con frontal >200 y trasero >300 mm. |
| NAV_GIRAR_ACOTADO = 4 | Solo opcion habilitada y retroceso completado correctamente; vector diferencial fijo, maximo 250 ms. |
| NAV_DETENIDO = 2 | Terminal; objetivos, integrales y PWM cero, sin reanudacion. |

Con la opcion en cero, el retroceso termina en DETENIDO como en B7.5. Se
rechazan los vectores de giro tambien en el backend, aun si se invoca la API.

Para considerar completado el retroceso debe haber llegado un objetivo vigente
al ciclo real de ambos motores y vencido normalmente su plazo de fase. Este
acuse software NO demuestra que las ruedas se hayan movido ni una distancia
recorrida. Un retroceso no consumido, cancelado, vencido por consigna, STOP,
Safety, modo o sesion no habilita giro.

Al vencer el retroceso, MotorControl pone cero sin cancelar prematuramente la
evaluacion de su fin normal. TaskControlRobot vuelve a comprobar el snapshot
completo y las condiciones del retroceso, incluido trasero >300 mm. Luego usa
TaskMotorControl_Stop para revocar el permiso anterior y limpiar objetivos/PI,
antes de entrar en giro. No emite en ese ciclo ni en el mismo tick. La siguiente
iteracion debe evaluar nuevamente todas las condiciones antes de emitir giro.

La ventana de giro empieza al entrar en ese subestado, incluido el intervalo
inicial con salida cero. Solo se entra si quedan al menos 250 ms reales de
sesion en ese instante. 250 ms restantes se admiten; 249 ms se rechazan. Nunca
se cambia el tick de inicio AUTO. Un retraso del scheduler no se compensa ni
extiende plazos. MotorControl comprueba ambos limites incluso sin otro ciclo
de navegacion. La anulacion se realiza al siguiente ciclo ejecutado; no es un
corte fisico exacto independiente del scheduler.

Al finalizar se mantiene DETENIDO aun si cambian los sensores. Solo STOP,
confirmacion IDLE y nueva solicitud AUTO permiten otra sesion. RESET no restaura
objetivos ni reinicia la maniobra. Como maximo hay un giro por sesion.

## Autorizacion y cancelacion

En cada decision de giro se exige AUTO, Safety habilitado, STOP no pendiente,
snapshot con edad <=100 ms, ambos ultrasonidos validos con edad individual
<=200 ms, distancias finitas >200 mm, ausencia de caida IR y plazos vigentes.
Se usa TaskSafety_EvaluateSnapshot sin cambiar sus reglas. Los limites de
300 mm son preventivo frontal de avance y condicion de retroceso; no prueban
espacio lateral durante giro. Las edades usan resta modular, admitiendo tick
cero y desbordamiento.

La ruta exclusiva sigue verificando identidad de TaskControlRobot, modo, tick,
generacion, token de sesion/fase y permiso de emision de un uso. El backend
relaciona el vector fijo con la fase exacta: un permiso de avance/retroceso no
autoriza giro y viceversa. Cada transicion revoca primero el token anterior;
una consigna vieja no habilita otra fase/sesion. La publicacion y el consumo
conservan las barreras frente a STOP/Safety, incluyendo eventos del mismo tick
y encolados tardios. Los cinco comandos manuales UART siguen rechazados en AUTO.
No hay nuevas colas, tareas, accesos directos a PWM ni espera bloqueante.

## Pruebas y compilacion normal

Desde la raiz, con GCC nativo, GNU Arm Embedded y MinGW make disponibles:

```powershell
& Firmware/stm32_control/Tests/safety_b11/run.bat
& Firmware/stm32_control/Tests/ultrasound/run.bat
Push-Location Firmware/stm32_control
New-Item -ItemType Directory -Force -Path build | Out-Null
mingw32-make -B -o build -j4 BUILD_DIR=build
Pop-Location
```

run.bat ejecuta los mismos fuentes reales con HAL/RTOS sustituidos: 354 casos
normales (350 anteriores intactos +4 de giro deshabilitado), 55 experimentales
izquierdos y 55 derechos. Las configuraciones habilitadas son ejecutables host;
no se genera otra variante ARM. Los 19 casos del driver ultrasonico se mantienen.

Se comprueban transiciones, cero entre fases, fase/vector/tarea/permiso unico,
consignas de otra fase y sesion, 249/250 ms, presupuesto global 250/249 ms,
datos viejos/invalidos, IR, Safety concurrente, STOP/cola llena, RESET, rechazo
manual, no reanudacion, tick cero, wrap de ticks y token. Las comprobaciones de
seccion critica conservan ausencia de colas/esperas y maximo seis escrituras
GPIO/CCR por transaccion. No miden duracion Cortex-M ni validan mecanica.

ELF normal unico: Firmware/stm32_control/build/stm32_control.elf.
Log completo: Firmware/stm32_control/build/b77_build.log.
No modificar Debug.launch ni limpiar despues de construir ese ELF.

## Limitaciones y validacion posterior

Los sensores frontal/trasero NO verifican el volumen lateral barrido por el
chasis. El giro no es una evasion segura ni completa. 250 ms no indican angulo,
distancia ni frenado; 300 mm no es distancia de frenado homologada. El limite
PWM y las edades sensoriales siguen siendo provisionales. CPU/scheduler
detenidos pueden mantener una salida: estos plazos no son corte independiente.

La primera validacion autorizada debe mantener desconectada la potencia de
motores y comprobar el firmware normal: avance, retroceso, DETENIDO, PWM cero,
STOP, RESET y rechazo manual en AUTO. Debe confirmar ausencia de giro. Una
eventual validacion del giro habilitado necesita otra autorizacion especifica,
revision del chasis y espacio lateral libre; no forma parte de esta etapa.
