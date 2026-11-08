# B7.3 - Primera navegacion: avanzar y detener

Base: 281e550, rama G1-add/NavegacionAutonoma. Implementacion pendiente de
revision y validacion fisica. No se graba firmware ni se opera hardware.

AUTO ya puede emitir avance. El comando n no debe interpretarse como la prueba
sin movimiento de B7.2. Mantener desconectada la potencia durante la primera
validacion autorizada. La consola ESP32 solo cambia esos textos obsoletos;
teclas, tramas, API UART y configuracion de flash permanecen iguales.

## Responsabilidades

- TaskControlRobot conserva la FSM principal y la cola de ocho eventos B7.2.
  Inicia una sesion al entrar efectivamente en AUTO y llama Navigation_Step
  despues de procesar eventos, con su periodo nominal de 20 ms y prioridad +3.
- mid_navigation contiene los tres subestados. Lee un snapshot consistente
  mediante TaskSensores_GetData, sin bloquear y fuera de secciones criticas.
- TaskSafety_EvaluateSnapshot expone la evaluacion existente como consulta pura.
  No registra fallas, enclava ni rearma. Safety conserva su autoridad y su codigo
  de evaluacion, umbrales y rearme; no se trasladan a navegacion.
- mid_motion traduce el unico movimiento autonomo permitido a +1600/+1600 cps,
  usando MOTION_DRIVE_CPS de B6. No son RPM ni velocidad lineal calibrada.
- TaskMotorControl conserva PI, encoders, timeout de 500 ms y PWM limitado por
  el driver al 10%. Agrega identidad de origen, modo y sesion a la autorizacion
  y al consumo, sin otra tarea ni cola.

## Estados y condiciones

| Estado | Comportamiento |
|---|---|
| NAV_ESPERAR_DATOS = 0 | Inicio de sesion, objetivos cero; espera datos utilizables dentro de los 2 s totales. |
| NAV_AVANZAR = 1 | Reevaluacion y una emision autorizada por ciclo, sin extender la sesion. |
| NAV_DETENIDO = 2 | Terminal: sin objetivos ni nuevas emisiones, aunque los datos vuelvan a ser seguros. |

Configuracion central en Core/Inc/mid_navigation.h:

```c
#define NAV_FRONT_STOP_MM 300U
#define NAV_SESSION_MAX_MS 2000U
```

Solo puede avanzar con AUTO activo, Safety habilitado, sin STOP pendiente,
snapshot disponible y seguro segun el predicado de Safety, distancias finitas
y frontal estrictamente mayor que 0,300 m. Se conservan snapshot <=100 ms,
ambos ultrasonicos validos con edades individuales <=200 ms, ambas distancias
>0,200 m y ausencia de caida IR. La consulta usa el hay_caida publicado, calculado
por TaskSensores a partir de los cuatro IR; no vuelve a leer GPIO.

Las edades se calculan con resta de TickType_t. Tick cero es legitimo. El tick
general no sustituye a los ticks individuales, que corresponden al disparo de
la ultima captura aceptada. Se respeta la alternancia sensorial ya validada.

Falta de informacion inicial mantiene ESPERAR_DATOS. Un obstaculo o caida
conocidos terminan la sesion. Cualquier dato invalido/antiguo/ausente durante
AVANZAR termina la sesion inmediatamente mediante TaskMotorControl_Stop, sin
esperar 500 ms. La navegacion detiene ante IR, pero solo Safety registra la
falla y decide su enclavamiento/rearme. La parada preventiva a 300 mm por si
sola no crea una falla Safety. El umbral Safety de 200 mm no se modifica.

## Autorizacion y concurrencia

Las APIs genericas TaskMotorControl_SetTargetSpeeds, la variante fechada y
Motion_Execute siguen restringidas a MANUAL. Pasar AUTONOMOUS como parametro
no las habilita. Los cinco comandos manuales siguen rechazandose durante AUTO,
incluido DETENER; el operador debe usar STOP para cancelar AUTO.

Navigation_Step prepara un permiso de un solo uso, asociado a la sesion y al
tick de decision. Motion_NavigationAdvance usa una ruta dedicada. El backend
comprueba identidad de TaskControlRobot, AUTO, Safety, STOP pendiente, origen
posterior a las barreras B6, permiso/sesion y exactamente +1600/+1600 cps.
No se permiten retroceso, giro ni velocidades arbitrarias por esta ruta.

Modo y sesion se conservan en la consigna y en el contexto consumido. El
backend comprueba nuevamente sesion y plazo en cada ciclo de 10 ms. Una
consigna AUTO no cruza hacia MANUAL, ni una sesion antigua hacia otra nueva,
incluso si se aislara esa barrera de la generacion de consignas.

TaskMotorControl_Stop revoca la sesion y su permiso antes de limpiar objetivos,
integrales y PWM, en la misma transaccion critica existente. STOP/Safety que
intervengan entre decision, admision y encolado invalidan cualquier mensaje
tardio. Las colas y lecturas sensoriales quedan fuera de secciones criticas.

El timeout de una consigna AUTO tambien termina la sesion. Una renovacion
legitima interrumpida antes de encolarse conserva la proteccion B1 de generacion,
sin cancelar por error una sesion nueva o vigente. No se cambian Kp=0,50,
Ki=0,10, periodo PI, lectura de encoder, limite PWM ni drivers.

## Tiempo y finalizacion

Los 2 s se cuentan desde la entrada valida en AUTO, incluyendo la espera. No
desde el primer avance ni desde la ultima emision. Edad >=2000 ms rechaza
publicacion y actuacion aunque una consigna tenga menos de 500 ms. El backend
tambien corta una consigna activa si navegacion no ejecuta otro ciclo a tiempo.

La parada se aplica al ejecutar las rutas de supervision: 20 ms nominales en
navegacion y 10 ms en MotorControl, mas jitter. No es un temporizador fisico de
corte ni una garantia ante CPU/scheduler detenidos. Los 2 s no detectan perdida
UART; no se agrega heartbeat, renovacion de sesion ni protocolo nuevo.

Tras DETENIDO, la FSM puede seguir indicando STATE=1: el subestado de navegacion
es independiente. Se requiere STOP, confirmar STATE=0 y una solicitud AUTO
nueva. RESET con Safety enclavado vuelve a IDLE tras validar recuperacion; no
restaura AUTO. RESET durante AUTO sano conserva ese modo de la FSM, pero cancela
la sesion de navegacion y la deja terminal, sin reanudacion automatica.

## Verificacion reproducible

Desde la raiz, con GCC nativo, GNU Arm Embedded y MinGW make disponibles:

```powershell
& Firmware/stm32_control/Tests/safety_b11/run.bat
& Firmware/stm32_control/Tests/ultrasound/run.bat
Push-Location Firmware/stm32_control
New-Item -ItemType Directory -Force -Path build/b73/firmware | Out-Null
mingw32-make -B -o build/b73/firmware -j4 BUILD_DIR=build/b73/firmware
Pop-Location
pio run -d Firmware/esp32_gateway -e esp32
```

El arnes conserva los 215 casos anteriores y agrega 66 de B7.3: 281 en total.
Incluye fuentes reales de navegacion, movimiento, FSM, Safety y backend. Sustituye
RTOS/HAL/publicacion sensorial para intercalar eventos; no copia algoritmos ni
afirma validacion fisica. test_navigation.c se incluye en test_safety.c y se
ejecuta con el run.bat existente. Los 19 casos ultrasonicos siguen separados.

Cobertura: 300/301 mm, datos iniciales ausentes, edades independientes exactas
y excedidas, tick cero/wrap, trasero, IR, no reanudacion, plazo desde entrada,
renovacion finita, timeout de consigna, deadlines en decision/publicacion/consumo,
STOP/cola llena, carreras, rearme, sesiones viejas, modo de la consigna,
identidad de tarea y permiso de un uso. Los hooks son exclusivos del arnes.

ELF STM32: Firmware/stm32_control/build/b73/firmware/stm32_control.elf.
ELF ESP32: Firmware/esp32_gateway/.pio/build/esp32/firmware.elf.
No reutilizar el ELF B7.2 para validar B7.3. No modificar Debug.launch.

## Validacion futura, sin potencia de motores

1. Solo tras revision/autorizacion para cargar los firmwares, mantener potencia
   de motores desconectada. Confirmar sensores, suelo IR y RESET hacia IDLE.
2. Con frontal >300 mm y trasero >200 mm, pulsar n. Observar STATE=1 y los
   subestados navigation.state: esperar -> avanzar -> detenido. Comprobar
   objetivos +1600/+1600 y CCR2/CCR3 <=840 con ARR=8399, luego ambos cero.
3. Confirmar que started_tick no cambia con las renovaciones y que la sesion
   termina al cumplir 2 s. Retirar/reponer obstaculos despues no debe reanudar.
4. En otra sesion, acercar un obstaculo al margen preventivo, conservando
   distancia >200 mm: objetivos y PWM cero, DETENIDO y ninguna falla nueva por
   esa sola condicion. No tomar 300 mm como distancia de frenado validada.
5. Comprobar STOP durante avance, rechazo de w/x/a/d/e en AUTO y necesidad de
   STOP -> IDLE -> AUTO para otra sesion. Verificar que RESET no restaure avance.
6. Ensayar condiciones sensoriales de fallo bajo supervision, sin potencia:
   Safety debe mantener su enclavamiento y rearme explicito existentes.

Variables permanentes observables: navigation.state, navigation.started_tick,
navigation.session, task_context.left_target_cps/right_target_cps,
task_context.left_pwm_command/right_pwm_command y htim4.Instance->CCR2/CCR3.
No hay historiales ni instrumentacion temporal. No pausar CPU con potencia
conectada suponiendo que el timeout detendra el puente.

Quedan pendientes frenado, geometria del chasis, consumo, jitter/pila y validacion
del margen real. No hay evasion, giros, retroceso automatico ni autorizacion de
movimiento autonomo fisico como resultado de las pruebas locales.
