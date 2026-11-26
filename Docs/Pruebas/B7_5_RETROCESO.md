# B7.5 - Separacion inicial de un obstaculo mediante retroceso acotado

Base: 845d793, rama G1-add/NavegacionAutonoma. Preparado para revision y
validacion posterior sin potencia. No se graba firmware ni se opera hardware.

## Transiciones

Se mantienen la FSM principal, tareas, colas y periodos existentes. Solo se
agrega NAV_RETROCEDER_ACOTADO al modulo mid_navigation.

| navigation.state | Comportamiento |
|---|---|
| NAV_ESPERAR_DATOS = 0 | Espera inicial sin movimiento, incluida en los 2 s. |
| NAV_AVANZAR = 1 | Avanza mientras el frontal sea >300 mm y los datos sean seguros. |
| NAV_DETENIDO = 2 | Terminal: no vuelve a emitir, aunque se recuperen las condiciones. |
| NAV_RETROCEDER_ACOTADO = 3 | Reevalua y emite retroceso hasta su limite de 300 ms o el fin de la sesion. |

El valor 3 de navigation.state NO es STATE=3 de STATUS. La FSM principal
permanece en STATE=1 durante AUTO; STATE=3 de la FSM sigue siendo SAFETY_STOP.

AUTO iniciado con un obstaculo frontal <=300 mm termina detenido, aunque el
trasero este despejado. No inicia un retroceso. Solo una sesion que ya alcanzo
NAV_AVANZAR puede considerar la nueva maniobra.

Al observar frontal <=300 mm durante AVANZAR, primero se usa la parada existente
para anular objetivos positivos, integrales, generacion y permisos, con PWM cero.
Solo si el snapshot completo es seguro y el trasero es >300 mm se cambia a
NAV_RETROCEDER_ACOTADO dentro de la misma seccion critica. En caso contrario se
mantiene la parada terminal. No hay operaciones de cola ni esperas dentro de
esa seccion.

La transicion rota la epoca del permiso, pero conserva navigation.started_tick.
Una consigna positiva pendiente queda invalidada y no puede entrar en la fase
negativa. No se emite en el ciclo de transicion ni en su mismo tick: el siguiente
ciclo con tick posterior debe reevaluar los sensores antes de publicar retroceso.
No se agrega un retardo ni se cambia el periodo de 20 ms.

## Parametros

Centralizados en Core/Inc/mid_navigation.h:

```c
#define NAV_FRONT_STOP_MM 300U
#define NAV_REAR_REVERSE_MIN_MM 300U
#define NAV_REVERSE_MAX_MS 300U
#define NAV_SESSION_MAX_MS 2000U
```

Las consignas reutilizan MOTION_DRIVE_CPS de B6, centralizada en mid_motion.h:
+1600/+1600 cps para avance, -1600/-1600 cps para retroceso. No se duplica una
configuracion de velocidad ni se aceptan valores numericos desde UART.

El retroceso exige en cada emision AUTO, Safety habilitado, ausencia de STOP,
snapshot disponible y reciente, ambos ultrasonidos validos y vigentes, distancias
finitas, sin caida IR, frontal >200 mm y trasero >300 mm. El requisito frontal
de 200 mm se obtiene del predicado Safety existente, que permanece intacto.
Se conservan snapshot <=100 ms y edades individuales <=200 ms; el tick general
no rejuvenece las mediciones. Tick cero y wrap usan resta modular de TickType_t.

Los 300 ms cuentan desde la entrada en la fase, incluyendo el intervalo inicial
sin emision. No prometen 300 ms de accionamiento efectivo, distancia recorrida
ni frenado. Despejar el frontal durante retroceso no devuelve a AVANZAR ni
extiende el tiempo. La sesion global sigue limitada a 2 s desde la entrada en
AUTO, incluida la espera inicial. La maniobra puede terminar antes de 300 ms.

## Autorizacion y cancelacion

Motion_NavigationReverse reutiliza el backend de B7.3. La autorizacion valida
identidad de TaskControlRobot, modo, fecha, generacion, sesion/fase y permiso
de un uso, ahora tambien ligado a la direccion. Solo admite velocidades iguales
en ambos motores y exactamente +/-1600 cps. Un permiso de avance no autoriza
retroceso y uno de retroceso no autoriza avance. No admite giros.

Las APIs genericas y los cinco comandos manuales siguen restringidos a MANUAL.
En AUTO, incluido el retroceso, w/x/a/d/e responden MOVE_REJECTED. Se usa t/STOP
para cancelar la navegacion. No se modifica el ESP32 ni el protocolo UART.

TaskMotorControl comprueba los plazos de fase y sesion al consumir y ejecutar
objetivos. Tambien termina una autorizacion vencida que llega a su primer
consumo, sin aplicar PWM. Una consigna vieja no cancela una fase o sesion nueva.
STOP, Safety, RESET y salida de AUTO conservan sus rutas de cancelacion; los
mensajes tardios quedan rechazados por generacion, modo y permiso.

Al alcanzar 300 ms, perder cualquier condicion o cumplir 2 s, se anulan ambos
objetivos y se pasa a DETENIDO. Los datos recuperados no reanudan. Se requiere
STOP -> confirmar IDLE -> AUTO para otra sesion. RESET no restaura la maniobra.
No cambian PI, limite PWM del 10%, timeout de 500 ms, encoders, sensores,
perifericos ni umbrales/politicas de Safety.

La ejecucion depende de las tareas de 20 ms y 10 ms mas jitter. No se afirma
un corte fisico exacto ni parada ante CPU o scheduler detenidos. No hay watchdog
UART ni renovacion de sesion por comunicacion. No pausar CPU con potencia
conectada suponiendo que un timeout detendra los motores.

## Pruebas y compilacion

```powershell
& Firmware/stm32_control/Tests/safety_b11/run.bat
& Firmware/stm32_control/Tests/ultrasound/run.bat
Push-Location Firmware/stm32_control
New-Item -ItemType Directory -Force -Path build/b75/firmware | Out-Null
mingw32-make -B -o build/b75/firmware -j4 BUILD_DIR=build/b75/firmware
Pop-Location
```

El arnes incluye test_navigation_reverse.c desde test_safety.c. Conserva 281
casos previos y agrega 69: total 350. Un caso B7.3 de parada terminal se ejecuta
ahora con trasero a 300 mm; con trasero despejado la transicion autorizada fue
cambiada expresamente por B7.5 y esta cubierta por los nuevos casos. No se
eliminan las comprobaciones de anulacion inmediata o de Safety de ese caso.
Los 19 casos ultrasonicos permanecen separados y sin cambios.

Se ejercitan los fuentes reales, con el mismo arnes RTOS/HAL determinista:
umbrales exactos 300/301 y 200/201, datos invalidos/antiguos, IR, cero antes de
retroceder, duracion 299/300 ms, plazo durante publicacion/consumo/PI, lease
global de 2 s, STOP/cola llena, Safety concurrente, rearme, direcciones y origen,
mensajes de fases/sesiones antiguas, modo MANUAL, wrap y tick cero. No se copian
algoritmos ni se agrega instrumentacion temporal de produccion.

ELF actualizado: Firmware/stm32_control/build/b75/firmware/stm32_control.elf.
No reutilizar el ELF B7.3 para probar retroceso. No modificar Debug.launch.

## Validacion posterior minima sin potencia

1. Tras revision y autorizacion para cargar el firmware, mantener VM/potencia de
   motores desconectada. Comprobar sensores, IR y rearme hasta IDLE seguro.
2. Iniciar AUTO con frontal >300 mm y trasero >300 mm. Confirmar avance y luego
   acercar el obstaculo frontal al intervalo (200,300] mm antes de los 2 s.
3. Observar primero objetivos/integrales/CCR cero, despues objetivos -1600/-1600,
   subestado 3 y CCR limitados a 840 con ARR=8399. Al terminar: subestado 2 y
   ambos CCR cero. navigation.started_tick no debe cambiar durante la maniobra.
4. Repetir desde una sesion nueva con frontal bloqueado al inicio: no debe
   retroceder. Con trasero <=300 mm, tampoco debe aceptar retroceso.
5. Verificar STOP durante retroceso, no reanudacion al retirar el obstaculo,
   rechazo manual en AUTO y rearme de Safety sin restaurar movimientos.

Variables existentes: navigation.state, navigation.started_tick,
navigation.reverse_started_tick, navigation.session, task_context.left_target_cps,
task_context.right_target_cps, motor_l.current_speed/motor_r.current_speed
(PWM firmado) y htim4.Instance->CCR2/CCR3. No confundir PWM firmado con velocidad
medida del encoder. Live Expressions puede omitir transitorios cortos; no tomar
una captura tardia como prueba del tiempo fisico de maniobra.

Sin sensores laterales no se elige lado ni se gira. Este retroceso no es una
evasion completa. Faltan validar volumen barrido, dinamica, corriente, frenado,
jitter/pila y comportamiento con el chasis. La temporizacion no calibra distancia
recorrida. Esta etapa no autoriza desplazamiento autonomo con potencia.
