# AUTO-1: avance unico de 100 mm estimados

Nota posterior AUTO-2: este documento conserva el perfil validado AUTO-1,
ahora seleccionable al compilar con `NAV_AUTO2_ENABLED=0`. La configuracion
operativa por defecto se describe en
[AUTO2_REACCION_OBSTACULOS.md](AUTO2_REACCION_OBSTACULOS.md). Los parametros y
resultados historicos siguientes corresponden exclusivamente a AUTO-1.

Perfil operativo sobre `acd1618`, sin nuevas tareas, cambios de protocolo ni
modificaciones de Safety, FSM, PI/PWM, odometria, IMU o mapa. Solo se actualiza
el texto explicativo del boton AUTO de la web para describir este perfil.

## Autorizacion y estados

El comando AUTO existente solicita IDLE -> AUTONOMOUS con las barreras actuales:
Safety habilitado, sin STOP pendiente, solicitud y generacion vigentes. Nunca
arranca automaticamente. Las ordenes manuales UART siguen rechazadas en AUTO.

`Navigation_Begin()` inicia ESPERAR_DATOS y el plazo total de 2000 ms. Con
sensores seguros y una muestra odometrica valida, de edad <=50 ms y tomada
estrictamente despues de entrar en AUTO, se captura una referencia de ambos
conteos acumulados antes de la primera consigna. Una muestra del mismo tick
que AUTO mantiene la espera: no se puede demostrar el orden de esos hechos.
La espera consume parte del plazo original, que nunca se renueva.

Con referencia aceptada se pasa a AVANZAR y se utiliza
`Motion_NavigationAdvance()` (+1600 cps por rueda), reevaluando las condiciones
en cada ciclo de la tarea de control (20 ms). No se escribe PWM desde navegacion.
La vigencia individual de consignas sigue siendo 500 ms desde su emision.

Objetivo alcanzado, obstaculo, perdida de datos, cancelacion o vencimiento llevan
a DETENIDO terminal. La parada de navegacion usa `TaskMotorControl_Stop()`:
revoca sesion/permisos, invalida la generacion, limpia objetivos e integrales y
escribe PWM cero mediante la serializacion existente. El backend tambien
revalida progreso y vigencia antes de admitir/aplicar objetivos pendientes;
ante rechazo cancela la sesion y limpia las salidas por su ruta habitual.

La FSM principal **permanece AUTONOMOUS** tras terminar normalmente. No vuelve
a avanzar aunque mejoren los datos. Para otra maniobra: STOP -> confirmar IDLE
mediante STATUS -> nueva solicitud AUTO. RESET no restaura una sesion y no
sustituye esa secuencia. Safety puede llevar la FSM a SAFETY_STOP, que conserva
su rearme explicito y sus requisitos actuales.

## Progreso odometrico

Sin reiniciar la odometria global, para cada rueda:

```text
delta_L = conteo_L_actual - conteo_L_referencia
delta_R = conteo_R_actual - conteo_R_referencia
mm_L = delta_L * pi * diametro_L_m * 1000 / CPR_L
mm_R = delta_R * pi * diametro_R_m * 1000 / CPR_R
progreso_mm = (mm_L + mm_R) / 2
detener si progreso_mm >= NAV_AUTO1_TARGET_MM (100)
```

Los factores provienen de `odometry_config.h`, igual que la odometria existente:
diametros 0,0254 m, CPR 1435,2 / 1421,6 y base 0,1492 m, sin modificarlos.
Geometria completa provisional permite la estimacion, sin declararla calibrada.
No se usa la orientacion de encoders o IMU para corregir trayectoria. Tampoco
se suma `increment_m`: el productor ejecuta cada 10 ms y el consumidor cada
20 ms, por lo que sumar solo el ultimo incremento perderia desplazamientos.

Se aceptan referencias absolutas negativas y ticks cero o con wrap. La resta
int64 se comprueba antes de ejecutarla. Se rechazan conteos que retrocedan en
esta maniobra, muestras que retrocedan temporalmente, cambios de conteos en un
mismo timestamp, geometria incompleta, estado odometrico distinto de 2,
edad > `ODOM_MAX_INTERVAL_MS` (50 ms), valores no finitos y saltos numericos
fuera de la extension ya admitida por odometria. El limite numerico por rueda
es dos veces `ODOM_POSITION_LIMIT_M`, convertido a mm; no representa una
velocidad maxima fisica. Se conservan ademas las guardas del productor.

`NAV_AUTO1_MAX_WHEEL_DIFFERENCE_MM = 10` limita la diferencia entre recorridos
de ruedas (10% del objetivo, valor provisional de consistencia). Si una sola
rueda progresa, se cancela antes de aceptar solo el promedio. No corrige el
rumbo, no cambia Safety ni detecta deslizamiento igual de ambas ruedas. Sin
progreso no se declara exito: se mantienen las verificaciones y se termina por
el plazo global de 2 segundos. No se agrega un detector permanente de bloqueo.

## Detencion y causas

Se conserva el umbral preventivo frontal de 300 mm y Safety de 200 mm. Ambos
ultrasonicos deben ser validos y recientes; tambien se conservan la vigencia
del snapshot, antiguedad individual e IR de Safety. La ausencia de eco puede
enclavar una falla de sensor: nunca se interpreta como espacio libre.

`Navigation_GetFinishReason()` expone un resultado de solo lectura para pruebas
y consumidores internos, sin agregar telemetria o diagnosticos periodicos:

| Resultado | Significado |
|---|---|
| `NAV_FINISH_NONE` | Sin finalizacion en la sesion actual. |
| `NAV_FINISH_TARGET` | Promedio >=100 mm con ambas ruedas coherentes. |
| `NAV_FINISH_CANCELLED` | STOP, RESET, salida de AUTO u otra revocacion; incluye el timeout permanente de consignas anterior al plazo global. |
| `NAV_FINISH_SAFETY` | Safety invalido la sesion activa. |
| `NAV_FINISH_TIMEOUT` | Vencimiento del plazo global de 2000 ms. |
| `NAV_FINISH_OBSTACLE` | Frontal preventivo <=300 mm. |
| `NAV_FINISH_SENSORS` | Datos sensoriales inseguros durante el avance. |
| `NAV_FINISH_ODOMETRY` | Odometria ausente, antigua, invalida o incoherente. |
| `NAV_FINISH_EMISSION` | La publicacion fue rechazada sin otra causa terminal previa. |

Se conserva la primera causa hasta `Navigation_Begin()` de una sesion nueva.
No es un nuevo codigo de falla Safety ni un evento del datalogger. Un rechazo
terminal del backend tampoco puede volver a habilitarse por una muestra fresca.

## Conservacion del perfil anterior y regresiones

`NAV_AUTO1_ODOMETRY` vale **1 por defecto** en `mid_navigation.h`: el firmware
operativo solo avanza y se detiene; no retrocede ni gira. Las funciones y
barreras anteriores permanecen. Compilar con `NAV_AUTO1_ODOMETRY=0` conserva el
perfil B7 exclusivamente para regresiones explicitas y trabajo posterior
AUTO-2; no es una opcion remota ni una autorizacion de ensayo fisico.
`NAV_EXPERIMENTAL_TURN_ENABLED` sigue en 0 por defecto.

`Tests/safety_b11/run.bat` ejecuta:

- Perfil operativo AUTO-1: regresiones comunes y nuevas pruebas de objetivo,
  referencia, datos, integridad numerica, plazos, carreras y backend real.
- Perfil anterior: todos los 428 casos existentes, sin relajar expectativas.
- Giros experimentales anteriores: 55 casos host por sentido, conservados.

Las pruebas incluyen el bucle real de MotorControl, alimentando odometria con
sus deltas y planificando navegacion cada 20 ms, asi como snapshots corruptos
inyectados solo en el arnes. Se verifica PWM cero, limpieza PI, tokens/sesiones
antiguos, cola llena, STOP/Safety concurrentes y ausencia de reactivacion.
Los datos sinteticos pertenecen exclusivamente a tests.

Comandos, desde la raiz (sin grabar placas):

```powershell
& Firmware/stm32_control/Tests/safety_b11/run.bat
powershell -NoProfile -ExecutionPolicy Bypass -File Firmware/stm32_control/Tests/odometry_w7/run.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Firmware/stm32_control/Tests/sensors_w5/run.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Firmware/stm32_control/Tests/imu_1/run.ps1
& Firmware/stm32_control/Tests/ultrasound/run.bat
pio run -d Firmware/esp32_gateway -e esp32
```

STM32: dentro de `Firmware/stm32_control`, ejecutar
`mingw32-make -B -o build -j4`. ELF normal: `build/stm32_control.elf`.
La variante operativa no requiere ninguna macro adicional.

Verificacion local de esta etapa (mocks y compilacion, sin hardware):

| Conjunto | Resultado |
|---|---|
| Perfil operativo, incluidos 64 casos AUTO-1 | 353/353 |
| Perfil B7 anterior completo | 428/428 |
| Giro experimental izquierdo / derecho | 55/55 por sentido |
| Odometria / adquisicion sensores / IMU | 28/28, 8/8, 48/48 |
| Driver ultrasonico | 19/19 |
| ESP32 nativo / JavaScript | 143/143, 160/160 grupos |

Compilaciones sin errores ni advertencias: STM32 text=98496, data=924,
bss=35984 bytes (RAM estatica data+bss=36908). ESP32 flash=930569/1048576,
RAM estatica=72320/327680 bytes. La unica ampliacion de RAM estatica STM32
respecto de la compilacion inicial es 72 bytes. No se agregan tareas, colas ni
heap para esta maniobra. Los informes `-fstack-usage` indican frames locales
de 120 bytes para `Navigation_Step`, 72 para `Navigation_IsCommandCurrent`
y 80 para el calculo odometrico; no miden el maximo real de pila ni el tiempo
de las secciones criticas, pendientes de observacion fisica.

## Validacion fisica posterior, solo despues de revision y autorizacion

1. Con el robot detenido, disponer de una superficie plana, sin caidas y con
   amplio espacio frontal/trasero. Confirmar por web STATUS y sensores validos,
   actuales y despejados; revisar que el mapa tenga odometria provisional valida.
2. Con el procedimiento de alimentacion supervisado del proyecto, rearmar si
   corresponde y confirmar IDLE, sin fallas ni bloqueo. Tener STOP accesible.
   No pausar la CPU ni depender de USB/depuracion durante la maniobra.
3. Anotar los conteos de `/api/map` y solicitar AUTO **una sola vez**. Observar
   un avance aproximadamente de 10 cm y la detencion dentro de la sesion de 2 s.
   No exigir precision milimetrica; no encadenar sesiones automaticamente.
4. Comprobar que permanece detenido y STATUS sigue AUTO (o Safety si aparecio
   una falla). Usar STOP y confirmar IDLE antes de considerar otra sesion.
   Consultar de nuevo los conteos para estimar el desplazamiento, sin tratar
   datos web retrasados como el instante exacto de parada.
5. En una validacion separada autorizada, solicitar STOP antes de completar
   el avance y comprobar que no se reactiva. No probar obstaculos o fallas
   desconectando protecciones durante una maniobra con potencia.

Los 100 mm son estimados mediante ruedas: deslizamiento, CPR, geometria,
deformacion y frenado afectan el desplazamiento fisico. La adquisicion y
actuacion son discretas; puede existir sobrepaso y movimiento residual. El
umbral frontal de 300 mm no es una distancia de frenado validada. La cancelacion
software requiere CPU y scheduler funcionando; no garantiza parada con CPU
bloqueada o puente averiado. El resultado fisico de AUTO-1 queda pendiente.
