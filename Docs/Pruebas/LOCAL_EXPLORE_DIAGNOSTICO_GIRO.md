# LOCAL_EXPLORE: diagnóstico y corrección mínima de wheel_disagreement

Base: G1-add/InterfazWeb, HEAD 79ddac6, conservando los cambios pendientes.
La revisión inicial reprodujo el rechazo de la guarda de 3 mm mediante
pruebas y documentación. La corrección posterior descrita a continuación
modifica únicamente esa validación para LOCAL_EXPLORE; no añade recuperación
automática, estados, tareas ni reinicios de referencias. La causa física de
la asimetría y la fiabilidad de los ensayos posteriores siguen sin comprobarse.

## Corrección vigente: presupuesto de asimetría localizado

`LOCAL_TURN_MAX_WHEEL_DIFFERENCE_MM = 4U`, en `mid_local_exploration.h`,
permite hasta 4 mm de diferencia acumulada exclusivamente durante giros de
LOCAL_EXPLORE y cuando **ambas ruedas ya recorrieron al menos 3 mm** en sus
sentidos esperados. Antes de ese progreso bilateral se conserva la guarda
original de 3 mm para detectar el arranque de una sola rueda.

El fundamento es un presupuesto experimental acotado de 2 mm de recorrido
longitudinal neto del centro en el modelo diferencial ideal, frente a los
1,5 mm del criterio anterior: `abs(R-L)/2`. No es una calibración física ni
garantiza posición, adherencia, ausencia de fallas de encoder o espacio lateral.
La cuantización por sí sola no justifica todo el milímetro adicional.
Se admite una pequeña asimetría mecánica sin tratarla automáticamente como
dato inválido; las discrepancias mayores siguen siendo terminales.

`Navigation_WheelToleranceMm(left_mm, right_mm)` centraliza el criterio en
la validación y en su clasificación diagnóstica. Lo usan tanto el planificador
como `Navigation_IsCommandCurrent()`, consultado por MotorControl antes de
aplicar consignas. No se modifica MotorControl ni se añade un bypass.

Se conservan sin cambios:

- AUTO_TURN, AUTO_EVADE y AUTO_CONTINUOUS: guarda de giro de 3 mm.
- Avance y retroceso: guarda de 10 mm y restantes autorizaciones existentes.
- Referencia acumulada original, objetivo angular y plazo de 800 ms.
- Rechazo de sentido contrario, odometría inválida/antigua, timestamps
  incoherentes, overflow, valores no finitos y falta de progreso.
- STOP, Safety, vigilancia del enlace, estados terminales y consignas antiguas.

La traza sintética que incorpora −251/+195 a los 520 ms ahora se acepta
como 9,563°. Con tres muestras adicionales de −5/+4 a 20 ms alcanza
10,141° a los 580 ms y ordena detenerse. No cambia el origen ni repite el giro.
Esas muestras adicionales son exclusivamente estímulos de prueba; no se
afirma que el robot físico haya completado esa trayectoria.

Casos vecinos con la geometría real: −268/+195 se admite; −269/+195 supera
4 mm y se rechaza. Incluso si el recorrido medio alcanza 10°, una discrepancia
mayor se rechaza antes de aceptar el objetivo. Una rueda que deja de contar
después del arranque también termina al superar ese presupuesto acumulado.

La asimetría persistente **dentro** del presupuesto puede aceptarse: no se
ha implementado un detector de deslizamiento ni de todas las fallas de encoder.
Solo uno de los tres ensayos informados tiene este diagnóstico confirmado.
La corrección no demuestra resolver las otras detenciones sin sus datos.

## Validación actual de software

- Safety/control/motores/navegación operativa: 694 pruebas.
- AUTO-1: 363; variantes experimentales izquierda/derecha: 720 + 720.
- B7 y giros históricos: 436 + 55 + 55.
- Odometría: 28; adquisición: 8; IMU: 48; ultrasónicos: 19.
- ESP32: 224 grupos nativos, incluidos cruces con tramas reales STM32.
- JavaScript: 176 grupos en V8 con DOM/fetch/reloj sustituidos.

Se conservan las comprobaciones históricas y se actualizan únicamente las
expectativas de LOCAL que dependían del límite anterior. Se agregan los
límites de 4 mm, arranque bilateral, rueda detenida después del arranque,
rechazo antes de completar el objetivo, STOP/Safety y datos no confiables.
No se cambia el firmware para satisfacer los mocks.

STM32 compila sin advertencias: text 119752, data 948, bss 38432 bytes
(80 bytes más de código; misma RAM estática). ESP32 compila sin cambios:
Flash 968161/1048576, RAM estática 80368/327680 bytes. Las cadenas HTTP
analizadas suman 3088 y 2048 bytes con stack de 4096; son cotas inferiores,
no mediciones de high-water mark ni garantía de todas las rutas del SDK.

Logs, resultados, diff incremental y manifiesto:
`Firmware/stm32_control/build/local_turn_tolerance/` (ignorado por Git).
No se grabaron placas ni se realizaron pruebas físicas u operaciones Git
de escritura. Tras revisión, actualizar STM32 y repetir LOCAL en un área
despejada y supervisada, conservar diagnóstico si termina y verificar STOP.
El software ESP32 permanece sin cambios respecto de la revisión anterior.

## Antecedente histórico: diagnóstico con la guarda anterior

**Las secciones siguientes conservan la evidencia previa a la corrección.**
Sus rechazos de 3 mm, resultados y propuesta de recuperación describen la
versión anterior; la política vigente y las verificaciones son las anteriores
en este documento. La recuperación compleja evaluada no se implementó ni
se requiere para aceptar el caso confirmado dentro del nuevo presupuesto.

## Resultado del análisis

Los valores −251/+195, interpretados como diferencias desde la referencia
del giro, exceden la guarda existente de 3 mm. No indican un sentido de giro
incorrecto. El fallo no demuestra por sí mismo una avería de encoder,
un error numérico ni una falla de Safety: rechaza la simetría esperada de
la maniobra. También es compatible con asimetría entre ruedas bajo carga,
contactos distintos con el piso o deslizamiento. Estas son hipótesis físicas.

No se atribuye a una causa física concreta el ensayo de 12 segmentos y 8
giros. No se realizaron mediciones nuevas sobre el robot.

## Guarda, geometría y cálculo

`Navigation_WheelToleranceMm()` en
`Firmware/stm32_control/Core/Src/mid_navigation.c` selecciona
`NAV_AUTO3_MAX_WHEEL_DIFFERENCE_MM = 3U` para GIRAR_ACOTADO en perfiles
AUTO_TURN, AUTO_EVADE, AUTO_CONTINUOUS y LOCAL_EXPLORE. Avance y retroceso
mantienen la guarda de 10 mm; los perfiles experimentales históricos
conservan sus propias condiciones. Las constantes están en `mid_navigation.h`.

La comparación es estricta: rechaza si `abs(left_mm-right_mm) > 3`.
Son recorridos acumulados desde el inicio de la fase, corregidos por el
signo esperado y por los factores diferentes de cada rueda. No compara
solo el incremento de una muestra ni exige varios rechazos consecutivos.

Con las constantes reales de `odometry_config.h`:

- Diámetros: 25,4 mm por rueda.
- CPR: izquierdo 1435,2; derecho 1421,6.
- Base entre ruedas: 149,2 mm.
- Configuración general aún identificada como provisional por sus flags.

```text
kL = pi * diametroL / CPR_L = 0,055599536756 mm/cuenta
kR = pi * diametroR / CPR_R = 0,056131439088 mm/cuenta
L = -delta_izquierda * kL          # giro izquierdo: consigna -1200 cps
R =  delta_derecha * kR           # giro izquierdo: consigna +1200 cps
theta = (L + R) / base            # radianes estimados mediante encoders
```

| Muestra | L (mm) | R (mm) | Diferencia (mm) | Ángulo estimado |
| --- | ---: | ---: | ---: | ---: |
| −251/+195, informada | 13,955484 | 10,945631 | **3,009853** | 9,562525° |
| −246/+191, ejemplo previo compatible | 13,677486 | 10,721105 | 2,956381 | **9,369546°** |
| −250/+195, vecina aceptada | 13,899884 | 10,945631 | 2,954254 | 9,541173° |
| −251/+196, vecina aceptable | 13,955484 | 11,001762 | 2,953722 | 9,584080° |

Los cálculos incluyen la representación float de las constantes de firmware.
El exceso de 0,009853 mm es menor que una cuenta de cualquier rueda. No es
un fallo de comparación de coma flotante: la desigualdad se cumple con
los parámetros actuales. Una cuenta adicional o ausente puede cruzar el límite.
El desbalance total tampoco debe confundirse con ese pequeño exceso: son
3,009853 mm, aproximadamente el 24,17 % del recorrido medio de 12,450557 mm.
La diferencia de CPR por sí sola no explica una diferencia de 56 cuentas.

El objetivo de 10° requiere un recorrido medio de 13,020157 mm por rueda
(aproximadamente 234,18 y 231,96 cuentas nominales). Los conteos rechazados
son compatibles con un giro parcialmente completado, no con un giro inverso.
La duración de 520 ms tampoco excede el timeout de giro de 800 ms.

La guarda de 3 mm se introdujo como límite provisional para giros pequeños:
en el modelo ideal con desbalance uniforme corresponde a unos 1,5 mm de
recorrido longitudinal neto del centro. No es una tolerancia calibrada del
encoder, una prueba de frenado ni una envolvente física de colisión.
Con una trayectoria variable o deslizamiento no garantiza esa posición real.

## Por qué se conserva 9,370° y termina toda la sesión

`Navigation_OdometryOutcome()` evalúa la discrepancia antes de actualizar
`progress` y `turn_mdeg`. Al rechazar, `Navigation_DiagnosticOdometry()` guarda
los conteos de la muestra fallida, pero el ángulo queda en el último valor
aceptado. Una muestra anterior de −246/+191 produce exactamente 9,370° al
redondear; es una reproducción compatible, no una muestra física recuperada.

Por tanto, no hay que usar 9,370° y −251/+195 como si fueran la misma muestra
angular válida. Con esos conteos nuevos el cálculo daría 9,563°, pero la
guarda impide aceptarlo como progreso fiable del giro.

La misma función se llama desde dos lugares:

1. `Navigation_ExploreStep()` termina con `Navigation_StopFor(ODOMETRY)`.
2. `Navigation_IsCommandCurrent()`, usado por MotorControl antes de aplicar
   consignas, rechaza la autorización y conserva la causa ODOMETRY.

`task_motor_control.c` limpia objetivos y revoca/cancela la sesión al recibir
ese rechazo. La regla recuperable actual para falta de progreso no se aplica
a un error ODOMETRY. Modificar solo el bucle de Navigation no resolvería el
rechazo anticipado del backend. El núcleo `mid_odometry` puede conservar
`state=2` porque una trayectoria diferencial asimétrica sigue siendo
matemáticamente válida; el rechazo es una guarda adicional de la maniobra.

## Recuperación evaluada, todavía no implementada

Detener y estabilizar es compatible con la arquitectura y necesario, pero
cuatro muestras quietas no prueban que un encoder esté sano ni que no haya
habido deslizamiento. Reiniciar referencias haría que la diferencia fuese
cero por construcción. Tampoco puede solicitarse otro giro completo de 10°
después de haber girado aproximadamente 9,56°.

Una propuesta posterior, a validar antes de implementarla, sería:

1. Solo para LOCAL_EXPLORE/GIRAR_ACOTADO y exclusivamente ante discrepancia,
   detener inmediatamente y revocar las consignas, conservando referencia,
   objetivo, plazo de 800 ms, magnitudes del rechazo y giro acumulado.
2. Aplicar la estabilización existente: cuatro muestras nuevas, 60 ms,
   rango máximo de una cuenta por rueda y timeout de 200 ms. STOP, Safety,
   error de signo, muestra inválida, overflow o pérdida del enlace prevalecen.
3. Recalcular desde la **referencia original**. Si la diferencia sigue siendo
   mayor que 3 mm, es persistente para esa fase y no se reanuda movimiento.
4. Solo si la coherencia se restablece con datos fiables podría evaluarse
   terminar el giro si ya alcanzó el objetivo o completar exclusivamente el
   ángulo restante. Una nueva referencia no debe borrar el progreso ni el
   desacuerdo acumulado original. No reiniciar el timeout.
5. Repetición del problema o ausencia de evidencia suficiente deben volver
   a una detención terminal; no convertir el restablecimiento de lecturas
   en una prueba de movimiento físico fiable.

A los 520 ms quedan 280 ms del presupuesto original. Una estabilización
de hasta 200 ms dejaría como máximo 80 ms antes de otras latencias: no se
puede garantizar un nuevo arranque y finalización en ese margen.
Esta propuesta exige definir y validar una política nueva de recuperación
de calidad de movimiento. No se elige una tolerancia mayor ni un retry ciego.

## Evidencia que falta y cómo obtenerla sin instrumental adicional

En una validación posterior, autorizada, conservar el mismo arranque STM32
y guardar antes de otra sesión:

- `/api/navigation`: referencia del giro en
  `turn_stabilization.reference_left_counts/reference_right_counts`,
  `reference_tick`, los deltas del rechazo y el estado terminal.
- `/api/map`: `pose.left_counts`, `pose.right_counts`, tick y antigüedad,
  después de que ambas ruedas se hayan detenido. Son strings int64: comparar
  contra la referencia original sin perder precisión.

Así puede determinarse si el desacuerdo desde la referencia original
permanece por encima de 3 mm o vuelve por debajo durante la detención.
STOP conserva el diagnóstico; una nueva sesión lo reemplaza. No reiniciar
placas entre esas capturas. No forzar manualmente las ruedas para alterar
los datos del evento. Las orientaciones IMU ya disponibles pueden aportar
comparación diagnóstica si tienen referencia y calibración válidas; no se
introducen en el control ni demuestran por sí solas la traslación del centro.

Si hace falta distinguir un salto de conteos de un crecimiento gradual,
puede proponerse después una captura RAM pequeña de las muestras previas
del giro, accesible por web y sin nueva tarea. No se implementa en esta etapa.
No se realizaron las pruebas físicas descritas ni se exige precisión milimétrica.

## Pruebas de esta revisión

Se añaden siete grupos al arnés real `Tests/safety_b11/test_local_exploration.c`:

1. Traza sintética de 520 ms que reproduce conteos y ángulo informado, con
   rechazo en la autorización real del backend, objetivos cero y sin Safety.
2. Convergencia aparente posterior al rechazo sin reactivación, referencia
   borrada ni consumo de una consigna antigua.
3. Muestras vecinas separadas por una cuenta, usando geometría de producción.
4. Asimetría transitoria inferior a 3 mm, recuperación y finalización normal.
5. Movimiento persistente de una sola rueda: detención terminal conservada.
6. Deslizamiento simulado con los mismos conteos: la odometría recibe los
   mismos datos y no distingue el movimiento físico real. Documenta una
   limitación de observabilidad, no una detección de deslizamiento implementada.
7. Recuperación mediante inicio explícito nuevo y estabilización; nunca
   reinicio automático de una sesión terminal.

Los conteos intermedios y la verdad física simulada existen solo en tests.
La reproducción no recupera la secuencia temporal física original.

Regresiones STM32: 685 operativas, 363 AUTO-1, 711+711 variantes de giro,
436 B7 y 55+55 giros históricos; odometría: 28. No se relajan tests de
Safety, STOP, enlace, motores ni perfiles anteriores.

Las compilaciones de ambos firmwares usan las configuraciones existentes.
Los logs, cálculos, diff incremental y manifiesto quedan en
`Firmware/stm32_control/build/local_turn_diagnosis/` (ignorado por Git).
No se grabaron placas, no hubo Git de escritura y no se modificó producción.
