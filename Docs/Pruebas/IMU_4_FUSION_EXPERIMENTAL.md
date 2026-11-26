# IMU-4: fusión complementaria experimental

Tercera estimación exclusivamente diagnóstica. No interviene en Safety, FSM,
navegación, PI/PWM, odometría original, posición o mapa. No cambia geometría,
CPR, offsets ni criterios de calibración/continuidad de IMU-2 e IMU-3.

## Arquitectura y tiempos

El filtro pertenece a la tarea existente `TaskIMU`, de 20 ms nominales. Reutiliza
el ángulo integrado de IMU-3 y su comparación con `Odometry_PoseAt()`. Las dos
fuentes corresponden al **mismo tick de adquisición**; normalmente se procesa
la muestra IMU anterior, para que el historial de encoders pueda interpolarla.
No agrega adquisiciones, tareas ni lecturas de encoder. No usa tiempos UART/HTTP
para integrar, ni modifica `mid_odometry`.

La referencia es el tick de finalización de una calibración IMU-2 exitosa,
identificado además por su solicitud. Dentro de ese arranque, los tres ángulos
comunes comienzan desde cero; la pose original de encoders no se reinicia.
Se espera la primera asociación válida. Si esta primera asociación ya requiere
un intervalo mayor de 40 ms desde la referencia, se rechaza en vez de inventar
una historia inicial. También se rechazan comparaciones futuras, atrasadas más
de 40 ms respecto de la adquisición actual o intervalos de fusión nulos/>40 ms.

## Ecuaciones y parámetro experimental

En grados, izquierda positiva y derecha negativa, como IMU-3. `G_k` es el
ángulo giroscópico continuo ya integrado con el Z corregido. `E_k` se obtiene
desenvolviendo los cambios de la orientación relativa de encoders:

```
E_k = E_anterior + wrap180(encoder_relativo_k - encoder_relativo_anterior)
prediccion = F_anterior + (G_k - G_anterior)
dt_ms = (TickType_t)(tick_k - tick_anterior) * 1000 / configTICK_RATE_HZ
alpha_k = tau_ms / (tau_ms + dt_ms)
innovacion = wrap180(E_k - prediccion)
F_k = prediccion + (1 - alpha_k) * innovacion
```

`IMU_FUSION_TAU_MS = 1000U` está centralizado en `task_imu.h`. A 20 ms,
`alpha ≈ 0,980392`: se conserva la respuesta rápida de la predicción del
giroscopio y se corrige hacia encoders en una escala aproximada de un segundo.
La inicialización informa ese coeficiente nominal; después se informa el
coeficiente de la última actualización real. Se recalcula con cada `dt`, sin
suponer que todos los ciclos duran exactamente 20 ms.

Es un punto de partida experimental, no un coeficiente identificado con un
modelo de ruido ni optimizado a partir de los errores físicos anteriores.
No se alteran ganancias de motores. No hay Kalman ni nueva compensación de
bias; el giroscopio mantiene los offsets medidos por las 250 muestras de IMU-2.

Los tres ángulos se conservan continuos en `double`. Solo la innovación y las
diferencias `wrap180(F-G)` y `wrap180(F-E)` se reducen a [-180°,180°). Se conserva
así la continuidad al cruzar ±180° y en varias vueltas. Desenvolver encoders
supone menos de 180° entre muestras asociadas; exactamente 180° se rechaza por
ambigüedad de signo. Sin observación adicional no se puede descubrir una vuelta
completa omitida entre muestras. Se aplican guardas finitas y de rango para
evitar conversiones fuera del rango de miligrados de `int32_t`.

La corrección no garantiza eliminar deriva, deslizamiento o errores de geometría.
Por ejemplo, con encoders quietos y deriva giroscópica constante de 1°/s, esta
configuración converge a un residuo próximo a 1°, no a cero. Un valor intermedio
entre fuentes no demuestra por sí mismo una mejora de exactitud.

## Estados y recuperación

| Estado STM32 | Significado | Ángulo publicado |
|---|---|---|
| 0 pending | Espera calibración/referencias/muestras sincronizadas | Ausente |
| 1 valid | Ambas fuentes válidas y actualización complementaria coherente | Presente |
| 2 degraded | Se perdió una pose de encoders asociable tras inicializar | Ausente |
| 3 invalid | IMU/calibración perdida, tiempo, referencia o límite numérico | Ausente |

Motivos: 0 ninguno, 1 IMU, 2 encoder, 3 tiempo, 4 numérico, 5 referencia.
Los estados degraded/invalid quedan enclavados en el estimador hasta una
**nueva calibración explícita y exitosa**; recuperar I²C o una pose no reanuda
la integración ni une intervalos ausentes. Esto no es un enclavamiento de
Safety y no cambia el movimiento permitido por el robot.

No existe fallback automático solo con encoders ni solo con giroscopio: el
campo `encoder_only` es siempre falso. Las fuentes originales siguen
disponibles por sus interfaces anteriores. La última referencia/tick/edad de
fusión se conservan para diagnóstico cuando existieron; sus ángulos se ocultan
al invalidarse. Recalibrar limpia ese historial y crea una referencia nueva,
sin saltar artificialmente una serie anterior ni modificar la odometría.

## Telemetría compatible y concurrencia

IMU-1/2/3 conservan sus consultas y respuestas exactas. La consulta adicional
`IMUF Q=N` es de solo lectura y responde una línea ASCII `FUS,` con 16 campos:

| Índice después de FUS | Contenido |
|---|---|
| 0–3 | versión 1, Q, estado, motivo |
| 4–7 | solicitud de referencia, tick de referencia, tick de muestra, edad en ms |
| 8–10 | ángulo gyro, ángulo encoder, ángulo fusionado; miligrados con signo |
| 11–12 | fusionado menos gyro/encoder, módulo 360°; miligrados con signo |
| 13–15 | alpha en partes por millón, tau en ms, geometría encoder (1 provisional/2 calibrada) |

Los ausentes son `-`, nunca mediciones inventadas de cero. Los ángulos solo
están presentes en estado valid. La línea cabe holgadamente en 192 bytes y no
alarga las tramas IMU-3 de hasta 240 bytes. Se conserva el plazo de respuesta
correlacionada de 200 ms y la tolerancia a LF/CRLF y fragmentación.

ESP32 consulta IMU y fusión independientemente, cada una no antes de 500 ms,
sin recuperar consultas atrasadas en ráfaga. Fusión va después de STATUS,
SENSORS, calibración e IMU y antes del siguiente fragmento de mapa. Conserva
el mutex TX, revalidación STOP/DETENER, pausa durante sesiones manuales y único
lector UART. El mailbox STM32 clasifica `IMUF` como diagnóstico de baja prioridad.
No hay transmisiones UART desde el endpoint HTTP de lectura.

`GET /api/imu` agrega un objeto `fusion`, preservando todos los campos previos:

* `experimental`, `received`, `connected`, `valid`, `state`, `reason`,
  `requires_calibration`, `encoder_only`.
* `reference_request`, `reference_tick`, `sample_tick`, `sample_age_ms`.
* `alpha`, `tau_ms`, `gyro_degrees`, `encoder_degrees`, `degrees`,
  `difference_gyro_deg`, `difference_encoder_deg`, `encoder_calibration`.

Las cinco cantidades angulares son `null` si no son utilizables. La API exige
también IMU/STATUS recientes y la misma referencia observada por IMU-3. Las
edades incluyen RTT conservador y tiempo transcurrido en ESP32; no se comparan
los relojes de placas distintas. Una respuesta IMU normal no rejuvenece una
respuesta de fusión antigua. Una pérdida de red no recalibra ni reinicia el
filtro; solo invalida su presentación hasta recibir datos recientes.

El panel vigente muestra la calibración manual y su resultado. Para comparar
ángulos, consultar el grupo temporal común de `fusion` en `/api/imu`. La
orientación de IMU-3 más reciente puede corresponder a otro tick. La geometría
de los encoders conserva su identificación provisional.

## Validación física posterior sin instrumentos angulares externos

Después de revisión y programación supervisada de ambas placas, reinstalar en
PCB y trabajar por batería/Wi-Fi. No hace falta monitor UART externo ni USB.
La implementación de esta etapa no programa placas ni ejecuta pruebas físicas.

1. Mantener el robot inmóvil en IDLE/SAFETY_STOP y calibrar desde la web. Esperar
   `fusion.valid=true`; registrar la referencia y los tres ángulos del objeto
   `fusion` en un mismo JSON. Son el grupo temporal común que se debe comparar.
2. Mantener reposo durante aproximadamente un minuto. Anotar deriva aparente de
   cada fuente y de la fusión, estabilidad y antigüedad. No interpretar solo una
   menor separación de fuentes como exactitud demostrada.
3. Girar manualmente el chasis unos 90° a izquierda, estimados visualmente, con
   las ruedas en contacto con el suelo. Esperar una actualización reciente al
   detenerse y registrar los tres valores. Comprobar signo positivo y continuidad.
4. Girar a derecha para volver aproximadamente al origen. Registrar residuos de
   cada estimación y repetir varias veces. La imprecisión del retorno manual
   forma parte de la comparación; no declarar como error instrumental exacto
   una diferencia de pocos grados.
5. Realizar un desplazamiento manual aproximadamente recto y giros adicionales,
   observando marcas o direcciones visuales existentes como referencias. Evaluar
   coherencia, ausencia de saltos, retorno y repetibilidad, sin exigir 90° exactos
   ni agregar instrumentos. Los ángulos del mapa no se alteran por la fusión.
6. Si se usan posteriormente movimientos MANUAL ya autorizados, la telemetría
   se pausa como antes durante la pulsación: soltar/DETENER y esperar datos
   recientes antes de anotar. La integración interna no depende de esa pausa.
7. Ante degraded/invalid, detener la prueba, inmovilizar y recalibrar; confirmar
   nueva referencia y cero. No empalmar series de calibraciones distintas.

La mejora se evalúa por reposo más estable, menor deriva/residuo repetible y
mejor coherencia con el retorno visual, considerando la incertidumbre manual.
Si solo aparece un valor intermedio o no hay una tendencia reproducible, la
mejora física queda sin demostrar; el filtro sigue siendo experimental. Esto
no impide continuar las pruebas ni autoriza usarlo para controlar movimiento.

También quedan por medir tiempos bajo carga, uso real del stack, efecto de
temperatura, deslizamiento y deriva prolongada. Los tests sintéticos verifican
el algoritmo y las barreras de validez, no esos resultados físicos.
