# IMU-3: orientación relativa y comparación con encoders

Alcance de este documento: estimaciones independientes de IMU-3. La extensión
[IMU-4](IMU_4_FUSION_EXPERIMENTAL.md) añade una tercera estimación diagnóstica
complementaria, conservando las estimaciones y el protocolo aquí descritos.

Implementación de software, pendiente de validación física de esta etapa. No
controla motores, no fusiona sensores y no modifica la odometría ni el mapa.

## Integración y referencia

`TaskIMU` conserva su tarea, prioridad, stack y período nominal de 20 ms. Usa
exclusivamente `Gyro_Z_RAW`, el offset medido por IMU-2 y los ticks reales de
inicio de las lecturas I²C exitosas. No usa los ángulos Kalman del driver ni
los intervalos de consulta UART/HTTP.

```
bias_z = suma_z / 250
w_k = (Gyro_Z_RAW_k - bias_z) / 131             [grados/s]
dt = (TickType_t)(tick_k - tick_anterior) / configTICK_RATE_HZ
angulo_k = angulo_anterior + (w_anterior + w_k) * dt / 2
```

El ángulo es relativo, acumulado sin normalizar y almacenado en `double`.
Izquierda/antihorario es positivo; derecha/horario es negativo, de acuerdo
con la polaridad Z confirmada físicamente. Se presupone rotación del chasis
aproximadamente horizontal: no es una estimación Euler 3D ni rumbo absoluto.
No hay zona muerta, filtro nuevo ni corrección automática de deriva.

Durante la calibración no se integra. Al completar exactamente la calibración
existente de 250 muestras, se pone a cero el ángulo y se usa esa última muestra
como primer extremo del siguiente trapecio. Una solicitud rechazada o una
ventana fallida no establecen una orientación válida.

La referencia se identifica con `reference_request` y `reference_tick` de STM32,
válidos dentro de ese arranque. El tick cero es legítimo. Un reinicio pierde
calibración y referencia; estos valores no son identificadores persistentes.

## Continuidad y validez

`IMU_YAW_MAX_GAP_MS` reutiliza los 40 ms de continuidad ya exigidos por IMU-2.
Se aceptan intervalos positivos de hasta 40 ms inclusive, calculados por resta
sin signo compatible con wrap de `TickType_t`. Un tick duplicado, un intervalo
mayor, una falla I²C, pérdida/cambio de calibración o reinicialización invalidan
la orientación. No se integra el intervalo ausente y no se reanuda con la
siguiente muestra: se requiere una calibración explícita nueva y exitosa.

La representación UART limita el ángulo al rango de miligrados de un `int32_t`
con margen de conversión. Si excede ese rango o deja de ser finito, se invalida;
no se envuelve ni satura el ángulo. RAW y velocidades corregidas siguen
disponibles según sus reglas previas, incluso si solo se perdió continuidad
de integración. Se conserva la edad de la última muestra utilizada cuando
existe referencia, pero una orientación inválida se serializa sin ángulo.

Un dato remoto viejo se oculta por las reglas existentes de telemetría
(1500 ms), sin confundir pérdida HTTP/UART con pérdida de calibración de STM32.
Si la tarea deja de publicar, el consumidor puede detectar desactualización;
no se afirma detección instantánea de una CPU detenida.

## Referencia común de comparación

Se consulta `Odometry_PoseAt(reference_tick)` sin reiniciar ni actualizar la
odometría. Para cada comparación se conserva una sola muestra IMU anterior
(ángulo y tick), de modo que el historial del lazo de motores de 10 ms pueda
encerrar ese tick. No se extrapola ni se usa la pose presente para una muestra
antigua. La comparación tiene su propio `sample_tick` y antigüedad; normalmente
es unos 20 ms anterior al ángulo acumulado mostrado.

```
delta_encoder = wrap180((theta_encoder(t) - theta_encoder(t_ref)) * 180/pi)
delta_gyro    = wrap180(angulo_gyro(t))
diferencia    = wrap180(delta_gyro - delta_encoder)
```

`wrap180` representa cambios módulo 360° en [-180°,180°). Evita saltos al cruzar
la discontinuidad ±pi de odometría, pero no permite comparar cantidades de
vueltas completas. `encoder_reference_deg` informa el origen absoluto de
encoders que se descontó; la GUI compara los deltas, nunca ángulos con orígenes
diferentes. La diferencia es entre estimaciones, no un error contra una verdad
física conocida. Se indica además si la geometría de encoders es provisional.

Si falta pose válida/interpolable para la referencia o la muestra, la
comparación no es válida aunque el giroscopio sí pueda seguir integrando.
La referencia original no se reemplaza por una pose posterior. Si no apareció
dentro del máximo horizonte posible del historial existente (64 × 50 ms),
se deja de buscar hasta recalibrar, evitando alias de ticks en el futuro.

## UART compatible

`IMU Q=N` y `IMU2 Q=N` conservan exactamente las respuestas v1/v2 y la orden
de calibración `IMUCAL N=N` no cambia. ESP32 consulta `IMU3 Q=N` para obtener
la extensión v3; sigue aceptando v1/v2 (orientación no disponible).

Una línea v3 contiene `IMU,` y 34 campos separados por comas, terminada en LF
(el parser también acepta CRLF). Los campos 0–19 son los mismos de IMU2 salvo
el número de versión 3. Las ausencias se representan con `-`, no con cero.

| Índice | Campo | Unidad / significado |
|---|---|---|
| 0 | versión | 3 |
| 1 | Q | identificador de consulta existente |
| 2–5 | estado, HAL, error I²C, edad RAW | contratos IMU-1, edad en ms |
| 6–11 | AX, AY, AZ, GX, GY, GZ | RAW con signo |
| 12–19 | estado, motivo, muestras, solicitud, edad, sumas X/Y/Z de calibración | contrato IMU-2 |
| 20 | estado orientación | 0 pendiente, 1 válida, 2 inválida |
| 21 | motivo orientación | 0 ninguno, 1 calibración, 2 I²C, 3 tiempo, 4 numérico, 5 reinicialización |
| 22 | tick referencia | tick STM32, o `-` si no hubo referencia |
| 23 | edad última muestra integrada | ms, o `-` si no hubo referencia |
| 24 | orientación acumulada | miligrados con signo, solo si válida |
| 25 | estado comparación | 0 espera, 1 válida, 2 odometría no disponible, 3 orientación no válida |
| 26–27 | edad y tick comparación | ms y tick STM32, solo si válida |
| 28–31 | delta gyro, delta encoder, diferencia, origen encoder | miligrados con signo, solo si comparación válida |
| 32 | geometría encoder | 1 provisional, 2 calibrada; `-` sin comparación |
| 33 | solicitud que originó la referencia angular | entero sin signo, o `-` sin referencia |

Todas las edades se calculan desde un único tick al construir la respuesta
coherente. El máximo conservador de 240 bytes con LF cabe en los buffers de
256 bytes y en una lectura UART de 255 bytes; no cambia el timeout de 200 ms
de correlación. La cuantización de ángulos se limita a la telemetría, con
truncamiento inferior a 0,001°. No se retroalimenta al integrador.

Se conservan la consulta de 500 ms, único lector UART, mutex TX y pausa durante
sesiones manuales. STOP, DETENER y operaciones conservan prioridad sobre
telemetría. En STM32, el mailbox permite que SENSORS/MAPPING desplacen una
consulta IMU pendiente; en ESP32 se conserva la consulta IMU antes del siguiente
fragmento de mapa para evitar inanición. Los handlers HTTP no envían nuevas
órdenes UART por esta extensión.

## GET /api/imu y dashboard

Se conservan todos los campos IMU-1/IMU-2. Se agregan:

* `orientation`: `available`, `state` (`unavailable/pending/valid/invalid`),
  `valid`, `requires_calibration`, `reason`, `degrees`, `sample_age_ms`,
  `reference_tick`, `reference_request`.
* `comparison`: `state` (`unavailable/waiting_reference/valid/odometry_unavailable/yaw_unavailable`),
  `valid`, `sample_tick`, `sample_age_ms`, `gyro_delta_deg`, `encoder_delta_deg`,
  `difference_deg`, `encoder_reference_deg`, `encoder_calibration` (0/1/2).

Los ángulos son `null` si no son utilizables. `state` conserva el estado
informado por STM32; `valid` exige además comunicación STATUS e IMU vigente.
`requires_calibration` se refiere al integrador de STM32, no a un fallo de Wi-Fi.
Las edades incluyen RTT conservador y tiempo transcurrido en ESP32, sin
restar relojes de placas diferentes. No se muestran valores antiguos como
orientaciones actuales. El panel vigente muestra el estado de la IMU y la
calibración manual. Consultar orientación acumulada, referencia, deltas comunes
y su diferencia en `/api/imu`.

## Validación física posterior, supervisada

1. Programar ambas placas por separado y reinstalarlas en la PCB, según el
   procedimiento del usuario. Esta implementación no programa hardware.
2. Con batería, Wi-Fi y chasis inmóvil, mantener el robot en En espera o
   Detenido por seguridad. No solicitar MANUAL/AUTO ni emitir movimientos.
3. Abrir el dashboard y `/api/imu`; calibrar explícitamente. Esperar calibración
   válida, `orientation.valid=true`, nueva referencia y ángulo cercano a cero.
4. Esperar `comparison.valid=true`. Registrar ambos deltas, tick de comparación
   y referencia. Si no hay pose válida, no comparar valores de orígenes distintos.
5. Girar manualmente el chasis sobre el suelo aproximadamente 90° a izquierda
   con ambas ruedas en contacto, sin accionamiento eléctrico; comprobar signo
   positivo en ambos deltas. Una consulta HTTP lenta no define el tiempo de
   integración; esperar datos recientes al terminar el giro.
6. Volver aproximadamente al punto angular inicial y observar residuo/deriva.
   Repetir a derecha (signo negativo). No se promete igualdad exacta: hay ruido,
   deriva térmica, resolución temporal, incertidumbre geométrica y deslizamiento.
7. Si `requires_calibration=true`, inmovilizar y recalibrar. No unir series con
   referencias diferentes ni interpretar recuperación de Wi-Fi como recalibración.

No usar estas mediciones para comandar movimiento todavía. El
stack real, tiempos bajo carga y deriva térmica permanecen por validar en PCB.
