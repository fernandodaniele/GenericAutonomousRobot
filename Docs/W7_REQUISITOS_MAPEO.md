# W7 pendiente obligatorio — Requisitos para mapeo 2D

W6 no implementa un mapa, odometría ni telemetría simulada. El mapa de ocupación
2D sigue siendo un objetivo obligatorio de la próxima etapa, sujeto a diseño
y autorización propios. Los datos actuales no bastan para ubicar obstáculos
de manera fiable en coordenadas del entorno.

## Datos existentes después de W6

| Fuente | Disponibilidad real | Limitación para mapeo |
| --- | --- | --- |
| Encoders TIM1/TIM2 | `drv_encoder.c`: `Encoder_ReadDelta()` lee diferencia firmada y actualiza `last_count`; TIM1 16 bits, TIM2 32 bits. `TaskMotorControl_Run()` consume ambos cada 10 ms y calcula cps con conversión protegida contra overflow. | No hay acumulados, pose ni telemetría pública de odometría. Un segundo consumidor del mismo handle robaría diferencias al PI. |
| HC-SR04 frontal/trasero | `TaskSensores_GetData()` copia la última instantánea coherente; validez, resultado disponible y timestamps independientes. Adquisición alternada, mínimo 70 ms entre disparos, nominal 140 ms por sensor, timeout 50 ms. | No simultáneos; timestamps de disparo/resultado según cada campo, no un instante de impacto exacto. Haz de apertura amplia, reflexiones y ausencia de observación lateral. |
| IR TCRT5000 | Cuatro estados normalizados, 1=suelo, 0=sin detección, timestamp IR cada 20 ms y condición agregada de caída. | Detectan suelo/caída, no distancia horizontal de obstáculos. No sustituir Safety. |
| MPU6050 | `main.c` crea `TaskIMU`; `task_imu.c` intenta inicialización, lee cada 20 ms y publica por cola; `TaskIMU_GetLatest()` copia sin consumir. Lectura fallida vacía la cola y reinicia con espera de 1000 ms. `MPU6050_t` contiene aceleración, giro, temperatura y Kalman X/Y. | No se publica por W5; no tiene timestamp/frescura explícitos. No hay rumbo absoluto, magnetómetro ni fusión de pose. Presencia de código no acredita validación física del IMU. |
| SENSORS/HTTP | W5 conserva mm, estados de adquisición y edades; consulta cada 500 ms cuando no hay control prioritario, pausada durante sesión manual. | Edad recibida y latencia de transporte; no hay pose asociada ni sincronización de relojes. La cadencia web no define la cadencia de adquisición. |

## Parámetros físicos por medir y documentar

- Radios efectivos de ambas ruedas bajo carga y separación efectiva entre sus
  contactos. `mid_kinematics.h` declara `ROBOT_WHEEL_BASE=200.0f`, pero es un
  valor de código, no geometría calibrada; esa capa no es la ruta actual de cps.
- Cuentas efectivas por vuelta de rueda, decodificación cuadratura y relación
  de transmisión. Las consignas ±1600 cps no son una velocidad lineal calibrada.
- Ejes de referencia, signos de avance/giro y origen de pose. Conservar el
  cableado y signos de encoder validados; no añadir inversiones arbitrarias.
- Posición y orientación real de cada HC-SR04 respecto al centro del robot,
  altura, apertura útil y zonas ciegas. Frontal y trasero son orientaciones
  nominales opuestas, no extrínsecos medidos. Medir también huella del chasis
  y volumen barrido en giros, sin atribuir cobertura lateral a esos sensores.
- Montaje/ejes del IMU, sesgos, ruido y estabilidad térmica, antes de fusionarlo.

## Pose y asociación temporal que faltan

La futura odometría deberá reutilizar deltas ya leídos por MotorControl sin
volver a consumir el encoder, manteniendo coherencia, acumulados o incrementos,
secuencia y timestamp de origen. Con ruedas calibradas, la base diferencial es
`ds=(dl+dr)/2` y `dtheta=(dr-dl)/b`; integrar una pose relativa `(x,y,theta)` con
convención explícita. No está implementado en W6.

Cada distancia debe asociarse a la pose correspondiente a su adquisición,
no a la posición al refrescar HTTP. Definir historial temporal acotado,
interpolación y tratamiento de muestras inválidas o fuera de la ventana. Usar
ticks compatibles con desbordamiento, identidad de arranque/secuencia y
conversión temporal explícita; no comparar directamente relojes ESP32 y STM32.
Determinar resolución y frecuencia de odometría/telemetría con presupuesto UART,
preservando STOP, DETENER, autorizaciones manuales, STATUS y Safety.

## Límites y validación necesaria

La odometría deriva por deslizamiento, holgura y diferencias de rueda. Un IMU
sin referencia externa tampoco elimina la deriva de rumbo. Los dos HC-SR04 no
identifican de forma única la superficie que produjo un eco y no equivalen a
un lidar puntual. Un mapa deberá expresar incertidumbre y espacio desconocido;
no convertir zonas sin muestra en libres ni generar mapas ficticios.

Antes de mapear: pruebas automatizadas de deltas/signos/wrap, pose y asociación
de timestamps; luego, con autorización, medir geometría y calibrar distancia y
giro frente a referencias reales. Definir reinicio del mapa, tamaño/resolución,
coste de memoria y criterios de ocupación en W7. Estos requisitos no autorizan
modificar motores, sensores o navegación durante W6.
