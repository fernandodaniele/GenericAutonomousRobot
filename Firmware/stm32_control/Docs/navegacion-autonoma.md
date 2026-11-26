# Navegación autónoma — etapas 0 a 2

Implementación del plan «Plan de cambios para la navegación autónoma — Pruebas_Grupo2».
El ESP32 y el datalogger no se tocan: USART2, la SD y el SPI1 quedan igual.

## Qué se agregó

| Etapa | Cambio | Archivos |
|---|---|---|
| 0 | Heap de FreeRTOS de 15 360 B a 32 768 B | `Core/Inc/FreeRTOSConfig.h` |
| 0 | `HAL_UART_ErrorCallback` rearma la recepción (una ráfaga ya no deja muerta la UART) | `Core/Src/main.c` (USER CODE 4) |
| 0 | CI también en `Pruebas_Grupo2` | `.github/workflows/main_ci.yml` |
| 1 | `Ultrasound_ReadStatus()`: OK / sin eco / eco largo (libre) / demasiado cerca | `drv_ultrasound.c/.h` |
| 1 | `task_sensors`: un HC-SR04 por turno cada 60 ms; TCRT5000 y giro Z del MPU6050 (I2C3, 3 reintentos) cada 20 ms | `task_sensors.c/.h` |
| 1 | Umbrales del TCRT5000 redefinibles con `-D` para calibrar | `drv_TCRT5000.c` |
| 2 | `mid_nav`: máquina de 5 estados sin HAL | `mid_nav.c/.h` |
| 2 | `Kinematics_Compute`: mm/s y rad/s → PWM | `mid_kinematics.c/.h` |
| 2 | `task_nav`: 20 ms, rampa, freno activo, freno si la foto tiene más de 300 ms | `task_nav.c/.h` |
| — | `START` / `STOP` por UART y eventos `EVT,<ms>,ALERTA|CLIFF|FAULT` en `defaultTask` | `freertos.c` |
| — | Prueba de `mid_nav` en la PC | `Test/test_mid_nav.c` |

## Arranque

Con `TASK_NAV_AUTOSTART = 1` (valor actual, `Core/Inc/task_nav.h`) el robot arranca
**solo, 3 s después de encenderse** (`TASK_NAV_AUTOSTART_DELAY_MS`). `START` / `STOP`
por UART (o desde el celular a través del ESP32) siguen funcionando. Con
`TASK_NAV_AUTOSTART = 0` queda en `IDLE` hasta recibir `START`.

## Navegación lenta y coherente (noviembre 2026)

### Movimiento
- **PWM proporcional** (`mid_kinematics.c`): cada rueda recibe un PWM proporcional a los
  mm/s pedidos, con un piso de arranque `ROBOT_WHEEL_MIN_PWM`: la rueda recibe 0 o al
  menos ese valor. Reemplaza el antiguo todo/nada al 80 %.
- **Velocidad única `NAV_SPEED_PCT`** (50 %): avance, retroceso y giros van a ese
  porcentaje del PWM máximo (PWM 4200). Para cambiarla se toca solo esa macro. El piso
  `ROBOT_WHEEL_MIN_PWM` (3000, provisorio) tiene que quedar por debajo de 4200; si no,
  el 50 % no se respeta.
- **Pausa `NAV_SETTLE_MS`** (300 ms, con freno) entre frenar y retroceder, entre
  retroceder y girar y entre girar y avanzar.

### Reglas de la máquina de estados
| Regla | Parámetro |
|---|---|
| El borde se confirma con 3 lecturas nuevas seguidas antes de `CLIFF_RECOVER` | `NAV_CLIFF_CONFIRM` |
| 3 `CLIFF_RECOVER` seguidos sin 1,5 s de `FORWARD` → `FAULT` frenado + `EVT,FAULT` | `NAV_CLIFF_LOOP_MAX`, `NAV_CLIFF_LOOP_RESET_MS` |
| Ese `FAULT` **no se recupera solo**: hay que mandar `STOP` y luego `START` (o resetear la placa). LD3 da 4 destellos | — |
| Lado sin eco al elegir el giro: 0 = espacio libre 0 (actual), 1 = "desconocido" | `NAV_NO_ECHO_AS_UNKNOWN` |

### Recorrido por tramos
No hay giros "buscando" espacio libre: cada decisión se toma **una sola vez**, con el robot
quieto.

1. Avanza recto a `NAV_SPEED_PCT` (50 %), manteniendo el rumbo con el giroscopio.
2. Frena cuando el frente baja de `NAV_OBSTACLE_MM` (30 cm; con la latencia de lectura
   debería quedar cerca de 25 cm) o al cumplir `NAV_STEP_MS` (3 s), o si un lateral queda a 5 cm o menos. Con `NAV_STEP_MS = 0`
   corre hasta la pared.
3. Espera quieto (`NAV_BRAKE_MS` + `NAV_SETTLE_MS`) y toma `NAV_SCAN_SAMPLES` lecturas por
   ultrasonido, con un máximo de `NAV_SCAN_MAX_MS`. Usa la mediana.
4. Decide con esas medianas:

| Frente | Lados | Acción |
|---|---|---|
| Libre (≥ `NAV_CLEAR_MM`, 35 cm) | Cualquiera | Otro tramo recto, con el mismo rumbo de referencia |
| Bloqueado | Derecha con ≥ `NAV_SIDE_FREE_MM` (40 cm) | Pivota 90° a la derecha y sigue recto |
| Bloqueado | Solo la izquierda con ≥ 40 cm | Pivota 90° a la izquierda y sigue recto |
| Bloqueado | Ninguno | Retrocede `NAV_SCAN_BACK_MS` y vuelve a medir (hasta 5 veces, luego `FAULT`) |

El colchón entre 30 y 35 cm evita que vuelva a frenar apenas arranca. Después de retroceder
vuelve a medir antes de moverse. El frontal sin ninguna lectura válida cuenta como bloqueado:
el robot no avanza a ciegas. Un lateral sin eco no cuenta como libre.

Con `NAV_SCAN_STUCK_MAX` (6) maniobras seguidas sin avanzar 1,5 s, entra en `FAULT` frenado
(4 destellos, salir con `STOP` y `START`).

### Giros por ángulo
Los giros **terminan por ángulo**, leído del rumbo del MPU6050. **Prueba actual:** en el panel,
un giro físico de ~90° se lee como ~10°, así que se usa `NAV_TURN_ANGLE_MDEG = 10000` (10° leídos)
con `NAV_TURN_COAST_MDEG = 0`. Si el factor se confirma con las 10 lecturas a cada lado, hay que
corregir la escala del giroscopio y volver a 90°. `NAV_TURN_TIMEOUT_MS` (1 s) es el tope de
seguridad: a ~60°/s, un giro que no corta por ángulo no queda dando vueltas.
`NAV_REQUIRE_IMU = 1`: sin MPU6050 el robot queda en `FAULT` (1 destello).

- **Todos los giros (escaneo y precipicio) son en el lugar:** cada rueda al 50 % en sentidos
  opuestos (`NAV_TURN_SPOT_RAD_S`). Se dejó el pivote: barría un círculo de radio B y chocaba
  de costado.

### Límite lateral
- Avanzando, si un lateral está a `NAV_SIDE_STOP_MM` (5 cm) o menos, frena y pasa directo a SCAN.
- Girando, si el lateral del lado del giro o el frente quedan a 5 cm o menos, corta el giro y
  vuelve a medir.
- Se usa la **última lectura propia** de cada lateral (se miden 1 de cada 4 fotos).
- `NAV_SIDE_BAD_MAX` (3) lecturas seguidas sin eco o `TOO_CLOSE` de un lateral también cuentan
  como bloqueado (`us_bad_count` en `task_sensors`).

### Precipicio
Retrocede lo mínimo (`NAV_CLIFF_BACKUP_MS`), gira 90° en el lugar alejándose del sensor que vio
el borde (con los dos, a la derecha) y vuelve a medir. Si al terminar el giro sigue viendo
borde, retrocede otra vez. Los ultrasonidos laterales no se usan para elegir el lado: no ven
el vacío y lo leen como espacio libre.

### Valores provisorios a verificar en el piso
| Macro | Valor | Qué medir |
|---|---|---|
| `ROBOT_WHEEL_MIN_PWM` | 3000 | Las dos ruedas arrancan desde parado con ese PWM; si no, subirlo (máximo 4000) |
| `NAV_CLIFF_BACKUP_MS` | 600 ms | El mínimo con el que ambos TCRT quedan sobre el piso, más ~1 cm |
| `NAV_SCAN_BACK_MS` | 1000 ms | Retroceso con todo bloqueado (atrás no hay sensores) |
| `NAV_TURN_COAST_MDEG` | 6000 | Cuántos grados se pasa de 90° en el campo `rumbo` |
| `NAV_TURN_TIMEOUT_MS` | 5000 ms | 1,5 veces lo que tarda un giro normal de 90° |
| `NAV_OBSTACLE_MM` | 300 mm | Que pare a ~25 cm de la pared (medir con regla) |

### Arranque suave
Desde parado, cada rueda salta a `TASK_NAV_SOFT_START_PWM` (4000, por debajo del PWM con el
que empieza a girar) y sube de a `TASK_NAV_RAMP_UP_STEP_PWM` (40) cada 20 ms: llega al 70 %
en casi 1 s. Frenar e invertir el sentido siguen siendo rápidos (`TASK_NAV_RAMP_STEP_PWM`).
Valores en `Core/Inc/task_nav.h`.

### Rumbo con el MPU6050
El MPU6050 (I2C3) está activo (`SENSORS_USE_IMU = 1`). Al conectarse mide el offset del giro Z
durante 1 s: **el robot tiene que estar quieto** ese segundo (el panel muestra "Calibrando").
Después integra el giro Z cada 20 ms y obtiene el rumbo.

Si al encender el MPU6050 no responde, la STM32 lo reintenta cada 2 s sin frenar la lectura de
los demás sensores. Esto cubre el caso de un módulo que recibe alimentación después que la STM32,
por ejemplo desde la batería de los motores. Si se conecta con el robot en marcha, calibra en
movimiento y el rumbo puede desviarse: conviene encender todo junto.

En cada tramo recto el robot mantiene un rumbo de referencia que se toma al recibir `START`,
al salir de `FAULT` y al terminar cada giro. Cortar y retomar un tramo **no** lo cambia: así el
desvío de una frenada no se vuelve el nuevo "recto". Corrige proporcionalmente al desvío
(`NAV_HEADING_KP`), con un máximo de `NAV_HEADING_MAX_RAD_S` y sin corregir desvíos menores a
2° (`NAV_HEADING_DEADBAND_MDEG`). Si el MPU6050 deja de responder, el robot entra en `FAULT`.
La corrección se desactiva con `NAV_HEADING_HOLD = 0`.

**Verificar el signo antes de la primera prueba:** con el robot quieto, girarlo a mano hacia
la izquierda (antihorario visto desde arriba). En el panel, "Giro Z" tiene que dar positivo y
el rumbo tiene que aumentar. Si da negativo, el módulo está montado al revés: poner
`SENSORS_GYRO_Z_SIGN = -1` en `task_sensors.c`. Con el signo equivocado, el robot gira cada
vez más en vez de enderezarse.

**Conexión** (I2C1, módulo GY-521): VCC a 3V3, GND a GND, SCL a **PB6**, SDA a **PB9**, AD0 a
GND o sin conectar (también se acepta AD0 en alto, dirección 0x69). En la Discovery el I2C1
también llega al codec de audio de la placa, que ya trae pull-ups de 4,7 kΩ y usa otra
dirección. No usar PB8 como SCL: es el PWM del motor derecho.

Para volver al I2C3 (SCL en **PA8**, SDA en **PC9**, con las pull-ups internas del STM32 de
unos 40 kΩ) poner `SENSORS_IMU_ON_I2C1 = 0` en `task_sensors.c`.

Al arrancar, la STM32 libera el bus I2C por si quedó trabado tras un reset y acepta los
clones compatibles (MPU6500 / MPU9250, que responden con otro identificador). Si no logra
usarlo, el panel indica la causa en "Giro Z":

| Mensaje en el panel | Causa probable |
|---|---|
| SDA o SCL en bajo | Módulo sin alimentación en ese momento, un cable a masa o en un pin equivocado |
| Calibrando | Conectado: mantener el robot quieto 1 s |
| No responde por I2C | SCL y SDA invertidos o en otros pines, o sin GND común |
| Responde con ID 0x..: modelo no reconocido | Otro sensor: pasar ese número para agregarlo |
| Respondía y dejó de responder | Conexión floja o cables largos |

### LED naranja (LD3)
| LED | Estado |
|---|---|
| Apagado | `IDLE` |
| Fijo | `FORWARD` |
| Parpadeo lento | `SCAN`: midiendo o maniobrando entre tramos |
| Parpadeo rápido | Recuperándose de un borde |
| 1 / 2 / 3 / 4 destellos | `FAULT`: MPU6050 / frontal sin eco / sensores sin actualizar / atrapado (bucle de retroceso o de escaneo) |

## Panel web de verificación

Conectarse a la red WiFi `GAR-Robot` (clave `robot1234`) y abrir `http://192.168.4.1`.
La página muestra el estado de la navegación, los ultrasonidos, los TCRT5000, el giro Z y
los motores, con gráficos de los últimos 60 s, un registro de cambios y los botones
`START` y `STOP`. Se edita en `Firmware/esp32_gateway/web/index.html` y queda embebida en
el firmware de la ESP32, que la actualiza consultando `GET /api/status`.

La STM32 manda esta línea por USART2 cada 200 ms, en todos los estados:

```text
TEL,<ms>,<estado>,<pwm_izq>,<pwm_der>,<freno>,<est_frente>,<mm_frente>,<est_izq>,<mm_izq>,<est_der>,<mm_der>,<tcrt_izq>,<tcrt_der>,<borde_izq>,<borde_der>,<giro_z>,<imu_ok>,<rumbo_decimas_grado>,<diag_imu>
```

Estados de ultrasonido: `O` = distancia válida, `F` = nada al alcance, `C` = pegado,
`N` = sin eco. El PWM es el que ordena la STM32: sin encoders, el panel no puede confirmar
que la rueda realmente gire.

## Pendiente antes de probar en el robot

1. **`.ioc`**: declarar FreeRTOS (CMSIS_V2, heap 32 768) y ADC1 en PC1/PC2 en CubeMX
   *antes* de regenerar código; si no, CubeMX borra FreeRTOS y el ADC.
2. **Calibrar TCRT5000 en el granito**: medir el ADC del izquierdo sobre el piso y
   mirando al vacío y ajustar `TCRT_THRESHOLD_CLIFF` (hoy 2000, provisorio) con la regla
   de `drv_TCRT5000.c`.
3. **Calibrar `ROBOT_WHEEL_MIN_PWM`** (hoy 3000, provisorio): el menor PWM con el que ambas
   ruedas arrancan desde parado en el granito. Tiene que quedar por debajo de 4200.
4. **Verificar los valores provisorios** de la tabla de "Recorrido por tramos" (retrocesos,
   giro de 90° y distancia de parada).
5. Revisar el signo del giro (`angular > 0` = izquierda) con las ruedas en el aire.
6. Medir el heap libre con `xPortGetFreeHeapSize()`.

## Prueba en la PC

```bash
cd Firmware/stm32_control
gcc -Wall -Wextra -ICore/Inc Test/test_mid_nav.c Core/Src/mid_nav.c -o test_mid_nav && ./test_mid_nav
gcc -Wall -Wextra -ICore/Inc Test/test_mid_nav_scan.c Core/Src/mid_nav.c -o test_mid_nav_scan && ./test_mid_nav_scan
```

## Umbrales del plan (30 / 40 cm)

| Regla | Parámetro | Valor |
|---|---|---|
| Frena de frente con menos de 30 cm (para parar cerca de 25 cm) | `NAV_OBSTACLE_MM` | 300 mm (antes 400) |
| Frente libre para avanzar | `NAV_CLEAR_MM` | 350 mm (antes 400) |
| Lado libre para doblar; prefiere la **derecha**, si no la izquierda. Gira 90° y sigue de largo por ese lado | `NAV_SIDE_FREE_MM` | 400 mm |
| Sin lado libre: retrocede y vuelve a medir, hasta 5 veces; luego `FAULT` | `NAV_SCAN_BACKUP_MAX` | 5 |
| Distancia de seguridad en `main.c` | `SAFE_DISTANCE` | 300 mm (antes 200) |

Un lateral sin eco no cuenta como libre; un eco largo sí. Falta medir la distancia de
frenado a `NAV_FWD_SPEED_MMS` para confirmar que el robot se detiene antes de 30 cm.
