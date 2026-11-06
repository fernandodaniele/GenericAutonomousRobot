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
| 1 | `task_sensors`: un HC-SR04 por turno cada 30 ms, TCRT5000 y giro Z del MPU6050 (I2C3, 3 reintentos) | `task_sensors.c/.h` |
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
- **Pausa `NAV_SETTLE_MS`** (300 ms, con freno) entre frenar y retroceder, entre
  retroceder y girar y entre girar y avanzar, en la evasiva y en la recuperación del
  borde. El lado de giro de la evasiva se decide al final de la pausa, con el robot
  quieto.

### Reglas de la máquina de estados
| Regla | Parámetro |
|---|---|
| El borde se confirma con 3 lecturas nuevas seguidas antes de `CLIFF_RECOVER` | `NAV_CLIFF_CONFIRM` |
| 3 `CLIFF_RECOVER` seguidos sin 2 s de `FORWARD` → `FAULT` frenado + `EVT,FAULT` | `NAV_CLIFF_LOOP_MAX`, `NAV_CLIFF_LOOP_RESET_MS` |
| Ese `FAULT` **no se recupera solo**: hay que mandar `STOP` y luego `START` (o resetear la placa). LD3 da 4 destellos | — |
| Giro de esquina de más de 8 s → rutina evasiva | `NAV_CORNER_MAX_MS` |
| Lado sin eco al elegir el giro: 0 = espacio libre 0 (actual), 1 = "desconocido" | `NAV_NO_ECHO_AS_UNKNOWN` |

### Ultrasonido derecho sin eco
Con el derecho siempre en `NO_ECHO`, la evasiva sin pared gira **siempre a la izquierda**
(el lado sin eco cuenta como espacio 0). Se dejó así a propósito: la izquierda es el lado
del único TCRT5000 activo (`SENSORS_USE_TCRT_RIGHT = 0`), y el derecho no tiene ni
ultrasonido ni infrarrojo. La opción `NAV_NO_ECHO_AS_UNKNOWN = 1` permite girar a la
derecha cuando la izquierda tiene menos de `NAV_CLEAR_MM`, pero eso lleva al robot hacia
el lado ciego: activarla solo cuando el ultrasonido derecho funcione.

### LED naranja (LD3)
| LED | Estado |
|---|---|
| Apagado | `IDLE` |
| Fijo | `FORWARD` |
| Parpadeo rápido | Evasiva o borde |
| 1 / 2 / 3 / 4 destellos | `FAULT`: MPU6050 / frontal sin eco / sensores sin actualizar / bucle de retroceso |

## Pendiente antes de probar en el robot

1. **`.ioc`**: declarar FreeRTOS (CMSIS_V2, heap 32 768) y ADC1 en PC1/PC2 en CubeMX
   *antes* de regenerar código; si no, CubeMX borra FreeRTOS y el ADC.
2. **Calibrar TCRT5000 en el granito**: medir el ADC del izquierdo sobre el piso y
   mirando al vacío y ajustar `TCRT_THRESHOLD_CLIFF` (hoy 2000, provisorio) con la regla
   de `drv_TCRT5000.c`.
3. **Calibrar `ROBOT_WHEEL_MIN_PWM`** (hoy 4200): el menor PWM con el que ambas ruedas
   arrancan desde parado en el granito.
4. **Medir `ROBOT_MAX_WHEEL_SPEED_MMS`** (hoy 50 mm/s, a ojo) y la distancia de frenado a
   35 mm/s; ajustar `NAV_FWD_SPEED_MMS` para que frene antes de 5 cm.
5. Revisar el signo del giro (`angular > 0` = izquierda) con las ruedas en el aire.
6. Medir el heap libre con `xPortGetFreeHeapSize()`.

## Prueba en la PC

```bash
cd Firmware/stm32_control
gcc -Wall -Wextra -ICore/Inc Test/test_mid_nav.c Core/Src/mid_nav.c -o test_mid_nav && ./test_mid_nav
```
