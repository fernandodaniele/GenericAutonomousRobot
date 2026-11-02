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

El robot arranca en `IDLE` y solo avanza con `START` por UART.

## Pendiente antes de probar en el robot

1. **`.ioc`**: declarar FreeRTOS (CMSIS_V2, heap 32 768) y ADC1 en PC1/PC2 en CubeMX
   *antes* de regenerar código; si no, CubeMX borra FreeRTOS y el ADC.
2. **Calibrar TCRT5000**: medir el ADC sobre el piso y sobre el borde y ajustar
   `TCRT_THRESHOLD_CLIFF` (hoy 2000).
3. **Medir `ROBOT_MAX_WHEEL_SPEED_MMS`** (hoy 500 mm/s provisorio) y la distancia de frenado;
   ajustar `NAV_FWD_SPEED_MMS` para que frene antes de 5 cm.
4. Revisar el signo del giro (`angular > 0` = izquierda) con las ruedas en el aire.
5. Medir el heap libre con `xPortGetFreeHeapSize()`.

## Prueba en la PC

```bash
cd Firmware/stm32_control
gcc -Wall -Wextra -ICore/Inc Test/test_mid_nav.c Core/Src/mid_nav.c -o test_mid_nav && ./test_mid_nav
```
