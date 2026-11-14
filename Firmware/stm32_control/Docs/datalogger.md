# Datalogger (rama `Pruebas_Grupo2_Datalogger`)

Guarda en la tarjeta SD un registro con fecha y hora de lo que hace el robot y muestra en
el panel web del ESP32 si cada módulo está funcionando.

## Conexiones

| Módulo | Señal | Pin STM32 | Alimentación |
|---|---|---|---|
| Tarjeta SD (SPI2) | SCK / MISO / MOSI / CS | PB13 / PB14 / PB15 / PC4 | 5 V si el módulo tiene regulador; capacitor 100 µF + 100 nF |
| Reloj DS1302 | CLK / DAT / RST | PD8 / PD9 / PD10 | 3,3 V, pila CR2032 en el módulo |

Al iniciar SPI2 se liberan PB10 y PC3 (micrófono MEMS de la Discovery), que `gpio.c`
dejaba en la función alternativa de SPI2.

## Capas

| Capa | Archivo | Qué hace |
|---|---|---|
| drv | `drv_sd_spi` | Inicializa la tarjeta (SD v2: SDSC y SDHC) y lee o escribe bloques de 512 B |
| drv | `drv_sd_diskio` | Funciones `disk_*` de FatFs sobre `drv_sd_spi` (un solo volumen, `0:`) |
| — | `Middlewares/Third_Party/FatFs` + `Core/Inc/ffconf.h` | FatFs R0.12c sin cambios; configuración propia |
| mid | `mid_clock` | Fecha y hora del DS1302: valida lo leído, pone en hora, mutex |
| mid | `mid_log` | Escribe cada línea en `GAR.LOG` (con `f_sync`) y por USB; mutex |
| task | `task_datalogger` | Prioridad Low: una fila de datos cada 200 ms y estado de módulos cada 1 s |

## Formato de `GAR.LOG`

```
[2026-11-14 17:30:00] [INF] datalogger: SD lista (SDHC), reloj en hora
[2026-11-14 17:30:00] [DAT] t_ms,us_f,front_mm,us_l,left_mm,us_r,right_mm,ir_l,ir_r,cliff_l,cliff_r,gyro_dps,heading_mdeg,state,pwm_l,pwm_r,brake
[2026-11-14 17:30:01] [DAT] 4212,O,1150,O,630,N,0,210,231,0,0,1,-90,FORWARD,4400,4400,0
```

Si el reloj no responde o no está en hora, el prefijo es `[boot+1234ms]`.
Ultrasonidos: `O` = válido, `F` = libre, `C` = pegado, `N` = sin eco.

Para pasarlo a CSV en la PC:

```sh
grep '\[DAT\]' GAR.LOG | sed 's/^.*\[DAT\] //' > datos.csv
```

## Protocolo nuevo con el ESP32

| Sentido | Línea | Contenido |
|---|---|---|
| STM32 → ESP32, cada 1 s | `SYS,<ms>,<sd>,<tipo_sd>,<lineas_sd>,<reloj_ok>,<año>,<mes>,<día>,<hora>,<min>,<seg>,<filas>` | `sd`: 0 = sin tarjeta, 1 = guardando, 2 = falló en marcha |
| ESP32 → STM32 | `TIME,AAAA,MM,DD,hh,mm,ss` | Pone en hora el DS1302 |
| STM32 → ESP32 | `EVT,<ms>,HORA_OK` / `EVT,<ms>,HORA_ERR` | Respuesta al `TIME` |

`PING`/`OK`, `START`/`STOP`, `TEL`, `MAP` y `EVT` no cambian.

## Panel web

La tarjeta **Estado de módulos** muestra cada módulo con un punto verde (responde), ámbar
(responde con advertencia) o rojo (falla): enlace con la STM32, los tres ultrasonidos, los
sensores de piso, el MPU6050, los motores, la tarjeta SD, el reloj y el datalogger. El botón
**Sincronizar hora del robot** copia al DS1302 la hora del celular.

## Comportamiento sin SD o sin reloj

- Sin tarjeta (o sin respuesta al arrancar): el robot navega igual, el datalogger genera las
  filas y solo salen por USB. La web muestra «Tarjeta SD: No detectada». La tarjeta se
  detecta solo al arrancar: después de ponerla, reiniciar.
- Si la SD falla en marcha: se deja de usar (web: «Falló») y el robot sigue.
- Sin reloj o sin hora: las líneas usan `boot+ms` y la web muestra «Reloj DS1302: Sin hora».

## Reglas

- Solo `mid_log` usa FatFs, y solo `mid_clock` usa el DS1302.
- No llamar `LOG_*` desde `task_nav` ni `task_sensors`: escribir en la SD tarda decenas de ms.
- `task_datalogger` lee con `TaskSensors_GetSnapshot` y `TaskNav_GetStatus` (no bloquean) y no
  consume la cola de eventos de navegación.
- La tarea que llame a `LOG_*` necesita al menos 1 KB de stack (`defaultTask` tiene 512 B: no lo usa).
