# Pruebas físicas pendientes — rama `G1-int/sd-storage` (Fases 2–5)

Todo el código de las Fases 1–5 **compila** (`make` en `Firmware/stm32_control`, vía
`GNUmakefile`), pero **nada se probó en placa**. Este documento es el checklist de banco.

Placa: **STM32F407G-DISC1**. Mapa de pines completo en [`PINMAP.md`](PINMAP.md).

---

## 0. Grabar el firmware

En la máquina de desarrollo no hay herramienta de flasheo instalada. Instalar una:

```sh
sudo pacman -S stlink          # ST-Link (el que trae la Discovery)
# o: sudo pacman -S openocd
```

```sh
cd Firmware/stm32_control
make                                   # genera build/stm32_control.bin
st-flash write build/stm32_control.bin 0x08000000
```

Alternativa: STM32CubeProgrammer (`STM32_Programmer_CLI -c port=SWD -w build/stm32_control.elf -rst`).

**Antes de mergear a `master`**: compilar también con la toolchain del CI
(`arm-none-eabi-gcc 10.3-2021.10`), más estricta que la 16.x local para algunos warnings.

---

## 1. Consola de depuración (USB CDC)

El firmware loguea por **USB CDC** (el micro-USB de "USB OTG", no el ST-Link).

- Abrir con **`screen /dev/ttyACM0 115200`** o **`minicom -D /dev/ttyACM0`**.
  Estos activan **DTR** → el logger empieza a transmitir.
- **`cat /dev/ttyACM0` NO activa DTR** → no vas a ver nada aunque el firmware ande.
- Al cerrar el terminal, el logger deja de mandar por CDC (pero sigue escribiendo a la SD).

Salida esperada tras el boot (~2 líneas/segundo):

```
[2026-01-01 12:00:00][INF] STM32 UART ready        (por UART, no CDC)
[2026-01-01 12:00:01][INF] hola desde task_example
[2026-01-01 12:00:01][INF] heartbeat, uptime=1 s
[2026-01-01 12:00:01][INF] MPU6050 OK              (o: WRN MPU6050 no responde...)
[2026-01-01 12:00:02][INF] roll=0.3 pitch=-1.1 cliff=0 dist=65535mm
...
```

---

## 2. RTC DS1302 (Fase 2)

**Cableado** (bit-bang, no SPI):

| Módulo | STM32 |
|---|---|
| CLK / SCLK | PD8 |
| DAT / I/O  | PD9 |
| RST / CE   | PD10 |
| VCC | 3V3 |
| GND | GND |
| (pila CR2032 en el módulo) | — |

**Qué verificar:**
- [ ] El firmware fija `2026-01-01 12:00:00` en cada arranque (bring-up).
- [ ] En la consola, el **timestamp del prefijo** de `mid_log` arranca en `12:00:00` y
      **avanza 1 s por segundo** (no congelado, no saltos).
- [ ] Si sale `[boot+Nms]` en vez de fecha → `DS1302_UpdateDateTime()` falla: revisar
      cableado y que el módulo tenga cristal + pila.
- [ ] Si la hora avanza mal (rápido/lento) → problema de timing del bit-bang
      (`DS1302_Delay()` en `ds1302.c`); es NOP-loop, a 168 MHz debería andar.

**Nota:** el DS1302 mantiene la hora con la pila entre reinicios, pero el firmware la
pisa en cada boot (`DS1302_SetTime` en `app_main.c`). Cuando se quiera hora real:
`TODO` en `app_main.c` — setear solo si el bit CH indica reloj detenido.

---

## 3. Logger → USB CDC + detección DTR (Fase 3)

- [ ] Con `screen`/`minicom` abierto: se ven las líneas con timestamp (ver §1).
- [ ] Al **cerrar** el terminal: el firmware deja de transmitir por CDC (no se acumula
      backlog, no se cuelga). La SD (si está) sigue recibiendo.
- [ ] Reabrir el terminal → vuelven a aparecer líneas nuevas (no las viejas).
- [ ] El prefijo de nivel funciona: `LOG_INFO`→`[INF]`, `LOG_WARN`→`[WRN]`, `LOG_ERROR`→`[ERR]`.

---

## 4. Tarjeta SD — black box (Fase 4)

**Cableado** (SPI2):

| Módulo SD | STM32 |
|---|---|
| SCK  | PB13 |
| MISO | PB14 |
| MOSI | PB15 |
| CS   | PC4  |
| VCC  | 3V3 (verificar que el módulo sea 3.3 V nativo o tenga level shifter) |
| GND  | GND  |

**Preparación:** la tarjeta debe venir **formateada FAT o FAT32 desde la PC**
(no hay `f_mkfs` en el firmware — `_USE_MKFS 0` en `ffconf.h`).

**Qué verificar:**
- [ ] Tras el boot, sacar la SD y leerla en la PC → debe existir **`GAR.LOG`** en la raíz.
- [ ] El contenido de `GAR.LOG` **coincide** con lo que se vio por la consola CDC (mismas
      líneas, mismos timestamps).
- [ ] Reiniciar la placa varias veces → `GAR.LOG` **acumula** (append), no se pisa.
- [ ] Sacar la SD con la placa encendida → el firmware no se cuelga (baja `s_sd_ok`,
      sigue solo por CDC).
- [ ] Si `GAR.LOG` no aparece → `f_mount`/`f_open` fallaron en `Log_Init` (`mid_log.c`):
      SD no detectada. Revisar cableado, formato, y que la tarjeta responda a CMD0/ACMD41.
      Para depurar, poner un breakpoint en `Log_Init` o un LED al setear `s_sd_ok`.

**Conocido:** `get_fattime()` es un stub (`_FS_NORTC 1`) → el **mtime** de `GAR.LOG` va a
ser fecha basura (1980). Cosmético. Pendiente: enganchar `get_fattime()` al DS1302.

---

## 5. Sensores en el lazo de control (Fase 5)

Tarea `task_sensors` (~20 ms). Loguea 1×/s: `roll=.. pitch=.. cliff=.. dist=..mm [STOP]`.

### 5.1 MPU6050 (IMU, I2C1)

**Cableado:** SCL = **PB6**, SDA = **PB9** (etiquetados `Audio_SCL/SDA` en la Discovery —
bus compartido con el códec CS43L22; sin colisión: CS43L22 = 0x94, MPU6050 = 0xD0).
Dirección 0xD0 = pin AD0 a GND. Pull-ups: la Discovery los tiene para el códec; si usás
un breakout externo, suele traer los suyos.

- [ ] `LOG_INFO("MPU6050 OK")` en el boot. Si sale `LOG_WARN("MPU6050 no responde...")`
      → no cableado / dirección / sin pull-ups. **No es fatal**: el robot corre sin IMU.
- [ ] Inclinar la placa → `roll` y `pitch` en el log cambian de forma coherente.
- [ ] Dejar la placa quieta unos segundos → los ángulos convergen (filtro Kalman).

### 5.2 TCRT5000 (infrarrojos de precipicio, GPIO)

**Cableado:** salida digital de cada módulo (el que tiene comparador + potenciómetro):

| Sensor | STM32 |
|---|---|
| Delantero izquierdo | PE7 |
| Delantero derecho   | PE8 |
| Trasero izquierdo   | PE9 |
| Trasero derecho     | PE10 |

- [ ] **Polaridad**: `drv_TCRT5000.c` tiene `#define TCRT_NIVEL_DETECCION GPIO_PIN_RESET`
      (asume: el módulo saca **LOW** cuando ve piso). Verificar:
      robot sobre la mesa (hay piso) → `cliff=0` en el log; levantar el frente → `cliff=1`.
      **Si está invertido**, cambiar a `GPIO_PIN_SET` y recompilar.
- [ ] **Pull**: `gpio.c` deja PE7–PE10 como `INPUT` **sin pull**. Si las lecturas son
      erráticas (entrada IR flotante), poner `GPIO_PULLUP` (en `gpio.c` o en el `.ioc`).
- [ ] Ajustar el **potenciómetro** de cada módulo a la altura real del robot para que
      distinga piso / borde de forma estable.

### 5.3 HC-SR04 (ultrasonido) — ya integrado, falta validar en el lazo

**Cableado:** TRIG = **PB4**, ECHO = **PA6** (TIM3_CH1 + DMA1_Stream4).
Ojo: el HC-SR04 saca 5 V en ECHO → **divisor resistivo a ~3.3 V** en PA6 (o módulo 3.3 V).

- [ ] `dist=` en el log muestra la distancia en mm a un objeto conocido (± unos cm).
- [ ] Sin objeto / fuera de rango → `dist=65535mm` (`SENSOR_DISTANCE_INVALID`).
- [ ] Mover la mano cerca/lejos → el valor sigue.

### 5.4 Consumidor de seguridad (`Motor_Stop`)

`task_sensors` frena **ambos motores** si `cliff_any` **o** `dist < SAFE_DISTANCE_MM` (200).

- [ ] Tapar un TCRT delantero **o** acercar la mano a < 20 cm del HC-SR04 → aparece
      **` STOP`** al final de la línea de log.
- [ ] (Opcional, prueba más profunda) Agregar temporalmente un `Motor_SetSpeed(&motor_l, 3000)`
      en `app_main.c` antes del scheduler, robot **sobre caballetes** (ruedas al aire),
      y verificar que al detectar precipicio/obstáculo el motor efectivamente para.
- [ ] Que `Motor_Stop` no genere hard-fault (los `motor_*.htim` apuntan a `&htim4`, OK).

**Nota:** todavía **no hay comando de "avanzar"** (la navegación es fase futura), así que
sin el paso opcional de arriba la prueba se limita a ver el flag `STOP` en el log.

---

## 6. UART STM32 ↔ ESP32 (pendiente del informe original)

- [ ] Al boot, el STM32 manda `"STM32 UART ready\n"` por **USART2** (TX = PA2, RX = PA3)
      hacia el ESP32. Verificar que el ESP32 lo recibe (tiene `uart_comm` en
      `Firmware/esp32_gateway`).
- [ ] Probar ida y vuelta de un mensaje corto.

---

## Resumen de ajustes probables (según qué falle)

| Síntoma | Dónde tocar |
|---|---|
| DS1302 da `[boot+Nms]` | cableado PD8/9/10, pila, cristal del módulo |
| TCRT `cliff` siempre 1 o siempre 0 | `TCRT_NIVEL_DETECCION` en `drv_TCRT5000.c` |
| TCRT lecturas erráticas | `GPIO_PULLUP` en `gpio.c` para PE7–PE10 |
| MPU6050 `no responde` | pull-ups I2C, dirección AD0, cableado PB6/PB9 |
| `GAR.LOG` no aparece | formato FAT de la SD, cableado SPI2, level shifter |
| ECHO del HC-SR04 raro | divisor resistivo de 5 V→3.3 V en PA6 |
| mtime de archivos = 1980 | esperado (`get_fattime` stub); enganchar al DS1302 |
