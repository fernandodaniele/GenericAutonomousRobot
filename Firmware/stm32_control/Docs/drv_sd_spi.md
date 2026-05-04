# drv_sd_spi — Driver SD card por SPI

Driver para tarjeta SD en modo SPI sobre STM32F407VG. Implementa el protocolo
SD SPI (inicialización, lectura y escritura de bloques de 512 bytes) y se integra
con FatFs a través de `user_diskio.c`.

---

## Prerequisitos de hardware y CubeMX

El driver requiere que SPI2 y un pin GPIO para CS estén configurados en CubeMX
antes de ser usado. Esos cambios generan el `hspi2` y las definiciones de pin que
el driver consume.

### Paso 1 — Liberar I2S2 (periférico compartido con SPI2)
- Clic en pin **PB10** → **Reset State**
- Clic en pin **PC3** → **Reset State**

### Paso 2 — Habilitar SPI2
- `Connectivity → SPI2 → Full-Duplex Master`
- NSS: Software
- Prescaler: **256** (164 kHz — velocidad segura para init de SD)
- First Bit: MSB
- CPOL: Low, CPHA: 1 Edge (Mode 0)
- CubeMX asigna automáticamente PB10/PC2/PC3

### Paso 3 — Pin CS de la SD
- Clic en pin **PC4** → **GPIO_Output**
- GPIO → User Label: `SD_CS`
- Output Level: **High** (CS inactivo = HIGH)

### Paso 4 — Middleware FatFs
- `Middleware and Software Packs → FATFS → Enable`
- Type: **User-defined**
- Configuration → dejar defaults (MAX_SS = 512 está bien para hello world)

### Paso 5 — GENERATE CODE


---

## Conexión física del módulo SD

```
Módulo SD SPI    →   STM32F407 Discovery
─────────────────────────────────────────
VCC (5 V)        →   5V pin
GND              →   GND
SCK              →   PB10
MISO             →   PC2
MOSI             →   PC3
CS               →   PC4
```

> Los módulos SD de bajo costo suelen tener un regulador de voltaje y nivel
> lógico integrados; en ese caso también pueden conectarse a 5 V.

---

## Dependencias en el proyecto

| Archivo | Rol |
|---|---|
| `Core/Inc/drv_sd_spi.h` | API pública del driver |
| `Core/Src/drv_sd_spi.c` | Implementación del protocolo SD SPI |
| `FATFS/Target/user_diskio.c` | Bridge FatFs ↔ driver |
| `FATFS/App/fatfs.c` | Inicialización del volumen FatFs (generado por CubeMX) |
| `spi.h` / `spi.c` | Handle `hspi2` (generado por CubeMX) |
| `main.h` | Definiciones `SD_CS_Pin` y `SD_CS_GPIO_Port` (generadas por CubeMX) |

El driver **no** necesita modificaciones en los archivos de CubeMX ni en el HAL.

---

## API

```c
// Inicializa la tarjeta SD. Llama a esta función antes de montar FatFs.
// hspi debe ser hspi2, configurado con prescaler 256.
// El driver sube la velocidad a 10.5 MHz internamente tras la init.
SD_Status_t SD_Init(SPI_HandleTypeDef *hspi);

// Lee un bloque de 512 bytes desde la dirección LBA block_addr.
// buf debe tener capacidad de al menos 512 bytes.
SD_Status_t SD_ReadBlock(uint8_t *buf, uint32_t block_addr);

// Escribe un bloque de 512 bytes en la dirección LBA block_addr.
SD_Status_t SD_WriteBlock(const uint8_t *buf, uint32_t block_addr);
```

**Valores de retorno:**

| Valor | Significado |
|---|---|
| `SD_OK` | Operación exitosa |
| `SD_ERROR` | Error de protocolo (respuesta inesperada de la tarjeta) |
| `SD_TIMEOUT` | La tarjeta no respondió dentro de los 500 ms |

El driver no se usa directamente desde la aplicación; `user_diskio.c` lo llama
como backend de FatFs. La aplicación usa únicamente la API de FatFs (`f_open`,
`f_read`, `f_write`, etc.).

---

## Uso desde la aplicación

```c
#include "fatfs.h"

// Llamar después de MX_SPI2_Init() y MX_FATFS_Init()
FATFS fs;
FIL   fil;
UINT  bw;

if (f_mount(&fs, USERPath, 1) == FR_OK) {
    if (f_open(&fil, "data.txt", FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
        f_write(&fil, "contenido\n", 10, &bw);
        f_close(&fil);
    }
    f_mount(NULL, USERPath, 0);  // desmontaje
}
```

El orden de inicialización en `main()` debe ser:

```
HAL_Init() → SystemClock_Config() → MX_GPIO_Init()
→ MX_SPI2_Init() → MX_FATFS_Init() → [tu código]
```

---

## Cómo funciona internamente

### Secuencia de inicialización SD SPI

```
1. ≥80 ciclos de clock con CS en alto     → activa modo SPI en la tarjeta
2. CMD0  (GO_IDLE_STATE)                  → reset; espera R1 = 0x01
3. CMD8  (SEND_IF_COND, arg 0x1AA)        → verifica rango de voltaje (SDv2)
4. ACMD41 con HCS=1  (loop hasta R1=0x00) → inicialización completa
5. CMD58 (READ_OCR)                       → detecta SDHC (bit CCS=1) o SDSC
6. CMD16 (SET_BLOCKLEN, 512)              → solo para SDSC; SDHC ya usa 512
7. Cambio de prescaler: 256 → 4           → sube de 164 kHz a 10.5 MHz
```

### Lectura de bloque (CMD17)

```
→ CMD17 con dirección LBA
← R1 = 0x00
← polling hasta token 0xFE
← 512 bytes de datos
← 2 bytes CRC (descartados)
```

### Escritura de bloque (CMD24)

```
→ CMD24 con dirección LBA
← R1 = 0x00
→ 1 byte dummy + token 0xFE + 512 bytes + 2 bytes CRC dummy
← token de respuesta (bits [4:0] == 0x05 = aceptado)
← polling hasta que MISO vuelve a 0xFF (fin de escritura en flash interna)
```

### Addressing: SDHC vs SDSC

- **SDHC/SDXC** (tarjetas > 2 GB): la dirección del comando es el número de
  bloque directamente. El driver detecta esto con el bit CCS del registro OCR.
- **SDSC** (tarjetas ≤ 2 GB): la dirección es en bytes (`block_addr × 512`).
  El driver aplica la conversión automáticamente.

---

## Diagnóstico y troubleshooting

El hello world en `main.c` envía el resultado por USB CDC al arrancar:

| Mensaje CDC | Causa probable |
|---|---|
| `SD: OK` | Todo funcionó correctamente |
| `SD: mount error` | `f_mount` falló — revisar columna siguiente |
| `SD: f_open error` | Sistema de archivos montado pero no puede abrir/crear archivo |

**`SD: mount error` — checklist:**

| Síntoma | Causa probable |
|---|---|
| No hay respuesta en CMD0 | CS al pin equivocado, o VCC insuficiente |
| CMD8 no responde con 0x01 | Tarjeta SDv1 o MMC (no soportadas por este driver) |
| ACMD41 timeout (> 1 s) | Tarjeta defectuosa o nivel lógico inadecuado |
| `f_mount` retorna `FR_NO_FILESYSTEM` | La tarjeta no está formateada en FAT32/FAT16 |
| `f_mount` retorna `FR_DISK_ERR` | Error en `disk_initialize` → ver SD_Init() |

**Verificación con osciloscopio/analizador lógico:**

- Al arrancar, CS debe bajar y aparecer ~80 pulsos de clock **con CS en alto** primero
- CMD0 = `0x40 0x00 0x00 0x00 0x00 0x95` sobre MOSI
- La tarjeta responde `0x01` sobre MISO dentro de los 8 ciclos siguientes

---

## Limitaciones conocidas

- Solo soporta **SDv2** (SDHC y SDSC). Las tarjetas SDv1 y MMC antiguas no son
  compatibles.
- No usa DMA — las transferencias son bloqueantes por polling. Suficiente para
  logging y black box; no apto para streaming de alta velocidad.
- Un único volumen (sin soporte multi-drive). FatFs se configura con
  `_VOLUMES = 1`.
- Sin soporte de detección de tarjeta (card detect pin). Si la tarjeta se
  retira en caliente, el sistema queda en estado indefinido hasta el próximo reset.
- La velocidad post-init es 10.5 MHz (prescaler /4). El máximo del modo SPI
  es 25 MHz; puede subirse cambiando `SPI_BAUDRATEPRESCALER_4` por
  `SPI_BAUDRATEPRESCALER_2` en `sd_set_high_speed()` si la calidad del cableado
  lo permite.
