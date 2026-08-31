# FATFS — vendorizado a mano (NO regenerar desde CubeMX)

Esta carpeta y `Middlewares/Third_Party/FatFs/` se agregaron **a mano**, no con la
opción FATFS de STM32CubeMX. Las fuentes se compilan vía `../GNUmakefile`
(`APP_C_SOURCES`), no vía `Makefile` (que queda como salida pura de CubeMX).
Ver `fix-plan.md` §5.

## Qué hay acá

- `Middlewares/Third_Party/FatFs/src/{ff,ff_gen_drv,diskio}.{c,h}`, `integer.h`:
  copiados tal cual del pack **STM32Cube FW_F4 V1.28.3** (FatFs **R0.12c**, rev 68300).
- `FATFS/Target/{ffconf.h, user_diskio.{c,h}}`: portados de `origin/add/sd-spi`.
  `user_diskio.c` cablea los callbacks a `drv_sd_spi` (`SD_Init(&hspi2)`,
  `SD_ReadBlock`, `SD_WriteBlock`).
- `FATFS/App/fatfs.{c,h}`: glue estilo CubeMX (`MX_FATFS_Init()` → `FATFS_LinkDriver`).
  `get_fattime()` es un stub (`_FS_NORTC = 1`).

## `ffconf.h` — afinado para log de solo-append

`_USE_LFN 0`, `_CODE_PAGE 1`, `_USE_MKFS 0`, `_USE_STRFUNC 0`, `_USE_FASTSEEK 0`,
`_FS_LOCK 0`, `_FS_NORTC 1`, `_FS_REENTRANT 0` (se serializa con el mutex de
`mid_log`).

Consecuencias:
- La tarjeta **debe venir formateada FAT/FAT32 desde la PC** (no hay `f_mkfs`).
- Solo nombres 8.3 (el log es `0:/GAR.LOG`).
- Los archivos no tienen mtime real (`get_fattime` stub). Los timestamps de cada
  línea sí salen del DS1302 vía `mid_log`.

## Si en el futuro se quiere pasar a FATFS "nativo" de CubeMX

Es una tarea deliberada, no un "Generate Code" al pasar:
1. Habilitar FATFS (modo *user-defined*) en el `.ioc`.
2. Regenerar y **descartar `main.c`** (`git checkout -- Core/Src/main.c`), luego
   reintegrar a mano.
3. Verificar que las opciones del panel FATFS de CubeMX reproducen este `ffconf.h`
   (si no, se revierte el afinado).
4. `user_diskio.c`: el glue a `drv_sd_spi` vive en secciones `USER CODE`, se preserva.
