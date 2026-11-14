/**
 * @file    ffconf.h
 * @brief   Configuracion de FatFs R0.12c para el datalogger.
 *
 * La biblioteca esta en Middlewares/Third_Party/FatFs/src y se usa tal cual.
 * Los nombres de las opciones (con guion bajo inicial) los fija FatFs y no
 * se pueden cambiar. Valores tomados de la configuracion de Grupo-1.
 *
 * Decisiones:
 *   - Lectura y escritura, sin formatear (la tarjeta se formatea en la PC).
 *   - Nombres cortos 8.3 (GAR.LOG): sin nombres largos, ahorra RAM.
 *   - Un solo volumen ("0:"), sectores de 512 bytes.
 *   - Sin reloj (_FS_NORTC = 1): la fecha de cada linea la pone mid_log.
 *   - Sin reentrancia: solo mid_log usa FatFs, protegido por su mutex.
 */

#ifndef _FFCONF
#define _FFCONF 68300   // Revision de la configuracion (debe coincidir con ff.h)

/* ===== Funciones disponibles ===== */
#define _FS_READONLY    0       // 0 = lectura y escritura
#define _FS_MINIMIZE    0       // 0 = API completa
#define _USE_STRFUNC    0       // Sin f_printf / f_puts (se usa snprintf)
#define _USE_FIND       0
#define _USE_MKFS       0       // No formatea
#define _USE_FASTSEEK   0
#define _USE_EXPAND     0
#define _USE_CHMOD      0
#define _USE_LABEL      0
#define _USE_FORWARD    0

/* ===== Nombres de archivo ===== */
#define _CODE_PAGE      1       // ASCII (solo nombres sin acentos)
#define _USE_LFN        0       // Solo nombres cortos 8.3
#define _MAX_LFN        255
#define _LFN_UNICODE    0
#define _STRF_ENCODE    3
#define _FS_RPATH       0       // Sin directorio actual relativo

/* ===== Volumenes ===== */
#define _VOLUMES            1
#define _STR_VOLUME_ID      0
#define _VOLUME_STRS        "SD"
#define _MULTI_PARTITION    0
#define _MIN_SS             512
#define _MAX_SS             512
#define _USE_TRIM           0
#define _FS_NOFSINFO        0

/* ===== Sistema ===== */
#define _FS_TINY        0       // Buffer de sector por archivo (mas rapido)
#define _FS_EXFAT       0       // Solo FAT12/16/32 (tarjetas de hasta 32 GB)
#define _FS_NORTC       1       // Sin reloj: fecha fija para los archivos
#define _NORTC_MON      11
#define _NORTC_MDAY     1
#define _NORTC_YEAR     2026
#define _FS_LOCK        0       // Sin control de archivos abiertos
#define _FS_REENTRANT   0       // Sin mutex interno (lo pone mid_log)
#define _FS_TIMEOUT     1000
#define _SYNC_t         void *

#endif /* _FFCONF */
