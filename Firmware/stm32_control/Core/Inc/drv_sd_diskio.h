/**
 * @file    drv_sd_diskio.h
 * @brief   Union entre FatFs y el driver de la SD (drv_sd_spi).
 *
 * Implementa las funciones de disco que pide FatFs (disk_initialize,
 * disk_status, disk_read, disk_write y disk_ioctl, declaradas en diskio.h)
 * sobre la tarjeta SD por SPI. Hay un solo disco: el volumen "0:".
 *
 * Reemplaza la capa generica de ST (ff_gen_drv, user_diskio, fatfs.c) que
 * usaba Grupo-1: con una sola tarjeta no hace falta.
 */

#ifndef DRV_SD_DISKIO_H
#define DRV_SD_DISKIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define SD_DISKIO_VOLUME    "0:"    // Ruta del volumen para f_mount y f_open

/**
 * @brief  Indica que SPI usa la tarjeta. Llamar antes de f_mount.
 * @param  hspi  Handle del SPI ya inicializado (MX_SPI2_Init).
 */
void SdDiskio_Init(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* DRV_SD_DISKIO_H */
