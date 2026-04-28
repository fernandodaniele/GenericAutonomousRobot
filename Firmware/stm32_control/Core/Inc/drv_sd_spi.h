/**
 * @file    drv_sd_spi.h
 * @brief   Driver de bajo nivel para tarjetas SD via SPI1.
 *
 * Implementa la secuencia de inicialización SPI (CMD0, CMD8, ACMD41, CMD58)
 * y expone el tipo de tarjeta detectado. Es la capa diskio de FATFS.
 *
 * Conexión hardware (SPI1, Discovery F407):
 *   SCK  → PA5   MISO → PA6   MOSI → PA7   CS → PA4 (I2S3_WS libre)
 *
 * @note  PA4 es I2S3_WS en la Discovery. Solo usar si I2S3 no está activo.
 */

#ifndef DRV_SD_SPI_H
#define DRV_SD_SPI_H

#include <stdint.h>

typedef enum {
    SD_OK = 0,
    SD_ERR_NO_RESPONSE,
    SD_ERR_TIMEOUT,
    SD_ERR_VOLTAGE,
} SD_Status_t;

typedef enum {
    SD_TYPE_UNKNOWN = 0,
    SD_TYPE_V1,
    SD_TYPE_V2,
    SD_TYPE_SDHC,
} SD_Type_t;

/**
 * @brief  Inicializa la tarjeta SD en modo SPI.
 *
 * Configura PA4 como CS, baja la velocidad SPI a ≤400 kHz para la secuencia
 * de init, y sube a 21 MHz una vez que la tarjeta está lista.
 *
 * @return SD_OK si la tarjeta respondió correctamente.
 */
SD_Status_t SD_Init(void);

/**
 * @brief  Retorna el tipo de tarjeta detectado tras un SD_Init() exitoso.
 */
SD_Type_t SD_GetType(void);

#endif /* DRV_SD_SPI_H */
