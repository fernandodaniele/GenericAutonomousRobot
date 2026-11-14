/**
 * @file    drv_sd_spi.h
 * @brief   Driver de bajo nivel para tarjetas SD en modo SPI.
 *
 * Inicializa la tarjeta (CMD0, CMD8, ACMD41, CMD58, CMD16) y lee o escribe
 * bloques de 512 bytes. Es la capa de disco que usa FatFs (drv_sd_diskio.c).
 *
 * Conexion (SPI2, Discovery F407):
 *   SCK -> PB13   MISO -> PB14   MOSI -> PB15   CS -> PC4 (SD_CS en main.h)
 *
 * Solo soporta tarjetas SD v2 (SDSC y SDHC/SDXC hasta 32 GB en FAT32).
 * Las tarjetas SD v1 y MMC devuelven SD_ERR_VERSION.
 */

#ifndef DRV_SD_SPI_H
#define DRV_SD_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define SD_BLOCK_SIZE   512u    // Bytes por bloque (sector) de la tarjeta

typedef enum {
    SD_OK = 0,
    SD_ERR_NO_RESPONSE,     // La tarjeta no contesto (no esta o mal conectada)
    SD_ERR_VERSION,         // Tarjeta SD v1 o MMC: no soportada
    SD_ERR_TIMEOUT,         // La tarjeta no termino a tiempo
    SD_ERR_COMMAND,         // La tarjeta rechazo el comando
    SD_ERR_WRITE,           // La tarjeta rechazo los datos escritos
    SD_ERR_NOT_READY        // Se uso antes de un SD_Init exitoso
} SD_Status_t;

typedef enum {
    SD_TYPE_UNKNOWN = 0,
    SD_TYPE_SDSC,           // Direccion por byte (hasta 2 GB)
    SD_TYPE_SDHC            // Direccion por bloque (SDHC y SDXC)
} SD_Type_t;

/**
 * @brief  Inicializa la tarjeta SD.
 *
 * Arranca el SPI a baja velocidad (la tarjeta exige <= 400 kHz durante la
 * inicializacion) y, si todo sale bien, lo sube a ~10,5 MHz.
 *
 * @param  hspi  Handle del SPI ya inicializado (MX_SPI2_Init).
 * @return SD_OK si la tarjeta quedo lista para leer y escribir.
 */
SD_Status_t SD_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief  Lee un bloque de 512 bytes.
 * @param  buffer       Destino, de al menos SD_BLOCK_SIZE bytes.
 * @param  block_index  Numero de bloque (LBA).
 */
SD_Status_t SD_ReadBlock(uint8_t *buffer, uint32_t block_index);

/**
 * @brief  Escribe un bloque de 512 bytes y espera a que la tarjeta lo grabe.
 * @param  buffer       Datos, SD_BLOCK_SIZE bytes.
 * @param  block_index  Numero de bloque (LBA).
 */
SD_Status_t SD_WriteBlock(const uint8_t *buffer, uint32_t block_index);

/**
 * @brief  Tipo de tarjeta detectado en el ultimo SD_Init exitoso.
 */
SD_Type_t SD_GetType(void);

/**
 * @brief  Indica si la tarjeta esta inicializada y lista.
 */
uint8_t SD_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_SD_SPI_H */
