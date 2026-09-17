#ifndef __DRV_SD_SPI_H__
#define __DRV_SD_SPI_H__

#include "main.h"

typedef enum {
    SD_OK      = 0,
    SD_ERROR   = 1,
    SD_TIMEOUT = 2,
} SD_Status_t;

/**
 * @brief  Inicializa la tarjeta SD en modo SPI. Sube la velocidad tras la init.
 * @param  hspi  Handle SPI a usar (prescaler 256 para init, sube a /4 post-init)
 * @retval SD_OK si la tarjeta fue detectada e inicializada correctamente
 */
SD_Status_t SD_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief  Lee un bloque de 512 bytes desde la tarjeta SD.
 * @param  buf        Buffer destino (debe tener al menos 512 bytes)
 * @param  block_addr Dirección del bloque (LBA)
 */
SD_Status_t SD_ReadBlock(uint8_t *buf, uint32_t block_addr);

/**
 * @brief  Escribe un bloque de 512 bytes en la tarjeta SD.
 * @param  buf        Buffer fuente con los datos a escribir
 * @param  block_addr Dirección del bloque (LBA)
 */
SD_Status_t SD_WriteBlock(const uint8_t *buf, uint32_t block_addr);

#endif /* __DRV_SD_SPI_H__ */
