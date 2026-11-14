/**
 * @file    drv_sd_diskio.c
 * @brief   Funciones de disco de FatFs sobre la tarjeta SD por SPI.
 *
 * Los nombres disk_* y los tipos (DSTATUS, DRESULT, BYTE...) los fija FatFs.
 */

#include "drv_sd_diskio.h"
#include "drv_sd_spi.h"
#include "ff.h"
#include "diskio.h"
#include <stddef.h>

#define SD_DISKIO_DRIVE     0u      // Unico disco fisico

static SPI_HandleTypeDef *diskio_spi = NULL;
static volatile DSTATUS diskio_status = STA_NOINIT;

void SdDiskio_Init(SPI_HandleTypeDef *hspi) {
    diskio_spi = hspi;
    diskio_status = STA_NOINIT;
}

/* Lee o escribe 'count' bloques seguidos; corta en el primer error */
static DRESULT diskio_transfer(BYTE *read_buffer, const BYTE *write_buffer,
                               DWORD sector, UINT count) {
    for (UINT i = 0u; i < count; i++) {
        SD_Status_t status;
        if (read_buffer != NULL) {
            status = SD_ReadBlock(&read_buffer[i * SD_BLOCK_SIZE], sector + i);
        } else {
            status = SD_WriteBlock(&write_buffer[i * SD_BLOCK_SIZE], sector + i);
        }
        if (status != SD_OK) {
            return RES_ERROR;
        }
    }
    return RES_OK;
}

DSTATUS disk_initialize(BYTE pdrv) {
    if (pdrv != SD_DISKIO_DRIVE) {
        return STA_NOINIT;
    }
    if (SD_Init(diskio_spi) == SD_OK) {
        diskio_status = 0u;
    } else {
        diskio_status = STA_NOINIT;
    }
    return diskio_status;
}

DSTATUS disk_status(BYTE pdrv) {
    if (pdrv != SD_DISKIO_DRIVE) {
        return STA_NOINIT;
    }
    return diskio_status;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
    if ((pdrv != SD_DISKIO_DRIVE) || (buff == NULL) || (count == 0u)) {
        return RES_PARERR;
    }
    if ((diskio_status & STA_NOINIT) != 0u) {
        return RES_NOTRDY;
    }
    return diskio_transfer(buff, NULL, sector, count);
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count) {
    if ((pdrv != SD_DISKIO_DRIVE) || (buff == NULL) || (count == 0u)) {
        return RES_PARERR;
    }
    if ((diskio_status & STA_NOINIT) != 0u) {
        return RES_NOTRDY;
    }
    return diskio_transfer(NULL, buff, sector, count);
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv != SD_DISKIO_DRIVE) {
        return RES_PARERR;
    }
    if ((diskio_status & STA_NOINIT) != 0u) {
        return RES_NOTRDY;
    }

    switch (cmd) {
        case CTRL_SYNC:
            // SD_WriteBlock ya espera a que la tarjeta termine de grabar
            return RES_OK;
        case GET_SECTOR_SIZE:
            *(WORD *)buff = (WORD)SD_BLOCK_SIZE;
            return RES_OK;
        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1u;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}
