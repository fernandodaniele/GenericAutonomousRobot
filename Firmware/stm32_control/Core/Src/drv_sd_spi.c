/**
 * @file    drv_sd_spi.c
 * @brief   Driver de bajo nivel para tarjetas SD en modo SPI.
 *
 * Basado en el driver de Grupo-1 (rama Grupo-1), reescrito con el estilo del
 * proyecto y con errores mas especificos.
 */

#include "drv_sd_spi.h"
#include "main.h"

/* ===== Tiempos y constantes del protocolo ===== */
#define SD_SPI_TIMEOUT_MS       10u     // Timeout de HAL por byte
#define SD_READY_TIMEOUT_MS     500u    // Espera de "tarjeta libre" y de datos
#define SD_INIT_TIMEOUT_MS      1000u   // Maximo para que ACMD41 termine
#define SD_POWER_UP_BYTES       10u     // 80 ciclos de reloj con CS en alto
#define SD_R1_POLL_BYTES        8u      // La respuesta R1 llega en <= 8 bytes

#define SD_DUMMY_BYTE           0xFFu
#define SD_TOKEN_START_BLOCK    0xFEu   // Inicio de un bloque de datos
#define SD_R1_IDLE              0x01u   // R1: en estado idle
#define SD_R1_READY             0x00u   // R1: sin errores
#define SD_R1_INVALID_MASK      0x80u   // Bit 7 en 1: todavia no es una respuesta
#define SD_DATA_RESP_MASK       0x1Fu
#define SD_DATA_ACCEPTED        0x05u   // Data response: 0b00101 = aceptado

/* Comandos SD en modo SPI */
#define SD_CMD0     0u      // GO_IDLE_STATE
#define SD_CMD8     8u      // SEND_IF_COND
#define SD_CMD16    16u     // SET_BLOCKLEN (solo SDSC)
#define SD_CMD17    17u     // READ_SINGLE_BLOCK
#define SD_CMD24    24u     // WRITE_BLOCK
#define SD_CMD55    55u     // APP_CMD (precede a un ACMD)
#define SD_CMD58    58u     // READ_OCR
#define SD_ACMD41   41u     // SD_SEND_OP_COND

#define SD_CMD8_ARG         0x000001AAu     // VHS = 2,7-3,6 V, patron 0xAA
#define SD_ACMD41_HCS       0x40000000u     // El host soporta SDHC
#define SD_OCR_CCS_BIT      0x40u           // Byte 0 del OCR: 1 = SDHC/SDXC

/* Prescalers sobre APB1 = 42 MHz */
#define SD_PRESCALER_INIT   SPI_BAUDRATEPRESCALER_128   // ~328 kHz
#define SD_PRESCALER_FAST   SPI_BAUDRATEPRESCALER_4     // ~10,5 MHz

static SPI_HandleTypeDef *sd_spi = NULL;
static SD_Type_t sd_type = SD_TYPE_UNKNOWN;
static uint8_t sd_ready = 0u;

/* ===== Acceso al bus ===== */

static uint8_t sd_transfer(uint8_t tx) {
    uint8_t rx = SD_DUMMY_BYTE;
    (void) HAL_SPI_TransmitReceive(sd_spi, &tx, &rx, 1u, SD_SPI_TIMEOUT_MS);
    return rx;
}

static void sd_clock_dummy(uint8_t count) {
    for (uint8_t i = 0u; i < count; i++) {
        (void) sd_transfer(SD_DUMMY_BYTE);
    }
}

static void sd_select(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void sd_deselect(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
    // Un byte extra con CS en alto para que la tarjeta suelte MISO
    (void) sd_transfer(SD_DUMMY_BYTE);
}

static void sd_set_prescaler(uint32_t prescaler) {
    sd_spi->Init.BaudRatePrescaler = prescaler;
    (void) HAL_SPI_Init(sd_spi);
}

/* Espera a que la tarjeta deje MISO en alto (fin de ocupado) */
static uint8_t sd_wait_ready(void) {
    uint32_t start = HAL_GetTick();
    uint8_t response;

    do {
        response = sd_transfer(SD_DUMMY_BYTE);
    } while ((response != SD_DUMMY_BYTE) && ((HAL_GetTick() - start) < SD_READY_TIMEOUT_MS));

    return response;
}

/* ===== Comandos ===== */

/* Envia un comando (CS ya seleccionado) y devuelve la respuesta R1 */
static uint8_t sd_send_command(uint8_t command, uint32_t argument) {
    // El CRC solo se verifica en CMD0 y CMD8; en el resto se manda 0xFF
    uint8_t crc = SD_DUMMY_BYTE;
    if (command == SD_CMD0) {
        crc = 0x95u;
    } else if (command == SD_CMD8) {
        crc = 0x87u;
    }

    (void) sd_wait_ready();

    (void) sd_transfer((uint8_t)(0x40u | command));
    (void) sd_transfer((uint8_t)(argument >> 24));
    (void) sd_transfer((uint8_t)(argument >> 16));
    (void) sd_transfer((uint8_t)(argument >> 8));
    (void) sd_transfer((uint8_t)argument);
    (void) sd_transfer(crc);

    uint8_t response = SD_DUMMY_BYTE;
    for (uint8_t i = 0u; i < SD_R1_POLL_BYTES; i++) {
        response = sd_transfer(SD_DUMMY_BYTE);
        if ((response & SD_R1_INVALID_MASK) == 0u) {
            break;
        }
    }
    return response;
}

/* Comando completo: selecciona, envia, deselecciona */
static uint8_t sd_command(uint8_t command, uint32_t argument) {
    sd_select();
    uint8_t response = sd_send_command(command, argument);
    sd_deselect();
    return response;
}

/* Lee los 4 bytes que siguen a R1 (respuestas R3 y R7) */
static void sd_read_trailer(uint8_t trailer[4]) {
    for (uint8_t i = 0u; i < 4u; i++) {
        trailer[i] = sd_transfer(SD_DUMMY_BYTE);
    }
}

/* SDSC usa direccion en bytes; SDHC/SDXC, numero de bloque */
static uint32_t sd_block_address(uint32_t block_index) {
    if (sd_type == SD_TYPE_SDHC) {
        return block_index;
    }
    return block_index * SD_BLOCK_SIZE;
}

/* ===== Pasos de la inicializacion ===== */

static SD_Status_t sd_check_voltage(void) {
    uint8_t trailer[4];

    sd_select();
    uint8_t response = sd_send_command(SD_CMD8, SD_CMD8_ARG);
    if (response != SD_R1_IDLE) {
        sd_deselect();
        return SD_ERR_VERSION;
    }
    sd_read_trailer(trailer);
    sd_deselect();

    // La tarjeta repite el rango de tension y el patron enviados
    if ((trailer[2] != 0x01u) || (trailer[3] != 0xAAu)) {
        return SD_ERR_VERSION;
    }
    return SD_OK;
}

static SD_Status_t sd_wait_initialized(void) {
    uint32_t start = HAL_GetTick();
    uint8_t response;

    do {
        (void) sd_command(SD_CMD55, 0u);
        response = sd_command(SD_ACMD41, SD_ACMD41_HCS);
        if ((HAL_GetTick() - start) > SD_INIT_TIMEOUT_MS) {
            return SD_ERR_TIMEOUT;
        }
    } while (response != SD_R1_READY);

    return SD_OK;
}

static void sd_detect_type(void) {
    uint8_t ocr[4];

    sd_type = SD_TYPE_SDSC;
    sd_select();
    if (sd_send_command(SD_CMD58, 0u) == SD_R1_READY) {
        sd_read_trailer(ocr);
        if ((ocr[0] & SD_OCR_CCS_BIT) != 0u) {
            sd_type = SD_TYPE_SDHC;
        }
    }
    sd_deselect();
}

/* ===== API publica ===== */

SD_Status_t SD_Init(SPI_HandleTypeDef *hspi) {
    SD_Status_t status;

    sd_spi = hspi;
    sd_type = SD_TYPE_UNKNOWN;
    sd_ready = 0u;
    if (sd_spi == NULL) {
        return SD_ERR_NOT_READY;
    }

    sd_set_prescaler(SD_PRESCALER_INIT);

    // >= 74 ciclos de reloj con CS en alto para entrar en modo SPI
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
    sd_clock_dummy(SD_POWER_UP_BYTES);

    if (sd_command(SD_CMD0, 0u) != SD_R1_IDLE) {
        return SD_ERR_NO_RESPONSE;
    }

    status = sd_check_voltage();
    if (status != SD_OK) {
        return status;
    }

    status = sd_wait_initialized();
    if (status != SD_OK) {
        return status;
    }

    sd_detect_type();

    // Las SDSC necesitan fijar el bloque en 512 bytes
    if ((sd_type == SD_TYPE_SDSC) && (sd_command(SD_CMD16, SD_BLOCK_SIZE) != SD_R1_READY)) {
        return SD_ERR_COMMAND;
    }

    sd_set_prescaler(SD_PRESCALER_FAST);
    sd_ready = 1u;
    return SD_OK;
}

SD_Status_t SD_ReadBlock(uint8_t *buffer, uint32_t block_index) {
    if ((sd_ready == 0u) || (buffer == NULL)) {
        return SD_ERR_NOT_READY;
    }

    sd_select();
    if (sd_send_command(SD_CMD17, sd_block_address(block_index)) != SD_R1_READY) {
        sd_deselect();
        return SD_ERR_COMMAND;
    }

    // Esperar el token de inicio de datos
    uint32_t start = HAL_GetTick();
    while (sd_transfer(SD_DUMMY_BYTE) != SD_TOKEN_START_BLOCK) {
        if ((HAL_GetTick() - start) > SD_READY_TIMEOUT_MS) {
            sd_deselect();
            return SD_ERR_TIMEOUT;
        }
    }

    for (uint32_t i = 0u; i < SD_BLOCK_SIZE; i++) {
        buffer[i] = sd_transfer(SD_DUMMY_BYTE);
    }
    sd_clock_dummy(2u);     // CRC de 16 bits, descartado

    sd_deselect();
    return SD_OK;
}

SD_Status_t SD_WriteBlock(const uint8_t *buffer, uint32_t block_index) {
    if ((sd_ready == 0u) || (buffer == NULL)) {
        return SD_ERR_NOT_READY;
    }

    sd_select();
    if (sd_send_command(SD_CMD24, sd_block_address(block_index)) != SD_R1_READY) {
        sd_deselect();
        return SD_ERR_COMMAND;
    }

    (void) sd_transfer(SD_DUMMY_BYTE);          // Separacion de 1 byte
    (void) sd_transfer(SD_TOKEN_START_BLOCK);
    for (uint32_t i = 0u; i < SD_BLOCK_SIZE; i++) {
        (void) sd_transfer(buffer[i]);
    }
    sd_clock_dummy(2u);                         // CRC de 16 bits (no se verifica)

    // Data response: llega dentro de los proximos bytes
    uint8_t response = SD_DUMMY_BYTE;
    for (uint8_t i = 0u; i < SD_R1_POLL_BYTES; i++) {
        response = sd_transfer(SD_DUMMY_BYTE);
        if (response != SD_DUMMY_BYTE) {
            break;
        }
    }
    if ((response & SD_DATA_RESP_MASK) != SD_DATA_ACCEPTED) {
        sd_deselect();
        return SD_ERR_WRITE;
    }

    // Mientras graba, la tarjeta mantiene MISO en bajo (ocupada)
    uint32_t start = HAL_GetTick();
    while (sd_transfer(SD_DUMMY_BYTE) == 0x00u) {
        if ((HAL_GetTick() - start) > SD_READY_TIMEOUT_MS) {
            sd_deselect();
            return SD_ERR_TIMEOUT;
        }
    }

    sd_deselect();
    return SD_OK;
}

SD_Type_t SD_GetType(void) {
    return sd_type;
}

uint8_t SD_IsReady(void) {
    return sd_ready;
}
