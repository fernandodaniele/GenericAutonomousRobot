#include "drv_sd_spi.h"

#define SD_TIMEOUT_MS   500
#define SD_DUMMY        0xFF
#define SD_TOKEN_START  0xFE
#define SD_BLOCK_SIZE   512

// Comandos SD en modo SPI
#define CMD0   0    // GO_IDLE_STATE
#define CMD8   8    // SEND_IF_COND
#define CMD16  16   // SET_BLOCKLEN (solo SDSC)
#define CMD17  17   // READ_SINGLE_BLOCK
#define CMD24  24   // WRITE_BLOCK
#define CMD55  55   // APP_CMD (precede a ACMD)
#define CMD58  58   // READ_OCR
#define ACMD41 41   // SD_SEND_OP_COND

static SPI_HandleTypeDef *sd_spi = NULL;
static uint8_t sd_is_hc = 0;  // 1 = SDHC/SDXC (addressing por bloque)

/* ---------- utilidades SPI ---------- */

static uint8_t spi_byte(uint8_t tx) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(sd_spi, &tx, &rx, 1, SD_TIMEOUT_MS);
    return rx;
}

static void spi_dummy(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) spi_byte(SD_DUMMY);
}

static void sd_select(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void sd_deselect(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
    spi_byte(SD_DUMMY);  // ciclo extra para liberar MISO
}

// Espera hasta que la tarjeta libere MISO (fin de busy o fin de comando previo)
static uint8_t sd_wait_ready(void) {
    uint32_t tick = HAL_GetTick();
    uint8_t r;
    do {
        r = spi_byte(SD_DUMMY);
    } while (r != 0xFF && (HAL_GetTick() - tick) < SD_TIMEOUT_MS);
    return r;
}

/* ---------- protocolo de comando ---------- */

// Envía un comando SD y retorna la respuesta R1
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg) {
    // CRC válido solo para CMD0 y CMD8; el resto usa 0xFF (CRC deshabilitado)
    uint8_t crc = (cmd == CMD0) ? 0x95 : (cmd == CMD8) ? 0x87 : 0xFF;

    sd_wait_ready();

    spi_byte(0x40 | cmd);
    spi_byte((arg >> 24) & 0xFF);
    spi_byte((arg >> 16) & 0xFF);
    spi_byte((arg >> 8)  & 0xFF);
    spi_byte( arg        & 0xFF);
    spi_byte(crc);

    // La respuesta puede tardar hasta 8 ciclos de clock
    uint8_t r = 0xFF;
    for (uint8_t i = 0; i < 8; i++) {
        r = spi_byte(SD_DUMMY);
        if (!(r & 0x80)) break;
    }
    return r;
}

static void sd_set_high_speed(void) {
    sd_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  // 42 MHz / 4 = 10.5 MHz
    HAL_SPI_Init(sd_spi);
}

/* ---------- API pública ---------- */

SD_Status_t SD_Init(SPI_HandleTypeDef *hspi) {
    sd_spi   = hspi;
    sd_is_hc = 0;

    // ≥74 ciclos de clock con CS en alto para que la tarjeta entre en modo SPI
    sd_deselect();
    spi_dummy(10);  // 80 ciclos

    // CMD0: resetear tarjeta → espera R1 = 0x01 (idle)
    sd_select();
    if (sd_cmd(CMD0, 0) != 0x01) { sd_deselect(); return SD_ERROR; }
    sd_deselect();

    // CMD8: comprobar rango de voltaje (solo SDv2 responde)
    // Argumento: VHS = 1 (2.7–3.6 V), check pattern = 0xAA
    sd_select();
    uint8_t r = sd_cmd(CMD8, 0x000001AA);
    if (r != 0x01) { sd_deselect(); return SD_ERROR; }  // SDv1/MMC no soportado

    uint8_t r7[4];
    for (int i = 0; i < 4; i++) r7[i] = spi_byte(SD_DUMMY);
    sd_deselect();

    // El campo de voltaje y el patrón deben ser el eco del argumento
    if (r7[2] != 0x01 || r7[3] != 0xAA) return SD_ERROR;

    // ACMD41 con HCS=1: inicializar la tarjeta (repetir hasta R1 = 0x00)
    uint32_t tick = HAL_GetTick();
    do {
        sd_select(); sd_cmd(CMD55, 0);             sd_deselect();
        sd_select(); r = sd_cmd(ACMD41, 0x40000000); sd_deselect();
        if ((HAL_GetTick() - tick) > 1000) return SD_TIMEOUT;
    } while (r != 0x00);

    // CMD58: leer OCR para saber si es SDHC/SDXC (bit CCS = bit 30)
    sd_select();
    if (sd_cmd(CMD58, 0) == 0x00) {
        uint8_t ocr[4];
        for (int i = 0; i < 4; i++) ocr[i] = spi_byte(SD_DUMMY);
        sd_is_hc = (ocr[0] & 0x40) ? 1 : 0;
    }
    sd_deselect();

    // CMD16: fijar tamaño de bloque en 512 bytes (requerido solo para SDSC)
    if (!sd_is_hc) {
        sd_select();
        if (sd_cmd(CMD16, 512) != 0x00) { sd_deselect(); return SD_ERROR; }
        sd_deselect();
    }

    sd_set_high_speed();
    return SD_OK;
}

SD_Status_t SD_ReadBlock(uint8_t *buf, uint32_t block_addr) {
    // SDHC usa dirección de bloque; SDSC usa dirección de byte
    uint32_t addr = sd_is_hc ? block_addr : block_addr * 512;

    sd_select();
    if (sd_cmd(CMD17, addr) != 0x00) { sd_deselect(); return SD_ERROR; }

    // Esperar token de inicio de datos (0xFE)
    uint32_t tick = HAL_GetTick();
    uint8_t token;
    do {
        token = spi_byte(SD_DUMMY);
        if ((HAL_GetTick() - tick) > SD_TIMEOUT_MS) { sd_deselect(); return SD_TIMEOUT; }
    } while (token != SD_TOKEN_START);

    for (int i = 0; i < SD_BLOCK_SIZE; i++) buf[i] = spi_byte(SD_DUMMY);
    spi_byte(SD_DUMMY);  // CRC byte 1 (descartado)
    spi_byte(SD_DUMMY);  // CRC byte 2 (descartado)

    sd_deselect();
    return SD_OK;
}

SD_Status_t SD_WriteBlock(const uint8_t *buf, uint32_t block_addr) {
    uint32_t addr = sd_is_hc ? block_addr : block_addr * 512;
    uint8_t resp;
    uint32_t tick;

    sd_select();
    if (sd_cmd(CMD24, addr) != 0x00) { sd_deselect(); return SD_ERROR; }

    spi_byte(SD_DUMMY);       // gap de 1 byte antes del token de datos
    spi_byte(SD_TOKEN_START);

    for (int i = 0; i < SD_BLOCK_SIZE; i++) spi_byte(buf[i]);
    spi_byte(SD_DUMMY);  // CRC dummy byte 1
    spi_byte(SD_DUMMY);  // CRC dummy byte 2

    // El token de respuesta de escritura puede tardar hasta 8 ciclos
    resp = 0xFF;
    for (int i = 0; i < 8; i++) {
        resp = spi_byte(SD_DUMMY);
        if (resp != 0xFF) break;
    }
    // Bits [4:0]: 00101 = aceptado, 01011 = error CRC, 01101 = error escritura
    if ((resp & 0x1F) != 0x05) { sd_deselect(); return SD_ERROR; }

    // Esperar fin de escritura (tarjeta señaliza busy manteniendo MISO en bajo)
    tick = HAL_GetTick();
    while (spi_byte(SD_DUMMY) == 0x00) {
        if ((HAL_GetTick() - tick) > SD_TIMEOUT_MS) { sd_deselect(); return SD_TIMEOUT; }
    }

    sd_deselect();
    return SD_OK;
}
