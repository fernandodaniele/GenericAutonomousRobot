/**
 * @file    drv_sd_spi.c
 * @brief   Driver de bajo nivel para tarjetas SD via SPI1.
 */

#include "drv_sd_spi.h"
#include "spi.h"
#include "stm32f4xx_hal.h"

/* CS: PA4 (I2S3_WS en Discovery — libre mientras I2S3 no esté inicializado) */
#define SD_CS_PORT  GPIOA
#define SD_CS_PIN   GPIO_PIN_4

#define SD_CS_LOW()   HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()  HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)

/* Comandos SD (byte de inicio = 0x40 | número de comando) */
#define SD_CMD0   (0x40u |  0u)   /* GO_IDLE_STATE        */
#define SD_CMD8   (0x40u |  8u)   /* SEND_IF_COND         */
#define SD_CMD55  (0x40u | 55u)   /* APP_CMD              */
#define SD_CMD58  (0x40u | 58u)   /* READ_OCR             */
#define SD_ACMD41 (0x40u | 41u)   /* SD_SEND_OP_COND      */

/* Bit de R1: idle state (esperado después de CMD0) */
#define R1_IDLE  0x01u

static SD_Type_t s_card_type = SD_TYPE_UNKNOWN;

/* ── SPI helpers ──────────────────────────────────────────────────────────── */

static uint8_t spi_byte(uint8_t tx) {
    uint8_t rx = 0;
    /* Timeout de 10 ms: evita colgarse si el bus SPI no responde */
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 10);
    return rx;
}

/* Cambia el baud rate de SPI1 en tiempo de ejecución.
 * Deshabilitar el periférico es obligatorio antes de tocar CR1.BR (RM 28.5.1). */
static void spi_set_prescaler(uint32_t prescaler_bits) {
    __HAL_SPI_DISABLE(&hspi2);
    MODIFY_REG(hspi2.Instance->CR1, SPI_CR1_BR_Msk, prescaler_bits);
    __HAL_SPI_ENABLE(&hspi2);
}

/* ── Protocolo SD ─────────────────────────────────────────────────────────── */

/* Espera hasta que la línea MISO esté libre (0xFF). Timeout 500 ms. */
static uint8_t sd_wait_ready(void) {
    uint32_t deadline = HAL_GetTick() + 500u;
    uint8_t r;
    do {
        r = spi_byte(0xFF);
    } while (r != 0xFF && HAL_GetTick() < deadline);
    return r;
}

/* Envía el paquete de 6 bytes del comando con CS ya en bajo.
 * Retorna el byte R1 (MSB=0 indica respuesta válida). */
static uint8_t sd_send_cmd_raw(uint8_t cmd, uint32_t arg, uint8_t crc) {
    spi_byte(cmd);
    spi_byte((uint8_t)(arg >> 24));
    spi_byte((uint8_t)(arg >> 16));
    spi_byte((uint8_t)(arg >>  8));
    spi_byte((uint8_t)(arg      ));
    spi_byte(crc);

    /* Espera respuesta: hasta 8 bytes de relleno (spec §7.5.1) */
    uint8_t r = 0xFF;
    for (int n = 8; n; n--) {
        r = spi_byte(0xFF);
        if (!(r & 0x80u)) break;
    }
    return r;
}

/* Manda un comando SD: assert CS → dummy byte → cmd → deassert CS → dummy. */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    sd_wait_ready();
    SD_CS_LOW();
    spi_byte(0xFF);
    uint8_t r = sd_send_cmd_raw(cmd, arg, crc);
    SD_CS_HIGH();
    spi_byte(0xFF);
    return r;
}

/* Igual que sd_cmd pero además lee 4 bytes de respuesta extendida (R3/R7).
 * El buffer 'out' solo se escribe si R1 es válido (≤ 0x01). */
static uint8_t sd_cmd_r3r7(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t out[4]) {
    sd_wait_ready();
    SD_CS_LOW();
    spi_byte(0xFF);
    uint8_t r = sd_send_cmd_raw(cmd, arg, crc);
    if (r <= 0x01u) {
        for (int i = 0; i < 4; i++) out[i] = spi_byte(0xFF);
    }
    SD_CS_HIGH();
    spi_byte(0xFF);
    return r;
}

/* ACMD = CMD55 (prefijo de aplicación) seguido del comando de aplicación. */
static uint8_t sd_acmd(uint8_t acmd, uint32_t arg) {
    sd_cmd(SD_CMD55, 0, 0x01u);
    return sd_cmd(acmd, arg, 0x01u);
}

/* ── API pública ──────────────────────────────────────────────────────────── */

SD_Status_t SD_Init(void) {
    /* Configura PA4 como salida push-pull para CS */
    GPIO_InitTypeDef g = {0};
    g.Pin   = SD_CS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_CS_PORT, &g);
    SD_CS_HIGH();

    /* 84 MHz / 256 ≈ 328 kHz — obligatorio para init (SD spec: ≤400 kHz) */
    spi_set_prescaler(SPI_BAUDRATEPRESCALER_256);

    /* ≥74 ciclos de reloj con CS en alto para despertar la tarjeta (spec §6.4.1) */
    for (int i = 0; i < 10; i++) spi_byte(0xFF);

    /* CMD0: reset de software → entra en modo SPI. Respuesta esperada: 0x01 */
    uint8_t r = sd_cmd(SD_CMD0, 0, 0x95u);
    if (r != R1_IDLE) return SD_ERR_NO_RESPONSE;

    /* CMD8: detecta SD v2. arg = 0x000001AA (rango de voltaje + check pattern) */
    uint8_t resp[4];
    r = sd_cmd_r3r7(SD_CMD8, 0x000001AAu, 0x87u, resp);

    if (r == R1_IDLE) {
        /* ── SD v2 ── */

        /* Verifica que el check pattern se repita (indica comunicación válida).
         * El campo de voltaje (resp[2]) varía entre fabricantes; no se verifica
         * ya que la tarjeta es alimentada a 3.3V que es compatible con el estándar. */
        if (resp[3] != 0xAAu) return SD_ERR_VOLTAGE;

        /* ACMD41 con HCS=1: loop hasta que la tarjeta salga del estado idle */
        uint32_t deadline = HAL_GetTick() + 1000u;
        do {
            r = sd_acmd(SD_ACMD41, 0x40000000u);
        } while (r == R1_IDLE && HAL_GetTick() < deadline);

        if (r != 0x00u) return SD_ERR_TIMEOUT;

        /* CMD58: lee OCR; bit 30 (CCS) distingue SDHC de SDSC */
        sd_cmd_r3r7(SD_CMD58, 0, 0x01u, resp);
        s_card_type = (resp[0] & 0x40u) ? SD_TYPE_SDHC : SD_TYPE_V2;

    } else {
        /* ── SD v1 (CMD8 no reconocido) ── */
        uint32_t deadline = HAL_GetTick() + 1000u;
        do {
            r = sd_acmd(SD_ACMD41, 0);
        } while (r == R1_IDLE && HAL_GetTick() < deadline);

        if (r != 0x00u) return SD_ERR_TIMEOUT;
        s_card_type = SD_TYPE_V1;
    }

    /* Tarjeta lista → sube a 21 MHz (84 / 4); SD estándar acepta hasta 25 MHz */
    spi_set_prescaler(SPI_BAUDRATEPRESCALER_4);

    return SD_OK;
}

SD_Type_t SD_GetType(void) {
    return s_card_type;
}
