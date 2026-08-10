#include "uart_comm.h"

#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ── Configuración de pines y puerto ─────────────────────────────────────── */
#define UART_PORT           UART_NUM_2
#define UART_TX_PIN         GPIO_NUM_17
#define UART_RX_PIN         GPIO_NUM_16
#define UART_BAUD_RATE      115200
#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 0

static const char *TAG = "ESP32_UART";

/* ── Implementación ──────────────────────────────────────────────────────── */

void Uart_Init(void) {
    uart_config_t uart_config = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART inicializada TX=%d RX=%d @ %d baud",
             UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
}

void Uart_SendPing(void) {
    const char *message = "PING\n";
    uart_write_bytes(UART_PORT, message, strlen(message));
    ESP_LOGI(TAG, "Enviado a STM32: PING");
}

void Uart_ReadResponse(void) {
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];

    int length = uart_read_bytes(UART_PORT, rx_buffer,
                                 UART_RX_BUFFER_SIZE - 1,
                                 pdMS_TO_TICKS(100));
    if (length > 0) {
        rx_buffer[length] = '\0';
        printf("[STM32] %s", (char *)rx_buffer);
    }
}
