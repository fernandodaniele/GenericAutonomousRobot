#include "uart_comm.h"
#include "stm32_status.h"

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

void Uart_SendStatus(void)
{
    const char *message = "STATUS\n";

    uart_write_bytes(UART_PORT, message, strlen(message));

    ESP_LOGI(TAG, "Enviado a STM32: STATUS");
}

bool Uart_SendControl(UartControlCommand command)
{
    const char *message;
    switch (command) {
        case UART_CONTROL_STOP: message = "STOP\n"; break;
        case UART_CONTROL_RESET: message = "RESET\n"; break;
        case UART_CONTROL_MANUAL: message = "MANUAL\n"; break;
        default: return false;
    }
    const size_t length = strlen(message);
    /* Una trama por llamada: tx_mux del SDK serializa consola y app_main. */
    const bool sent = uart_write_bytes(UART_PORT, message, length) == (int)length;
    if (sent) ESP_LOGI(TAG, "Entregado a UART: %s", message);
    else ESP_LOGE(TAG, "Fallo de entrega UART; sin reintento");
    return sent;
}

void Uart_SendReset(void) { (void)Uart_SendControl(UART_CONTROL_RESET); }
void Uart_SendStop(void) { (void)Uart_SendControl(UART_CONTROL_STOP); }
void Uart_SendManual(void) { (void)Uart_SendControl(UART_CONTROL_MANUAL); }

/* Una pulsacion explicita = una trama; sin temporizador de renovacion. */
static void Uart_SendMotion(const char *message) {
    uart_write_bytes(UART_PORT, message, strlen(message));
    ESP_LOGI(TAG, "Movimiento enviado: %s", message);
}

void Uart_SendForward(void) { Uart_SendMotion("AVANZAR\n"); }
void Uart_SendReverse(void) { Uart_SendMotion("RETROCEDER\n"); }
void Uart_SendLeft(void) { Uart_SendMotion("GIRAR_IZQUIERDA\n"); }
void Uart_SendRight(void) { Uart_SendMotion("GIRAR_DERECHA\n"); }
void Uart_SendHalt(void) { Uart_SendMotion("DETENER\n"); }

void Uart_ReadResponse(void) {
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];

    int length = uart_read_bytes(UART_PORT, rx_buffer,
                                 UART_RX_BUFFER_SIZE - 1,
                                 pdMS_TO_TICKS(100));
    if (length > 0) {
        Stm32Status_Feed(rx_buffer, (size_t)length);
        rx_buffer[length] = '\0';
        printf("[STM32] %s", (char *)rx_buffer);
    }
}
