#include "uart_comm.h"
#include "stm32_status.h"
#include "stm32_sensors.h"
#include "stm32_mapping.h"
#include "web_commands.h"

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* ── Configuración de pines y puerto ─────────────────────────────────────── */
#define UART_PORT           UART_NUM_2
#define UART_TX_PIN         GPIO_NUM_17
#define UART_RX_PIN         GPIO_NUM_16
#define UART_BAUD_RATE      115200
#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 0

static const char *TAG = "ESP32_UART";
static StaticSemaphore_t s_tx_storage;
static SemaphoreHandle_t s_tx_gate;

/* Todos los productores pasan por este turno, siempre fuera de portMUX.
 * Esperar FIFO vacio ANTES de validar el permiso evita esperas de cola tras
 * la comprobacion final. Las tramas son menores que el FIFO del ESP32. */
static bool TxBegin(void)
{
    if (!s_tx_gate || xSemaphoreTake(s_tx_gate, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    if (uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(20)) != ESP_OK) {
        xSemaphoreGive(s_tx_gate); return false;
    }
    return true;
}
static bool TxWrite(const char *message)
{
    const size_t length = strlen(message);
    return uart_write_bytes(UART_PORT, message, length) == (int)length;
}
static bool SendOrdinary(const char *message)
{
    if (!TxBegin()) return false;
    const bool sent = TxWrite(message); xSemaphoreGive(s_tx_gate); return sent;
}

/* ── Implementación ──────────────────────────────────────────────────────── */

void Uart_Init(void) {
    if (!s_tx_gate) s_tx_gate = xSemaphoreCreateMutexStatic(&s_tx_storage);
    if (!s_tx_gate) ESP_LOGE(TAG, "No se pudo crear el turno TX; transmisiones inhibidas");
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
    if (!SendOrdinary(message)) { ESP_LOGW(TAG, "PING no entregado a UART"); return; }
    ESP_LOGI(TAG, "Enviado a STM32: PING");
}

void Uart_SendStatus(void)
{
    const char *message = "STATUS\n";

    if (!SendOrdinary(message)) { ESP_LOGW(TAG, "STATUS no entregado a UART"); return; }

    ESP_LOGI(TAG, "Enviado a STM32: STATUS");
}

void Uart_PollSensors(void)
{
    if (!Stm32Sensors_PollDue()) return;
    Stm32Sensors_DeferPoll(); /* Tambien ante omision/error: sin rafagas/reintentos. */
    if (!WebCommands_CanPollSensors() || !TxBegin()) return;
    /* Revalidar despues de cualquier espera del turno TX. */
    if (WebCommands_CanPollSensors()) {
        char message[24];
        uint32_t query = Stm32Sensors_BeginQuery();
        int length = snprintf(message, sizeof(message), "SENSORS Q=%" PRIu32 "\n", query);
        if (length <= 0 || (size_t)length >= sizeof(message) || !TxWrite(message)) {
            Stm32Sensors_QueryFailed();
            ESP_LOGW(TAG, "SENSORS no entregado a UART; sin reintento inmediato");
        }
    }
    xSemaphoreGive(s_tx_gate);
}

void Uart_PollMapping(void)
{
    if(!Stm32Mapping_PollDue()) return;
    Stm32Mapping_DeferPoll();
    if(!WebCommands_CanPollMapping()||!TxBegin()) return;
    if(WebCommands_CanPollMapping()) {
        char message[40];unsigned part;
        uint32_t query=Stm32Mapping_BeginQuery(&part);
        int n=snprintf(message,sizeof(message),"MAPPING Q=%" PRIu32 " S=%u\n",query,part);
        if(n<=0||(size_t)n>=sizeof(message)||!TxWrite(message)) Stm32Mapping_QueryFailed();
    }
    xSemaphoreGive(s_tx_gate);
}

bool Uart_SendControl(UartControlCommand command)
{
    const char *message;
    switch (command) {
        case UART_CONTROL_STOP: message = "STOP\n"; break;
        case UART_CONTROL_RESET: message = "RESET\n"; break;
        case UART_CONTROL_MANUAL: message = "MANUAL\n"; break;
        case UART_CONTROL_AUTO: message = "AUTO\n"; break;
        default: return false;
    }
    WebManual_ControlBegin(command);
    bool sent = false;
    if (TxBegin()) {
        const bool halted = !WebManual_TakeHalt() || TxWrite("DETENER\n");
        if ((halted || command == UART_CONTROL_STOP) &&
            (command != UART_CONTROL_AUTO || WebCommands_ClaimAuto())) sent = TxWrite(message);
        xSemaphoreGive(s_tx_gate);
    }
    WebManual_ControlEnd();
    if (sent) ESP_LOGI(TAG, "Entregado a UART: %s", message);
    else ESP_LOGE(TAG, "Fallo de entrega UART; sin reintento");
    return sent;
}

void Uart_SendReset(void) { (void)Uart_SendControl(UART_CONTROL_RESET); }
void Uart_SendStop(void) { (void)Uart_SendControl(UART_CONTROL_STOP); }
void Uart_SendManual(void) { (void)Uart_SendControl(UART_CONTROL_MANUAL); }

void Uart_SendAuto(void)
{
    if (WebCommands_RequestConsoleAuto()) ESP_LOGI(TAG, "AUTO encolado; pendiente de validar y transmitir");
    else ESP_LOGW(TAG, "AUTO rechazado: estado no seguro, antiguo o control pendiente");
}

/* Una pulsacion explicita = una trama; sin temporizador de renovacion. */
static void Uart_SendMotion(const char *message) {
    if (!TxBegin()) return;
    const bool allowed = WebManual_ConsoleAllowed();
    if (allowed) (void)TxWrite(message);
    xSemaphoreGive(s_tx_gate);
    if (allowed) ESP_LOGI(TAG, "Movimiento de consola enviado: %s", message);
    else ESP_LOGW(TAG, "Direccion de consola rechazada: autorizacion web/control activo");
}

void Uart_SendForward(void) { Uart_SendMotion("AVANZAR\n"); }
void Uart_SendReverse(void) { Uart_SendMotion("RETROCEDER\n"); }
void Uart_SendLeft(void) { Uart_SendMotion("GIRAR_IZQUIERDA\n"); }
void Uart_SendRight(void) { Uart_SendMotion("GIRAR_DERECHA\n"); }
void Uart_SendHalt(void) { WebManual_RequestHalt(); (void)Uart_SendPendingHalt(); }

bool Uart_SendPendingHalt(void)
{
    if (!TxBegin()) {
        if (WebManual_TakeHalt()) ESP_LOGW(TAG, "DETENER consumido sin entrega a UART");
        return false;
    }
    const bool requested = WebManual_TakeHalt();
    const bool sent = !requested || TxWrite("DETENER\n");
    xSemaphoreGive(s_tx_gate);
    if (requested) {
        if (sent) ESP_LOGI(TAG, "Entregado a UART: DETENER");
        else ESP_LOGW(TAG, "DETENER consumido sin entrega a UART");
    }
    return sent;
}

bool Uart_SendLeasedMotion(UartMotionCommand command, uint64_t generation, uint32_t sequence)
{
    static const char *messages[] = { "AVANZAR\n", "RETROCEDER\n", "GIRAR_IZQUIERDA\n", "GIRAR_DERECHA\n" };
    if ((unsigned)command >= sizeof(messages) / sizeof(messages[0])) return false;
    if (!TxBegin()) { WebManual_TxFailed(); return false; }
    bool sent = false;
    if (WebManual_Claim(generation, sequence, command)) {
        sent = TxWrite(messages[command]);
        if (!sent) WebManual_TxFailed();
    }
    xSemaphoreGive(s_tx_gate);
    if (sent) ESP_LOGI(TAG, "Entregado a UART: %.*s",
                       (int)(strlen(messages[command]) - 1U), messages[command]);
    return sent;
}

void Uart_ReadResponse(void) {
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];

    int length = uart_read_bytes(UART_PORT, rx_buffer,
                                 UART_RX_BUFFER_SIZE - 1,
                                 pdMS_TO_TICKS(100));
    if (length > 0) {
        Stm32Status_Feed(rx_buffer, (size_t)length);
        Stm32Sensors_Feed(rx_buffer, (size_t)length);
        Stm32Mapping_Feed(rx_buffer, (size_t)length);
        rx_buffer[length] = '\0';
        printf("[STM32] %s", (char *)rx_buffer);
    }
}
