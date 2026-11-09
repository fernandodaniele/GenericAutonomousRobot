#include "web_commands.h"
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Un solo pendiente, mas el envio en curso. Historial acotado de idempotencia.
 * HTTP nunca espera UART ni respuesta STM32; no hay tareas/colas adicionales. */
static portMUX_TYPE s_command_lock = portMUX_INITIALIZER_UNLOCKED;
static WebCommandReceipt s_receipts[WEB_COMMAND_HISTORY];
static unsigned s_next_receipt;
static int s_pending = -1, s_sending = -1, s_latest = -1;

const char *WebCommands_Name(UartControlCommand command)
{
    switch (command) {
        case UART_CONTROL_STOP: return "STOP";
        case UART_CONTROL_RESET: return "RESET";
        case UART_CONTROL_MANUAL: return "MANUAL";
        default: return "";
    }
}

const char *WebCommands_Phase(WebCommandPhase phase)
{
    static const char *names[] = { "queued", "sending", "transmitted", "failed", "cancelled" };
    return (unsigned)phase < sizeof(names) / sizeof(names[0]) ? names[phase] : "failed";
}

WebCommandResult WebCommands_Submit(const char *id, UartControlCommand command, WebCommandReceipt *receipt)
{
    if (!id || !receipt || !*WebCommands_Name(command) || strlen(id) != WEB_COMMAND_ID_LENGTH) return WEB_CMD_INVALID;
    for (unsigned i = 0; i < WEB_COMMAND_ID_LENGTH; ++i)
        if (!((id[i] >= '0' && id[i] <= '9') || (id[i] >= 'a' && id[i] <= 'f'))) return WEB_CMD_INVALID;

    WebCommandResult result;
    taskENTER_CRITICAL(&s_command_lock);
    for (unsigned i = 0; i < WEB_COMMAND_HISTORY; ++i) {
        if (strcmp(s_receipts[i].id, id) == 0) {
            *receipt = s_receipts[i];
            result = s_receipts[i].command == command ? WEB_CMD_DUPLICATE : WEB_CMD_CONFLICT;
            taskEXIT_CRITICAL(&s_command_lock);
            return result;
        }
    }
    if ((s_pending >= 0 || s_sending >= 0) && command != UART_CONTROL_STOP) {
        taskEXIT_CRITICAL(&s_command_lock);
        return WEB_CMD_BUSY;
    }
    /* STOP reemplaza solo lo que aun no fue tomado para transmitir. Nunca
     * promete deshacer una trama ya enviada por la consola u otro cliente. */
    if (s_pending >= 0) s_receipts[s_pending].phase = WEB_CMD_CANCELLED;
    unsigned slot = s_next_receipt;
    if ((int)slot == s_sending) slot = (slot + 1U) % WEB_COMMAND_HISTORY;
    s_next_receipt = (slot + 1U) % WEB_COMMAND_HISTORY;
    WebCommandReceipt *entry = &s_receipts[slot];
    memcpy(entry->id, id, WEB_COMMAND_ID_LENGTH + 1U);
    entry->command = command;
    entry->phase = WEB_CMD_QUEUED;
    entry->sent_us = 0;
    s_latest = s_pending = (int)slot;
    *receipt = *entry;
    taskEXIT_CRITICAL(&s_command_lock);
    return WEB_CMD_ACCEPTED;
}

void WebCommands_ProcessPending(void)
{
    taskENTER_CRITICAL(&s_command_lock);
    const int slot = s_pending;
    if (slot < 0) { taskEXIT_CRITICAL(&s_command_lock); return; }
    s_pending = -1;
    s_sending = slot;
    s_receipts[slot].phase = WEB_CMD_SENDING;
    const UartControlCommand command = s_receipts[slot].command;
    taskEXIT_CRITICAL(&s_command_lock);

    const bool sent = Uart_SendControl(command);
    const int64_t sent_us = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    s_receipts[slot].phase = sent ? WEB_CMD_TRANSMITTED : WEB_CMD_FAILED;
    s_receipts[slot].sent_us = sent_us;
    s_sending = -1;
    taskEXIT_CRITICAL(&s_command_lock);
}

bool WebCommands_GetLatest(WebCommandReceipt *receipt)
{
    if (!receipt) return false;
    taskENTER_CRITICAL(&s_command_lock);
    const bool found = s_latest >= 0;
    if (found) *receipt = s_receipts[s_latest];
    taskEXIT_CRITICAL(&s_command_lock);
    return found;
}
