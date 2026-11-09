#ifndef WEB_COMMANDS_H
#define WEB_COMMANDS_H
#include "uart_comm.h"
#include <stdint.h>

#define WEB_COMMAND_ID_LENGTH 32U
#define WEB_COMMAND_HISTORY 8U
typedef enum { WEB_CMD_QUEUED, WEB_CMD_SENDING, WEB_CMD_TRANSMITTED, WEB_CMD_FAILED, WEB_CMD_CANCELLED } WebCommandPhase;
typedef struct {
    char id[WEB_COMMAND_ID_LENGTH + 1U];
    UartControlCommand command;
    WebCommandPhase phase;
    int64_t sent_us;
} WebCommandReceipt;
typedef enum { WEB_CMD_ACCEPTED, WEB_CMD_DUPLICATE, WEB_CMD_BUSY, WEB_CMD_CONFLICT, WEB_CMD_INVALID } WebCommandResult;

WebCommandResult WebCommands_Submit(const char *id, UartControlCommand command, WebCommandReceipt *receipt);
/* Solo app_main; ninguna E/S se realiza dentro del bloqueo. */
void WebCommands_ProcessPending(void);
bool WebCommands_GetLatest(WebCommandReceipt *receipt);
const char *WebCommands_Name(UartControlCommand command);
const char *WebCommands_Phase(WebCommandPhase phase);
#endif
