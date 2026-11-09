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
    int64_t requested_us;
    uint64_t auto_epoch;
} WebCommandReceipt;
typedef enum { WEB_CMD_ACCEPTED, WEB_CMD_DUPLICATE, WEB_CMD_BUSY, WEB_CMD_CONFLICT, WEB_CMD_INVALID, WEB_CMD_UNSAFE } WebCommandResult;

#define WEB_AUTO_STATUS_MS 2000U
#define WEB_AUTO_REQUEST_MS 500U
/* AUTO usa la misma cola desde web y consola; revalidar bajo el turno UART. */
bool WebCommands_RequestConsoleAuto(void);
bool WebCommands_ClaimAuto(void);

WebCommandResult WebCommands_Submit(const char *id, UartControlCommand command, WebCommandReceipt *receipt);
/* Solo app_main; ninguna E/S se realiza dentro del bloqueo. */
void WebCommands_ProcessPending(void);
bool WebCommands_GetLatest(WebCommandReceipt *receipt);
const char *WebCommands_Name(UartControlCommand command);
const char *WebCommands_Phase(WebCommandPhase phase);

#define WEB_MANUAL_LEASE_MS 300U
#define WEB_MANUAL_STATUS_MS 2000U
typedef enum { WEB_MANUAL_OK, WEB_MANUAL_INVALID, WEB_MANUAL_BUSY, WEB_MANUAL_UNSAFE, WEB_MANUAL_EXPIRED, WEB_MANUAL_REPLAY } WebManualResult;
WebManualResult WebManual_Open(const char *request_id, char token[33]);
WebManualResult WebManual_Renew(const char *token, uint32_t sequence, UartMotionCommand command);
WebManualResult WebManual_End(const char *token);
WebManualResult WebManual_Halt(const char *request_id);
void WebManual_RequestHalt(void); /* Consola: parada siempre permitida. */
void WebManual_Process(void); /* Solo app_main. */
bool WebManual_Active(void);
/* Consulta opcional: false ante cualquier control operativo o sesion manual. */
bool WebCommands_CanPollSensors(void);
/* Arbitraje TX: invocadas con el mutex UART adquirido; ninguna realiza E/S. */
bool WebManual_Claim(uint64_t generation, uint32_t sequence, UartMotionCommand direction);
bool WebManual_TakeHalt(void);
void WebManual_TxFailed(void);
bool WebManual_ConsoleAllowed(void);
void WebManual_ControlBegin(UartControlCommand command);
void WebManual_ControlEnd(void);
#endif
