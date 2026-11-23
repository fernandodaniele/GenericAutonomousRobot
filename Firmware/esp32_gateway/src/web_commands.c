#include "web_commands.h"
#include <string.h>
#include "esp_timer.h"
#include "esp_random.h"
#include "stm32_status.h"
#include "stm32_exploration.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Un solo pendiente, mas el envio en curso. Historial acotado de idempotencia.
 * HTTP nunca espera UART ni respuesta STM32; no hay tareas/colas adicionales. */
static portMUX_TYPE s_command_lock = portMUX_INITIALIZER_UNLOCKED;
static WebCommandReceipt s_receipts[WEB_COMMAND_HISTORY];
static unsigned s_next_receipt;
static int s_pending = -1, s_sending = -1, s_latest = -1;
static uint64_t s_auto_epoch;
static int64_t s_auto_barrier;
static bool s_auto_has_barrier;

static void AutoInvalidateLocked(int64_t now)
{
    ++s_auto_epoch;
    s_auto_barrier = now;
    s_auto_has_barrier = true;
}

static bool AutoSafeLocked(const Stm32StatusSnapshot *status, int64_t now)
{
    return status->valid && status->state == 0U && status->faults == 0U && status->stop == 0U &&
        now >= status->received_us && now - status->received_us < (int64_t)WEB_AUTO_STATUS_MS * 1000 &&
        (!s_auto_has_barrier || status->received_us > s_auto_barrier);
}

/* Mismo cerrojo que W3: no se anidan secciones criticas entre arbitros. */
static struct {
    bool active, pending, halt;
    char token[33];
    uint64_t generation;
    uint32_t sequence;
    int64_t expires_us;
    UartMotionCommand direction;
} s_manual;
static char s_manual_ids[8][33];
static unsigned s_manual_next, s_manual_controls;
static int64_t s_manual_barrier;
static bool s_manual_has_barrier;
static void ManualRevokeLocked(bool halt, int64_t now)
{
    if (halt && (s_manual.active || s_manual.pending)) s_manual.halt = true;
    s_manual.active = s_manual.pending = false;
    s_manual.token[0] = '\0';
    ++s_manual.generation;
    s_manual_barrier = now;
    s_manual_has_barrier = true;
}

const char *WebCommands_Name(UartControlCommand command)
{
    switch (command) {
        case UART_CONTROL_STOP: return "STOP";
        case UART_CONTROL_RESET: return "RESET";
        case UART_CONTROL_MANUAL: return "MANUAL";
        case UART_CONTROL_AUTO: return "AUTO";
        case UART_CONTROL_AUTO_TURN: return "AUTO_TURN";
        case UART_CONTROL_AUTO_EVADE: return "AUTO_EVADE";
        case UART_CONTROL_AUTO_CONTINUOUS: return "AUTO_CONTINUOUS";
        case UART_CONTROL_AUTO_EXPLORE: return "AUTO_EXPLORE";
        default: return "";
    }
}

const char *WebCommands_Phase(WebCommandPhase phase)
{
    static const char *names[] = { "queued", "sending", "transmitted", "failed", "cancelled" };
    return (unsigned)phase < sizeof(names) / sizeof(names[0]) ? names[phase] : "failed";
}

static WebCommandResult WebCommands_SubmitOwned(const char *id, UartControlCommand command, WebCommandReceipt *receipt,const char *owner)
{
    if (!id || !receipt || !*WebCommands_Name(command) || strlen(id) != WEB_COMMAND_ID_LENGTH) return WEB_CMD_INVALID;
    for (unsigned i = 0; i < WEB_COMMAND_ID_LENGTH; ++i)
        if (!((id[i] >= '0' && id[i] <= '9') || (id[i] >= 'a' && id[i] <= 'f'))) return WEB_CMD_INVALID;

    if(command==UART_CONTROL_AUTO_EXPLORE){
        if(!owner || strlen(owner)!=32U || !strcmp(owner,id))return WEB_CMD_INVALID;
        for(unsigned i=0;i<32U;++i)if(!((owner[i]>='0'&&owner[i]<='9')||(owner[i]>='a'&&owner[i]<='f')))return WEB_CMD_INVALID;
    }
    WebCommandResult result;
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    const int64_t manual_now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    for (unsigned i = 0; i < WEB_COMMAND_HISTORY; ++i) {
        if (strcmp(s_receipts[i].id, id) == 0) {
            *receipt = s_receipts[i];
            result = s_receipts[i].command == command && (command!=UART_CONTROL_AUTO_EXPLORE || !strcmp(s_receipts[i].explore_owner,owner)) ? WEB_CMD_DUPLICATE : WEB_CMD_CONFLICT;
            taskEXIT_CRITICAL(&s_command_lock);
            return result;
        }
    }
    if ((s_pending >= 0 || s_sending >= 0) && command != UART_CONTROL_STOP) {
        taskEXIT_CRITICAL(&s_command_lock);
        return WEB_CMD_BUSY;
    }
    if (Uart_ControlIsAuto(command) &&
        (!AutoSafeLocked(&status, manual_now) || s_manual_controls != 0U)) {
        taskEXIT_CRITICAL(&s_command_lock);
        return WEB_CMD_UNSAFE;
    }
    /* STOP reemplaza solo lo que aun no fue tomado para transmitir. Nunca
     * promete deshacer una trama ya enviada por la consola u otro cliente. */
    if (s_pending >= 0) s_receipts[s_pending].phase = WEB_CMD_CANCELLED;
    unsigned slot = s_next_receipt;
    if ((int)slot == s_sending) slot = (slot + 1U) % WEB_COMMAND_HISTORY;
    s_next_receipt = (slot + 1U) % WEB_COMMAND_HISTORY;
    WebCommandReceipt *entry = &s_receipts[slot];
    ManualRevokeLocked(command != UART_CONTROL_STOP, manual_now);
    if (!Uart_ControlIsAuto(command)) AutoInvalidateLocked(manual_now);
    if (command == UART_CONTROL_STOP) s_manual.halt = false;
    memcpy(entry->id, id, WEB_COMMAND_ID_LENGTH + 1U);
    entry->explore_owner[0]=0;if(command==UART_CONTROL_AUTO_EXPLORE)memcpy(entry->explore_owner,owner,33);
    entry->command = command;
    entry->phase = WEB_CMD_QUEUED;
    entry->sent_us = 0;
    entry->requested_us = manual_now;
    entry->auto_epoch = s_auto_epoch;
    s_latest = s_pending = (int)slot;
    *receipt = *entry;
    taskEXIT_CRITICAL(&s_command_lock);
    if(!Uart_ControlIsAuto(command))Stm32Exploration_Revoke();
    return WEB_CMD_ACCEPTED;
}

WebCommandResult WebCommands_Submit(const char *id,UartControlCommand command,WebCommandReceipt *receipt){return WebCommands_SubmitOwned(id,command,receipt,NULL);}
WebCommandResult WebCommands_SubmitExplore(const char *id,const char *owner,WebCommandReceipt *receipt){return WebCommands_SubmitOwned(id,UART_CONTROL_AUTO_EXPLORE,receipt,owner);}
bool WebCommands_RequestConsoleAuto(void)
{
    uint8_t random[16]; char id[33];
    static const char hex[] = "0123456789abcdef";
    esp_fill_random(random, sizeof(random));
    for (unsigned i = 0; i < sizeof(random); ++i) {
        id[2*i] = hex[random[i] >> 4]; id[2*i+1] = hex[random[i] & 15U];
    }
    id[32] = '\0';
    WebCommandReceipt receipt;
    return WebCommands_Submit(id, UART_CONTROL_AUTO, &receipt) == WEB_CMD_ACCEPTED;
}

bool WebCommands_ExploreOwner(char owner[33]){
    taskENTER_CRITICAL(&s_command_lock);bool ok=s_sending>=0 && s_pending<0 && s_receipts[s_sending].command==UART_CONTROL_AUTO_EXPLORE &&
        s_receipts[s_sending].phase==WEB_CMD_SENDING && s_receipts[s_sending].auto_epoch+1U==s_auto_epoch;
    if(ok)memcpy(owner,s_receipts[s_sending].explore_owner,33);
    taskEXIT_CRITICAL(&s_command_lock);return ok;
}
bool WebCommands_ClaimAuto(UartControlCommand command)
{
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    bool allowed = false;
    if (s_sending >= 0) {
        WebCommandReceipt *entry = &s_receipts[s_sending];
        allowed = entry->command == command && Uart_ControlIsAuto(command) && entry->auto_epoch == s_auto_epoch &&
            entry->phase == WEB_CMD_SENDING && now >= entry->requested_us &&
            now - entry->requested_us < (int64_t)WEB_AUTO_REQUEST_MS * 1000 &&
            AutoSafeLocked(&status, now) && s_manual_controls == 1U && !s_manual.active && !s_manual.pending;
        /* Permiso de un solo uso, incluso si luego falla la UART. */
        if (allowed) ++s_auto_epoch;
    }
    taskEXIT_CRITICAL(&s_command_lock);
    return allowed;
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

static bool ManualHex(const char *value)
{
    if (!value || strlen(value) != 32U) return false;
    for (unsigned i = 0; i < 32; ++i)
        if (!((value[i] >= '0' && value[i] <= '9') || (value[i] >= 'a' && value[i] <= 'f'))) return false;
    return true;
}

static bool ManualSafeLocked(const Stm32StatusSnapshot *status, int64_t now)
{
    return status->valid && status->state == 2U && status->faults == 0U && status->stop == 0U &&
        now >= status->received_us && now - status->received_us < (int64_t)WEB_MANUAL_STATUS_MS * 1000 &&
        (!s_manual_has_barrier || status->received_us > s_manual_barrier) &&
        s_pending < 0 && s_sending < 0 && s_manual_controls == 0U;
}

static void ManualExpireLocked(const Stm32StatusSnapshot *status, int64_t now)
{
    if (s_manual.active && (now >= s_manual.expires_us || !ManualSafeLocked(status, now)))
        ManualRevokeLocked(true, now);
}

static bool ManualRememberLocked(const char *id)
{
    for (unsigned i = 0; i < 8U; ++i) if (strcmp(id, s_manual_ids[i]) == 0) return false;
    memcpy(s_manual_ids[s_manual_next], id, 33U);
    s_manual_next = (s_manual_next + 1U) % 8U;
    return true;
}

WebManualResult WebManual_Open(const char *id, char token[33])
{
    if (!ManualHex(id) || !token) return WEB_MANUAL_INVALID;
    uint8_t random[16]; char candidate[33];
    esp_fill_random(random, sizeof(random)); /* Entropia hardware con Wi-Fi activo. */
    static const char hex[] = "0123456789abcdef";
    for (unsigned i = 0; i < sizeof(random); ++i) { candidate[2*i] = hex[random[i] >> 4]; candidate[2*i+1] = hex[random[i] & 15]; }
    candidate[32] = '\0';
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    ManualExpireLocked(&status, now);
    WebManualResult result = WEB_MANUAL_OK;
    if (!ManualRememberLocked(id)) result = WEB_MANUAL_REPLAY;
    else if (!ManualSafeLocked(&status, now)) result = WEB_MANUAL_UNSAFE;
    else if (s_manual.active || s_manual.halt) result = WEB_MANUAL_BUSY;
    else {
        memcpy(s_manual.token, candidate, sizeof(candidate)); memcpy(token, candidate, sizeof(candidate));
        ++s_manual.generation; s_manual.sequence = 0U;
        s_manual.active = true; s_manual.pending = false;
        s_manual.expires_us = now + WEB_MANUAL_LEASE_MS * 1000;
    }
    taskEXIT_CRITICAL(&s_command_lock);
    return result;
}

WebManualResult WebManual_Renew(const char *token, uint32_t sequence, UartMotionCommand direction)
{
    if (!ManualHex(token) || sequence == 0U || (unsigned)direction > UART_MOTION_RIGHT) return WEB_MANUAL_INVALID;
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    ManualExpireLocked(&status, now);
    WebManualResult result = WEB_MANUAL_OK;
    if (!s_manual.active || strcmp(s_manual.token, token) != 0) result = WEB_MANUAL_EXPIRED;
    else if (sequence <= s_manual.sequence) result = WEB_MANUAL_REPLAY;
    else {
        s_manual.sequence = sequence; s_manual.direction = direction;
        s_manual.pending = true; /* Reemplaza, nunca acumula. */
        s_manual.expires_us = now + WEB_MANUAL_LEASE_MS * 1000;
    }
    taskEXIT_CRITICAL(&s_command_lock);
    return result;
}

WebManualResult WebManual_End(const char *token)
{
    if (!ManualHex(token)) return WEB_MANUAL_INVALID;
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    ManualExpireLocked(&status, now);
    const bool owner = s_manual.active && strcmp(token, s_manual.token) == 0;
    if (owner) { ManualRevokeLocked(true, now); AutoInvalidateLocked(now); }
    taskEXIT_CRITICAL(&s_command_lock);
    return owner ? WEB_MANUAL_OK : WEB_MANUAL_EXPIRED;
}

void WebManual_RequestHalt(void)
{
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    ManualRevokeLocked(true, now); s_manual.halt = true; AutoInvalidateLocked(now);
    taskEXIT_CRITICAL(&s_command_lock);
}

WebManualResult WebManual_Halt(const char *id)
{
    if (!ManualHex(id)) return WEB_MANUAL_INVALID;
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    bool fresh = ManualRememberLocked(id);
    if (fresh) { ManualRevokeLocked(true, now); s_manual.halt = true; AutoInvalidateLocked(now); }
    taskEXIT_CRITICAL(&s_command_lock);
    return fresh ? WEB_MANUAL_OK : WEB_MANUAL_REPLAY;
}

bool WebManual_Claim(uint64_t generation, uint32_t sequence, UartMotionCommand direction)
{
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    ManualExpireLocked(&status, now);
    const bool allowed = s_manual.active && s_manual.pending && !s_manual.halt &&
        s_manual.generation == generation && s_manual.sequence == sequence && s_manual.direction == direction;
    if (allowed) s_manual.pending = false;
    taskEXIT_CRITICAL(&s_command_lock);
    return allowed;
}

bool WebManual_TakeHalt(void)
{
    taskENTER_CRITICAL(&s_command_lock);
    const bool halt = s_manual.halt; s_manual.halt = false;
    taskEXIT_CRITICAL(&s_command_lock); return halt;
}

void WebManual_TxFailed(void)
{
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    ManualRevokeLocked(true, now);
    taskEXIT_CRITICAL(&s_command_lock);
}

bool WebManual_ConsoleAllowed(void)
{
    taskENTER_CRITICAL(&s_command_lock);
    const bool allowed = !s_manual.active && !s_manual.halt && s_pending < 0 && s_sending < 0 && !s_manual_controls;
    taskEXIT_CRITICAL(&s_command_lock); return allowed;
}

void WebManual_ControlBegin(UartControlCommand command)
{
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    ++s_manual_controls; ManualRevokeLocked(command != UART_CONTROL_STOP, now);
    if (!Uart_ControlIsAuto(command)) AutoInvalidateLocked(now);
    if (command == UART_CONTROL_STOP) s_manual.halt = false;
    taskEXIT_CRITICAL(&s_command_lock);
}

void WebManual_ControlEnd(void)
{
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    --s_manual_controls; s_manual_barrier = now; s_manual_has_barrier = true;
    AutoInvalidateLocked(now);
    taskEXIT_CRITICAL(&s_command_lock);
}

bool WebManual_Active(void)
{
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    ManualExpireLocked(&status, now); const bool active = s_manual.active;
    taskEXIT_CRITICAL(&s_command_lock); return active;
}

void WebManual_Process(void)
{
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    ManualExpireLocked(&status, now);
    const bool halt = s_manual.halt, pending = s_manual.pending;
    const uint64_t generation = s_manual.generation;
    const uint32_t sequence = s_manual.sequence;
    const UartMotionCommand direction = s_manual.direction;
    taskEXIT_CRITICAL(&s_command_lock);
    if (halt) (void)Uart_SendPendingHalt();
    else if (pending) (void)Uart_SendLeasedMotion(direction, generation, sequence);
}

bool WebCommands_CanPollSensors(void)
{
    taskENTER_CRITICAL(&s_command_lock);
    bool idle = !s_manual.active && !s_manual.pending && !s_manual.halt &&
                s_pending < 0 && s_sending < 0 && s_manual_controls == 0U;
    taskEXIT_CRITICAL(&s_command_lock);
    return idle;
}

bool WebCommands_CanPollMapping(void)
{
    const int64_t now=esp_timer_get_time();
    taskENTER_CRITICAL(&s_command_lock);
    bool allowed=!s_manual.pending&&!s_manual.halt&&s_pending<0&&s_sending<0&&s_manual_controls==0U&&
        (!s_manual.active||s_manual.expires_us-now>150000);
    taskEXIT_CRITICAL(&s_command_lock);
    return allowed;
}
