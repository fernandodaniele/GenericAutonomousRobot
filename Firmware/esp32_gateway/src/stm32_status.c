#include "stm32_status.h"

#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define STATUS_LINE_CAPACITY 96U

/* El ensamblador y el temporizador tienen un solo propietario: app_main. */
static char s_line[STATUS_LINE_CAPACITY];
static size_t s_length;
static bool s_discard_line;
static bool s_poll_started;
static int64_t s_last_poll_us;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static struct {
    bool valid;
    uint32_t state;
    uint32_t faults;
    uint32_t stop;
    int64_t received_us;
} s_status;

static bool ReadUnsigned(const char **cursor, uint32_t *value)
{
    const char *p = *cursor;
    uint32_t number = 0U;
    if (*p < '0' || *p > '9') return false;
    do {
        const uint32_t digit = (uint32_t)(*p - '0');
        if (number > (UINT32_MAX - digit) / 10U) return false;
        number = number * 10U + digit;
        ++p;
    } while (*p >= '0' && *p <= '9');
    *cursor = p;
    *value = number;
    return true;
}

static void AcceptLine(void)
{
    uint32_t state, faults, stop;
    const char *p = s_line;
    if (strncmp(p, "STATE=", 6U) != 0) return;
    p += 6;
    if (!ReadUnsigned(&p, &state) || strncmp(p, " FAULTS=", 8U) != 0) return;
    p += 8;
    if (!ReadUnsigned(&p, &faults) || strncmp(p, " STOP=", 6U) != 0) return;
    p += 6;
    if (!ReadUnsigned(&p, &stop) || *p != '\0' || state > 3U || stop > 1U) return;

    /* Es el instante de recepcion completa, no la edad de sensores STM32.
     * Se conserva toda la mascara, incluso bits futuros sin definicion local. */
    const int64_t received_us = esp_timer_get_time();
    taskENTER_CRITICAL(&s_lock);
    s_status.state = state;
    s_status.faults = faults;
    s_status.stop = stop;
    s_status.received_us = received_us;
    s_status.valid = true;
    taskEXIT_CRITICAL(&s_lock);
}

void Stm32Status_Feed(const uint8_t *bytes, size_t length)
{
    if (bytes == NULL) return;
    for (size_t i = 0U; i < length; ++i) {
        const uint8_t byte = bytes[i];
        if (byte == '\n') {
            if (!s_discard_line) {
                if (s_length > 0U && s_line[s_length - 1U] == '\r') --s_length;
                s_line[s_length] = '\0';
                AcceptLine();
            }
            s_length = 0U;
            s_discard_line = false;
        } else if (!s_discard_line) {
            if (byte == 0U || s_length >= sizeof(s_line) - 1U) {
                /* Descartar hasta LF: nunca analizar una cola de linea larga. */
                s_discard_line = true;
            } else {
                s_line[s_length++] = (char)byte;
            }
        }
    }
}

void Stm32Status_GetSnapshot(Stm32StatusSnapshot *snapshot)
{
    if (snapshot == NULL) return;
    int64_t received_us;
    taskENTER_CRITICAL(&s_lock);
    snapshot->valid = s_status.valid;
    snapshot->state = s_status.state;
    snapshot->faults = s_status.faults;
    snapshot->stop = s_status.stop;
    received_us = s_status.received_us;
    snapshot->received_us = received_us;
    taskEXIT_CRITICAL(&s_lock);

    const int64_t elapsed = esp_timer_get_time() - received_us;
    snapshot->age_ms = snapshot->valid && elapsed > 0 ? (uint64_t)elapsed / 1000U : 0U;
    snapshot->connected = snapshot->valid && snapshot->age_ms < STM32_STATUS_STALE_MS;
}

bool Stm32Status_PollDue(void)
{
    const int64_t now = esp_timer_get_time();
    if (!s_poll_started || now - s_last_poll_us >= (int64_t)STM32_STATUS_POLL_MS * 1000) {
        s_poll_started = true;
        s_last_poll_us = now;
        return true;
    }
    return false;
}
