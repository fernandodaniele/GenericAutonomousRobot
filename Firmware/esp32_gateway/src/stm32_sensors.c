#include "stm32_sensors.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE s_sensor_lock = portMUX_INITIALIZER_UNLOCKED;
static Stm32SensorsSnapshot s_sensor_data;
static int64_t s_sensor_received_us;
static char s_sensor_line[96];
static size_t s_sensor_length;
static bool s_sensor_discard;
static uint32_t s_sensor_counter, s_sensor_pending;
static int64_t s_sensor_sent_us, s_sensor_poll_us;

bool Stm32Sensors_PollDue(void) {
    return esp_timer_get_time() - s_sensor_poll_us >= (int64_t)SENSORS_POLL_MS * 1000;
}
void Stm32Sensors_DeferPoll(void) { s_sensor_poll_us = esp_timer_get_time(); }
uint32_t Stm32Sensors_BeginQuery(void) {
    if (++s_sensor_counter == 0U) ++s_sensor_counter;
    s_sensor_pending = s_sensor_counter;
    s_sensor_sent_us = s_sensor_poll_us = esp_timer_get_time();
    return s_sensor_pending;
}
void Stm32Sensors_QueryFailed(void) { s_sensor_pending = 0U; }

/* Un entero decimal o '-' por campo; sin signos, espacios o overflow. */
static bool Sensors_Field(const char **cursor, uint32_t *value, bool *present, bool last) {
    const char *p = *cursor;
    *value = 0U;
    *present = *p != '-';
    if (!*present) ++p;
    else {
        if (*p < '0' || *p > '9') return false;
        do {
            uint32_t digit = (uint32_t)(*p - '0');
            if (*value > (UINT32_MAX - digit) / 10U) return false;
            *value = *value * 10U + digit;
            ++p;
        } while (*p >= '0' && *p <= '9');
    }
    if (*p != (last ? '\0' : ',')) return false;
    *cursor = last ? p : p + 1;
    return true;
}

static void Sensors_Accept(void) {
    if (strncmp(s_sensor_line, "SENSORS,", 8U) != 0) return;
    const char *p = s_sensor_line + 8;
    uint32_t v[13]; bool has[13];
    for (unsigned i = 0; i < 13U; ++i)
        if (!Sensors_Field(&p, &v[i], &has[i], i == 12U)) return;
    if (!has[0] || v[0] != 1U || !has[1] || v[1] == 0U ||
        !has[2] || v[2] > 1U || !has[4] || v[4] > 2U || !has[7] || v[7] > 2U) return;
    if (v[2] == 0U) {
        if (v[4] || v[7]) return;
        for (unsigned i = 3; i < 13U; ++i)
            if (i != 4U && i != 7U && has[i]) return;
    } else {
        if (!has[3] || !has[10] || v[10] > 15U || !has[11] || !has[12] ||
            v[12] > 1U || v[12] != (v[10] != 15U ? 1U : 0U)) return;
        for (unsigned i = 4; i <= 7; i += 3) {
            if (v[i] == 0U) { if (has[i+1] || has[i+2]) return; }
            else if (!has[i+2] || (v[i] == 2U && has[i+1]) ||
                     (v[i] == 1U && (!has[i+1] || v[i+1] < 20U || v[i+1] > 4000U))) return;
        }
    }
    const int64_t now = esp_timer_get_time();
    if (s_sensor_pending != v[1] || now < s_sensor_sent_us ||
        now - s_sensor_sent_us >= (int64_t)SENSORS_REPLY_MS * 1000) return;
    /* Sumar todo el RTT sobreestima conservadoramente el transporte; nunca
     * comparar el tick STM32 con el reloj ESP32 ni rejuvenecer una respuesta. */
    uint32_t rtt_ms = (uint32_t)((now - s_sensor_sent_us + 999) / 1000);
    Stm32SensorsSnapshot data = { .valid = true, .available = v[2] != 0U, .query = v[1],
        .snapshot_age = v[3], .front_state = v[4], .front_mm = v[5], .front_age = v[6],
        .rear_state = v[7], .rear_mm = v[8], .rear_age = v[9],
        .ir_mask = v[10], .ir_age = v[11], .cliff = v[12] };
    uint32_t *ages[] = { &data.snapshot_age, &data.front_age, &data.rear_age, &data.ir_age };
    for (unsigned i = 0; i < 4; ++i)
        *ages[i] = *ages[i] > UINT32_MAX - rtt_ms ? UINT32_MAX : *ages[i] + rtt_ms;
    s_sensor_pending = 0U; /* Una respuesta por consulta; duplicados no refrescan. */
    taskENTER_CRITICAL(&s_sensor_lock);
    s_sensor_data = data; s_sensor_received_us = now;
    taskEXIT_CRITICAL(&s_sensor_lock);
}

void Stm32Sensors_Feed(const uint8_t *bytes, size_t length) {
    if (!bytes) return;
    for (size_t i = 0; i < length; ++i) {
        uint8_t c = bytes[i];
        if (c == '\n') {
            if (!s_sensor_discard) {
                if (s_sensor_length && s_sensor_line[s_sensor_length - 1U] == '\r') --s_sensor_length;
                s_sensor_line[s_sensor_length] = '\0'; Sensors_Accept();
            }
            s_sensor_length = 0U; s_sensor_discard = false;
        } else if (!s_sensor_discard) {
            if (c == 0U || s_sensor_length >= sizeof(s_sensor_line) - 1U) s_sensor_discard = true;
            else s_sensor_line[s_sensor_length++] = (char)c;
        }
    }
}

static uint32_t Sensors_AddAge(uint32_t age, uint64_t elapsed) {
    return elapsed >= UINT32_MAX || age > UINT32_MAX - (uint32_t)elapsed ? UINT32_MAX : age + (uint32_t)elapsed;
}
void Stm32Sensors_GetSnapshot(Stm32SensorsSnapshot *out) {
    if (!out) return;
    int64_t received;
    taskENTER_CRITICAL(&s_sensor_lock);
    *out = s_sensor_data; received = s_sensor_received_us;
    taskEXIT_CRITICAL(&s_sensor_lock);
    int64_t elapsed = esp_timer_get_time() - received;
    out->age_ms = out->valid && elapsed > 0 ? (uint64_t)elapsed / 1000U : 0U;
    out->connected = out->valid && out->age_ms < SENSORS_LINK_MS;
    out->snapshot_age = Sensors_AddAge(out->snapshot_age, out->age_ms);
    out->front_age = Sensors_AddAge(out->front_age, out->age_ms);
    out->rear_age = Sensors_AddAge(out->rear_age, out->age_ms);
    out->ir_age = Sensors_AddAge(out->ir_age, out->age_ms);
}

static const char *Sensors_Number(char text[12], bool available, uint64_t number) {
    if (!available) return "null";
    uint32_t bounded = number > UINT32_MAX ? (uint32_t)UINT32_MAX : (uint32_t)number;
    int length = snprintf(text, 12U, "%" PRIu32, bounded);
    return length > 0 && length < 12 ? text : "null";
}
int Stm32Sensors_Json(char *out, size_t capacity, bool stm32_connected, bool paused) {
    Stm32SensorsSnapshot s; Stm32Sensors_GetSnapshot(&s);
    const char *yes = "true", *no = "false";
    const char *states[] = { "no_sample", "valid", "invalid" };
    bool old = !stm32_connected || !s.connected || !s.available || s.snapshot_age > SENSORS_SNAPSHOT_AGE_MS;
    char nums[7][12];
    int length = snprintf(out, capacity,
        "{\"valid\":%s,\"connected\":%s,\"stm32_connected\":%s,\"paused\":%s,\"age_ms\":%s,"
        "\"snapshot_available\":%s,\"snapshot_age_ms\":%s,"
        "\"limits_ms\":{\"snapshot\":%u,\"ultrasound\":%u,\"ir\":%u},"
        "\"front\":{\"state\":\"%s\",\"valid\":%s,\"failed\":%s,\"stale\":%s,\"distance_mm\":%s,\"age_ms\":%s},"
        "\"rear\":{\"state\":\"%s\",\"valid\":%s,\"failed\":%s,\"stale\":%s,\"distance_mm\":%s,\"age_ms\":%s},"
        "\"ir\":{\"available\":%s,\"stale\":%s,\"age_ms\":%s,\"floor\":[%s,%s,%s,%s],\"cliff\":%s}}",
        s.valid ? yes : no, s.connected ? yes : no, stm32_connected ? yes : no, paused ? yes : no,
        Sensors_Number(nums[0], s.valid, s.age_ms), s.available ? yes : no,
        Sensors_Number(nums[1], s.available, s.snapshot_age), SENSORS_SNAPSHOT_AGE_MS, SENSORS_ULTRASOUND_AGE_MS, SENSORS_IR_AGE_MS,
        states[s.front_state], s.front_state == 1U ? yes : no, s.front_state == 2U ? yes : no,
        old || s.front_age > SENSORS_ULTRASOUND_AGE_MS ? yes : no,
        Sensors_Number(nums[2], s.front_state == 1U, s.front_mm), Sensors_Number(nums[3], s.front_state != 0U, s.front_age),
        states[s.rear_state], s.rear_state == 1U ? yes : no, s.rear_state == 2U ? yes : no,
        old || s.rear_age > SENSORS_ULTRASOUND_AGE_MS ? yes : no,
        Sensors_Number(nums[4], s.rear_state == 1U, s.rear_mm), Sensors_Number(nums[5], s.rear_state != 0U, s.rear_age),
        s.available ? yes : no, old || s.ir_age > SENSORS_IR_AGE_MS ? yes : no,
        Sensors_Number(nums[6], s.available, s.ir_age),
        !s.available ? "null" : s.ir_mask & 1U ? yes : no,
        !s.available ? "null" : s.ir_mask & 2U ? yes : no,
        !s.available ? "null" : s.ir_mask & 4U ? yes : no,
        !s.available ? "null" : s.ir_mask & 8U ? yes : no,
        !s.available ? "null" : s.cliff ? yes : no);
    return length >= 0 && (size_t)length < capacity ? length : -1;
}
