#include "stm32_imu.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    bool received, has_sample;
    uint32_t state, hal_status, error, sample_age;
    int16_t raw[6];
    int64_t received_us;
} ImuSnapshot;
static portMUX_TYPE s_imu_lock = portMUX_INITIALIZER_UNLOCKED;
static ImuSnapshot s_imu;
static char s_imu_line[128];
static size_t s_imu_length;
static bool s_imu_discard;
static uint32_t s_imu_counter, s_imu_pending;
static int64_t s_imu_sent, s_imu_poll;

bool Stm32Imu_PollDue(void) { return esp_timer_get_time() - s_imu_poll >= IMU_POLL_MS * 1000; }
void Stm32Imu_DeferPoll(void) { s_imu_poll = esp_timer_get_time(); }
uint32_t Stm32Imu_BeginQuery(void) {
    if (++s_imu_counter == 0U) ++s_imu_counter;
    s_imu_pending = s_imu_counter;
    s_imu_sent = s_imu_poll = esp_timer_get_time();
    return s_imu_pending;
}
void Stm32Imu_QueryFailed(void) { s_imu_pending = 0U; }

static bool ImuField(const char **cursor, int64_t *value, bool *present, bool last) {
    const char *p = *cursor;
    bool negative = *p == '-';
    if (negative) ++p;
    *present = !(negative && (*p == ',' || *p == '\0'));
    uint32_t number = 0U;
    if (*present) {
        if (*p < '0' || *p > '9') return false;
        do {
            uint32_t digit = (uint32_t)(*p - '0');
            if (number > (UINT32_MAX - digit) / 10U) return false;
            number = number * 10U + digit;
            ++p;
        } while (*p >= '0' && *p <= '9');
        if (negative && number == 0U) return false;
    }
    *value = negative ? -(int64_t)number : (int64_t)number;
    if (*p != (last ? '\0' : ',')) return false;
    *cursor = last ? p : p + 1;
    return true;
}

static void ImuAccept(void) {
    if (strncmp(s_imu_line, "IMU,", 4U) != 0) return;
    const char *p = s_imu_line + 4;
    int64_t v[12]; bool has[12];
    for (unsigned i = 0; i < 12U; ++i)
        if (!ImuField(&p, &v[i], &has[i], i == 11U)) return;
    for (unsigned i = 0; i < 5U; ++i) if (!has[i] || v[i] < 0) return;
    if (v[0] != 1 || v[1] == 0 || v[2] > 4 || v[3] > 3 || (has[5] && v[5] < 0)) return;
    bool measurement = v[2] == 2;
    if ((v[2] <= 2 && (v[3] || v[4])) || (v[2] >= 3 && v[3] == 0)) return;
    if (measurement && !has[5]) return;
    for (unsigned i = 6; i < 12U; ++i) {
        if (has[i] != measurement) return;
        if (has[i] && (v[i] < INT16_MIN || v[i] > INT16_MAX)) return;
    }
    int64_t now = esp_timer_get_time();
    if (s_imu_pending != (uint32_t)v[1] || now < s_imu_sent ||
        now - s_imu_sent >= IMU_REPLY_MS * 1000) return;
    uint32_t rtt = (uint32_t)((now - s_imu_sent + 999) / 1000);
    ImuSnapshot next = { .received = true, .has_sample = has[5], .state = (uint32_t)v[2],
        .hal_status = (uint32_t)v[3], .error = (uint32_t)v[4], .sample_age = (uint32_t)v[5], .received_us = now };
    if (next.sample_age > UINT32_MAX - rtt) next.sample_age = UINT32_MAX;
    else next.sample_age += rtt; /* Conservative transport age; never mix board clocks. */
    for (unsigned i = 0; i < 6U; ++i) if (measurement) next.raw[i] = (int16_t)v[i+6];
    s_imu_pending = 0U;
    taskENTER_CRITICAL(&s_imu_lock);
    s_imu = next;
    taskEXIT_CRITICAL(&s_imu_lock);
}

void Stm32Imu_Feed(const uint8_t *bytes, size_t length) {
    if (!bytes) return;
    for (size_t i = 0; i < length; ++i) {
        uint8_t c = bytes[i];
        if (c == '\n') {
            if (!s_imu_discard) {
                if (s_imu_length && s_imu_line[s_imu_length-1] == '\r') --s_imu_length;
                s_imu_line[s_imu_length] = '\0'; ImuAccept();
            }
            s_imu_length = 0U; s_imu_discard = false;
        } else if (!s_imu_discard) {
            if (c == 0U || s_imu_length >= sizeof(s_imu_line)-1U) s_imu_discard = true;
            else s_imu_line[s_imu_length++] = (char)c;
        }
    }
}

int Stm32Imu_Json(char *out, size_t capacity, bool stm32_connected, bool paused) {
    static const char *states[] = { "initializing", "ready", "valid", "init_error", "read_error" };
    ImuSnapshot s;
    taskENTER_CRITICAL(&s_imu_lock);
    s = s_imu;
    taskEXIT_CRITICAL(&s_imu_lock);
    int64_t now = esp_timer_get_time();
    uint64_t elapsed = s.received && now >= s.received_us ? (uint64_t)(now-s.received_us)/1000U : UINT32_MAX;
    uint32_t response_age = elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
    uint32_t age = s.sample_age > UINT32_MAX-response_age ? UINT32_MAX : s.sample_age+response_age;
    bool connected = s.received && response_age < IMU_LINK_MS && stm32_connected;
    bool valid = connected && s.state == 2U && s.has_sample && age <= IMU_SAMPLE_MAX_AGE_MS;
    char axes[160] = "null,null,null],\"gyro_dps\":[null,null,null";
    if (valid) {
        int n = snprintf(axes, sizeof(axes), "%.6f,%.6f,%.6f],\"gyro_dps\":[%.6f,%.6f,%.6f",
            s.raw[0]/16384.0, s.raw[1]/16384.0, s.raw[2]/16384.0,
            s.raw[3]/131.0, s.raw[4]/131.0, s.raw[5]/131.0);
        if (n < 0 || (size_t)n >= sizeof(axes)) return -1;
    }
    char response[12] = "null", sample[12] = "null", hal[12] = "null", error[12] = "null";
    if (s.received) {
        (void)snprintf(response, sizeof(response), "%" PRIu32, response_age);
        (void)snprintf(hal, sizeof(hal), "%" PRIu32, s.hal_status);
        (void)snprintf(error, sizeof(error), "%" PRIu32, s.error);
    }
    if (s.has_sample) (void)snprintf(sample, sizeof(sample), "%" PRIu32, age);
    int n = snprintf(out, capacity,
        "{\"received\":%s,\"connected\":%s,\"stm32_connected\":%s,\"paused\":%s,\"valid\":%s,\"stale\":%s,"
        "\"state\":\"%s\",\"hal_status\":%s,\"i2c_error\":%s,\"response_age_ms\":%s,\"sample_age_ms\":%s,"
        "\"max_age_ms\":%u,\"accel_g\":[%s]}",
        s.received ? "true" : "false", connected ? "true" : "false", stm32_connected ? "true" : "false",
        paused ? "true" : "false", valid ? "true" : "false", valid ? "false" : "true",
        s.received ? states[s.state] : "no_data", hal, error, response, sample, IMU_SAMPLE_MAX_AGE_MS, axes);
    return n >= 0 && (size_t)n < capacity ? n : -1;
}
