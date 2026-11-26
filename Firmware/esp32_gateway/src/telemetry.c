#include "telemetry.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TOKEN_SIZE      16
#define EVENT_COUNT     8
#define US_COUNT        3
#define LINK_TIMEOUT_MS 1500
#define MAP_COUNT       16    // Puntos del mapa guardados (la web los pide cada 250 ms)

typedef struct {
    char status;              // O = valido, F = libre, C = pegado, N = sin eco
    unsigned long mm;
} Range_t;

typedef struct {
    bool valid;
    int64_t rx_us;            // Momento de recepcion en el ESP32
    unsigned long stm_ms;     // Reloj de la STM32
    char state[TOKEN_SIZE];
    int pwm[2];               // Izquierdo, derecho
    int brake;
    Range_t us[US_COUNT];     // Frente, izquierda, derecha
    unsigned tcrt[2];
    int cliff[2];
    int gyro_z;
    int imu_ok;
    int heading_ddeg;         // Rumbo en decimas de grado (+ = izquierda)
    int imu_diag;             // Diagnostico del MPU6050 (IMU_DIAG_* en la STM32)
} Sample_t;

#define IMU_DIAG_UNKNOWN (-99)   // STM32 con firmware anterior, sin diagnostico

typedef struct {
    uint32_t id;
    unsigned long ms;
    char text[TOKEN_SIZE];
} Event_t;

/* Estado de los modulos del datalogger (linea SYS de la STM32) */
typedef struct {
    bool valid;
    int64_t rx_us;
    unsigned long stm_ms;
    int sd;                   // 0 = sin tarjeta, 1 = guardando, 2 = fallo en marcha
    int sd_type;              // 0 = desconocido, 1 = SDSC, 2 = SDHC
    unsigned long sd_lines;   // Lineas guardadas en GAR.LOG
    int clock_ok;
    int date[6];              // Anio, mes, dia, hora, minuto, segundo
    unsigned long rows;       // Filas de datos generadas
} System_t;

/* Linea MAP de la STM32: pose estimada y ultima lectura de cada ultrasonido */
typedef struct {
    uint32_t id;              // 0 = vacio
    unsigned long ms;
    long x_mm, y_mm, theta_mrad;
    long range_mm[US_COUNT];  // Frente, izquierda, derecha (9999 = libre, 0 = sin lectura)
} MapPoint_t;

static SemaphoreHandle_t lock;
static Sample_t latest;
static System_t system_status;
static Event_t events[EVENT_COUNT];
static uint32_t next_event_id = 1;
static MapPoint_t map_points[MAP_COUNT];
static uint32_t next_map_id = 1;

void Telemetry_Init(void) {
    lock = xSemaphoreCreateMutex();
    configASSERT(lock);
}

/* Lo que va dentro de comillas en el JSON: solo letras, numeros y guion bajo */
static bool is_token(const char *text) {
    if (*text == '\0') {
        return false;
    }
    for (; *text != '\0'; text++) {
        if (!isalnum((unsigned char)*text) && *text != '_') {
            return false;
        }
    }
    return true;
}

static bool is_range_status(char status) {
    return status == 'O' || status == 'F' || status == 'C' || status == 'N';
}

static bool parse_sample(const char *line, Sample_t *s) {
    int fields = sscanf(line,
                        "TEL,%lu,%15[^,],%d,%d,%d,%c,%lu,%c,%lu,%c,%lu,%u,%u,%d,%d,%d,%d,%d,%d",
                        &s->stm_ms, s->state, &s->pwm[0], &s->pwm[1], &s->brake,
                        &s->us[0].status, &s->us[0].mm,
                        &s->us[1].status, &s->us[1].mm,
                        &s->us[2].status, &s->us[2].mm,
                        &s->tcrt[0], &s->tcrt[1],
                        &s->cliff[0], &s->cliff[1],
                        &s->gyro_z, &s->imu_ok, &s->heading_ddeg, &s->imu_diag);
    if (fields == 17) {
        s->heading_ddeg = 0;   // STM32 con firmware anterior, sin rumbo
    }
    if (fields <= 18) {
        s->imu_diag = IMU_DIAG_UNKNOWN;
    }
    if (fields < 17 || !is_token(s->state)) {
        return false;
    }
    for (int i = 0; i < US_COUNT; i++) {
        if (!is_range_status(s->us[i].status)) {
            return false;
        }
    }
    return true;
}

static void handle_sample(const char *line) {
    Sample_t sample;
    if (!parse_sample(line, &sample)) {
        return;   // Linea cortada o con ruido: se descarta
    }
    sample.valid = true;
    sample.rx_us = esp_timer_get_time();

    xSemaphoreTake(lock, portMAX_DELAY);
    latest = sample;
    xSemaphoreGive(lock);
}

static void handle_system(const char *line) {
    System_t sys;
    int fields = sscanf(line, "SYS,%lu,%d,%d,%lu,%d,%d,%d,%d,%d,%d,%d,%lu",
                        &sys.stm_ms, &sys.sd, &sys.sd_type, &sys.sd_lines, &sys.clock_ok,
                        &sys.date[0], &sys.date[1], &sys.date[2],
                        &sys.date[3], &sys.date[4], &sys.date[5], &sys.rows);
    if (fields != 12 || sys.sd < 0 || sys.sd > 2) {
        return;   // Linea cortada o con ruido: se descarta
    }
    sys.valid = true;
    sys.rx_us = esp_timer_get_time();

    xSemaphoreTake(lock, portMAX_DELAY);
    system_status = sys;
    xSemaphoreGive(lock);
}

static void handle_event(const char *line) {
    Event_t event;
    if (sscanf(line, "EVT,%lu,%15s", &event.ms, event.text) != 2 || !is_token(event.text)) {
        return;
    }

    xSemaphoreTake(lock, portMAX_DELAY);
    event.id = next_event_id++;
    events[event.id % EVENT_COUNT] = event;
    xSemaphoreGive(lock);
}

static void handle_map(const char *line) {
    MapPoint_t p;
    if (sscanf(line, "MAP,%lu,%ld,%ld,%ld,%ld,%ld,%ld", &p.ms, &p.x_mm, &p.y_mm, &p.theta_mrad,
               &p.range_mm[0], &p.range_mm[1], &p.range_mm[2]) != 7) {
        return;   // Linea cortada o con ruido: se descarta
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    p.id = next_map_id++;
    map_points[p.id % MAP_COUNT] = p;
    xSemaphoreGive(lock);
}

bool Telemetry_HandleLine(const char *line) {
    if (strncmp(line, "TEL,", 4) == 0) {
        handle_sample(line);
        return true;
    }
    if (strncmp(line, "EVT,", 4) == 0) {
        handle_event(line);
        return true;
    }
    if (strncmp(line, "SYS,", 4) == 0) {
        handle_system(line);
        return true;
    }
    if (strncmp(line, "MAP,", 4) == 0) {
        handle_map(line);
        return true;
    }
    return false;
}

size_t Telemetry_ToJson(char *out, size_t size) {
    Sample_t s;
    System_t sys;
    Event_t copy[EVENT_COUNT];
    MapPoint_t map_copy[MAP_COUNT];

    xSemaphoreTake(lock, portMAX_DELAY);
    s = latest;
    sys = system_status;
    memcpy(copy, events, sizeof(copy));
    memcpy(map_copy, map_points, sizeof(map_copy));
    xSemaphoreGive(lock);

    // El estado de modulos llega cada 1 s: se da por viejo a los 3 s
    const long sys_age_ms = sys.valid ? (long)((esp_timer_get_time() - sys.rx_us) / 1000) : -1;
    const bool sys_fresh = sys.valid && sys_age_ms < 3000;

    const long age_ms = s.valid ? (long)((esp_timer_get_time() - s.rx_us) / 1000) : -1;
    const bool link = s.valid && age_ms < LINK_TIMEOUT_MS;

    int used = snprintf(out, size,
        "{\"link\":%s,\"age\":%ld,\"ms\":%lu,\"state\":\"%s\","
        "\"pwm\":[%d,%d],\"brake\":%s,"
        "\"us\":[{\"s\":\"%c\",\"mm\":%lu},{\"s\":\"%c\",\"mm\":%lu},{\"s\":\"%c\",\"mm\":%lu}],"
        "\"tcrt\":[%u,%u],\"cliff\":[%d,%d],\"gyro\":%d,\"imu\":%s,\"heading\":%d,\"imuDiag\":%d,\"events\":[",
        link ? "true" : "false", age_ms, s.stm_ms, s.valid ? s.state : "",
        s.pwm[0], s.pwm[1], s.brake ? "true" : "false",
        s.valid ? s.us[0].status : 'N', s.us[0].mm,
        s.valid ? s.us[1].status : 'N', s.us[1].mm,
        s.valid ? s.us[2].status : 'N', s.us[2].mm,
        s.tcrt[0], s.tcrt[1], s.cliff[0], s.cliff[1],
        s.gyro_z, s.imu_ok ? "true" : "false", s.heading_ddeg,
        s.valid ? s.imu_diag : IMU_DIAG_UNKNOWN);

    bool first = true;
    for (int i = 0; i < EVENT_COUNT && used > 0 && (size_t)used < size; i++) {
        if (copy[i].id == 0) {
            continue;
        }
        used += snprintf(out + used, size - used, "%s{\"id\":%lu,\"ms\":%lu,\"t\":\"%s\"}",
                         first ? "" : ",", (unsigned long)copy[i].id, copy[i].ms, copy[i].text);
        first = false;
    }
    if (used > 0 && (size_t)used < size) {
        used += snprintf(out + used, size - used,
            "],\"sys\":{\"ok\":%s,\"sd\":%d,\"sdType\":%d,\"sdLines\":%lu,"
            "\"clock\":%s,\"date\":[%d,%d,%d,%d,%d,%d],\"rows\":%lu},\"map\":[",
            sys_fresh ? "true" : "false", sys.sd, sys.sd_type, sys.sd_lines,
            sys.clock_ok ? "true" : "false",
            sys.date[0], sys.date[1], sys.date[2], sys.date[3], sys.date[4], sys.date[5],
            sys.rows);
    }
    /* Puntos del mapa: [id, ms, x_mm, y_mm, theta_mrad, frente, izq, der] */
    first = true;
    for (int i = 0; i < MAP_COUNT && used > 0 && (size_t)used < size; i++) {
        const MapPoint_t *p = &map_copy[i];
        if (p->id == 0) {
            continue;
        }
        used += snprintf(out + used, size - used, "%s[%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld]",
                         first ? "" : ",", (unsigned long)p->id, p->ms, p->x_mm, p->y_mm,
                         p->theta_mrad, p->range_mm[0], p->range_mm[1], p->range_mm[2]);
        first = false;
    }
    if (used > 0 && (size_t)used < size) {
        used += snprintf(out + used, size - used, "]}");
    }

    return (used > 0 && (size_t)used < size) ? (size_t)used : 0;
}
