/**
 * @file    mid_clock.c
 * @brief   Fecha y hora del robot a partir del RTC DS1302.
 */

#include "mid_clock.h"
#include "ds1302.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define CLOCK_FORMAT_24H    0u      // DS1302_24H_FORMAT del driver
#define CLOCK_DAYS_IN_WEEK  7u

static Ds1302_t clock_rtc;
static SemaphoreHandle_t clock_mutex = NULL;
static volatile bool clock_valid = false;

/* ===== Exclusion mutua ===== */

static bool clock_scheduler_running(void) {
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

static void clock_lock(void) {
    if ((clock_mutex != NULL) && clock_scheduler_running()) {
        (void) xSemaphoreTake(clock_mutex, portMAX_DELAY);
    }
}

static void clock_unlock(void) {
    if ((clock_mutex != NULL) && clock_scheduler_running()) {
        (void) xSemaphoreGive(clock_mutex);
    }
}

/* ===== Validacion y conversion ===== */

static uint8_t clock_bcd(uint8_t tens, uint8_t units) {
    return (uint8_t)((tens * 10u) + units);
}

static uint8_t clock_days_in_month(uint16_t year, uint8_t month) {
    static const uint8_t days[12] = { 31u, 28u, 31u, 30u, 31u, 30u, 31u, 31u, 30u, 31u, 30u, 31u };
    if ((month == 2u) && ((year % 4u) == 0u)) {
        return 29u;     // 2000 a 2099: todo multiplo de 4 es bisiesto
    }
    return days[month - 1u];
}

static bool clock_is_valid(const ClockDateTime_t *t) {
    if ((t->year < CLOCK_YEAR_MIN) || (t->year > CLOCK_YEAR_MAX)) {
        return false;
    }
    if ((t->month < 1u) || (t->month > 12u)) {
        return false;
    }
    if ((t->day < 1u) || (t->day > clock_days_in_month(t->year, t->month))) {
        return false;
    }
    return (t->hour < 24u) && (t->minute < 60u) && (t->second < 60u);
}

/* Dia de la semana del DS1302: 1 = domingo ... 7 = sabado (Sakamoto) */
static uint8_t clock_weekday(const ClockDateTime_t *t) {
    static const uint8_t offset[12] = { 0u, 3u, 2u, 5u, 0u, 3u, 5u, 1u, 4u, 6u, 2u, 4u };
    uint16_t year = t->year;
    if (t->month < 3u) {
        year--;
    }
    uint16_t day = (uint16_t)(year + (year / 4u) - (year / 100u) + (year / 400u)
                              + offset[t->month - 1u] + t->day);
    return (uint8_t)((day % CLOCK_DAYS_IN_WEEK) + 1u);
}

/* Convierte los registros leidos (BCD) a ClockDateTime_t */
static bool clock_from_registers(const RtcRegister_t *r, ClockDateTime_t *out) {
    // Reloj detenido: nunca se puso en hora o se quedo sin pila
    if (r->seconds.bits.clock_halt != 0u) {
        return false;
    }
    // Solo se carga en formato 24 h (Clock_Set)
    if (r->hour.hour_24.bits.hour_format != CLOCK_FORMAT_24H) {
        return false;
    }

    out->second = clock_bcd(r->seconds.bits.seconds_tens, r->seconds.bits.seconds_units);
    out->minute = clock_bcd(r->minutes.bits.minutes_tens, r->minutes.bits.minutes_units);
    out->hour = clock_bcd(r->hour.hour_24.bits.hour_tens, r->hour.hour_24.bits.hour_units);
    out->day = clock_bcd(r->month_day.bits.month_day_tens, r->month_day.bits.month_day_units);
    out->month = clock_bcd(r->month.bits.month_tens, r->month.bits.month_units);
    out->year = (uint16_t)(CLOCK_YEAR_MIN
                           + clock_bcd(r->year.bits.year_tens, r->year.bits.year_units));

    // Unidades BCD > 9 (por ejemplo 0xFF de un DS1302 desconectado) quedan fuera de rango
    return clock_is_valid(out);
}

/* ===== API publica ===== */

bool Clock_Init(void) {
    const Ds1302Config_t config = {
        .reset_pin = {
            .mcu_port = RST_DS1302_GPIO_Port,
            .pin = RST_DS1302_Pin
        }
    };
    ClockDateTime_t now;

    if (clock_mutex == NULL) {
        clock_mutex = xSemaphoreCreateMutex();
    }
    (void) DS1302_Init(&clock_rtc, &config);
    return Clock_Read(&now);
}

bool Clock_Read(ClockDateTime_t *out) {
    ClockDateTime_t value;
    bool valid = false;

    if (out == NULL) {
        return false;
    }

    clock_lock();
    if (DS1302_UpdateDateTime(&clock_rtc) == DS1302_OK) {
        valid = clock_from_registers(&clock_rtc.data.received_data, &value);
    }
    clock_unlock();

    clock_valid = valid;
    if (valid) {
        *out = value;
    }
    return valid;
}

bool Clock_Set(const ClockDateTime_t *value) {
    bool written = false;

    if ((value == NULL) || !clock_is_valid(value)) {
        return false;
    }

    clock_lock();
    written = (DS1302_SetTime(&clock_rtc, CLOCK_FORMAT_24H, value->hour, value->minute,
                              value->second, 0u, clock_weekday(value), value->day,
                              value->month, (int)value->year) == DS1302_OK);
    clock_unlock();

    return written;
}

bool Clock_IsValid(void) {
    return clock_valid;
}
