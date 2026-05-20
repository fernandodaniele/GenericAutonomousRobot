/*
 * drv_TCRT5000.c
 *
 *  Created on: April, 2026
 *      Author: Matías H. Costamagna
 */

#include "drv_TCRT5000.h"
#include "adc.h"
#include "stm32f4xx_hal.h"

/* ================= CONFIGURACIÓN ================= */

#define TCRT_LEFT_CHANNEL    ADC_CHANNEL_11
#define TCRT_RIGHT_CHANNEL   ADC_CHANNEL_12

#define FILTER_SIZE 10

#define TCRT_THRESHOLD_LINE    800
#define TCRT_THRESHOLD_CLIFF   2000
#define TCRT_MARGIN            50

/* ================= TIPOS INTERNOS ================= */

typedef struct
{
    uint32_t channel;
    uint16_t raw_value;
} drv_tcrt5000_sensor_t;

/* ================= VARIABLES PRIVADAS ================= */

static drv_tcrt5000_sensor_t tcrt_left_sensor;
static drv_tcrt5000_sensor_t tcrt_right_sensor;

static uint16_t filter_left[FILTER_SIZE];
static uint16_t filter_right[FILTER_SIZE];
static uint8_t filter_index = 0;

/* ================= PROTOTIPOS PRIVADOS ================= */

static uint16_t drv_tcrt5000_read_sensor(drv_tcrt5000_sensor_t *sensor);
static uint16_t filter_average(uint16_t *buffer);
static tcrt_state_t classify(uint16_t val);
static bool is_unstable(uint16_t val);

/* ================= INIT ================= */

void drv_tcrt5000_init(void)
{
    tcrt_left_sensor.channel = TCRT_LEFT_CHANNEL;
    tcrt_right_sensor.channel = TCRT_RIGHT_CHANNEL;

    uint16_t left_init  = drv_tcrt5000_read_sensor(&tcrt_left_sensor);
    uint16_t right_init = drv_tcrt5000_read_sensor(&tcrt_right_sensor);

    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
        filter_left[i]  = left_init;
        filter_right[i] = right_init;
    }
}

/* ================= UPDATE ================= */

void drv_tcrt5000_update(void)
{
    uint16_t left_value  = drv_tcrt5000_read_sensor(&tcrt_left_sensor);
    uint16_t right_value = drv_tcrt5000_read_sensor(&tcrt_right_sensor);

    filter_left[filter_index]  = left_value;
    filter_right[filter_index] = right_value;

    filter_index++;
    if (filter_index >= FILTER_SIZE)
        filter_index = 0;
}

/* ================= LECTURA ================= */

uint16_t drv_tcrt5000_read_left(void)
{
    return filter_average(filter_left);
}

uint16_t drv_tcrt5000_read_right(void)
{
    return filter_average(filter_right);
}

/* ================= INTERPRETACIÓN ================= */

tcrt_reading_t drv_tcrt5000_get_left(void)
{
    tcrt_reading_t r;

    r.value = drv_tcrt5000_read_left();
    r.state = classify(r.value);
    r.unstable = is_unstable(r.value);

    return r;
}

tcrt_reading_t drv_tcrt5000_get_right(void)
{
    tcrt_reading_t r;

    r.value = drv_tcrt5000_read_right();
    r.state = classify(r.value);
    r.unstable = is_unstable(r.value);

    return r;
}


/* ================= PRIVADAS ================= */

static uint16_t drv_tcrt5000_read_sensor(drv_tcrt5000_sensor_t *sensor)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = sensor->channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        return sensor->raw_value;

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        return sensor->raw_value;

    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
        return sensor->raw_value;

    sensor->raw_value = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    return sensor->raw_value;
}

static uint16_t filter_average(uint16_t *buffer)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < FILTER_SIZE; i++)
        sum += buffer[i];

    return (uint16_t)(sum / FILTER_SIZE);
}

static tcrt_state_t classify(uint16_t val)
{
    if (val < TCRT_THRESHOLD_LINE)
        return TCRT_STATE_LINE;

    if (val > TCRT_THRESHOLD_CLIFF)
        return TCRT_STATE_CLIFF;

    return TCRT_STATE_FLOOR;
}

static bool is_unstable(uint16_t val)
{
    if ((val > (TCRT_THRESHOLD_LINE - TCRT_MARGIN)) &&
        (val < (TCRT_THRESHOLD_LINE + TCRT_MARGIN)))
        return true;

    if ((val > (TCRT_THRESHOLD_CLIFF - TCRT_MARGIN)) &&
        (val < (TCRT_THRESHOLD_CLIFF + TCRT_MARGIN)))
        return true;

    return false;
}
