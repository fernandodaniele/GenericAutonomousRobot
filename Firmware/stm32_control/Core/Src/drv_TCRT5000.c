/*
 * drv_TCRT5000.h
 *
 *  Created on: April, 2026
 *      Author: Matías H. Costamagna
 */

#include "drv_TCRT5000.h"

/* Configuración de canales ADC */
#define TCRT_LEFT_CHANNEL    ADC_CHANNEL_11
#define TCRT_RIGHT_CHANNEL   ADC_CHANNEL_12

/* Tamaño del filtro */
#define FILTER_SIZE 10

/* Estructura interna del sensor */
typedef struct
{
    uint32_t channel;
    uint16_t raw_value;
} drv_tcrt5000_sensor_t;

/* Instancias privadas */
static drv_tcrt5000_sensor_t tcrt_left_sensor;
static drv_tcrt5000_sensor_t tcrt_right_sensor;

/* Buffers de filtrado */
static uint16_t filter_left[FILTER_SIZE];
static uint16_t filter_right[FILTER_SIZE];
static uint8_t filter_index = 0;

/* Prototipos privados */
static uint16_t drv_tcrt5000_read_sensor(drv_tcrt5000_sensor_t *sensor);
static uint16_t filter_average(uint16_t *buffer);

/* Inicialización */
void drv_tcrt5000_init(void)
{
    tcrt_left_sensor.channel = TCRT_LEFT_CHANNEL;
    tcrt_left_sensor.raw_value = 0;

    tcrt_right_sensor.channel = TCRT_RIGHT_CHANNEL;
    tcrt_right_sensor.raw_value = 0;
}

/* Actualización periódica */
void drv_tcrt5000_update(void)
{
    uint16_t left_value;
    uint16_t right_value;

    left_value  = drv_tcrt5000_read_sensor(&tcrt_left_sensor);
    right_value = drv_tcrt5000_read_sensor(&tcrt_right_sensor);

    filter_left[filter_index]  = left_value;
    filter_right[filter_index] = right_value;

    filter_index++;
    if (filter_index >= FILTER_SIZE)
    {
        filter_index = 0;
    }
}

/* Lectura filtrada */
uint16_t drv_tcrt5000_read_left(void)
{
    return filter_average(filter_left);
}

uint16_t drv_tcrt5000_read_right(void)
{
    return filter_average(filter_right);
}

/* ================= FUNCIONES PRIVADAS ================= */

/* Lectura ADC de un sensor */
static uint16_t drv_tcrt5000_read_sensor(drv_tcrt5000_sensor_t *sensor)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = sensor->channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

    sensor->raw_value = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    return sensor->raw_value;
}

/* Promedio simple */
static uint16_t filter_average(uint16_t *buffer)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
        sum += buffer[i];
    }

    return (uint16_t)(sum / FILTER_SIZE);
}
