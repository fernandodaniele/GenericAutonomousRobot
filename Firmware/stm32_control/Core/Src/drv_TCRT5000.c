/*
 * drv_TCRT5000.h
 *
 *  Created on: April, 2026
 *      Author: Matías H. Costamagna
 */

#include "drv_TCRT5000.h"

/* Buffer DMA (actualizado automáticamente por el ADC) */
static uint16_t adc_buffer[TCRT_NUM_SENSORS];

/* Buffers para filtrado */
#define FILTER_SIZE 4

static uint16_t filter_left[FILTER_SIZE] = {0};
static uint16_t filter_right[FILTER_SIZE] = {0};

static uint8_t filter_index = 0;

/* Inicialización */
void drv_tcrt5000_init(void)
{
    /* Arranca ADC en modo DMA circular */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, TCRT_NUM_SENSORS);
}

/* Actualización de filtro (llamar periódicamente) */
void drv_tcrt5000_update(void)
{
    filter_left[filter_index]  = adc_buffer[TCRT_LEFT_INDEX];
    filter_right[filter_index] = adc_buffer[TCRT_RIGHT_INDEX];

    filter_index++;
    if (filter_index >= FILTER_SIZE)
        filter_index = 0;
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

/* Lecturas */
uint16_t drv_tcrt5000_read_left(void)
{
    return filter_average(filter_left);
}

uint16_t drv_tcrt5000_read_right(void)
{
    return filter_average(filter_right);
}
