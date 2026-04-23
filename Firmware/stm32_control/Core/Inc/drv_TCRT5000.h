/*
 * drv_TCRT5000.h
 *
 *  Created on: April, 2026
 *      Author: Matías H. Costamagna
 */

#ifndef CORE_INC_DRV_TCRT5000_H_
#define CORE_INC_DRV_TCRT5000_H_

#include "stm32f4xx_hal.h"
#include "adc.h"

/* Cantidad de sensores */
#define TCRT_NUM_SENSORS 2

/* API */
void drv_tcrt5000_init(void);
void drv_tcrt5000_update(void);

uint16_t drv_tcrt5000_read_left(void);
uint16_t drv_tcrt5000_read_right(void);

#endif
