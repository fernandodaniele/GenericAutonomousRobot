/*
 * drv_ultrasound.h
 *
 *  Created on: Feb 15, 2026
 *      Author: Fernando E. Daniele
 */

#ifndef CORE_INC_DRV_ULTRASOUND_H_
#define CORE_INC_DRV_ULTRASOUND_H_

#include "stm32f4xx_hal.h"

typedef struct {
    GPIO_TypeDef* trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef* echo_port;
    uint16_t echo_pin;
    TIM_HandleTypeDef* timer; // Timer to measure microseconds (e.g. TIM2)
} Ultrasound_t;

/**
 * @brief Initializes the sensor (make sure the timer is configured to 1MHz)
 */
void Ultrasound_Init(Ultrasound_t* dev);

/**
 * @brief Performs a reading and returns the distance in millimeters.
 * @return Distance in mm. Returns 0 if error or out of range.
 */
uint32_t Ultrasound_Read_mm(Ultrasound_t* dev);

#endif /* CORE_INC_DRV_ULTRASOUND_H_ */
