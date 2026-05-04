/*
 * drv_motor.h
 *
 *  Created on: Feb 15, 2026
 *      Author: Fernando E. Daniele
 */

#ifndef CORE_INC_DRV_MOTOR_H_
#define CORE_INC_DRV_MOTOR_H_


#include "tim.h"
#include "gpio.h"

// Definitions for speed limits (0 to 1000)
#define MOTOR_MAX_SPEED 1000
#define MOTOR_MIN_SPEED -1000

typedef struct {
    TIM_HandleTypeDef* htim;    // PWM Timer (e.g. &htim4)
    uint32_t channel;           // Channel (e.g. TIM_CHANNEL_1)

    GPIO_TypeDef* port_a;       // Port Pin Direction A
    uint16_t pin_a;             // Pin Direction A

    GPIO_TypeDef* port_b;       // Port Pin Direction B
    uint16_t pin_b;             // Pin Direction B

    int16_t current_speed;      // Current state (-1000 to 1000)
} MotorHandle_t;

/**
 * @brief Initializes the motor PWM.
 */
void Motor_Init(MotorHandle_t* motor);

/**
 * @brief Sets the motor speed.
 * @param speed Value between -1000 (backward) and 1000 (forward). 0 is brake.
 */
void Motor_SetSpeed(MotorHandle_t* motor, int16_t speed);

/**
 * @brief Stops the motor immediately.
 */
void Motor_Stop(MotorHandle_t* motor);

#endif /* CORE_INC_DRV_MOTOR_H_ */
