/*
 * drv_motor.h
 *
 *  Created on: Abr 11, 2026
 *      Author: Leonel Gonzalez
 */

//Protección contra inclusiones múltiples
#ifndef CORE_INC_DRV_MOTOR_H_
#define CORE_INC_DRV_MOTOR_H_

#include "tim.h"
#include "gpio.h"

// Definiciones de límites de velocidad (0 a 8399)
#define MOTOR_MAX_SPEED 8399
#define MOTOR_MIN_SPEED -8399

typedef struct {
    TIM_HandleTypeDef* htim;    // PWM Timer (e.g. &htim4) TIM 4 -> APB1, PWM 10KHz
    uint32_t channel;           // Canal (e.g. TIM_CHANNEL_1)

    GPIO_TypeDef* port_a;       // Puerto del pin de dirección A
    uint16_t pin_a;             // Pin de dirección A

    GPIO_TypeDef* port_b;       // Puerto del pin de dirección B
    uint16_t pin_b;             // Pin de dirección B

    int16_t current_speed;      // Velocidad actual (-8399 to 8399)
} MotorHandle_t;

/**
 * @brief Inicializa el PWM del motor
 */
void Motor_Init(MotorHandle_t* motor);

/**
 * @brief Configura la velocidad del motor.
 * @param speed Valor entre -8399 (hacia atrás) y 8399 (hacia adelante). 0 es libre.
 */
void Motor_SetSpeed(MotorHandle_t* motor, int16_t speed);

/**
 * @brief Detiene el motor inmediatamente.
 */
void Motor_Stop(MotorHandle_t* motor);

#endif /* CORE_INC_DRV_MOTOR_H_ */
