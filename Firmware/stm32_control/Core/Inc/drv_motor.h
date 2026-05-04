/*
 * drv_ultrasound.h
 *
 *  Created on: Apr 18, 2026
 *      Author: Parucci, Santiago
 */

//Si no está definido se define el archivo. Evita que se incluya mas de una vez

#ifndef CORE_INC_DRV_MOTOR_H_
#define CORE_INC_DRV_MOTOR_H_

#include "tim.h"
#include "gpio.h"

// Escala de velocidad entre 0 a 8399
#define MOTOR_MAX_SPEED 8399
#define MOTOR_MIN_SPEED -8399

typedef struct {
    TIM_HandleTypeDef* htim;   	// Timer PWM
    uint32_t channel;          	// Canal PWM

    GPIO_TypeDef* port_a;      	// Puerto pin A
    uint16_t pin_a;				// Pin dirección A

    GPIO_TypeDef* port_b;      	// Puerto pin B
    uint16_t pin_b;				// Pin dirección B

    int16_t current_speed;     	// Estado actual de velocidad
} MotorHandle_t;

/**
 * @brief Inicializa el motor
 */
void Motor_Init(MotorHandle_t* motor);

/**
 * @brief Inicializa funcion que setea velocidad
 * @param speed -8399 a 8399
 */
void Motor_SetSpeed(MotorHandle_t* motor, int16_t speed);

/**
 * @brief Inicializa funcion que frena el motor
 */
void Motor_Stop(MotorHandle_t* motor);

#endif /* CORE_INC_DRV_MOTOR_H_ */
