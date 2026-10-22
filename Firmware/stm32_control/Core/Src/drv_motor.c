/*
 * drv_motor.c
 *
 *  Created on: Apr 18, 2026
 *      Author: Parucci, Santiago
 *
 *  Adaptado al driver L9110S (IA = PWM, IB = sentido).
 */


#include "../Inc/drv_motor.h"

// Compare mayor al ARR: la salida PWM queda siempre en alto
#define MOTOR_PWM_ALWAYS_HIGH (MOTOR_MAX_SPEED + 1)

/**
 * @brief Freno activo: IA=1, IB=1 (ambas salidas del L9110S en bajo)
 */
static void Motor_Brake(MotorHandle_t* motor)
{
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, MOTOR_PWM_ALWAYS_HIGH);
}

/**
 * @brief Inicializa el motor
 */
void Motor_Init(MotorHandle_t* motor)
{
    // Estado inicial de velocidad en 0
    motor->current_speed = 0;

    // Freno activo antes de arrancar el PWM. Sin velocidad al iniciar
    Motor_Brake(motor);

    // Arranca PWM en canal
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
}


/**
 * @brief Setea velocidad del motor
 */
void Motor_SetSpeed(MotorHandle_t* motor, int16_t speed)
{
    // Saturación para no exceder de límites
    if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
    if (speed < MOTOR_MIN_SPEED) speed = MOTOR_MIN_SPEED;

    // Guarda el estado de velocidad
    motor->current_speed = speed;

    if (speed > 0)
    {
        // Adelante: IB=0, el motor gira mientras IA está en 1
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint32_t)speed);
    }
    else if (speed < 0)
    {
        // Atrás: IB=1, el motor gira mientras IA está en 0 -> ciclo invertido
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel,
                              (uint32_t)(MOTOR_MAX_SPEED + speed));
    }
    else
    {
        // Freno
        Motor_Brake(motor);
    }
}


/**
 * @brief Frena el motor
 */
void Motor_Stop(MotorHandle_t* motor)
{
    motor->current_speed = 0;

    // Freno activo
    Motor_Brake(motor);
}
