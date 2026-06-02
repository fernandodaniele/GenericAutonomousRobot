/*
 * drv_ultrasound.h
 *
 *  Created on: Apr 18, 2026
 *      Author: Parucci, Santiago
 */


#include "../Inc/drv_motor.h"

/**
 * @brief Inicializa el motor
 */
void Motor_Init(MotorHandle_t* motor)
{
    // Arranca PWM en canal
    HAL_TIM_PWM_Start(motor->htim, motor->channel);

    // Estado inicial de velocidad en 0
    motor->current_speed = 0;

    // Freno activo (IN1=1, IN2=1). Sin velocidad al iniciar
    HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_SET);

    // PWM en 0
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
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

    // Valor absoluto → PWM
    uint16_t pwm = (speed >= 0) ? speed : -speed;

    // Configuración de dirección o de freno
    if (speed > 0)
    {
        // Adelante
        HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_RESET);
    }
    else if (speed < 0)
    {
        // Atrás
        HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_SET);
    }
    else
    {
        // Freno
        HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_SET);
    }

    // Cálculo del PWM usando el valor absoluto de la velocidad
    uint16_t pwm_value = (speed > 0) ? speed : -speed;

    // Aplicar PWM
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pwm);
}


/**
 * @brief Frena el motor
 */
void Motor_Stop(MotorHandle_t* motor)
{
    motor->current_speed = 0;

    // Freno activo
    HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_SET);

    // PWM en 0
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
}
