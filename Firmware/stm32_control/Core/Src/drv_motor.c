/*
 * drv_motor.c
 *
 *  Created on: Abr 11, 2026
 *      Author: Leonel Gonzalez
 */


#include "drv_motor.h"

void Motor_Init(MotorHandle_t* motor)
{
	//Arranca con velocidad 0
	motor->current_speed = 0;

	//Ambas señales de direccion en bajo
	HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_RESET);

	//Duty cycle inicial en 0 (sin señal PWM)
	__HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);

	//Arranca el PWM en el canal correspondiente
	HAL_TIM_PWM_Start(motor->htim, motor->channel);
}

void Motor_SetSpeed(MotorHandle_t* motor, int16_t speed)
{
	int16_t applied_speed = speed;

	//Saturacion de velocidad para asegurar límites válidos
	if(applied_speed > MOTOR_MAX_SPEED) applied_speed = MOTOR_MAX_SPEED;
	if(applied_speed < MOTOR_MIN_SPEED) applied_speed = MOTOR_MIN_SPEED;

	//Caso especial: velocidad cero (motor en reposo sin movimiento)
	if(applied_speed == 0)
	{
		HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_RESET);

		__HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);

		motor->current_speed = 0;
		return;
	}

	// Configuración de la dirección según el signo de la velocidad
	if(applied_speed > 0)
	{
		HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_SET);
	}

	// Cálculo del PWM usando el valor absoluto de la velocidad
	uint16_t pwm_value = (applied_speed > 0) ? applied_speed : -applied_speed; //si es positivo lo dejo igual, si es negativo lo hago positivo
	// Aplica el duty cycle correspondiente
	__HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pwm_value);

	// Guarda el estado real aplicado (con signo)
	motor->current_speed = applied_speed;

}

void Motor_Stop(MotorHandle_t* motor)
{
	// Activa ambas señales de dirección (freno activo)
    HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);

    motor->current_speed = 0;
}

