/*
 * drv_motor.c
 *
 *  Created on: Abr 11, 2026
 *      Author: Leonel Gonzalez
 */


#include "drv_motor.h"
#include "task_safety.h"
#include "FreeRTOS.h"
#include "task.h"

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

uint16_t Motor_GetPwmLimit(const MotorHandle_t *motor)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->htim);
    uint64_t limit = (((uint64_t)arr + 1U) * MOTOR_PWM_MAX_PERCENT) / 100U;
    if (limit > arr) limit = arr;
    if (limit > MOTOR_MAX_SPEED) limit = MOTOR_MAX_SPEED;
    if (limit > INT16_MAX) limit = INT16_MAX;
    return (uint16_t)limit;
}

void Motor_SetSpeed(MotorHandle_t* motor, int16_t speed)
{
	// Serializa comprobacion y escrituras con ReportFault/STOP, solo desde tarea.
	taskENTER_CRITICAL();
	if (TaskSafety_IsStopActive()) {
		Motor_Stop(motor);
		taskEXIT_CRITICAL();
		return;
	}
	int16_t applied_speed = speed;

	// B4: todas las rutas normales/directas pasan por este techo comun.
	// Cero y Safety no necesitan calcular el limite para detenerse.
	if (applied_speed != 0) {
		int16_t limit = (int16_t)Motor_GetPwmLimit(motor);
		if (applied_speed > limit) applied_speed = limit;
		if (applied_speed < -limit) applied_speed = (int16_t)-limit;
	}

	//Caso especial: velocidad cero (motor en reposo sin movimiento)
	if(applied_speed == 0)
	{
		HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_RESET);

		__HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);

		motor->current_speed = 0;
		taskEXIT_CRITICAL();
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
	taskEXIT_CRITICAL();

}

void Motor_Stop(MotorHandle_t* motor)
{
	taskENTER_CRITICAL();
	// Activa ambas señales de dirección (freno activo)
    HAL_GPIO_WritePin(motor->port_a, motor->pin_a, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->port_b, motor->pin_b, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);

    motor->current_speed = 0;
	taskEXIT_CRITICAL();
}

