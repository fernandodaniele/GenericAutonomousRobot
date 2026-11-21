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
#include <stdint.h>

/* B4: techo PROVISIONAL comun. Ajustar solo tras identificar/validar motores.
 * No compensa VM ni representa una especificacion electrica del motor.
 */
#ifndef MOTOR_PWM_MAX_PERCENT
#define MOTOR_PWM_MAX_PERCENT 40U
#endif
#if MOTOR_PWM_MAX_PERCENT > 100 || MOTOR_PWM_MAX_PERCENT < 0
#error "MOTOR_PWM_MAX_PERCENT must be between 0 and 100"
#endif

// Capacidad absoluta de compare; el techo de actuacion lo fija el porcentaje.
#define MOTOR_MAX_SPEED 8399
#define MOTOR_MIN_SPEED -8399

typedef struct {
    TIM_HandleTypeDef* htim;    // PWM Timer (e.g. &htim4) TIM 4 -> APB1, PWM 10KHz
    uint32_t channel;           // Canal (e.g. TIM_CHANNEL_1)

    GPIO_TypeDef* port_a;       // Puerto del pin de dirección A
    uint16_t pin_a;             // Pin de dirección A

    GPIO_TypeDef* port_b;       // Puerto del pin de dirección B
    uint16_t pin_b;             // Pin de dirección B

    int16_t current_speed;      // PWM aplicado firmado; NO velocidad del encoder.
} MotorHandle_t;

/**
 * @brief Inicializa el PWM del motor
 */
void Motor_Init(MotorHandle_t* motor);

/* Limite de compare segun ARR real, porcentaje comun y capacidad del driver.
 * Sin escrituras ni esperas; usar en la misma transaccion que PI/salida.
 */
uint16_t Motor_GetPwmLimit(const MotorHandle_t *motor);

/**
 * @brief Configura la velocidad del motor.
 * @note Solo desde tareas; comprobacion de bloqueo y PWM serializados.
 * @param speed PWM firmado; se limita simetricamente a Motor_GetPwmLimit().
 */
void Motor_SetSpeed(MotorHandle_t* motor, int16_t speed);

/**
 * @brief Detiene el motor inmediatamente.
 * @note Solo desde tareas; no espera ni accede a colas.
 */
void Motor_Stop(MotorHandle_t* motor);

#endif /* CORE_INC_DRV_MOTOR_H_ */
