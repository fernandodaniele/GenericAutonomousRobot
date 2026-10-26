/*
 * drv_motor.h
 *
 *  Created on: Apr 18, 2026
 *      Author: Parucci, Santiago
 *  Updated on: Oct 26, 2026
 *      Adaptacion para driver de motores L9110S
 */

#ifndef CORE_INC_DRV_MOTOR_H_
#define CORE_INC_DRV_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

// Escala de velocidad entre -8399 y 8399 (periodo TIM4)
#define MOTOR_MAX_SPEED  8399
#define MOTOR_MIN_SPEED -8399

/**
 * @brief Estructura de control para un canal de motor con driver L9110S.
 *        L9110S utiliza dos entradas por motor:
 *        - Entrada A: Conectada al canal de PWM (velocidad y sentido directo)
 *        - Entrada B: Conectada a un pin GPIO de direccion
 */
typedef struct {
    TIM_HandleTypeDef* htim;    // Timer PWM (ej. &htim4)
    uint32_t channel;           // Canal PWM (ej. TIM_CHANNEL_2 en PB7)

    GPIO_TypeDef* dir_port;     // Puerto GPIO del pin de direccion (ej. GPIOD)
    uint16_t dir_pin;           // Pin de direccion (ej. GPIO_PIN_0 para A-IB)

    uint8_t inverted;           // Inversion mecanica de giro (1 = invertido, 0 = normal)
    int16_t current_speed;      // Estado actual de velocidad (-8399 a 8399)

    /* Campos de retrocompatibilidad con codigo previo */
    GPIO_TypeDef* port_a;
    uint16_t pin_a;
    GPIO_TypeDef* port_b;
    uint16_t pin_b;
} MotorHandle_t;

/**
 * @brief Inicializa el motor (PWM en 0 y direccion en bajo)
 */
void Motor_Init(MotorHandle_t* motor);

/**
 * @brief Configura la velocidad y sentido de giro del motor L9110S
 * @param motor Puntero a la estructura MotorHandle_t
 * @param speed Rango de -8399 (max retroceso) a 8399 (max avance). 0 = parada suave
 */
void Motor_SetSpeed(MotorHandle_t* motor, int16_t speed);

/**
 * @brief Detiene el motor (desactiva ambas entradas -> parada por inercia / coast)
 */
void Motor_Stop(MotorHandle_t* motor);

/**
 * @brief Aplica freno dinamico activo (ambas entradas en nivel alto)
 */
void Motor_Brake(MotorHandle_t* motor);

/**
 * @brief Inicializa los dos motores del robot (izquierdo y derecho) con la configuracion por defecto
 */
void Motors_InitAll(void);

/**
 * @brief Fija la velocidad de ambos motores a la vez
 * @param speed_left Velocidad motor izquierdo (-8399 a 8399)
 * @param speed_right Velocidad motor derecho (-8399 a 8399)
 */
void Motors_SetSpeeds(int16_t speed_left, int16_t speed_right);

/**
 * @brief Detiene ambos motores
 */
void Motors_StopAll(void);

/**
 * @brief Obtiene el descriptor del motor izquierdo
 */
MotorHandle_t* Motor_GetLeft(void);

/**
 * @brief Obtiene el descriptor del motor derecho
 */
MotorHandle_t* Motor_GetRight(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_DRV_MOTOR_H_ */
