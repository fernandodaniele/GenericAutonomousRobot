/*
 * drv_motor.c
 *
 *  Created on: Apr 18, 2026
 *      Author: Parucci, Santiago
 *  Updated on: Oct 26, 2026
 *      Implementacion para driver de motores L9110S
 */

#include "drv_motor.h"
#include "main.h"

/* Instancias estaticas para ambos motores del robot */
static MotorHandle_t g_motor_left;
static MotorHandle_t g_motor_right;

void Motor_Init(MotorHandle_t* motor) {
    if (!motor || !motor->htim) return;

    // Sincronizar campos de retrocompatibilidad si se usaron
    if (!motor->dir_port && motor->port_a) {
        motor->dir_port = motor->port_a;
        motor->dir_pin  = motor->pin_a;
    }

    // Iniciar canal PWM en el temporizador
    HAL_TIM_PWM_Start(motor->htim, motor->channel);

    // Estado inicial: velocidad 0, PWM = 0, DIR = 0 (parada suave)
    motor->current_speed = 0;
    if (motor->dir_port) {
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
}

void Motor_SetSpeed(MotorHandle_t* motor, int16_t speed) {
    if (!motor || !motor->htim) return;

    // Saturacion de limites
    if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
    if (speed < MOTOR_MIN_SPEED) speed = MOTOR_MIN_SPEED;

    motor->current_speed = speed;

    // Aplicar inversion mecanica si esta configurada
    int16_t eff_speed = motor->inverted ? -speed : speed;

    if (eff_speed > 0) {
        // AVANCE:
        // Entrada B (DIR) = LOW (0)
        // Entrada A (PWM) = Ciclo de trabajo proporcional a eff_speed
        if (motor->dir_port) {
            HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
        }
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint16_t)eff_speed);
    } else if (eff_speed < 0) {
        // RETROCESO:
        // Entrada B (DIR) = HIGH (1)
        // Entrada A (PWM) = Ciclo de trabajo invertido: (MOTOR_MAX_SPEED - abs(eff_speed))
        // Cuando PWM esta en LOW, el motor recibe (0, 1) -> conduccion en reversa.
        // Cuando PWM esta en HIGH, el motor recibe (1, 1) -> freno.
        // El tiempo efectivo de traccion inversa es proporcional a abs(eff_speed).
        uint16_t abs_speed = (uint16_t)(-eff_speed);
        uint16_t compare_val = MOTOR_MAX_SPEED - abs_speed;

        if (motor->dir_port) {
            HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
        }
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, compare_val);
    } else {
        // PARADA (0):
        // Entrada B (DIR) = LOW (0)
        // Entrada A (PWM) = 0
        // Ambas entradas en (0, 0) -> Parada suave (sin consumo)
        if (motor->dir_port) {
            HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
        }
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
    }
}

void Motor_Stop(MotorHandle_t* motor) {
    if (!motor || !motor->htim) return;
    motor->current_speed = 0;
    if (motor->dir_port) {
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
}

void Motor_Brake(MotorHandle_t* motor) {
    if (!motor || !motor->htim) return;
    motor->current_speed = 0;
    // Freno dinamico: ambas entradas en nivel alto (1, 1)
    if (motor->dir_port) {
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
    }
    // Compare = ARR + 1 -> la salida PWM queda siempre en alto (sin el tick en bajo por periodo)
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, MOTOR_MAX_SPEED + 1);
}

void Motors_InitAll(void) {
    extern TIM_HandleTypeDef htim4;

    // Motor Izquierdo: PWM en PB7 (TIM4_CH2), DIR en PD0
    g_motor_left.htim      = &htim4;
    g_motor_left.channel   = TIM_CHANNEL_2;
    g_motor_left.dir_port  = MOTOR_L_DIR_GPIO_Port;
    g_motor_left.dir_pin   = MOTOR_L_DIR_Pin;
    g_motor_left.inverted  = 0;
    Motor_Init(&g_motor_left);

    // Motor Derecho: PWM en PB8 (TIM4_CH3), DIR en PD2
    g_motor_right.htim     = &htim4;
    g_motor_right.channel  = TIM_CHANNEL_3;
    g_motor_right.dir_port = MOTOR_R_DIR_GPIO_Port;
    g_motor_right.dir_pin  = MOTOR_R_DIR_Pin;
    g_motor_right.inverted = 0;
    Motor_Init(&g_motor_right);
}

void Motors_SetSpeeds(int16_t speed_left, int16_t speed_right) {
    Motor_SetSpeed(&g_motor_left, speed_left);
    Motor_SetSpeed(&g_motor_right, speed_right);
}

void Motors_StopAll(void) {
    Motor_Stop(&g_motor_left);
    Motor_Stop(&g_motor_right);
}

MotorHandle_t* Motor_GetLeft(void) {
    return &g_motor_left;
}

MotorHandle_t* Motor_GetRight(void) {
    return &g_motor_right;
}
