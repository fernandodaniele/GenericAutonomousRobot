#ifndef TASK_MOTOR_CONTROL_H_
#define TASK_MOTOR_CONTROL_H_

#include <stdint.h>

#include "drv_encoder.h"
#include "drv_motor.h"

/**
 * @brief Crea la tarea periódica de medición y gestión de velocidades de los motores.
 * @param encoder_left Encoder del motor izquierdo.
 * @param encoder_right Encoder del motor derecho.
 * @param motor_left Driver del motor izquierdo.
 * @param motor_right Driver del motor derecho.
 */
void TaskMotorControl_Create(EncoderHandle_t *encoder_left,
                             EncoderHandle_t *encoder_right,
                             MotorHandle_t *motor_left,
                             MotorHandle_t *motor_right);

/**
 * @brief Establece las velocidades objetivo de ambos motores.
 * @param left_speed_cps Velocidad objetivo del motor izquierdo en cps.
 * @param right_speed_cps Velocidad objetivo del motor derecho en cps.
 * @note Las órdenes se reciben mediante una cola de FreeRTOS.
 *       Deben renovarse periódicamente, ya que vencen después de 500 ms.
 * @note La regulación automática del PWM mediante PI/PID está pendiente.
 *       Actualmente los objetivos se almacenan, pero no se aplican
 *       a los motores.
 */
void TaskMotorControl_SetTargetSpeeds(int32_t left_speed_cps,
                                      int32_t right_speed_cps);

#endif /* TASK_MOTOR_CONTROL_H_ */
