#ifndef TASK_MOTOR_CONTROL_H_
#define TASK_MOTOR_CONTROL_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task_control_robot.h"

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
 *       Vencen a los 500 ms desde su emision en esta llamada, no su consumo.
 *       El PI aplica PWM solo en modo habilitado y sin bloqueo de seguridad.
 *       Solo desde tareas; cada llamada aceptada emite una consigna nueva.
 *       Emisiones/paradas se ordenan por generacion, incluso dentro de un tick.
 *       B7.2: solo MANUAL admite actuacion; AUTO permanece detenido.
 */
void TaskMotorControl_SetTargetSpeeds(int32_t left_speed_cps,
                                      int32_t right_speed_cps);

/* Emision con fecha de origen (p.ej. primer byte UART), sin rejuvenecer cola.
 * Exige modo indicado y origen estrictamente posterior al ultimo Stop y a la
 * entrada al modo. Igual tick se rechaza. Cero/cero usa Stop B1 inmediato.
 * true confirma admision/publicacion, no movimiento fisico ni ausencia futura
 * de fallas. Contexto de tarea, nunca desde ISR.
 */
bool TaskMotorControl_TrySetTargetSpeedsAt(int32_t left_speed_cps, int32_t right_speed_cps,
                                         TickType_t issued_tick, RobotControlState_t required_mode);

/**
 * @brief Invalida consignas previas, reinicia PI y pone ambos PWM a cero.
 * @note Solo desde tareas. Breve, sin colas ni esperas; admite seccion critica
 *       externa. La epoca invalida incluso un encolado previo que llegue tarde.
 */
void TaskMotorControl_Stop(void);

#endif /* TASK_MOTOR_CONTROL_H_ */
