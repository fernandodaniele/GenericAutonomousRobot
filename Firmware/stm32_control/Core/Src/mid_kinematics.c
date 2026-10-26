/*
 * mid_kinematics.c
 *
 *  Created on: Feb 15, 2026
 *      Author: Fernando E. Daniele
 *  Updated on: Oct 26, 2026
 *      Implementacion de cinematica diferencial
 */

#include "mid_kinematics.h"
#include "drv_motor.h"

void Kinematics_Compute(RobotCommand_t cmd, int16_t* out_l_pwm, int16_t* out_r_pwm) {
    if (!out_l_pwm || !out_r_pwm) return;

    // Modelo cinematico diferencial:
    // v_l = linear_vel - (angular_vel * ROBOT_WHEEL_BASE / 2.0f)
    // v_r = linear_vel + (angular_vel * ROBOT_WHEEL_BASE / 2.0f)
    float v_l = cmd.linear_vel - (cmd.angular_vel * (ROBOT_WHEEL_BASE / 2.0f));
    float v_r = cmd.linear_vel + (cmd.angular_vel * (ROBOT_WHEEL_BASE / 2.0f));

    // Saturacion de limites PWM (-8399 a 8399)
    if (v_l > (float)MOTOR_MAX_SPEED) v_l = (float)MOTOR_MAX_SPEED;
    if (v_l < (float)MOTOR_MIN_SPEED) v_l = (float)MOTOR_MIN_SPEED;

    if (v_r > (float)MOTOR_MAX_SPEED) v_r = (float)MOTOR_MAX_SPEED;
    if (v_r < (float)MOTOR_MIN_SPEED) v_r = (float)MOTOR_MIN_SPEED;

    *out_l_pwm = (int16_t)v_l;
    *out_r_pwm = (int16_t)v_r;
}
