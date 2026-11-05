/*
 * mid_kinematics.c
 *
 *  Created on: Feb 15, 2026
 *      Author: Fernando E. Daniele
 *  Updated on: Oct 26, 2026
 *      Implementacion de cinematica diferencial
 *  Updated on: Nov 2, 2026
 *      Correccion de unidades: mm/s y rad/s -> cuentas de PWM
 */

#include "mid_kinematics.h"
#include "drv_motor.h"

static float on_off_pwm(float pwm)
{
    if (pwm >= (float)ROBOT_WHEEL_ON_PWM) return (float)MOTOR_MAX_SPEED;
    if (pwm <= -(float)ROBOT_WHEEL_ON_PWM) return (float)MOTOR_MIN_SPEED;
    return 0.0f;
}

void Kinematics_Compute(RobotCommand_t cmd, int16_t* out_l_pwm, int16_t* out_r_pwm) {
    if (!out_l_pwm || !out_r_pwm) return;

    // Modelo cinematico diferencial:
    // v_l = linear_vel - (angular_vel * ROBOT_WHEEL_BASE / 2.0f)
    // v_r = linear_vel + (angular_vel * ROBOT_WHEEL_BASE / 2.0f)
    float v_l = cmd.linear_vel - (cmd.angular_vel * (ROBOT_WHEEL_BASE / 2.0f));
    float v_r = cmd.linear_vel + (cmd.angular_vel * (ROBOT_WHEEL_BASE / 2.0f));

    // Unidades: v_l y v_r estan en mm/s (linear_vel en mm/s, angular_vel en rad/s,
    // ROBOT_WHEEL_BASE en mm). Se convierten a cuentas de PWM antes de saturar.
    const float mms_to_pwm = (float)MOTOR_MAX_SPEED / ROBOT_MAX_WHEEL_SPEED_MMS;
    v_l *= mms_to_pwm;
    v_r *= mms_to_pwm;
    v_l = on_off_pwm(v_l);
    v_r = on_off_pwm(v_r);

    // Saturacion de limites PWM (-8399 a 8399)
    if (v_l > (float)MOTOR_MAX_SPEED) v_l = (float)MOTOR_MAX_SPEED;
    if (v_l < (float)MOTOR_MIN_SPEED) v_l = (float)MOTOR_MIN_SPEED;

    if (v_r > (float)MOTOR_MAX_SPEED) v_r = (float)MOTOR_MAX_SPEED;
    if (v_r < (float)MOTOR_MIN_SPEED) v_r = (float)MOTOR_MIN_SPEED;

    *out_l_pwm = (int16_t)v_l;
    *out_r_pwm = (int16_t)v_r;
}
