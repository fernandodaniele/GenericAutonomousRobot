/*
 * mid_kinematics.c
 *
 *  Created on: Feb 15, 2026
 *      Author: Fernando E. Daniele
 *  Updated on: Oct 26, 2026
 *      Implementacion de cinematica diferencial
 *  Updated on: Nov 2, 2026
 *      Correccion de unidades: mm/s y rad/s -> cuentas de PWM
 *  Updated on: Nov 6, 2026
 *      PWM proporcional con piso de arranque (reemplaza el todo/nada)
 */

#include "mid_kinematics.h"
#include "drv_motor.h"

/* PWM proporcional con piso de arranque: 0 o |pwm| >= ROBOT_WHEEL_MIN_PWM.
 * Conserva el signo (el sentido de giro no cambia). */
static float floor_pwm(float pwm)
{
    if (pwm > -ROBOT_WHEEL_ZERO_PWM && pwm < ROBOT_WHEEL_ZERO_PWM) return 0.0f;
    if (pwm > 0.0f && pwm < (float)ROBOT_WHEEL_MIN_PWM) return (float)ROBOT_WHEEL_MIN_PWM;
    if (pwm < 0.0f && pwm > -(float)ROBOT_WHEEL_MIN_PWM) return -(float)ROBOT_WHEEL_MIN_PWM;
    return pwm;
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
    v_l = floor_pwm(v_l);
    v_r = floor_pwm(v_r);

    // Saturacion de limites PWM (-8399 a 8399)
    if (v_l > (float)MOTOR_MAX_SPEED) v_l = (float)MOTOR_MAX_SPEED;
    if (v_l < (float)MOTOR_MIN_SPEED) v_l = (float)MOTOR_MIN_SPEED;

    if (v_r > (float)MOTOR_MAX_SPEED) v_r = (float)MOTOR_MAX_SPEED;
    if (v_r < (float)MOTOR_MIN_SPEED) v_r = (float)MOTOR_MIN_SPEED;

    *out_l_pwm = (int16_t)v_l;
    *out_r_pwm = (int16_t)v_r;
}
