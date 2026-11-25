/*
 * mid_kinematics.h
 *
 *  Created on: Feb 15, 2026
 *      Author: Fernando E. Daniele
 */

#ifndef CORE_INC_MID_KINEMATICS_H_
#define CORE_INC_MID_KINEMATICS_H_

#include <stdint.h>

/* Valor de un esqueleto sin uso en el control. La geometría de odometría
 * está en odometry_config.h. */
#define ROBOT_WHEEL_BASE 200.0f

typedef struct {
    float linear_vel;
    float angular_vel;
} RobotCommand_t;

/* Esqueleto sin implementación. No escribe las salidas. */
void Kinematics_Compute(RobotCommand_t cmd, int16_t* out_l_pwm, int16_t* out_r_pwm);

#endif /* CORE_INC_MID_KINEMATICS_H_ */
