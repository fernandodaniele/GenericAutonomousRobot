/*
 * mid_kinematics.h
 *
 *  Created on: Feb 15, 2026
 *      Author: Fernando E. Daniele
 */

#ifndef CORE_INC_MID_KINEMATICS_H_
#define CORE_INC_MID_KINEMATICS_H_

#include <stdint.h>

// Physical configuration of the robot (in mm or arbitrary scale units)
#define ROBOT_WHEEL_BASE 200.0f  // Distance between wheels in mm

typedef struct {
    float linear_vel;   // Desired linear velocity
    float angular_vel;  // Desired angular velocity
} RobotCommand_t;

/**
 * @brief Calculates the velocities of each wheel based on robot commands.
 * @param cmd Structure with desired velocities.
 * @param out_l_pwm Pointer where the value for the left motor will be saved.
 * @param out_r_pwm Pointer where the value for the right motor will be saved.
 */
void Kinematics_Compute(RobotCommand_t cmd, int16_t* out_l_pwm, int16_t* out_r_pwm);

#endif /* CORE_INC_MID_KINEMATICS_H_ */
