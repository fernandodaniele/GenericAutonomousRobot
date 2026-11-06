/*
 * mid_kinematics.h
 *
 *  Created on: Feb 15, 2026
 *      Author: Fernando E. Daniele
 */

#ifndef CORE_INC_MID_KINEMATICS_H_
#define CORE_INC_MID_KINEMATICS_H_

#include <stdint.h>

// Physical configuration of the robot
#define ROBOT_WHEEL_BASE 200.0f  // Distance between wheels in mm

// Velocidad de rueda (mm/s) que se obtiene con PWM maximo (MOTOR_MAX_SPEED).
// Medido a ojo por el grupo: a lo sumo 50 mm/s. Ajustar si se mide mejor.
#ifndef ROBOT_MAX_WHEEL_SPEED_MMS
#define ROBOT_MAX_WHEEL_SPEED_MMS 50.0f
#endif

// PWM proporcional a la velocidad pedida, con un piso de arranque: una rueda
// recibe 0 o un PWM de al menos este valor (en modulo), porque por debajo los
// motores no arrancan desde parado.
// Valor inicial 4200 (50 %): CALIBRAR en el granito como el menor PWM con el
// que ambas ruedas arrancan desde parado.
#ifndef ROBOT_WHEEL_MIN_PWM
#define ROBOT_WHEEL_MIN_PWM 4200
#endif
// Pedidos por debajo de esto (en cuentas de PWM) se toman como 0: rueda parada.
#ifndef ROBOT_WHEEL_ZERO_PWM
#define ROBOT_WHEEL_ZERO_PWM 1.0f
#endif

typedef struct {
    float linear_vel;   // Desired linear velocity in mm/s
    float angular_vel;  // Desired angular velocity in rad/s (+ = turn left)
} RobotCommand_t;

/**
 * @brief Calculates the velocities of each wheel based on robot commands.
 * @param cmd Structure with desired velocities.
 * @param out_l_pwm Pointer where the value for the left motor will be saved.
 * @param out_r_pwm Pointer where the value for the right motor will be saved.
 */
void Kinematics_Compute(RobotCommand_t cmd, int16_t* out_l_pwm, int16_t* out_r_pwm);

#endif /* CORE_INC_MID_KINEMATICS_H_ */
