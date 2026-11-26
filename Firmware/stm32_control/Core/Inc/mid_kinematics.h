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
// Conjunto nuevo: motor Pololu de 100 RPM con rueda de 40 mm de diametro:
//   100 RPM / 60 * pi * 40 mm = 209 mm/s (nominal, sin carga).
// Con carga va a ser algo menor: ajustar si se mide en el piso.
#ifndef ROBOT_MAX_WHEEL_SPEED_MMS
#define ROBOT_MAX_WHEEL_SPEED_MMS 209.0f
#endif

// PWM proporcional a la velocidad pedida, con un piso de arranque: una rueda
// recibe 0 o un PWM de al menos este valor (en modulo), porque por debajo los
// motores no arrancan desde parado.
// PROVISORIO 3000 (~36 %): CALIBRAR en el piso como el menor PWM con el que
// ambas ruedas arrancan desde parado (maximo 4000). Tiene que quedar por debajo
// de 4200 (50 %): si no, NAV_SPEED_PCT = 50 no se respeta.
#ifndef ROBOT_WHEEL_MIN_PWM
#define ROBOT_WHEEL_MIN_PWM 3000
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
