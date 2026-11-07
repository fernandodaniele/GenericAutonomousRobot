#ifndef MID_MOTION_H_
#define MID_MOTION_H_

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task_control_robot.h"

/* Provisionales en cuentas/s, sin conversion mecanica ni velocidades UART. */
#define MOTION_DRIVE_CPS 1600
#define MOTION_TURN_CPS 1200

typedef enum {
    MOTION_FORWARD = 0,
    MOTION_REVERSE,
    MOTION_TURN_LEFT,
    MOTION_TURN_RIGHT,
    MOTION_HALT
} MotionCommand_t;

/* Desde tarea. Una llamada = una emision, nunca una renovacion periodica.
 * UART debe exigir MANUAL. Un futuro productor AUTO debe validar sus datos y
 * pasar su instante de emision real; esta funcion no cambia modos ni rearma.
 */
bool Motion_Execute(MotionCommand_t command, RobotControlState_t required_mode,
                    TickType_t issued_tick);

#endif
