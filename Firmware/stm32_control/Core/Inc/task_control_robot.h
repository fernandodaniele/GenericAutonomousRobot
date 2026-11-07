#ifndef TASK_CONTROL_ROBOT_H_
#define TASK_CONTROL_ROBOT_H_

#include <stdbool.h>
#include "FreeRTOS.h"

/**
 * @brief Estados de funcionamiento del robot.
 */
typedef enum {
    ROBOT_STATE_IDLE = 0,
    ROBOT_STATE_AUTONOMOUS,
    ROBOT_STATE_MANUAL,
    ROBOT_STATE_SAFETY_STOP
} RobotControlState_t;

/**
 * @brief Eventos que provocan transiciones de estado.
 */
typedef enum {
    ROBOT_EVENT_START_AUTO = 0,
    ROBOT_EVENT_START_MANUAL,
    ROBOT_EVENT_STOP,
    ROBOT_EVENT_SAFETY_STOP,
    ROBOT_EVENT_SAFETY_RESET
} RobotControlEvent_t;

/**
 * @brief Crea la tarea FreeRTOS de control del robot.
 */
void TaskControlRobot_Create(void);

/**
 * @brief Solicita una transición mediante un evento.
 * @param event Evento solicitado.
 * @return true si el evento fue aceptado para procesamiento.
 */
bool TaskControlRobot_SendEvent(RobotControlEvent_t event);

/**
 * @brief Obtiene el estado actual del robot.
 * @return Estado de la máquina de estados.
 */
RobotControlState_t TaskControlRobot_GetState(void);

/* Solicitud STOP recibida pero aun no procesada por la FSM. */
bool TaskControlRobot_IsStopPending(void);

/* Frescura respecto de la entrada efectiva al modo. Igual tick se rechaza
 * conservadoramente: no puede probarse el orden entre recepcion y transicion.
 */
bool TaskControlRobot_AcceptsMotionAt(RobotControlState_t mode, TickType_t issued_tick);

#endif /* TASK_CONTROL_ROBOT_H_ */
