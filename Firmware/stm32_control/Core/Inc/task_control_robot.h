#ifndef TASK_CONTROL_ROBOT_H_
#define TASK_CONTROL_ROBOT_H_

#include <stdbool.h>

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

#endif /* TASK_CONTROL_ROBOT_H_ */