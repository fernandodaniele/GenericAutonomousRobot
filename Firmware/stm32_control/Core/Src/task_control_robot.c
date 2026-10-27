#include "task_control_robot.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "task_safety.h"

#define TASK_CONTROL_PERIOD_MS 20U
#define TASK_CONTROL_STACK_SIZE 256U
#define TASK_CONTROL_PRIORITY (tskIDLE_PRIORITY + 3U)
#define TASK_CONTROL_QUEUE_LENGTH 8U

// Estado inicial del robot.
static volatile RobotControlState_t robot_state = ROBOT_STATE_IDLE;

// Cola de eventos para la maquina de estados.
static QueueHandle_t event_queue = NULL;

/**
 * @brief Procesa las transiciones de la maquina de estados.
 */
static void TaskControlRobot_ProcessEvent(RobotControlEvent_t event) {

    // El rearme se procesa fuera de la seccion critica porque
    // TaskSafety_RequestReset consulta la cola de sensores.
    if (event == ROBOT_EVENT_SAFETY_RESET) {

        if (TaskControlRobot_GetState() == ROBOT_STATE_SAFETY_STOP &&
            TaskSafety_RequestReset()) {

            // Descarta eventos anteriores al rearme.
            (void)xQueueReset(event_queue);

            taskENTER_CRITICAL();

            // Verifica nuevamente que no haya aparecido una parada.
            if (!TaskSafety_IsStopActive()) {
                robot_state = ROBOT_STATE_IDLE;
            }

            taskEXIT_CRITICAL();
        }

        return;
    }

    taskENTER_CRITICAL();

    switch (event) {

        case ROBOT_EVENT_START_AUTO:
            if (robot_state == ROBOT_STATE_IDLE &&
                !TaskSafety_IsStopActive()) {

                robot_state = ROBOT_STATE_AUTONOMOUS;
            }
            break;

        case ROBOT_EVENT_START_MANUAL:
            if (robot_state == ROBOT_STATE_IDLE &&
                !TaskSafety_IsStopActive()) {

                robot_state = ROBOT_STATE_MANUAL;
            }
            break;

        case ROBOT_EVENT_STOP:
            if (robot_state != ROBOT_STATE_SAFETY_STOP) {
                robot_state = ROBOT_STATE_IDLE;
            }
            break;

        case ROBOT_EVENT_SAFETY_STOP:
            // Ignorar eventos de parada antiguos,
            // anteriores a un rearme exitoso.
            if (TaskSafety_IsStopActive()) {
                robot_state = ROBOT_STATE_SAFETY_STOP;
            }
            break;

        case ROBOT_EVENT_SAFETY_RESET:
            // Procesado al comienzo de la funcion.
            break;

        default:
            break;
    }

    taskEXIT_CRITICAL();
}

/**
 * @brief Tarea periodica de control del robot.
 */
static void TaskControlRobot_Run(void *parameters) {
    (void)parameters;

    RobotControlEvent_t event;

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {

        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS));

        // Procesa todos los eventos pendientes.
        while (xQueueReceive(event_queue, &event, 0U) == pdTRUE) {
            TaskControlRobot_ProcessEvent(event);
        }

        // Pendiente: incorporar logica de navegacion
        // y coordinacion con las demas tareas.
    }
}

/**
 * @brief Crea la tarea FreeRTOS de control del robot.
 */
void TaskControlRobot_Create(void) {

    if (event_queue != NULL) {
        Error_Handler();
        return;
    }

    robot_state = ROBOT_STATE_IDLE;

    event_queue = xQueueCreate(
        TASK_CONTROL_QUEUE_LENGTH,
        sizeof(RobotControlEvent_t)
    );

    if (event_queue == NULL) {
        Error_Handler();
        return;
    }

    BaseType_t result = xTaskCreate(
        TaskControlRobot_Run,
        "ControlRobot",
        TASK_CONTROL_STACK_SIZE,
        NULL,
        TASK_CONTROL_PRIORITY,
        NULL
    );

    if (result != pdPASS) {
        vQueueDelete(event_queue);
        event_queue = NULL;
        Error_Handler();
    }
}

/**
 * @brief Envia un evento a la maquina de estados.
 */
bool TaskControlRobot_SendEvent(RobotControlEvent_t event) {

    if (event_queue == NULL ||
        event < ROBOT_EVENT_START_AUTO ||
        event > ROBOT_EVENT_SAFETY_RESET) {
        return false;
    }

    return (xQueueSend(event_queue, &event, 0U) == pdTRUE);
}

/**
 * @brief Obtiene el estado actual del robot.
 */
RobotControlState_t TaskControlRobot_GetState(void) {

    RobotControlState_t state;

    taskENTER_CRITICAL();
    state = robot_state;
    taskEXIT_CRITICAL();

    return state;
}