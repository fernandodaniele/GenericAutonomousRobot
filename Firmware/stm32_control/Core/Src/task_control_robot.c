#include "task_control_robot.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "task_safety.h"
#include "task_motor_control.h"

#define TASK_CONTROL_PERIOD_MS 20U
#define TASK_CONTROL_STACK_SIZE 256U
#define TASK_CONTROL_PRIORITY (tskIDLE_PRIORITY + 3U)
#define TASK_CONTROL_QUEUE_LENGTH 8U

// Estado inicial del robot.
static volatile RobotControlState_t robot_state = ROBOT_STATE_IDLE;
static bool stop_pending = false;
static TickType_t mode_entry_tick = 0U;

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
                mode_entry_tick = xTaskGetTickCount();
            }

            taskEXIT_CRITICAL();
        }

        return;
    }

    taskENTER_CRITICAL();

    switch (event) {

        case ROBOT_EVENT_START_AUTO:
            if (robot_state == ROBOT_STATE_IDLE &&
                !TaskSafety_IsStopActive() && !stop_pending) {

                robot_state = ROBOT_STATE_AUTONOMOUS;
                mode_entry_tick = xTaskGetTickCount();
            }
            break;

        case ROBOT_EVENT_START_MANUAL:
            if (robot_state == ROBOT_STATE_IDLE &&
                !TaskSafety_IsStopActive() && !stop_pending) {

                robot_state = ROBOT_STATE_MANUAL;
                mode_entry_tick = xTaskGetTickCount();
            }
            break;

        case ROBOT_EVENT_STOP:
            if (robot_state != ROBOT_STATE_SAFETY_STOP) {
                TaskMotorControl_Stop();
                robot_state = ROBOT_STATE_IDLE;
                mode_entry_tick = xTaskGetTickCount();
            }
            stop_pending = false;
            break;

        case ROBOT_EVENT_SAFETY_STOP:
            // Ignorar eventos de parada antiguos,
            // anteriores a un rearme exitoso.
            if (TaskSafety_IsStopActive()) {
                if (robot_state != ROBOT_STATE_SAFETY_STOP) {
                    TaskMotorControl_Stop();
                }
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
        /* Incluso con cola llena, una solicitud STOP ya anulo los motores y
         * debe completar la transicion a IDLE. No depende de encolarla de nuevo.
         */
        if (TaskControlRobot_IsStopPending()) TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);

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
    stop_pending = false;
    mode_entry_tick = xTaskGetTickCount();

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

    if (event == ROBOT_EVENT_STOP) {
        /* Stop, marca y encolado forman una operacion de tarea; IRQ activas,
         * cola no bloqueante FUERA de la seccion critica. */
        vTaskSuspendAll();
        taskENTER_CRITICAL();
        stop_pending = true;
        TaskMotorControl_Stop();
        taskEXIT_CRITICAL();
        bool queued = xQueueSend(event_queue, &event, 0U) == pdTRUE;
        (void)xTaskResumeAll();
        return queued;
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

bool TaskControlRobot_IsStopPending(void) {
    taskENTER_CRITICAL();
    bool pending = stop_pending;
    taskEXIT_CRITICAL();
    return pending;
}

bool TaskControlRobot_AcceptsMotionAt(RobotControlState_t mode, TickType_t issued_tick) {
    taskENTER_CRITICAL();
    TickType_t now = xTaskGetTickCount();
    bool accepted = (mode == ROBOT_STATE_MANUAL || mode == ROBOT_STATE_AUTONOMOUS) &&
        robot_state == mode && !stop_pending &&
        (TickType_t)(now - issued_tick) < (TickType_t)(now - mode_entry_tick);
    taskEXIT_CRITICAL();
    return accepted;
}
