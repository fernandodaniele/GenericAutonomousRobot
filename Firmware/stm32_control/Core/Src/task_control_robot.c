#include "task_control_robot.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "task_safety.h"
#include "task_motor_control.h"
#include "mid_log.h"
#include "mid_navigation.h"

#define TASK_CONTROL_PERIOD_MS 20U
#define TASK_CONTROL_STACK_SIZE 256U
#define TASK_CONTROL_PRIORITY (tskIDLE_PRIORITY + 3U)
#define TASK_CONTROL_QUEUE_LENGTH 8U
/* Misma ventana de frescura de origen que las ordenes UART B6. */
#define TASK_CONTROL_AUTO_REQUEST_TIMEOUT_MS 500U

// Estado inicial del robot.
static volatile RobotControlState_t robot_state = ROBOT_STATE_IDLE;
static bool stop_pending = false;
static TickType_t mode_entry_tick = 0U;
static uint32_t auto_request_epoch = 0U;
static TickType_t auto_barrier_tick = 0U;

typedef struct {
    RobotControlEvent_t event;
    TickType_t issued_tick;
    uint32_t auto_epoch;
} TaskControlRobotEventMessage_t;

// Cola de eventos para la maquina de estados.
static QueueHandle_t event_queue = NULL;
static TaskHandle_t control_task_handle = NULL;

/* Solo desde tarea; admite una seccion critica externa. */
static void TaskControlRobot_InvalidateAuto(void) {
    taskENTER_CRITICAL();
    ++auto_request_epoch;
    auto_barrier_tick = xTaskGetTickCount();
    taskEXIT_CRITICAL();
}

/* Requiere seccion critica: comparar edades, no timestamps absolutos.
 * Igual tick que una barrera es ambiguo y se rechaza como en B6.
 */
static bool TaskControlRobot_AutoRequestCurrent(const TaskControlRobotEventMessage_t *message) {
    TickType_t now = xTaskGetTickCount();
    TickType_t age = (TickType_t)(now - message->issued_tick);
    return message->auto_epoch == auto_request_epoch &&
        age < pdMS_TO_TICKS(TASK_CONTROL_AUTO_REQUEST_TIMEOUT_MS) &&
        age < (TickType_t)(now - auto_barrier_tick) &&
        age < (TickType_t)(now - mode_entry_tick);
}

/**
 * @brief Procesa las transiciones de la maquina de estados.
 */
static void TaskControlRobot_ProcessEventAt(RobotControlEvent_t event,
        const TaskControlRobotEventMessage_t *auto_message) {
    RobotControlState_t previous_state;
    RobotControlState_t next_state;

    // El rearme se procesa fuera de la seccion critica porque
    // TaskSafety_RequestReset consulta la cola de sensores.
    if (event == ROBOT_EVENT_SAFETY_RESET) {
        TaskControlRobot_InvalidateAuto();
        if (TaskControlRobot_GetState() == ROBOT_STATE_AUTONOMOUS) TaskMotorControl_Stop();

        if (TaskControlRobot_GetState() == ROBOT_STATE_SAFETY_STOP &&
            TaskSafety_RequestReset()) {

            // Descarta eventos anteriores al rearme.
            (void)xQueueReset(event_queue);

            taskENTER_CRITICAL();

            previous_state = robot_state;
            // Verifica nuevamente que no haya aparecido una parada.
            if (!TaskSafety_IsStopActive()) {
                robot_state = ROBOT_STATE_IDLE;
                mode_entry_tick = xTaskGetTickCount();
            }

            next_state = robot_state;
            taskEXIT_CRITICAL();
            if (previous_state != next_state)
                (void)Log_EventPublish(110U, (uint32_t)previous_state, (uint32_t)next_state, (uint32_t)event);
        }

        return;
    }

    taskENTER_CRITICAL();

    previous_state = robot_state;
    switch (event) {

        case ROBOT_EVENT_START_AUTO:
            /* Validacion y transicion atomicas; el registro queda fuera del lock. */
            if ((auto_message == NULL || TaskControlRobot_AutoRequestCurrent(auto_message)) &&
                robot_state == ROBOT_STATE_IDLE &&
                !TaskSafety_IsStopActive() && !stop_pending) {

                TaskMotorControl_Stop();
                robot_state = ROBOT_STATE_AUTONOMOUS;
                mode_entry_tick = xTaskGetTickCount();
                Navigation_Begin(mode_entry_tick);
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
            TaskControlRobot_InvalidateAuto();
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
                    TaskControlRobot_InvalidateAuto();
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

    next_state = robot_state;
    taskEXIT_CRITICAL();
    if (previous_state != next_state)
        (void)Log_EventPublish(110U, (uint32_t)previous_state, (uint32_t)next_state, (uint32_t)event);
}

/**
 * @brief Tarea periodica de control del robot.
 */
static void TaskControlRobot_ProcessEvent(RobotControlEvent_t event) {
    TaskControlRobot_ProcessEventAt(event, NULL);
}

static void TaskControlRobot_Run(void *parameters) {
    (void)parameters;

    TaskControlRobotEventMessage_t message;

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {

        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS));

        // Procesa todos los eventos pendientes.
        while (xQueueReceive(event_queue, &message, 0U) == pdTRUE) {
            if (message.event == ROBOT_EVENT_START_AUTO) {
                TaskControlRobot_ProcessEventAt(message.event, &message);
            } else {
                TaskControlRobot_ProcessEvent(message.event);
            }
        }
        /* Incluso con cola llena, una solicitud STOP ya anulo los motores y
         * debe completar la transicion a IDLE. No depende de encolarla de nuevo.
         */
        if (TaskControlRobot_IsStopPending()) TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);

        Navigation_Step();
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
    auto_request_epoch = 0U;
    auto_barrier_tick = mode_entry_tick;

    event_queue = xQueueCreate(
        TASK_CONTROL_QUEUE_LENGTH,
        sizeof(TaskControlRobotEventMessage_t)
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
        &control_task_handle
    );

    if (result != pdPASS) {
        vQueueDelete(event_queue);
        event_queue = NULL;
        control_task_handle = NULL;
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

    if (event == ROBOT_EVENT_START_AUTO) {
        return TaskControlRobot_RequestAutoAt(xTaskGetTickCount());
    }

    TaskControlRobotEventMessage_t message = { .event = event };
    if (event == ROBOT_EVENT_STOP || event == ROBOT_EVENT_SAFETY_RESET) {
        /* Barrera y encolado STOP/RESET forman una operacion de tarea;
         * IRQ activas, cola no bloqueante FUERA de la seccion critica. */
        vTaskSuspendAll();
        taskENTER_CRITICAL();
        RobotControlState_t requested_state = robot_state;
        TaskControlRobot_InvalidateAuto();
        if (event == ROBOT_EVENT_STOP || robot_state == ROBOT_STATE_AUTONOMOUS) {
            if (event == ROBOT_EVENT_STOP) stop_pending = true;
            TaskMotorControl_Stop();
        }
        taskEXIT_CRITICAL();
        bool queued = xQueueSend(event_queue, &message, 0U) == pdTRUE;
        /* Una solicitud STOP real, aun con cola llena; RESET no es STOP. */
        if (event == ROBOT_EVENT_STOP)
            (void)Log_EventPublish(300U, (uint32_t)requested_state, 0U, 0U);
        (void)xTaskResumeAll();
        return queued;
    }
    return (xQueueSend(event_queue, &message, 0U) == pdTRUE);
}

bool TaskControlRobot_RequestAutoAt(TickType_t issued_tick) {
    TaskControlRobotEventMessage_t message = {
        .event = ROBOT_EVENT_START_AUTO, .issued_tick = issued_tick
    };
    taskENTER_CRITICAL();
    message.auto_epoch = auto_request_epoch;
    bool allowed = event_queue != NULL && robot_state == ROBOT_STATE_IDLE &&
        !TaskSafety_IsStopActive() && !stop_pending &&
        TaskControlRobot_AutoRequestCurrent(&message);
    taskEXIT_CRITICAL();
    /* Un STOP/RESET entre admision y encolado invalida message.auto_epoch. */
    return allowed && xQueueSend(event_queue, &message, 0U) == pdTRUE;
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

bool TaskControlRobot_IsCurrentTask(void) {
    return control_task_handle != NULL && xTaskGetCurrentTaskHandle() == control_task_handle;
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
