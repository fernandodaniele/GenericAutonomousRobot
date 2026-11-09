#include "task_safety.h"
#include "task_control_robot.h"
#include "task_sensores.h"
#include "task_motor_control.h"
#include "mid_log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#define TASK_SAFETY_PERIOD_MS 20U
#define TASK_SAFETY_STACK_SIZE 256U
#define TASK_SAFETY_PRIORITY (tskIDLE_PRIORITY + 6U)
#define TASK_SAFETY_OBSTACLE_DISTANCE_M 0.20f

// El sistema comienza con el movimiento bloqueado.
static volatile bool safety_stop_active = true;
static volatile uint32_t safety_faults = SAFETY_FAULT_NONE;
static uint32_t safety_fault_revision = 0U;
static TickType_t safety_fault_tick = 0U;
static bool safety_has_fault = false;

static TaskHandle_t safety_task_handle = NULL;

/* Evaluacion unica para supervision y rearme. No accede a colas ni espera. */
static uint32_t TaskSafety_SnapshotFaults(const SensoresDatos_t *data,
                                         TickType_t now) {
    uint32_t faults = SAFETY_FAULT_NONE;
    if ((TickType_t)(now - data->timestamp) >
        pdMS_TO_TICKS(TASK_SAFETY_SENSOR_TIMEOUT_MS)) {
        return SAFETY_FAULT_SENSOR;
    }
    if ((data->ultrasonido_valido == 0U) ||
        (data->ultrasonido_trasero_valido == 0U) ||
        ((TickType_t)(now - data->timestamp_ultrasonido_frontal) >
         pdMS_TO_TICKS(TASK_SAFETY_ULTRASOUND_MAX_AGE_MS)) ||
        ((TickType_t)(now - data->timestamp_ultrasonido_trasero) >
         pdMS_TO_TICKS(TASK_SAFETY_ULTRASOUND_MAX_AGE_MS))) {
        faults |= SAFETY_FAULT_SENSOR;
    }
    // La negacion de > tambien rechaza NaN, sin aprobar una distancia insegura.
    if (((data->ultrasonido_valido != 0U) &&
         !(data->distancia_m > TASK_SAFETY_OBSTACLE_DISTANCE_M)) ||
        ((data->ultrasonido_trasero_valido != 0U) &&
         !(data->distancia_trasera_m > TASK_SAFETY_OBSTACLE_DISTANCE_M))) {
        faults |= SAFETY_FAULT_OBSTACLE;
    }
    if (data->hay_caida != 0U) {
        faults |= SAFETY_FAULT_CLIFF;
    }
    return faults;
}

uint32_t TaskSafety_EvaluateSnapshot(const SensoresDatos_t *data, TickType_t now) {
    return data != NULL ? TaskSafety_SnapshotFaults(data, now) : SAFETY_FAULT_SENSOR;
}

/**
 * @brief Supervisa el estado de seguridad del robot.
 */
static void TaskSafety_Run(void *parameters) {
    (void)parameters;

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t startup_tick = last_wake_time;

    /* Una vez desde tarea, despues de Log_Init y antes de supervisar sensores. */
    (void)Log_EventPublish(100U, (uint32_t)TaskControlRobot_GetState(),
                           TaskSafety_GetFaults(), TaskSafety_IsStopActive() ? 1U : 0U);

    for (;;) {
        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(TASK_SAFETY_PERIOD_MS));

        // Consulta la ultima medicion de sensores.
        SensoresDatos_t sensores;
        BaseType_t available = TaskSensores_GetData(&sensores);
        TickType_t ahora = xTaskGetTickCount();

        if (available != pdTRUE) {

            // Permitir que TaskSensores publique su primer estado.
            if ((ahora - startup_tick) >
                pdMS_TO_TICKS(TASK_SAFETY_SENSOR_TIMEOUT_MS)) {

                TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
            }

        } else {
            uint32_t faults = TaskSafety_SnapshotFaults(&sensores, ahora);
            if (faults != SAFETY_FAULT_NONE) {
                TaskSafety_ReportFault((SafetyFault_t)faults);
            }
        }

        // Solicita el estado de parada mientras exista bloqueo.
        // Si la cola esta llena, vuelve a intentarlo.
        if (TaskSafety_IsStopActive() &&
            TaskControlRobot_GetState() != ROBOT_STATE_SAFETY_STOP) {

            (void)TaskControlRobot_SendEvent(
                ROBOT_EVENT_SAFETY_STOP
            );
        }
    }
}

/**
 * @brief Crea la tarea FreeRTOS de seguridad.
 */
void TaskSafety_Create(void) {
    if (safety_task_handle != NULL) {
        Error_Handler();
        return;
    }

    safety_faults = SAFETY_FAULT_NONE;
    safety_stop_active = true;
    safety_fault_revision = 0U;
    safety_fault_tick = xTaskGetTickCount();
    safety_has_fault = false;

    BaseType_t result = xTaskCreate(
        TaskSafety_Run,
        "Safety",
        TASK_SAFETY_STACK_SIZE,
        NULL,
        TASK_SAFETY_PRIORITY,
        &safety_task_handle
    );

    if (result != pdPASS) {
        safety_task_handle = NULL;
        Error_Handler();
    }
}

/**
 * @brief Registra una o varias condiciones peligrosas.
 */
void TaskSafety_ReportFault(SafetyFault_t fault) {
    if (fault == SAFETY_FAULT_NONE) {
        return;
    }

    taskENTER_CRITICAL();

    uint32_t new_faults = (uint32_t)fault & ~safety_faults;
    safety_faults |= (uint32_t)fault;
    uint32_t latched_faults = safety_faults;
    safety_fault_revision++;
    safety_fault_tick = xTaskGetTickCount();
    safety_has_fault = true;
    safety_stop_active = true;
    // Misma transaccion que el bloqueo: invalida consignas y pone ambos PWM a 0.
    TaskMotorControl_Stop();

    taskEXIT_CRITICAL();

    /* Observar solo bits nuevos; conservar todas las notificaciones de Safety.
     * Publicacion fuera de la seccion critica, sin reintentar si no hay lugar. */
    if ((new_faults & SAFETY_FAULT_CLIFF) != 0U)
        (void)Log_EventPublish(201U, SAFETY_FAULT_CLIFF, latched_faults, 0U);
    if ((new_faults & SAFETY_FAULT_OBSTACLE) != 0U)
        (void)Log_EventPublish(202U, SAFETY_FAULT_OBSTACLE, latched_faults, 0U);
    if ((new_faults & SAFETY_FAULT_SENSOR) != 0U)
        (void)Log_EventPublish(203U, SAFETY_FAULT_SENSOR, latched_faults, 0U);
}

/**
 * @brief Consulta si el movimiento esta bloqueado.
 */
bool TaskSafety_IsStopActive(void) {
    bool active;

    taskENTER_CRITICAL();
    active = safety_stop_active;
    taskEXIT_CRITICAL();

    return active;
}

/**
 * @brief Obtiene la mascara de fallas registradas.
 */
uint32_t TaskSafety_GetFaults(void) {
    uint32_t faults;

    taskENTER_CRITICAL();
    faults = safety_faults;
    taskEXIT_CRITICAL();

    return faults;
}

/**
 * @brief Solicita el rearme del sistema.
 */
bool TaskSafety_RequestReset(void) {

    SensoresDatos_t sensores;
    uint32_t revision_inicial;

    // Registrar la version de las fallas antes de verificar sensores.
    taskENTER_CRITICAL();
    revision_inicial = safety_fault_revision;
    taskEXIT_CRITICAL();

    // Debe existir informacion publicada por TaskSensores.
    if (TaskSensores_GetData(&sensores) != pdTRUE) {
        return false;
    }

    if (TaskSafety_SnapshotFaults(&sensores, xTaskGetTickCount()) !=
        SAFETY_FAULT_NONE) {
        return false;
    }

    bool reset = false;
    uint32_t previous_faults = SAFETY_FAULT_NONE;
    /* Solo TaskSensores publica, desde tarea. Suspender el scheduler impide
     * otra publicacion entre esta segunda copia y la decision. Las IRQ siguen
     * activas; peek es no bloqueante y queda FUERA de la seccion critica.
     */
    vTaskSuspendAll();
    if (TaskSensores_GetData(&sensores) == pdTRUE) {
        taskENTER_CRITICAL();
        TickType_t now = xTaskGetTickCount();
        TickType_t fault_age = (TickType_t)(now - safety_fault_tick);
        // Igual tick es ambiguo: se rechaza, incluso si la captura fue posterior.
        bool new_captures = !safety_has_fault ||
            (((TickType_t)(now - sensores.timestamp_ultrasonido_frontal) < fault_age) &&
             ((TickType_t)(now - sensores.timestamp_ultrasonido_trasero) < fault_age));

        if ((safety_fault_revision == revision_inicial) &&
            ((safety_faults & SAFETY_FAULT_EMERGENCY) == 0U) &&
            (TaskSafety_SnapshotFaults(&sensores, now) == SAFETY_FAULT_NONE) &&
            new_captures) {
            TaskMotorControl_Stop();
            previous_faults = safety_faults;
            safety_faults = SAFETY_FAULT_NONE;
            safety_stop_active = false;
            reset = true;
        }
        taskEXIT_CRITICAL();
    }
    (void)xTaskResumeAll();
    bool completed = reset && !TaskSafety_IsStopActive();
    if (completed) (void)Log_EventPublish(210U, previous_faults, 0U, 0U);
    return completed;
}
