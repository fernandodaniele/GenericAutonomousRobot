#include "task_safety.h"
#include "task_control_robot.h"
#include "task_sensores.h"

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

static TaskHandle_t safety_task_handle = NULL;

/**
 * @brief Supervisa el estado de seguridad del robot.
 */
static void TaskSafety_Run(void *parameters) {
    (void)parameters;

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(TASK_SAFETY_PERIOD_MS));

        // Consulta la ultima medicion de sensores.
        SensoresDatos_t sensores;

        if (TaskSensores_GetData(&sensores) == pdTRUE) {

            // Registra un obstaculo si la distancia es valida
            // y se encuentra dentro del umbral de seguridad.
            if ((sensores.ultrasonido_valido != 0U) &&
                (sensores.distancia_m <= TASK_SAFETY_OBSTACLE_DISTANCE_M)) {

                TaskSafety_ReportFault(SAFETY_FAULT_OBSTACLE);
            }

            // Prueba inicial: supervisa solo el TCRT5000 delantero izquierdo.
            // Pendiente ampliar a los cuatro sensores cuando esten conectados.
            if (sensores.ir.del_izq == TCRT_NO_DETECTA) {

                TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);
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

    safety_faults |= (uint32_t)fault;
    safety_stop_active = true;

    taskEXIT_CRITICAL();
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
    // Pendiente: verificar sensores y condiciones de seguridad.
    // Hasta entonces, el rearme permanece deshabilitado.
    return false;
}