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
#define TASK_SAFETY_SENSOR_TIMEOUT_MS 100U

// El sistema comienza con el movimiento bloqueado.
static volatile bool safety_stop_active = true;
static volatile uint32_t safety_faults = SAFETY_FAULT_NONE;
static uint32_t safety_fault_revision = 0U;

static TaskHandle_t safety_task_handle = NULL;

/**
 * @brief Supervisa el estado de seguridad del robot.
 */
static void TaskSafety_Run(void *parameters) {
    (void)parameters;

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t startup_tick = last_wake_time;

    for (;;) {
        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(TASK_SAFETY_PERIOD_MS));

        // Consulta la ultima medicion de sensores.
        SensoresDatos_t sensores;
        TickType_t ahora = xTaskGetTickCount();

        if (TaskSensores_GetData(&sensores) != pdTRUE) {

            // Permitir que TaskSensores publique su primer estado.
            if ((ahora - startup_tick) >
                pdMS_TO_TICKS(TASK_SAFETY_SENSOR_TIMEOUT_MS)) {

                TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
            }

        } else if ((ahora - sensores.timestamp) >
                pdMS_TO_TICKS(TASK_SAFETY_SENSOR_TIMEOUT_MS)) {

            // La informacion de los sensores dejo de actualizarse.
            TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);

        } else {

            // Deteccion de obstaculos.
            if ((sensores.ultrasonido_valido != 0U) &&
                (sensores.distancia_m <= TASK_SAFETY_OBSTACLE_DISTANCE_M)) {

                TaskSafety_ReportFault(SAFETY_FAULT_OBSTACLE);
            }

            // Si el movimiento esta habilitado y se pierde la medicion
            // ultrasonica, detener el robot por falta de informacion confiable.
            if (!TaskSafety_IsStopActive() &&
                sensores.ultrasonido_valido == 0U) {

                TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
            }

            // Deteccion de caidas mediante los cuatro TCRT5000.
            if (sensores.hay_caida != 0U) {

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
    safety_fault_revision++;
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

    // No aceptar datos desactualizados.
    TickType_t ahora = xTaskGetTickCount();

    if ((ahora - sensores.timestamp) >
        pdMS_TO_TICKS(TASK_SAFETY_SENSOR_TIMEOUT_MS)) {
        return false;
    }

    // Los cuatro sensores infrarrojos deben detectar superficie.
    if (sensores.hay_caida != 0U) {
        return false;
    }

    // Se requiere una medicion ultrasonica valida
    // y una distancia superior al umbral de seguridad.
    if ((sensores.ultrasonido_valido == 0U) ||
        (sensores.distancia_m <= TASK_SAFETY_OBSTACLE_DISTANCE_M)) {
        return false;
    }

    taskENTER_CRITICAL();

    // No borrar fallas registradas durante la verificacion.
    // Tampoco permitir rearme con emergencia registrada.
    if ((safety_fault_revision != revision_inicial) ||
        ((safety_faults & SAFETY_FAULT_EMERGENCY) != 0U)) {

        taskEXIT_CRITICAL();
        return false;
    }

    // Todas las condiciones verificadas permiten liberar el bloqueo.
    safety_faults = SAFETY_FAULT_NONE;
    safety_stop_active = false;

    taskEXIT_CRITICAL();

    return true;
}
