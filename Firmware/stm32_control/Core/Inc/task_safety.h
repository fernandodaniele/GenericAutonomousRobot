#ifndef TASK_SAFETY_H_
#define TASK_SAFETY_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Motivos que pueden provocar una parada de seguridad.
 */
typedef enum {
    SAFETY_FAULT_NONE         = 0U,
    SAFETY_FAULT_CLIFF        = (1U << 0),
    SAFETY_FAULT_OBSTACLE     = (1U << 1),
    SAFETY_FAULT_SENSOR       = (1U << 2),
    SAFETY_FAULT_EMERGENCY    = (1U << 3)
} SafetyFault_t;

/**
 * @brief Crea la tarea FreeRTOS de supervision de seguridad.
 */
void TaskSafety_Create(void);

/**
 * @brief Registra una condicion de riesgo.
 * @param fault Motivo de la parada.
 * @note Interfaz para llamadas desde tareas, no desde ISR.
 */
void TaskSafety_ReportFault(SafetyFault_t fault);

/**
 * @brief Indica si existe una parada de seguridad activa.
 * @return true si la seguridad mantiene bloqueado el movimiento.
 */
bool TaskSafety_IsStopActive(void);

/**
 * @brief Obtiene los motivos de parada registrados.
 * @return Mascara de fallas activas.
 */
uint32_t TaskSafety_GetFaults(void);

/**
 * @brief Solicita el rearme del sistema de seguridad.
 * @return true si se cumplen las condiciones para el rearme.
 */
bool TaskSafety_RequestReset(void);

#endif /* TASK_SAFETY_H_ */