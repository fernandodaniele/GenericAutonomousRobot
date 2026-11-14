/**
 * @file    task_datalogger.h
 * @brief   Tarea del datalogger: guarda una fila de datos cada 200 ms.
 *
 * Cada TASK_DATALOGGER_PERIOD_MS escribe con LOG_DATA una fila CSV con los
 * sensores y la navegacion (en GAR.LOG si hay SD, y por USB si esta abierto).
 * Una vez por segundo actualiza el estado de los modulos (SD, reloj y log)
 * que se manda al ESP32 en la linea SYS.
 *
 * Solo lee con funciones que no bloquean (TaskSensors_GetSnapshot y
 * TaskNav_GetStatus): no afecta a la navegacion. Es la tarea de menor
 * prioridad de la aplicacion.
 */

#ifndef TASK_DATALOGGER_H
#define TASK_DATALOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "mid_clock.h"
#include "mid_log.h"

#define TASK_DATALOGGER_PERIOD_MS       200u    // Una fila cada 200 ms (5 por segundo)
#define TASK_DATALOGGER_STATUS_MS       1000u   // Estado de modulos cada 1 s

/* Columnas de cada fila (se escribe una vez por arranque) */
#define TASK_DATALOGGER_HEADER \
    "t_ms,us_f,front_mm,us_l,left_mm,us_r,right_mm,ir_l,ir_r,cliff_l,cliff_r," \
    "gyro_dps,heading_mdeg,state,pwm_l,pwm_r,brake"

/** @brief  Estado de los modulos del datalogger, para la web. */
typedef struct {
    bool clock_ok;              // El DS1302 responde con una fecha valida
    ClockDateTime_t now;        // Ultima fecha leida (valida si clock_ok)
    LogStatus_t log;            // Estado de la SD y lineas guardadas
    uint32_t rows;              // Filas de datos generadas desde el arranque
} DataloggerStatus_t;

/**
 * @brief  Crea la tarea. Llamar despues de Log_Init y antes de osKernelStart.
 */
void TaskDatalogger_Create(void);

/**
 * @brief  Copia el ultimo estado de los modulos (no bloquea).
 */
void TaskDatalogger_GetStatus(DataloggerStatus_t *out);

#ifdef __cplusplus
}
#endif

#endif /* TASK_DATALOGGER_H */
