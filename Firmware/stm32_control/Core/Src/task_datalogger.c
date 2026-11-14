/**
 * @file    task_datalogger.c
 * @brief   Tarea del datalogger: guarda una fila de datos cada 200 ms.
 */

#include "task_datalogger.h"
#include "task_sensors.h"
#include "task_nav.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static osThreadId_t datalogger_task_handle;
static const osThreadAttr_t datalogger_task_attributes = {
    .name = "datalogger",
    .stack_size = 2048,                         // Linea de 160 B + FatFs
    .priority = (osPriority_t) osPriorityLow,   // Por debajo de sensores, nav y USART
};

static DataloggerStatus_t datalogger_status;
static uint32_t datalogger_rows = 0u;

static void DataloggerTask(void *argument);

void TaskDatalogger_Create(void) {
    memset(&datalogger_status, 0, sizeof(datalogger_status));
    datalogger_task_handle = osThreadNew(DataloggerTask, NULL, &datalogger_task_attributes);
    if (datalogger_task_handle == NULL) {
        Error_Handler();
    }
}

void TaskDatalogger_GetStatus(DataloggerStatus_t *out) {
    if (out == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    *out = datalogger_status;
    taskEXIT_CRITICAL();
}

/* Letra del motivo de la lectura de ultrasonido (igual que la linea TEL) */
static char datalogger_range_code(UltrasoundStatus_t status) {
    switch (status) {
        case ULTRASOUND_STATUS_OK:        return 'O';
        case ULTRASOUND_STATUS_FAR:       return 'F';
        case ULTRASOUND_STATUS_TOO_CLOSE: return 'C';
        default:                          return 'N';
    }
}

/* Una fila CSV con los sensores y la navegacion */
static void datalogger_write_row(void) {
    SensorSnapshot_t snap;

    if (!TaskSensors_GetSnapshot(&snap)) {
        return;
    }
    NavStatus_t nav = TaskNav_GetStatus();

    LOG_DATA("%lu,%c,%lu,%c,%lu,%c,%lu,%u,%u,%d,%d,%d,%ld,%s,%d,%d,%d",
             (unsigned long)snap.timestamp_ms,
             datalogger_range_code(snap.us_last_status[ULTRASOUND_FRONT]),
             (unsigned long)snap.dist_mm[ULTRASOUND_FRONT],
             datalogger_range_code(snap.us_last_status[ULTRASOUND_LEFT]),
             (unsigned long)snap.dist_mm[ULTRASOUND_LEFT],
             datalogger_range_code(snap.us_last_status[ULTRASOUND_RIGHT]),
             (unsigned long)snap.dist_mm[ULTRASOUND_RIGHT],
             (unsigned)snap.tcrt_left, (unsigned)snap.tcrt_right,
             snap.cliff_left ? 1 : 0, snap.cliff_right ? 1 : 0,
             (int)snap.gyro_z_dps, (long)snap.heading_mdeg,
             Nav_StateName(nav.state),
             (int)nav.pwm_left, (int)nav.pwm_right, nav.brake ? 1 : 0);
    datalogger_rows++;
}

/* Estado de los modulos para la linea SYS */
static void datalogger_update_status(void) {
    DataloggerStatus_t status;

    memset(&status, 0, sizeof(status));
    status.clock_ok = Clock_Read(&status.now);
    Log_GetStatus(&status.log);
    status.rows = datalogger_rows;

    taskENTER_CRITICAL();
    datalogger_status = status;
    taskEXIT_CRITICAL();
}

static void DataloggerTask(void *argument) {
    (void) argument;
    const uint32_t rows_per_status = TASK_DATALOGGER_STATUS_MS / TASK_DATALOGGER_PERIOD_MS;
    uint32_t cycle = 0u;

    // Encabezado de columnas: una vez por arranque
    LOG_INFO("datalogger: arranque, una fila cada %u ms", (unsigned)TASK_DATALOGGER_PERIOD_MS);
    LOG_DATA("%s", TASK_DATALOGGER_HEADER);
    datalogger_update_status();

    uint32_t next_wake = osKernelGetTickCount();
    for (;;) {
        next_wake += pdMS_TO_TICKS(TASK_DATALOGGER_PERIOD_MS);

        datalogger_write_row();

        cycle++;
        if (cycle >= rows_per_status) {
            cycle = 0u;
            datalogger_update_status();
        }

        // Si escribir en la SD tardo mas que el periodo, no acumular atraso
        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(next_wake - now) <= 0) {
            next_wake = now;
        } else {
            (void) osDelayUntil(next_wake);
        }
    }
}
