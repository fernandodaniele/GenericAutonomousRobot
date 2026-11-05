/*
 * task_sensors.c
 *
 *  Created on: Nov 2, 2026
 *      Author: Grupo 2
 *
 *  Lee un HC-SR04 por turno (cada 30 ms), los dos TCRT5000 y el giro Z del
 *  MPU6050 en I2C3, y publica la foto en una cola de un elemento
 *  (xQueueOverwrite / xQueuePeek): nadie espera a nadie.
 */

#include "task_sensors.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "i2c.h"
#include "drv_TCRT5000.h"
#include "drv_gyro.h"
#include <string.h>

/* El MPU6050 no se usa por ahora: 0 = no se inicializa ni se lee.
 * Poner en 1 para volver a usarlo (I2C3: PA8 SCL / PC9 SDA). */
#ifndef SENSORS_USE_IMU
#define SENSORS_USE_IMU          0
#endif
/* TCRT5000 habilitados: 0 = ese sensor nunca informa precipicio.
 * El derecho (PC2) esta anulado porque falla; poner en 1 cuando se repare. */
#ifndef SENSORS_USE_TCRT_LEFT
#define SENSORS_USE_TCRT_LEFT    1
#endif
#ifndef SENSORS_USE_TCRT_RIGHT
#define SENSORS_USE_TCRT_RIGHT   0
#endif
#define SENSORS_IMU_I2C          (&hi2c3)
#define SENSORS_IMU_INIT_RETRIES 3

static osThreadId_t sensors_task_handle;
static const osThreadAttr_t sensors_task_attributes = {
    .name = "sensors",
    .stack_size = 2048,
    .priority = (osPriority_t) osPriorityHigh,
};

static QueueHandle_t snapshot_queue;

static void SensorsTask(void* argument);

void TaskSensors_Create(void)
{
    snapshot_queue = xQueueCreate(1, sizeof(SensorSnapshot_t));
    if (snapshot_queue == NULL) {
        Error_Handler();
    }

    sensors_task_handle = osThreadNew(SensorsTask, NULL, &sensors_task_attributes);
    if (sensors_task_handle == NULL) {
        Error_Handler();
    }
}

bool TaskSensors_GetSnapshot(SensorSnapshot_t* out)
{
    if (!out || !snapshot_queue) {
        return false;
    }
    return xQueuePeek(snapshot_queue, out, 0) == pdTRUE;
}

#if SENSORS_USE_IMU
static bool imu_init(void)
{
    for (uint8_t i = 0; i < SENSORS_IMU_INIT_RETRIES; i++) {
        if (MPU6050_Init(SENSORS_IMU_I2C) == 0) {
            return true;
        }
        osDelay(50);
    }
    return false;
}
#endif

static void SensorsTask(void* argument)
{
    (void) argument;

    SensorSnapshot_t snap;
    memset(&snap, 0, sizeof(snap));
    for (uint8_t i = 0; i < ULTRASOUND_COUNT; i++) {
        snap.us_status[i] = ULTRASOUND_STATUS_NO_ECHO;
    }

#if SENSORS_USE_IMU
    MPU6050_t imu;
    memset(&imu, 0, sizeof(imu));
#endif

    drv_tcrt5000_init();
#if SENSORS_USE_IMU
    snap.imu_ok = imu_init();
#else
    snap.imu_ok = false;
#endif

    UltrasoundIndex_t turn = ULTRASOUND_FRONT;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        /* Un ultrasonido por turno para que los ecos no se crucen */
        uint32_t mm = 0;
        UltrasoundStatus_t st = Ultrasound_ReadStatusIndex(turn, &mm);
        snap.us_status[turn] = st;
        snap.dist_mm[turn] = mm;
        if (turn == ULTRASOUND_FRONT) {
            if (st == ULTRASOUND_STATUS_NO_ECHO) {
                if (snap.front_no_echo_count < UINT8_MAX) {
                    snap.front_no_echo_count++;
                }
            } else {
                snap.front_no_echo_count = 0;
            }
        }
        turn = (UltrasoundIndex_t)((turn + 1) % ULTRASOUND_COUNT);

        /* TCRT5000: filtro de promedio movil del driver */
        drv_tcrt5000_update();
        tcrt_reading_t l = drv_tcrt5000_get_left();
        tcrt_reading_t r = drv_tcrt5000_get_right();
        snap.tcrt_left = l.value;
        snap.tcrt_right = r.value;
        snap.cliff_left = SENSORS_USE_TCRT_LEFT && (l.state == TCRT_STATE_CLIFF);
        snap.cliff_right = SENSORS_USE_TCRT_RIGHT && (r.state == TCRT_STATE_CLIFF);

        /* MPU6050: solo el giro, no hace falta el angulo */
#if SENSORS_USE_IMU
        if (snap.imu_ok) {
            MPU6050_Read_Gyro(SENSORS_IMU_I2C, &imu);
            snap.gyro_z_dps = (int16_t)imu.Gz;
        }
#endif

        snap.timestamp_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
        xQueueOverwrite(snapshot_queue, &snap);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_SENSORS_PERIOD_MS));
    }
}
