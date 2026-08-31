/**
 * @file    task_sensors.c
 * @brief   Tarea de sensores (MPU6050 + TCRT5000 + HC-SR04) y seguridad.
 *
 * Un solo hilo, ~20 ms de período:
 *   - lee IMU por I2C1 (si el MPU6050 respondió en el init),
 *   - lee los TCRT5000 (GPIO) para detección de precipicio,
 *   - maneja el HC-SR04 con una máquina de estados no bloqueante,
 *   - frena ambos motores ante precipicio u obstáculo cercano (consumidor),
 *   - publica un snapshot protegido por mutex y loguea 1×/s.
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "task_sensors.h"
#include "mpu6050.h"
#include "drv_TCRT5000.h"
#include "drv_ultrasound.h"
#include "drv_motor.h"
#include "i2c.h"
#include "mid_log.h"

#define SENSOR_PERIOD_MS   20U
#define US_INTERVAL_MS     60U     /* entre disparos del HC-SR04 */
#define LOG_INTERVAL_MS    1000U   /* throttle del LOG_INFO */

static SensorData_t      s_data;
static SemaphoreHandle_t s_mtx;
static MotorHandle_t    *s_ml;
static MotorHandle_t    *s_mr;

void TaskSensors_Get(SensorData_t *out)
{
    if (out == NULL)
    {
        return;
    }
    if (s_mtx != NULL)
    {
        (void)xSemaphoreTake(s_mtx, portMAX_DELAY);
    }
    *out = s_data;
    if (s_mtx != NULL)
    {
        (void)xSemaphoreGive(s_mtx);
    }
}

static void vTaskSensors(void *pv)
{
    (void)pv;

    MPU6050_t  imu;
    /* Presencia real en el bus antes de creer en MPU6050_Init (que no chequea
     * el retorno de HAL_I2C y puede dar falso positivo si el chip no está). */
    uint8_t    imu_ok  = (HAL_I2C_IsDeviceReady(&hi2c1, 0xD0U, 3U, 20U) == HAL_OK) ? 1U : 0U;
    uint8_t    us_busy = 0U;
    TickType_t us_t0   = 0U;
    TickType_t log_t0  = 0U;
    uint16_t   dist_mm = SENSOR_DISTANCE_INVALID;

    if (imu_ok)
    {
        (void)MPU6050_Init(&hi2c1);
        LOG_INFO("MPU6050 OK");
    }
    else
    {
        LOG_WARN("MPU6050 no responde (I2C1 PB6/PB9); sigo sin IMU");
    }

    TCRT_Init();

    HCSR04_Trigger();
    us_busy = 1U;
    us_t0   = xTaskGetTickCount();

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();

        /* --- IMU --- */
        if (imu_ok)
        {
            MPU6050_Read_All(&hi2c1, &imu);
        }

        /* --- Precipicio (TCRT5000) --- */
        uint8_t cf = TCRT_HayCaidaAdelante();
        uint8_t cr = TCRT_HayCaidaAtras();
        uint8_t ca = (cf || cr) ? 1U : 0U;

        /* --- HC-SR04: máquina de estados no bloqueante --- */
        if (us_busy)
        {
            if (HCSR04_IsReady())
            {
                float d_m = HCSR04_GetDistance();
                dist_mm = (d_m > 0.01f && d_m < 4.0f)
                        ? (uint16_t)(d_m * 1000.0f)
                        : SENSOR_DISTANCE_INVALID;
                us_busy = 0U;
            }
            else if ((now - us_t0) > pdMS_TO_TICKS(US_INTERVAL_MS))
            {
                dist_mm = SENSOR_DISTANCE_INVALID;   /* sin eco */
                us_busy = 0U;
            }
        }
        if (!us_busy && (now - us_t0) >= pdMS_TO_TICKS(US_INTERVAL_MS))
        {
            HCSR04_Trigger();
            us_busy = 1U;
            us_t0   = now;
        }

        /* --- Seguridad: frenar ante precipicio u obstáculo cercano --- */
        uint8_t stop = (ca != 0U) ||
                       ((dist_mm != SENSOR_DISTANCE_INVALID) && (dist_mm < SAFE_DISTANCE_MM));
        if (stop)
        {
            Motor_Stop(s_ml);
            Motor_Stop(s_mr);
        }

        /* --- Publicar snapshot --- */
        if (s_mtx != NULL)
        {
            (void)xSemaphoreTake(s_mtx, portMAX_DELAY);
        }
        s_data.imu_ok = imu_ok;
        if (imu_ok)
        {
            s_data.accel_g[0]      = (float)imu.Ax;
            s_data.accel_g[1]      = (float)imu.Ay;
            s_data.accel_g[2]      = (float)imu.Az;
            s_data.gyro_dps[0]     = (float)imu.Gx;
            s_data.gyro_dps[1]     = (float)imu.Gy;
            s_data.gyro_dps[2]     = (float)imu.Gz;
            s_data.temp_c          = imu.Temperature;
            s_data.kalman_roll_deg  = (float)imu.KalmanAngleX;
            s_data.kalman_pitch_deg = (float)imu.KalmanAngleY;
        }
        s_data.cliff_front = cf;
        s_data.cliff_rear  = cr;
        s_data.cliff_any   = ca;
        s_data.distance_mm = dist_mm;
        s_data.safety_stop = stop;
        if (s_mtx != NULL)
        {
            (void)xSemaphoreGive(s_mtx);
        }

        /* --- Log 1x/s --- */
        if ((now - log_t0) >= pdMS_TO_TICKS(LOG_INTERVAL_MS))
        {
            log_t0 = now;
            if (imu_ok)
            {
                LOG_INFO("roll=%.1f pitch=%.1f cliff=%u dist=%umm%s",
                         (double)s_data.kalman_roll_deg,
                         (double)s_data.kalman_pitch_deg,
                         (unsigned)ca, (unsigned)dist_mm,
                         stop ? " STOP" : "");
            }
            else
            {
                LOG_INFO("cliff=%u dist=%umm%s",
                         (unsigned)ca, (unsigned)dist_mm, stop ? " STOP" : "");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

void TaskSensors_Create(MotorHandle_t *left, MotorHandle_t *right)
{
    s_ml = left;
    s_mr = right;
    if (s_mtx == NULL)
    {
        s_mtx = xSemaphoreCreateMutex();
    }

    /* 768 words: MPU6050 usa double + Kalman + trig, y LOG_INFO con %f
     * (formateo float de newlib) consume bastante pila. */
    if (xTaskCreate(vTaskSensors, "sensors", 768, NULL,
                    tskIDLE_PRIORITY + 2, NULL) != pdPASS)
    {
        Error_Handler();
    }
}
