#ifndef TASK_SENSORS_H
#define TASK_SENSORS_H

#include <stdint.h>
#include "drv_motor.h"

/** Distancia por debajo de la cual se frenan los motores. */
#define SAFE_DISTANCE_MM         200U
/** Valor de distancia cuando no hay medición válida del HC-SR04. */
#define SENSOR_DISTANCE_INVALID  0xFFFFU

/**
 * @brief  Última lectura consolidada de los sensores.
 *
 * Se actualiza en la tarea de sensores (~20 ms). Copiar con TaskSensors_Get().
 */
typedef struct {
    uint8_t  imu_ok;               /**< 1 si el MPU6050 respondió en el init. */
    float    accel_g[3];           /**< Ax, Ay, Az en g. */
    float    gyro_dps[3];          /**< Gx, Gy, Gz en °/s. */
    float    temp_c;               /**< Temperatura del IMU en °C. */
    float    kalman_roll_deg;      /**< Roll filtrado (KalmanAngleX). */
    float    kalman_pitch_deg;     /**< Pitch filtrado (KalmanAngleY). */

    uint8_t  cliff_front;          /**< 1 si algún TCRT delantero no ve piso. */
    uint8_t  cliff_rear;           /**< 1 si algún TCRT trasero no ve piso. */
    uint8_t  cliff_any;            /**< cliff_front || cliff_rear. */

    uint16_t distance_mm;          /**< Distancia HC-SR04 en mm, o SENSOR_DISTANCE_INVALID. */

    uint8_t  safety_stop;          /**< 1 si la última evaluación frenó los motores. */
} SensorData_t;

/**
 * @brief  Crea la tarea de sensores + seguridad.
 *
 * Lee MPU6050 (I2C1), TCRT5000 (GPIO) y HC-SR04, y frena ambos motores ante
 * precipicio o distancia < SAFE_DISTANCE_MM.
 *
 * @param left,right  Motores ya inicializados (Motor_Init).
 */
void TaskSensors_Create(MotorHandle_t *left, MotorHandle_t *right);

/**
 * @brief  Copia atómica del último estado de sensores.
 * @param  out  Destino (no NULL).
 */
void TaskSensors_Get(SensorData_t *out);

#endif /* TASK_SENSORS_H */
