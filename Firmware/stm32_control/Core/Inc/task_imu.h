#ifndef TASK_IMU_H_
#define TASK_IMU_H_

#include <stdbool.h>
#include "mpu6050.h"

/**
 * @brief Crea la tarea periódica de adquisición de datos de la IMU.
 * @param hi2c Interfaz I2C utilizada por el MPU6050.
 */
void TaskIMU_Create(I2C_HandleTypeDef *hi2c);

/**
 * @brief Obtiene la última medición disponible de la IMU.
 * @param data Estructura donde se copiarán las mediciones.
 * @return true si existe una medición válida, false en caso contrario.
 * @note No bloquea la tarea que realiza la consulta.
 *       Las mediciones se comparten mediante una cola FreeRTOS.
 */
bool TaskIMU_GetLatest(MPU6050_t *data);

#endif /* TASK_IMU_H_ */