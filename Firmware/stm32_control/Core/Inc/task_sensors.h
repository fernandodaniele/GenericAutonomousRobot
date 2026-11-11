/*
 * task_sensors.h
 *
 *  Created on: Nov 2, 2026
 *      Author: Grupo 2
 *
 *  Tarea de sensores: unica duena de ADC1 (TCRT5000), I2C3 (MPU6050) y de los
 *  tres HC-SR04. Publica una "foto" de sensores con marca de tiempo que el
 *  resto de las tareas lee sin bloquearse.
 */

#ifndef CORE_INC_TASK_SENSORS_H_
#define CORE_INC_TASK_SENSORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "drv_ultrasound.h"

#define TASK_SENSORS_PERIOD_MS   20

/* Diagnostico del MPU6050 (SensorSnapshot_t.imu_diag). Un valor >= 0 es el
 * WHO_AM_I que respondio: con imu_ok en false, es un modelo no reconocido. */
#define IMU_DIAG_LINE_LOW    (-3)   // SDA o SCL quedan en bajo: cables, alimentacion o pull-ups
#define IMU_DIAG_LOST        (-2)   // Respondia y dejo de responder
#define IMU_DIAG_NO_ACK      (-1)   // Nadie responde en 0x68 ni en 0x69
#define IMU_DIAG_DISABLED    (-4)   // SENSORS_USE_IMU = 0

/**
 * @brief Foto de sensores publicada por task_sensors.
 */
typedef struct {
    uint32_t timestamp_ms;                         // Tick de la ultima actualizacion
    uint32_t dist_mm[ULTRASOUND_COUNT];            // Ultima distancia por sensor
    UltrasoundStatus_t us_status[ULTRASOUND_COUNT];// Motivo de la lectura de este ciclo (solo el sensor de turno)
    UltrasoundStatus_t us_last_status[ULTRASOUND_COUNT]; // Motivo de la ultima lectura de cada sensor
    uint8_t front_no_echo_count;                   // Lecturas frontales sin eco seguidas
    uint16_t tcrt_left;                            // Valor ADC filtrado
    uint16_t tcrt_right;
    bool cliff_left;                               // TCRT izquierdo ve precipicio
    bool cliff_right;
    int16_t gyro_z_dps;                            // Velocidad angular Z en grados/s (+ = izquierda)
    int32_t heading_mdeg;                          // Rumbo integrado desde el encendido, milesimas de grado (+ = izquierda)
    bool imu_ok;                                   // MPU6050 inicializado
    int16_t imu_diag;                              // Ver IMU_DIAG_*; >= 0: WHO_AM_I leido
} SensorSnapshot_t;

/**
 * @brief Crea la tarea de sensores y la cola de la foto.
 *        Llamar antes de osKernelStart().
 */
void TaskSensors_Create(void);

/**
 * @brief Copia la ultima foto de sensores (no bloquea).
 * @return true si ya hay al menos una foto publicada.
 */
bool TaskSensors_GetSnapshot(SensorSnapshot_t* out);

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_TASK_SENSORS_H_ */
