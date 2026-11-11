/*
 * task_sensors.c
 *
 *  Created on: Nov 2, 2026
 *      Author: Grupo 2
 *
 *  Lee un HC-SR04 por turno (cada 60 ms) priorizando el frente mediante intercalado,
 *  y cada 20 ms los dos TCRT5000 y el giro Z del MPU6050 en I2C3. Publica la foto en
 *  una cola de un elemento (xQueueOverwrite / xQueuePeek): nadie espera a nadie.
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

/* MPU6050: giro Z para mantener el rumbo. 0 = no se inicializa ni se lee. */
#ifndef SENSORS_USE_IMU
#define SENSORS_USE_IMU          1
#endif
/* Sentido del giro Z: con +1, girar el robot a la izquierda (antihorario visto
 * desde arriba) tiene que dar valores positivos en el panel. Si da negativos,
 * el modulo esta montado al reves: poner -1. */
#ifndef SENSORS_GYRO_Z_SIGN
#define SENSORS_GYRO_Z_SIGN      1
#endif
#define SENSORS_GYRO_LSB_PER_DPS 131.0f  // Escala +/- 250 grados/s
#define SENSORS_GYRO_CAL_SAMPLES 100     // 1 s quieto al encender para medir el offset
#define SENSORS_IMU_FAIL_MAX     5       // Lecturas I2C fallidas seguidas: IMU fuera

/* TCRT5000 habilitados */
#ifndef SENSORS_USE_TCRT_LEFT
#define SENSORS_USE_TCRT_LEFT    1
#endif
#ifndef SENSORS_USE_TCRT_RIGHT
#define SENSORS_USE_TCRT_RIGHT   1
#endif

#define SENSORS_IMU_I2C          (&hi2c3)
#define SENSORS_IMU_INIT_RETRIES 3

/* Configuración del filtro de promedio móvil para el frente */
#define FRONT_MAX_SAMPLES        3

/* Un disparo de ultrasonido cada 3 ciclos de 20 ms: el HC-SR04 necesita unos
 * 60 ms entre mediciones para que el eco de un sensor no lo lea el siguiente. */
#define SENSORS_CYCLES_PER_PING  3

static osThreadId_t sensors_task_handle;
static const osThreadAttr_t sensors_task_attributes = {
    .name = "sensors",
    .stack_size = 2048,
    .priority = (osPriority_t) osPriorityHigh,
};

static QueueHandle_t snapshot_queue;

/* SECUENCIA DE PRIORIDAD: El frente se lee una vez por medio, intercalado con los demás */
static const UltrasoundIndex_t us_sequence[] = {
    ULTRASOUND_FRONT, ULTRASOUND_LEFT,
    ULTRASOUND_FRONT, ULTRASOUND_RIGHT,
};
#define SEQUENCE_LENGTH (sizeof(us_sequence) / sizeof(us_sequence[0]))

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

/* Offset del giro Z (cuentas crudas): promedio con el robot quieto */
static float gyro_calibrate(void)
{
    int32_t sum = 0;
    uint16_t count = 0;
    for (uint16_t i = 0; i < SENSORS_GYRO_CAL_SAMPLES; i++) {
        int16_t raw;
        if (MPU6050_Read_GyroZ_Raw(SENSORS_IMU_I2C, &raw) == 0) {
            sum += raw;
            count++;
        }
        osDelay(10);
    }
    return (count > 0u) ? (float)sum / (float)count : 0.0f;
}
#endif

static void SensorsTask(void* argument)
{
    (void) argument;

    SensorSnapshot_t snap;
    memset(&snap, 0, sizeof(snap));

    // Inicializar estados por defecto
    for (uint8_t i = 0; i < ULTRASOUND_COUNT; i++) {
        snap.us_status[i] = ULTRASOUND_STATUS_NO_ECHO;
        snap.us_last_status[i] = ULTRASOUND_STATUS_NO_ECHO;
    }

    drv_tcrt5000_init();
#if SENSORS_USE_IMU
    float gyro_bias = 0.0f;
    float heading_deg = 0.0f;
    uint8_t imu_fails = 0;
    snap.imu_ok = imu_init();
    if (snap.imu_ok) {
        gyro_bias = gyro_calibrate();
    }
    uint32_t last_gyro_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
#else
    snap.imu_ok = false;
#endif

    /* Variables de control para el orden secuencial prioritario */
    uint8_t seq_idx = 0;
    uint8_t ping_cycle = 0;

    /* Variables locales para el filtro de promedio móvil del frente */
    uint32_t front_samples[FRONT_MAX_SAMPLES] = {0};
    uint8_t front_sample_idx = 0;
    uint8_t front_samples_count = 0;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        /* 1. OBLIGATORIO: Limpiamos los estados de TODOS los ultrasonidos en este ciclo.
         * Así, el mapeador sabrá que el único sensor con información NUEVA en este milisegundo
         * es el que marque "ULTRASOUND_STATUS_OK" o "NO_ECHO". Los demás quedan invalidados. */
        for (uint8_t i = 0; i < ULTRASOUND_COUNT; i++) {
            snap.us_status[i] = ULTRASOUND_STATUS_NO_ECHO;
        }

        if (ping_cycle == 0u) {
            /* 2. Determinar a quién le toca según nuestra secuencia de prioridad */
            UltrasoundIndex_t turn = us_sequence[seq_idx];

            /* 3. Realizar la lectura física del sensor de este turno */
            uint32_t mm = 0;
            UltrasoundStatus_t st = Ultrasound_ReadStatusIndex(turn, &mm);

            snap.us_status[turn] = st;
            snap.us_last_status[turn] = st;
            snap.dist_mm[turn] = mm;

            /* 4. Lógica especial con Promedio Móvil si es el turno del SENSOR FRONTAL */
            if (turn == ULTRASOUND_FRONT) {
                if (st == ULTRASOUND_STATUS_NO_ECHO) {
                    if (snap.front_no_echo_count < UINT8_MAX) {
                        snap.front_no_echo_count++;
                    }
                } else {
                    snap.front_no_echo_count = 0;
                }

                if (st == ULTRASOUND_STATUS_OK) {
                    // Insertar nueva muestra en el buffer circular
                    front_samples[front_sample_idx] = mm;
                    front_sample_idx = (front_sample_idx + 1) % FRONT_MAX_SAMPLES;
                    if (front_samples_count < FRONT_MAX_SAMPLES) {
                        front_samples_count++;
                    }

                    // Calcular el promedio acumulado
                    uint32_t sum = 0;
                    for (uint8_t s = 0; s < front_samples_count; s++) {
                        sum += front_samples[s];
                    }

                    // Sobrescribimos el valor instantáneo por la media filtrada
                    snap.dist_mm[ULTRASOUND_FRONT] = sum / front_samples_count;
                } else {
                    // Sin distancia valida (libre: mm = 0): no debe entrar al promedio
                    front_samples_count = 0;
                    front_sample_idx = 0;
                }
            }

            /* 5. Avanzar el índice de la secuencia de prioridad de forma circular */
            seq_idx = (uint8_t)((seq_idx + 1) % SEQUENCE_LENGTH);
        }
        ping_cycle = (uint8_t)((ping_cycle + 1u) % SENSORS_CYCLES_PER_PING);

        /* 6. Leer los TCRT5000 (Filtro interno del driver) */
        drv_tcrt5000_update();
        tcrt_reading_t l = drv_tcrt5000_get_left();
        tcrt_reading_t r = drv_tcrt5000_get_right();
        snap.tcrt_left = l.value;
        snap.tcrt_right = r.value;
        snap.cliff_left = SENSORS_USE_TCRT_LEFT && (l.state == TCRT_STATE_CLIFF);
        snap.cliff_right = SENSORS_USE_TCRT_RIGHT && (r.state == TCRT_STATE_CLIFF);

        /* 7. Leer el Giroscopio MPU6050 */
#if SENSORS_USE_IMU
        if (snap.imu_ok) {
            int16_t raw;
            const uint32_t now_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (MPU6050_Read_GyroZ_Raw(SENSORS_IMU_I2C, &raw) == 0) {
                const float dps = (float)SENSORS_GYRO_Z_SIGN
                                * ((float)raw - gyro_bias) / SENSORS_GYRO_LSB_PER_DPS;
                heading_deg += dps * (float)(now_ms - last_gyro_ms) / 1000.0f;
                last_gyro_ms = now_ms;
                imu_fails = 0;
                snap.gyro_z_dps = (int16_t)(dps + ((dps >= 0.0f) ? 0.5f : -0.5f));
                snap.heading_mdeg = (int32_t)(heading_deg * 1000.0f);
            } else if (++imu_fails >= SENSORS_IMU_FAIL_MAX) {
                snap.imu_ok = false;   // Sin IMU la navegacion sigue sin corregir el rumbo
                snap.gyro_z_dps = 0;
            }
        }
#endif

        /* 8. Estampa de tiempo y publicación en la cola */
        snap.timestamp_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
        xQueueOverwrite(snapshot_queue, &snap);

        /* 9. Espera determinista de FreeRTOS para liberar la CPU */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_SENSORS_PERIOD_MS));
    }
}
