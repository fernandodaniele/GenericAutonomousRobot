#ifndef TASK_IMU_H_
#define TASK_IMU_H_

#include <stdbool.h>
#include "mpu6050.h"
#include "FreeRTOS.h"

typedef enum {
    IMU_INITIALIZING = 0,
    IMU_READY = 1,
    IMU_VALID = 2,
    IMU_INIT_ERROR = 3,
    IMU_READ_ERROR = 4
} TaskIMUState_t;

/* Calibration acceptance limits, NOT offsets or Safety/motor parameters.
 * Initial conservative lab criteria; tune only after observing real noise.
 * 250 acquisitions at 20 ms span 4980 ms between first and last samples. */
#define IMU_CAL_SAMPLES 250U
#define IMU_CAL_MIN_SPAN_MS 4900U
#define IMU_CAL_MAX_SPAN_MS 6000U
#define IMU_CAL_MAX_GAP_MS 40U
#define IMU_CAL_GYRO_ABS_RAW 655       /* 5 deg/s at 131 LSB/(deg/s). */
#define IMU_CAL_GYRO_RANGE_RAW 131     /* 1 deg/s peak-to-peak per axis. */
#define IMU_CAL_GYRO_STD_RAW 26        /* About 0.20 deg/s per-axis stddev. */
#define IMU_CAL_ACCEL_RANGE_RAW 819    /* About 0.05 g peak-to-peak per axis. */
#define IMU_CAL_ACCEL_NORM_MIN2 217432719LL /* (0.9 * 16384)^2, rounded. */
#define IMU_CAL_ACCEL_NORM_MAX2 324806902LL /* (1.1 * 16384)^2, rounded. */

enum { IMU_CAL_PENDING=0, IMU_CAL_RUNNING=1, IMU_CAL_VALID=2, IMU_CAL_FAILED=3 };
enum { IMU_CAL_NONE=0, IMU_CAL_MOTION=1, IMU_CAL_I2C=2, IMU_CAL_TIMING=3,
       IMU_CAL_MODE=4, IMU_CAL_INSUFFICIENT=5 };
typedef struct {
    uint32_t request;
    TickType_t completed_tick;
    int32_t sum[3]; /* Accepted raw sums / 250 = measured fractional bias. */
    uint16_t samples;
    uint8_t state, reason;
} TaskIMUCalibration_t;

/* Ángulo relativo acumulado en grados, positivo hacia la izquierda. El
 * giro integrado conserva el límite de continuidad de 40 ms de calibración. */
#define IMU_YAW_MAX_GAP_MS IMU_CAL_MAX_GAP_MS
enum { IMU_YAW_PENDING=0, IMU_YAW_VALID=1, IMU_YAW_INVALID=2 };
enum { IMU_YAW_NONE=0, IMU_YAW_CALIBRATION=1, IMU_YAW_I2C=2,
       IMU_YAW_TIMING=3, IMU_YAW_NUMERIC=4, IMU_YAW_REINITIALIZED=5 };
enum { IMU_COMPARE_WAITING=0, IMU_COMPARE_VALID=1, IMU_COMPARE_ODOMETRY=2,
       IMU_COMPARE_YAW=3 };
typedef struct {
    double degrees;
    TickType_t reference_tick, sample_tick, comparison_tick;
    uint32_t reference_request; /* Together with reference_tick, scoped to boot. */
    float gyro_delta_deg, encoder_delta_deg, difference_deg, encoder_reference_deg;
    uint8_t state, reason, comparison_state, encoder_calibration;
} TaskIMUOrientation_t;

/* Diagnostic complementary filter only. tau=1 s gives alpha=1000/1020 at
 * 20 ms; recomputed from actual dt, not a calibrated noise/traction model. */
#define IMU_FUSION_TAU_MS 1000U
#if IMU_FUSION_TAU_MS == 0
#error IMU_FUSION_TAU_MS must be positive
#endif
enum { IMU_FUSION_PENDING=0, IMU_FUSION_VALID=1, IMU_FUSION_DEGRADED=2, IMU_FUSION_INVALID=3 };
enum { IMU_FUSION_NONE=0, IMU_FUSION_IMU=1, IMU_FUSION_ENCODER=2,
       IMU_FUSION_TIMING=3, IMU_FUSION_NUMERIC=4, IMU_FUSION_REFERENCE=5 };
typedef struct {
    double degrees, gyro_degrees, encoder_degrees; /* Continuous common origin. */
    TickType_t reference_tick, sample_tick;
    uint32_t reference_request;
    float alpha;
    float difference_gyro_deg, difference_encoder_deg; /* Wrapped residuals. */
    uint8_t state, reason, encoder_calibration;
} TaskIMUFusion_t;

/* Copia coherente de la cola. IMU_VALID identifica una adquisición válida.
 * sample_tick marca el inicio de la última adquisición exitosa y admite cero.
 * Los ejes crudos corresponden al sensor. */
typedef struct {
    MPU6050_t data;
    TickType_t sample_tick;
    uint32_t i2c_error;
    TaskIMUState_t state;
    HAL_StatusTypeDef hal_status;
    bool has_sample;
    TaskIMUCalibration_t calibration;
    TaskIMUOrientation_t orientation;
    TaskIMUFusion_t fusion;
} TaskIMUSnapshot_t;

bool TaskIMU_GetSnapshot(TaskIMUSnapshot_t *snapshot);
/* Explicit task-context request, <=500 ms old; IDLE/SAFETY_STOP only.
 * Queues calibration only, never changes FSM, Safety or motor targets. */
bool TaskIMU_RequestCalibration(uint32_t request, TickType_t issued_tick);

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
