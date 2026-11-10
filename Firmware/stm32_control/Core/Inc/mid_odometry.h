#ifndef MID_ODOMETRY_H
#define MID_ODOMETRY_H
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#define ODOM_HISTORY_LENGTH 64U
#define ODOM_MAX_INTERVAL_MS 50U
#define ODOM_POSITION_LIMIT_M 100.0f
typedef enum {
    ODOM_GEOMETRY_INCOMPLETE = 0,
    ODOM_GEOMETRY_PROVISIONAL = 1,
    ODOM_GEOMETRY_CALIBRATED = 2
} OdometryCalibration;
typedef struct {
    float left_radius_m, right_radius_m, left_cpr, right_cpr, track_m;
    bool confirmed, sensors_confirmed; /* Confirmaciones fisicas, no habilitacion de metrica. */
    float sensor_x[2], sensor_y[2], sensor_yaw[2], beam_half_angle;
} OdometryConfig;
typedef struct {
    TickType_t tick;
    int64_t left_count, right_count;
    float x_m, y_m, theta_rad, increment_m, increment_rad;
    uint8_t state; /* 0 sin muestra, 1 no calibrada, 2 valida, 3 discontinuidad */
    bool calibrated; /* true exclusivamente con ambas confirmaciones fisicas. */
    OdometryCalibration calibration;
} OdometrySnapshot;
typedef struct {
    TickType_t tick;
    uint32_t sequence;
    float x_m, y_m, yaw_rad, distance_m, half_angle;
    uint8_t state; /* 0 sin muestra, 1 adquisicion fallida, 2 sin pose, 3 asociada */
} OdometryRay;
/* Init solo al arrancar MotorControl, Update solo por ese productor, fuera
 * de la transaccion PI/PWM. No leer encoders nuevamente. Sin heap ni E/S. */
void Odometry_Init(TickType_t tick);
void Odometry_InitConfig(const OdometryConfig *config, TickType_t tick);
void Odometry_Update(int32_t left_delta, int32_t right_delta, TickType_t tick);
void Odometry_GetSnapshot(OdometrySnapshot *out);
bool Odometry_PoseAt(TickType_t tick, float *x, float *y, float *theta);
/* Observador de un intento resuelto; no altera datos de Safety/SENSORS. */
void Odometry_Observe(unsigned sensor, float distance, TickType_t trigger, bool valid);
void Odometry_GetRay(unsigned sensor, OdometryRay *out);
#endif
