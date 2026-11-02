/*
 * mid_nav.h
 *
 *  Created on: Nov 2, 2026
 *      Author: Grupo 2
 *
 *  Modulo de decision de la navegacion autonoma. Maquina de cinco estados
 *  SIN dependencias de HAL ni de FreeRTOS: se puede compilar y probar en la PC
 *  (ver Test/test_mid_nav.c).
 *
 *  Reglas: avanzar mientras no haya nada a menos de 5 cm, esquivar el
 *  precipicio y emitir una alerta ante un giro brusco.
 */

#ifndef CORE_INC_MID_NAV_H_
#define CORE_INC_MID_NAV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ===== Parametros (ajustar con la distancia de frenado medida) ===== */
#ifndef NAV_OBSTACLE_MM
#define NAV_OBSTACLE_MM        50    // Entra en AVOID_OBSTACLE por debajo de esto
#endif
#ifndef NAV_CLEAR_MM
#define NAV_CLEAR_MM           80    // Sale de AVOID_OBSTACLE con este margen
#endif
#ifndef NAV_FWD_SPEED_MMS
#define NAV_FWD_SPEED_MMS      120.0f
#endif
#ifndef NAV_BACK_SPEED_MMS
#define NAV_BACK_SPEED_MMS     (-100.0f)
#endif
#ifndef NAV_TURN_RAD_S
#define NAV_TURN_RAD_S         1.5f
#endif
#ifndef NAV_BRAKE_MS
#define NAV_BRAKE_MS           150
#endif
#ifndef NAV_BACKUP_MS
#define NAV_BACKUP_MS          400
#endif
#ifndef NAV_CLIFF_TURN_MS
#define NAV_CLIFF_TURN_MS      500
#endif
#ifndef NAV_FRONT_NO_ECHO_MAX
#define NAV_FRONT_NO_ECHO_MAX  3
#endif
#ifndef NAV_GYRO_ALERT_DPS
#define NAV_GYRO_ALERT_DPS     200
#endif
#ifndef NAV_ALERT_HOLDOFF_MS
#define NAV_ALERT_HOLDOFF_MS   500
#endif

typedef enum {
    NAV_STATE_IDLE = 0,
    NAV_STATE_FORWARD,
    NAV_STATE_AVOID_OBSTACLE,
    NAV_STATE_CLIFF_RECOVER,
    NAV_STATE_FAULT
} NavState_t;

typedef enum {
    NAV_CMD_NONE = 0,
    NAV_CMD_START,
    NAV_CMD_STOP
} NavCommand_t;

/* Lectura del frontal, independiente del driver (sin HAL) */
typedef enum {
    NAV_RANGE_OK = 0,
    NAV_RANGE_NO_ECHO,
    NAV_RANGE_FAR,
    NAV_RANGE_TOO_CLOSE
} NavRange_t;

/* Eventos emitidos (mascara de bits) */
#define NAV_EVT_ALERT   (1u << 0)   // Giro brusco
#define NAV_EVT_CLIFF   (1u << 1)   // Entro en CLIFF_RECOVER
#define NAV_EVT_FAULT   (1u << 2)   // Entro en FAULT

typedef struct {
    uint32_t now_ms;
    NavCommand_t command;
    bool snapshot_stale;           // Foto de sensores vieja (> 300 ms)
    NavRange_t front;
    uint32_t front_mm;
    NavRange_t left;
    uint32_t left_mm;
    NavRange_t right;
    uint32_t right_mm;
    uint8_t front_no_echo_count;
    bool cliff_left;
    bool cliff_right;
    int16_t gyro_z_dps;
    bool imu_ok;
} NavInput_t;

typedef struct {
    NavState_t state;
    float linear_mms;              // Velocidad lineal pedida (mm/s)
    float angular_rads;            // Velocidad angular pedida (rad/s, + = izquierda)
    bool brake;                    // true: freno activo, ignorar velocidades
    uint32_t events;               // NAV_EVT_* ocurridos en este paso
} NavOutput_t;

typedef struct {
    NavState_t state;
    uint32_t state_enter_ms;
    uint8_t phase;                 // Sub-fase dentro de AVOID/CLIFF
    uint32_t phase_enter_ms;
    int8_t turn_dir;               // +1 izquierda, -1 derecha
    uint32_t last_alert_ms;
    bool alert_armed;
} NavContext_t;

void Nav_Init(NavContext_t* ctx);

/**
 * @brief Ejecuta un paso de la maquina de estados.
 */
NavOutput_t Nav_Step(NavContext_t* ctx, const NavInput_t* in);

/**
 * @brief Nombre corto del estado (para telemetria).
 */
const char* Nav_StateName(NavState_t s);

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_MID_NAV_H_ */
