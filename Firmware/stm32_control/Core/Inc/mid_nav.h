/*
 * mid_nav.h
 *
 *  Created on: Nov 2, 2026
 *      Author: Grupo 2
 *
 *  Modulo de decision de la navegacion autonoma. Maquina de estados SIN
 *  dependencias de HAL ni de FreeRTOS: se puede compilar y probar en la PC
 *  (ver Test/test_mid_nav.c y Test/test_mid_nav_scan.c).
 *
 *  Modo por defecto (NAV_STEP_SCAN_ENABLE = 1): avanzar recto NAV_STEP_MM,
 *  frenar, medir con el robot quieto y decidir si seguir, doblar o retroceder.
 *
 *  Modo continuo (NAV_STEP_SCAN_ENABLE = 0): seguir la pared a NAV_WALL_MM y
 *  girar en las esquinas; a menos de NAV_OBSTACLE_MM de frente, maniobra
 *  evasiva.
 *
 *  En ambos modos se esquiva el precipicio y se emite una alerta ante un giro
 *  brusco.
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
#define NAV_OBSTACLE_MM        180    // De frente: maniobra evasiva por debajo de esto
#endif
#ifndef NAV_CLEAR_MM
#define NAV_CLEAR_MM           250   // Frente libre: termina el giro o la evasiva
#endif
#ifndef NAV_WALL_MM
#define NAV_WALL_MM            100   // Distancia a mantener con la pared
#endif
#ifndef NAV_WALL_ACQUIRE_MM
#define NAV_WALL_ACQUIRE_MM    300   // Toma como pared lo que vea a menos de esto
#endif
#ifndef NAV_WALL_DEADBAND_MM
#define NAV_WALL_DEADBAND_MM   20     // Dentro de +/- esto va recto
#endif
#ifndef NAV_WALL_MAX_RAD_S
#define NAV_WALL_MAX_RAD_S     0.1f   // Correccion al seguir la pared: frena la rueda interna (25 mm/s, sobre el piso de PWM)
#endif
#ifndef NAV_CORNER_RAD_S
#define NAV_CORNER_RAD_S       0.35f  // Giro en esquinas (35 mm/s por rueda)
#endif
#ifndef NAV_EVADE_BACK_MS
#define NAV_EVADE_BACK_MS      1150  // Retroceso de la evasiva (misma distancia a 35 mm/s)
#endif
#ifndef NAV_EVADE_TURN_MIN_MS
#define NAV_EVADE_TURN_MIN_MS  1450  // Giro minimo antes de volver a avanzar (mismo angulo a 0,35 rad/s)
#endif
#ifndef NAV_EVADE_TURN_MAX_MS
#define NAV_EVADE_TURN_MAX_MS  5000  // Si sigue bloqueado: retrocede otra vez
#endif
#ifndef NAV_FWD_SPEED_MMS
#define NAV_FWD_SPEED_MMS      35.0f    // ~70 % de PWM (proporcional, ver mid_kinematics.h)
#endif
#ifndef NAV_BACK_SPEED_MMS
#define NAV_BACK_SPEED_MMS     (-35.0f)
#endif
#ifndef NAV_TURN_RAD_S
#define NAV_TURN_RAD_S         0.35f    // 35 mm/s por rueda en el giro
#endif
#ifndef NAV_BRAKE_MS
#define NAV_BRAKE_MS           400
#endif
#ifndef NAV_BACKUP_MS
#define NAV_BACKUP_MS          2150
#endif
#ifndef NAV_CLIFF_TURN_MS
#define NAV_CLIFF_TURN_MS      4300
#endif
/* Pausa con freno entre frenar y retroceder, entre retroceder y girar y
 * entre girar y avanzar: el robot queda quieto y los ultrasonidos leen
 * estable antes de decidir. */
#ifndef NAV_SETTLE_MS
#define NAV_SETTLE_MS          300
#endif
/* Precipicio confirmado: lecturas de sensores seguidas con borde antes de
 * entrar en CLIFF_RECOVER (evita falsos positivos del granito). */
#ifndef NAV_CLIFF_CONFIRM
#define NAV_CLIFF_CONFIRM      3
#endif
/* Bucle de retroceso: tras este numero de CLIFF_RECOVER seguidos sin haber
 * avanzado NAV_CLIFF_LOOP_RESET_MS en FORWARD, pasa a FAULT (frenado, se
 * sale con STOP y START). */
#ifndef NAV_CLIFF_LOOP_MAX
#define NAV_CLIFF_LOOP_MAX     3
#endif
#ifndef NAV_CLIFF_LOOP_RESET_MS
#define NAV_CLIFF_LOOP_RESET_MS 2000
#endif
/* Giro de esquina mas largo que esto: pasa a la rutina evasiva */
#ifndef NAV_CORNER_MAX_MS
#define NAV_CORNER_MAX_MS      8000
#endif
/* Lado sin eco al elegir hacia donde girar:
 * 0 = cuenta como espacio libre 0 (actual: con el derecho sin eco gira
 *     siempre a la izquierda, el lado que tiene el unico TCRT activo).
 * 1 = cuenta como "desconocido" (NAV_CLEAR_MM): solo gana el lado medido si
 *     tiene mas que eso. Puede girar hacia el lado sin TCRT ni ultrasonido. */
#ifndef NAV_NO_ECHO_AS_UNKNOWN
#define NAV_NO_ECHO_AS_UNKNOWN 0
#endif
/* 0 = sin eco del frontal sigue andando (a ciegas de frente) en vez de frenar.
 * 1 = se detiene hasta que el sensor vuelva a responder. */
#ifndef NAV_STOP_ON_NO_ECHO
#define NAV_STOP_ON_NO_ECHO    0
#endif
#ifndef NAV_FRONT_NO_ECHO_MAX
#define NAV_FRONT_NO_ECHO_MAX  3
#endif
/* 1 = sin MPU6050 el robot entra en FAULT (lo que pide el plan).
 * 0 = se navega igual sin MPU6050 (solo se pierde la alerta de giro brusco). */
#ifndef NAV_REQUIRE_IMU
#define NAV_REQUIRE_IMU        0
#endif
#ifndef NAV_GYRO_ALERT_DPS
#define NAV_GYRO_ALERT_DPS     200
#endif
#ifndef NAV_ALERT_HOLDOFF_MS
#define NAV_ALERT_HOLDOFF_MS   500
#endif

/* ===== Avance por tramos con escaneo =====
 * 1 = avanza recto NAV_STEP_MM, frena, mide con el robot quieto y decide:
 *     adelante, doblar hacia el lado libre o retroceder.
 * 0 = seguimiento de pared continuo (comportamiento anterior). */
#ifndef NAV_STEP_SCAN_ENABLE
#define NAV_STEP_SCAN_ENABLE   1
#endif
/* Sin encoders el tramo se mide por tiempo: si el robot no recorre 20 cm,
 * ajustar NAV_STEP_MS con lo medido en el piso. */
#ifndef NAV_STEP_MM
#define NAV_STEP_MM            200
#endif
#ifndef NAV_STEP_MS
#define NAV_STEP_MS            ((uint32_t)((float)NAV_STEP_MM * 1000.0f / NAV_FWD_SPEED_MMS))
#endif
/* Giro de 90 grados: (pi / 2) / NAV_TURN_RAD_S. Calibrar en el piso. */
#ifndef NAV_SCAN_TURN_MS
#define NAV_SCAN_TURN_MS       4488
#endif
/* Retroceso con todo bloqueado: unos 10 cm a 35 mm/s (atras no hay sensores) */
#ifndef NAV_SCAN_BACK_MS
#define NAV_SCAN_BACK_MS       2857
#endif
/* Lecturas validas por sensor; se usa la mediana */
#ifndef NAV_SCAN_SAMPLES
#define NAV_SCAN_SAMPLES       3
#endif
/* Tiempo maximo midiendo: un sensor sin eco no traba la decision.
 * Cada lateral se lee cada 240 ms: 3 lecturas entran con margen. */
#ifndef NAV_SCAN_MAX_MS
#define NAV_SCAN_MAX_MS        1200
#endif
/* Giros o retrocesos seguidos sin avanzar antes de FAULT (salir con STOP y START) */
#ifndef NAV_SCAN_STUCK_MAX
#define NAV_SCAN_STUCK_MAX     6
#endif

typedef enum {
    NAV_STATE_IDLE = 0,
    NAV_STATE_FORWARD,
    NAV_STATE_AVOID_OBSTACLE,
    NAV_STATE_CLIFF_RECOVER,
    NAV_STATE_FAULT,
    NAV_STATE_SCAN
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
    uint32_t sensor_ms;            // Marca de tiempo de la foto de sensores (una lectura nueva la cambia)
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
    bool fault_latched;            // FAULT por bucle de retroceso (requiere STOP)
} NavOutput_t;

typedef struct {
    NavState_t state;
    uint32_t state_enter_ms;
    uint8_t phase;                 // Sub-fase dentro de AVOID/CLIFF (MPHASE_*)
    uint32_t phase_enter_ms;
    int8_t turn_dir;               // +1 izquierda, -1 derecha
    uint32_t last_alert_ms;
    bool alert_armed;
    int8_t wall_side;              // Pared seguida: +1 izquierda, -1 derecha, 0 ninguna
    bool corner;                   // Girando en una esquina
    uint32_t corner_enter_ms;      // Inicio del giro de esquina
    uint32_t last_sensor_ms;       // Ultima foto contada para el borde
    uint8_t cliff_hits;            // Lecturas seguidas con borde
    uint8_t cliff_streak;          // CLIFF_RECOVER seguidos sin avanzar
    bool fault_latched;            // FAULT que no se recupera solo
    uint16_t scan_samples[3][NAV_SCAN_SAMPLES]; // Espacio libre (mm) ordenado: frente, izquierda, derecha
    uint8_t scan_n[3];             // Lecturas validas por sensor en este escaneo
    uint8_t scan_streak;           // Giros o retrocesos seguidos sin avanzar
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
