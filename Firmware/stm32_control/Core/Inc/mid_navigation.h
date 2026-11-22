#ifndef MID_NAVIGATION_H_
#define MID_NAVIGATION_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "mid_motion.h"

/* Base odometrica AUTO-1/AUTO-2.
 * 0 conserva el perfil B7 para regresiones explicitas.
 * AUTO-1/B7 remain compile-time regressions; AUTO-3 is an explicit session request. */
#ifndef NAV_AUTO1_ODOMETRY
#define NAV_AUTO1_ODOMETRY 1
#endif
#ifndef NAV_AUTO2_ENABLED
#define NAV_AUTO2_ENABLED NAV_AUTO1_ODOMETRY
#endif
#if (NAV_AUTO2_ENABLED != 0 && NAV_AUTO2_ENABLED != 1) || (NAV_AUTO2_ENABLED && !NAV_AUTO1_ODOMETRY)
#error "AUTO-2 requires the odometry profile"
#endif
/* AUTO-2 operativo por defecto. AUTO-1: compilar NAV_AUTO2_ENABLED=0.
 * AUTO-3 requests select a new session only; never change an active profile. */
#define NAV_AUTO2_REVERSE_TARGET_MM 20U
#define NAV_AUTO2_TURN_TARGET_DEG 10U
#define NAV_AUTO2_PROGRESS_WINDOW_MS 200U
#define NAV_AUTO2_MIN_PROGRESS_MM 1U
#define NAV_AUTO2_ADVANCE_MAX_MS 2000U
/* AUTO-2E: observaciones de Navigation_Step (20 ms), sin espera bloqueante. */
#define NAV_SETTLE_WINDOW_MS 60U
#define NAV_SETTLE_MIN_OBSERVATIONS 4U
#define NAV_SETTLE_MAX_SPAN_COUNTS 1U
#define NAV_SETTLE_TIMEOUT_MS 200U
#if NAV_AUTO1_ODOMETRY != 0 && NAV_AUTO1_ODOMETRY != 1
#error "NAV_AUTO1_ODOMETRY must be 0 or 1"
#endif
#define NAV_AUTO1_TARGET_MM 100U
/* Guarda provisional: 10% del objetivo. Detiene una discrepancia entre ruedas
 * antes de aceptar solo el promedio. No corrige rumbo, no cambia umbrales de
 * Safety y no detecta deslizamiento igual en ambas ruedas. */
#define NAV_AUTO1_MAX_WHEEL_DIFFERENCE_MM 10U

/* Legacy experiment for regression builds only. Operational AUTO-3 instead
 * requires an explicit one-shot AUTO_TURN/AUTO_EVADE request. */
#ifndef NAV_EXPERIMENTAL_TURN_ENABLED
#define NAV_EXPERIMENTAL_TURN_ENABLED 0
#endif
#ifndef NAV_TURN_LEFT
#define NAV_TURN_LEFT 1
#endif
#if (NAV_EXPERIMENTAL_TURN_ENABLED != 0 && NAV_EXPERIMENTAL_TURN_ENABLED != 1) || \
    (NAV_TURN_LEFT != 0 && NAV_TURN_LEFT != 1)
#error "Navigation experimental options must be 0 or 1"
#endif

/* Provisionales: no representan distancia de frenado ni watchdog UART. */
#if NAV_AUTO2_ENABLED
/* AUTO-2F: margenes experimentales de navegacion, Safety conserva 200 mm.
 * AUTO-1/B7 mantienen sus umbrales/plazos validados. */
#define NAV_FRONT_STOP_MM 275U
#define NAV_REAR_REVERSE_MIN_MM 250U
#define NAV_REVERSE_MAX_MS 800U
#define NAV_SESSION_MAX_MS 3000U
#else
#define NAV_FRONT_STOP_MM 300U
#define NAV_REAR_REVERSE_MIN_MM 300U
#define NAV_REVERSE_MAX_MS 300U
#define NAV_SESSION_MAX_MS 2000U
#endif
#define NAV_TURN_MAX_MS 250U /* Historical experimental profiles. */
#define NAV_AUTO3_TURN_MAX_MS 800U
#define NAV_AUTO3_SESSION_MAX_MS 4200U
#define NAV_AUTO3_ISOLATED_MAX_MS 1400U /* data 200 + settle 200 + turn 800 + transitions 200 */
#define NAV_AUTO3_MAX_WHEEL_DIFFERENCE_MM 3U /* center translation <=1.5 mm in the ideal model */
/* Explicit one-shot requests only; ordinary AUTO always keeps AUTO-2F. */
typedef enum { NAV_PROFILE_TURN = 3, NAV_PROFILE_EVASION = 4 } NavigationExperimentalProfile_t;
#if NAV_TURN_LEFT
#define NAV_TURN_LEFT_CPS (-MOTION_TURN_CPS)
#define NAV_TURN_RIGHT_CPS MOTION_TURN_CPS
#else
#define NAV_TURN_LEFT_CPS MOTION_TURN_CPS
#define NAV_TURN_RIGHT_CPS (-MOTION_TURN_CPS)
#endif

typedef enum {
    NAV_ESPERAR_DATOS = 0,
    NAV_AVANZAR,
    NAV_DETENIDO,
    NAV_RETROCEDER_ACOTADO,
    NAV_GIRAR_ACOTADO,
    NAV_DETENER_Y_ESPERAR
} NavigationState_t;

typedef enum {
    NAV_FINISH_NONE=0,
    NAV_FINISH_TARGET,
    NAV_FINISH_CANCELLED,
    NAV_FINISH_SAFETY,
    NAV_FINISH_TIMEOUT,
    NAV_FINISH_OBSTACLE,
    NAV_FINISH_SENSORS,
    NAV_FINISH_ODOMETRY,
    NAV_FINISH_EMISSION,
    NAV_FINISH_EVASION_COMPLETED,
    NAV_FINISH_EVASION_NOT_AUTHORIZED,
    NAV_FINISH_OPERATOR_STOP,
    NAV_FINISH_NO_PROGRESS
} NavigationFinishReason_t;

/* Begin solo al entrar efectivamente en AUTO; nunca al renovar consignas. */
void Navigation_Begin(TickType_t entered_tick);
void Navigation_BeginProfile(TickType_t entered_tick, uint8_t profile);
bool Navigation_TurnEnabled(void);
/* Revoca sesion/permisos, sin escribir salidas; Stop del controlador lo invoca. */
void Navigation_Cancel(void);
/* Cancela solo la sesion indicada; una consigna vieja no cancela otra nueva. */
void Navigation_CancelSession(uint32_t session);
void Navigation_Step(void);
NavigationState_t Navigation_GetState(void);
/* Resultado terminal de solo lectura; se limpia en Navigation_Begin(). */
NavigationFinishReason_t Navigation_GetFinishReason(void);

/* AUTO-2D: observacion, nunca permisos de movimiento. Un solo registro RAM. */
enum {
    NAV_DIAG_SESSION=1U, NAV_DIAG_OBSTACLE=2U, NAV_DIAG_REVERSE_CONSIDERED=4U,
    NAV_DIAG_REVERSE_AUTHORIZED=8U, NAV_DIAG_REVERSE_REQUESTED=16U,
    NAV_DIAG_REVERSE_PUBLISHED=32U, NAV_DIAG_REVERSE_APPLIED=64U,
    NAV_DIAG_REVERSE_COMPLETED=128U, NAV_DIAG_ADVANCE_PROGRESS=256U,
    NAV_DIAG_REVERSE_PROGRESS=512U
};
typedef enum {
    NAV_ODOM_OK=0, NAV_ODOM_UNAVAILABLE, NAV_ODOM_STALE, NAV_ODOM_TIMESTAMP,
    NAV_ODOM_DIRECTION, NAV_ODOM_OVERFLOW, NAV_ODOM_NONFINITE,
    NAV_ODOM_DISAGREEMENT, NAV_ODOM_RANGE
} NavigationOdometryError_t;
typedef struct {
    /* 0 no solicitada, 1 esperando, 2 confirmada, 3 timeout, 4 cancelada. */
    uint8_t settle_state;
    uint16_t settle_observations, settle_window_observations;
    uint32_t settle_elapsed_ms, settle_window_ms, settle_left_span, settle_right_span;
    bool reverse_reference_available, reverse_entered;
    TickType_t reverse_reference_tick, reverse_entry_tick;
    int64_t reverse_reference_left, reverse_reference_right;
} NavigationStabilization_t;
typedef struct {
    uint32_t session_id, started_tick, elapsed_ms, phase_elapsed_ms;
    TickType_t observed_tick, decision_tick;
    NavigationState_t state, last_phase;
    NavigationFinishReason_t reason;
    uint16_t flags, front_mm, rear_mm;
    uint8_t profile, sensor_flags; /* disponible=1, frontal valido=2, trasero=4, caida=8 */
    uint32_t snapshot_age_ms, front_age_ms, rear_age_ms;
    int32_t advance_um, reverse_um;
    NavigationOdometryError_t odometry_error;
    bool deltas_available;
    int64_t left_delta, right_delta;
    /* Anonymous aliases preserve the existing reverse diagnostic fields. */
    union {
        NavigationStabilization_t reverse_wait;
        struct {
            /* 0 no solicitada, 1 esperando, 2 confirmada, 3 timeout, 4 cancelada. */
            uint8_t settle_state;
            uint16_t settle_observations, settle_window_observations;
            uint32_t settle_elapsed_ms, settle_window_ms, settle_left_span, settle_right_span;
            bool reverse_reference_available, reverse_entered;
            TickType_t reverse_reference_tick, reverse_entry_tick;
            int64_t reverse_reference_left, reverse_reference_right;
        };
    };
    NavigationStabilization_t turn_wait;
    uint8_t turn_flags; /* requested=1, published=2, applied=4, completed=8, measured=16 */
    uint32_t turn_mdeg, turn_elapsed_ms;
} NavigationDiagnostics_t;
void Navigation_GetDiagnostics(NavigationDiagnostics_t *out);
/* Solo TaskComms: copia bajo lock y formatea fuera. <=255 bytes con LF. */
int Navigation_FormatDiagnostics(char *out, size_t capacity, uint32_t query);
/* Parte complementaria de la MISMA copia/Q tomada al formatear NAV. */
int Navigation_FormatStabilization(char *out, size_t capacity, uint32_t query);
int Navigation_FormatTurn(char *out, size_t capacity, uint32_t query);

/* Barreras internas del backend: permiso de emision de un solo uso y sesion
 * vigente tambien durante el consumo/PI. No consultan colas ni esperan. */
bool Navigation_ConsumeEmission(uint32_t session, TickType_t issued_tick, TickType_t now,
                                NavigationState_t phase);
bool Navigation_IsCommandCurrent(uint32_t session, TickType_t now);

#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
/* Acuse del backend despues de aplicar ambos objetivos. No acredita recorrido. */
void Navigation_CommandApplied(uint32_t session, TickType_t now);
/* Fin normal de fase: deja PWM a cero, pendiente de evaluar su finalizacion.
 * No se usa para timeout de consigna, STOP, Safety o cambio de modo. */
bool Navigation_CompleteCommand(uint32_t session, TickType_t now);
#endif

#endif
