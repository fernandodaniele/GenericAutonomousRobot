/*
 * mid_nav.c
 *
 *  Created on: Nov 2, 2026
 *      Author: Grupo 2
 *
 *  Maquina de estados de la navegacion autonoma (sin HAL).
 *
 *  IDLE           -> FORWARD         : comando START
 *  FORWARD        -> AVOID_OBSTACLE  : menos de 5 cm de frente o "demasiado cerca"
 *  FORWARD/AVOID  -> CLIFF_RECOVER   : algun TCRT5000 ve precipicio
 *  AVOID_OBSTACLE -> FORWARD         : tras frenar, retroceder y girar, camino libre
 *  CLIFF_RECOVER  -> FORWARD         : ambos TCRT5000 ven suelo tras la maniobra
 *  activo         -> FAULT           : frontal sin eco N veces, foto vieja o sin MPU6050
 *  cualquiera     -> IDLE            : comando STOP
 */

#include "mid_nav.h"
#include <stddef.h>

enum { PHASE_BRAKE = 0, PHASE_BACKUP = 1, PHASE_TURN = 2 };   // AVOID_OBSTACLE
enum { CPHASE_BRAKE = 0, CPHASE_BACKUP = 1, CPHASE_TURN = 2 }; // CLIFF_RECOVER

static void enter_state(NavContext_t* ctx, NavState_t s, uint32_t now)
{
    ctx->state = s;
    ctx->state_enter_ms = now;
    ctx->phase = 0;
    ctx->phase_enter_ms = now;
}

static void enter_phase(NavContext_t* ctx, uint8_t phase, uint32_t now)
{
    ctx->phase = phase;
    ctx->phase_enter_ms = now;
}

static uint32_t elapsed(uint32_t now, uint32_t since)
{
    return now - since;   // aritmetica sin signo: soporta el desborde del tick
}

static bool front_blocked(const NavInput_t* in)
{
    if (in->front == NAV_RANGE_TOO_CLOSE) {
        return true;
    }
    return (in->front == NAV_RANGE_OK) && (in->front_mm < NAV_OBSTACLE_MM);
}

static bool front_clear(const NavInput_t* in)
{
    if (in->front == NAV_RANGE_FAR) {
        return true;
    }
    return (in->front == NAV_RANGE_OK) && (in->front_mm >= NAV_CLEAR_MM);
}

/* Distancia "libre" para elegir el lado de giro. Sin eco cuenta como 0. */
static uint32_t free_space(NavRange_t r, uint32_t mm)
{
    switch (r) {
        case NAV_RANGE_FAR: return 4000u;
        case NAV_RANGE_OK:  return mm;
        default:            return 0u;
    }
}

static int8_t freer_side(const NavInput_t* in)
{
    return (free_space(in->left, in->left_mm) >= free_space(in->right, in->right_mm)) ? 1 : -1;
}

/* Se aleja de la pared seguida; sin pared, va al lado mas libre */
static int8_t evade_dir(const NavContext_t* ctx, const NavInput_t* in)
{
    return (ctx->wall_side != 0) ? (int8_t)(-ctx->wall_side) : freer_side(in);
}

static bool fault_condition(const NavInput_t* in)
{
    return in->snapshot_stale
        || (NAV_STOP_ON_NO_ECHO && in->front_no_echo_count >= NAV_FRONT_NO_ECHO_MAX)
        || (NAV_REQUIRE_IMU && !in->imu_ok);
}

static void set_brake(NavOutput_t* out)
{
    out->brake = true;
    out->linear_mms = 0.0f;
    out->angular_rads = 0.0f;
}

/* Rutina evasiva: frenar -> retroceder -> girar al lado mas libre -> seguir.
 * Si al girar no se libera el camino, vuelve a retroceder. */
static void evade_step(NavContext_t* ctx, const NavInput_t* in, NavOutput_t* out)
{
    const uint32_t now = in->now_ms;
    const uint32_t t = elapsed(now, ctx->phase_enter_ms);

    if (ctx->phase == PHASE_BRAKE) {
        set_brake(out);
        if (t >= NAV_BRAKE_MS) {
            enter_phase(ctx, PHASE_BACKUP, now);
        }
    } else if (ctx->phase == PHASE_BACKUP) {
        out->linear_mms = NAV_BACK_SPEED_MMS;
        if (t >= NAV_EVADE_BACK_MS) {
            ctx->turn_dir = evade_dir(ctx, in);
            enter_phase(ctx, PHASE_TURN, now);
        }
    } else if (t >= NAV_EVADE_TURN_MIN_MS && front_clear(in)) {
        enter_state(ctx, NAV_STATE_FORWARD, now);
        out->linear_mms = NAV_FWD_SPEED_MMS;
    } else if (t >= NAV_EVADE_TURN_MAX_MS) {
        enter_phase(ctx, PHASE_BACKUP, now);
        out->linear_mms = NAV_BACK_SPEED_MMS;
    } else {
        out->angular_rads = (float)ctx->turn_dir * NAV_TURN_RAD_S;
    }
}

/* Distancia a una pared lateral; sin lectura util devuelve "lejos" */
static uint32_t side_mm(NavRange_t r, uint32_t mm)
{
    if (r == NAV_RANGE_TOO_CLOSE) return 0u;
    return (r == NAV_RANGE_OK) ? mm : 4000u;
}

/* Recorrido para mapear: busca una pared, la sigue a NAV_WALL_MM y gira en
 * el lugar en las esquinas. */
static void follow_step(NavContext_t* ctx, const NavInput_t* in, NavOutput_t* out)
{
    if (ctx->wall_side == 0) {
        uint32_t l = side_mm(in->left, in->left_mm);
        uint32_t r = side_mm(in->right, in->right_mm);
        if (l < NAV_WALL_ACQUIRE_MM || r < NAV_WALL_ACQUIRE_MM) {
            ctx->wall_side = (l <= r) ? 1 : -1;
        }
    }

    if (ctx->corner && front_clear(in)) {
        ctx->corner = false;
    } else if (!ctx->corner && in->front == NAV_RANGE_OK && in->front_mm <= NAV_WALL_MM) {
        if (ctx->wall_side == 0) {
            ctx->wall_side = (int8_t)(-freer_side(in));
        }
        ctx->corner = true;
    }
    if (ctx->corner) {
        out->angular_rads = (float)(-ctx->wall_side) * NAV_CORNER_RAD_S;
        return;
    }

    out->linear_mms = NAV_FWD_SPEED_MMS;
    if (ctx->wall_side == 0) {
        return;
    }
    NavRange_t r = (ctx->wall_side > 0) ? in->left : in->right;
    uint32_t mm = (ctx->wall_side > 0) ? in->left_mm : in->right_mm;
    if (r == NAV_RANGE_NO_ECHO) {
        return;   // Sensor lateral sin respuesta: sigue derecho
    }
    int32_t err = (int32_t)side_mm(r, mm) - (int32_t)NAV_WALL_MM;
    if (err > NAV_WALL_DEADBAND_MM) {
        out->angular_rads = (float)ctx->wall_side * NAV_WALL_MAX_RAD_S;
    } else if (err < -NAV_WALL_DEADBAND_MM) {
        out->angular_rads = (float)(-ctx->wall_side) * NAV_WALL_MAX_RAD_S;
    }
}

void Nav_Init(NavContext_t* ctx)
{
    if (!ctx) return;
    ctx->state = NAV_STATE_IDLE;
    ctx->state_enter_ms = 0;
    ctx->phase = 0;
    ctx->phase_enter_ms = 0;
    ctx->turn_dir = 1;
    ctx->last_alert_ms = 0;
    ctx->alert_armed = false;
    ctx->wall_side = 0;
    ctx->corner = false;
}

NavOutput_t Nav_Step(NavContext_t* ctx, const NavInput_t* in)
{
    NavOutput_t out = { NAV_STATE_IDLE, 0.0f, 0.0f, false, 0u };
    if (!ctx || !in) {
        out.brake = true;
        return out;
    }
    const uint32_t now = in->now_ms;

    /* --- Giro brusco: no cambia el estado, solo avisa --- */
    int32_t gz = in->gyro_z_dps;
    if (gz < 0) gz = -gz;
    if (in->imu_ok && !in->snapshot_stale && gz > NAV_GYRO_ALERT_DPS) {
        if (!ctx->alert_armed || elapsed(now, ctx->last_alert_ms) >= NAV_ALERT_HOLDOFF_MS) {
            out.events |= NAV_EVT_ALERT;
            ctx->last_alert_ms = now;
            ctx->alert_armed = true;
        }
    }

    /* --- Comandos --- */
    if (in->command == NAV_CMD_STOP) {
        enter_state(ctx, NAV_STATE_IDLE, now);
    } else if (in->command == NAV_CMD_START && ctx->state == NAV_STATE_IDLE) {
        enter_state(ctx, NAV_STATE_FORWARD, now);
    }

    /* --- Falla resuelta: vuelve a andar solo --- */
    if (ctx->state == NAV_STATE_FAULT && !fault_condition(in)) {
        enter_state(ctx, NAV_STATE_FORWARD, now);
    }

    /* --- Fallas: tienen prioridad sobre cualquier maniobra --- */
    if (ctx->state != NAV_STATE_IDLE && ctx->state != NAV_STATE_FAULT && fault_condition(in)) {
        enter_state(ctx, NAV_STATE_FAULT, now);
        out.events |= NAV_EVT_FAULT;
    }

    /* --- Precipicio: prioridad sobre el obstaculo --- */
    if ((ctx->state == NAV_STATE_FORWARD || ctx->state == NAV_STATE_AVOID_OBSTACLE)
        && (in->cliff_left || in->cliff_right)) {
        enter_state(ctx, NAV_STATE_CLIFF_RECOVER, now);
        /* Girar alejandose del lado que vio el borde */
        ctx->turn_dir = in->cliff_left ? -1 : 1;
        /* Olvida la pared: buscarla de nuevo podria llevarlo otra vez al borde */
        ctx->wall_side = 0;
        ctx->corner = false;
        out.events |= NAV_EVT_CLIFF;
    }

    switch (ctx->state) {
        case NAV_STATE_IDLE:
            /* Motores detenidos (sin freno: rueda libre) */
            break;

        case NAV_STATE_FORWARD:
            if (front_blocked(in)) {
                enter_state(ctx, NAV_STATE_AVOID_OBSTACLE, now);
                ctx->turn_dir = evade_dir(ctx, in);
                ctx->corner = false;
                set_brake(&out);
            } else {
                follow_step(ctx, in, &out);
            }
            break;

        case NAV_STATE_AVOID_OBSTACLE:
            evade_step(ctx, in, &out);
            break;

        case NAV_STATE_CLIFF_RECOVER:
            if (ctx->phase == CPHASE_BRAKE) {
                set_brake(&out);
                if (elapsed(now, ctx->phase_enter_ms) >= NAV_BRAKE_MS) {
                    enter_phase(ctx, CPHASE_BACKUP, now);
                }
            } else if (ctx->phase == CPHASE_BACKUP) {
                out.linear_mms = NAV_BACK_SPEED_MMS;
                if (elapsed(now, ctx->phase_enter_ms) >= NAV_BACKUP_MS) {
                    enter_phase(ctx, CPHASE_TURN, now);
                }
            } else {
                out.angular_rads = (float)ctx->turn_dir * NAV_TURN_RAD_S;
                if (elapsed(now, ctx->phase_enter_ms) >= NAV_CLIFF_TURN_MS) {
                    if (!in->cliff_left && !in->cliff_right) {
                        enter_state(ctx, NAV_STATE_FORWARD, now);
                        out.angular_rads = 0.0f;
                        out.linear_mms = NAV_FWD_SPEED_MMS;
                    } else {
                        /* Sigue viendo borde: retroceder otra vez */
                        enter_phase(ctx, CPHASE_BACKUP, now);
                        out.angular_rads = 0.0f;
                        out.linear_mms = NAV_BACK_SPEED_MMS;
                    }
                }
            }
            break;

        case NAV_STATE_FAULT:
        default:
            set_brake(&out);
            break;
    }

    out.state = ctx->state;
    return out;
}

const char* Nav_StateName(NavState_t s)
{
    switch (s) {
        case NAV_STATE_IDLE:           return "IDLE";
        case NAV_STATE_FORWARD:        return "FORWARD";
        case NAV_STATE_AVOID_OBSTACLE: return "AVOID";
        case NAV_STATE_CLIFF_RECOVER:  return "CLIFF";
        case NAV_STATE_FAULT:          return "FAULT";
        default:                       return "?";
    }
}
