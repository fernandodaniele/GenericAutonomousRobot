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
 *  FORWARD/AVOID  -> CLIFF_RECOVER   : algun TCRT5000 ve precipicio en NAV_CLIFF_CONFIRM lecturas seguidas
 *  CLIFF_RECOVER  -> FAULT           : NAV_CLIFF_LOOP_MAX retrocesos seguidos sin avanzar 2 s (bucle)
 *  FORWARD (esquina) -> AVOID        : giro de esquina mas largo que NAV_CORNER_MAX_MS
 *  AVOID_OBSTACLE -> FORWARD         : tras frenar, retroceder y girar (con pausas), camino libre
 *  CLIFF_RECOVER  -> FORWARD         : ambos TCRT5000 ven suelo tras la maniobra
 *  activo         -> FAULT           : frontal sin eco N veces, foto vieja o sin MPU6050
 *  cualquiera     -> IDLE            : comando STOP
 */

#include "mid_nav.h"
#include <stddef.h>

/* Sub-fases de las maniobras (AVOID_OBSTACLE y CLIFF_RECOVER):
 * frenar -> pausa -> retroceder -> pausa -> girar -> pausa -> FORWARD */
enum {
    MPHASE_BRAKE = 0,
    MPHASE_SETTLE_BACK,   // pausa antes de retroceder
    MPHASE_BACKUP,
    MPHASE_SETTLE_TURN,   // pausa antes de girar
    MPHASE_TURN,
    MPHASE_SETTLE_FWD     // pausa antes de volver a avanzar
};

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

/* Distancia "libre" para elegir el lado de giro. Sin eco cuenta como 0, o
 * como "desconocido" (NAV_CLEAR_MM) con NAV_NO_ECHO_AS_UNKNOWN. */
static uint32_t free_space(NavRange_t r, uint32_t mm)
{
    switch (r) {
        case NAV_RANGE_FAR:     return 4000u;
        case NAV_RANGE_OK:      return mm;
        case NAV_RANGE_NO_ECHO: return NAV_NO_ECHO_AS_UNKNOWN ? (uint32_t)NAV_CLEAR_MM : 0u;
        default:                return 0u;   /* TOO_CLOSE */
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

/* Cuenta un retroceso del borde. Con NAV_CLIFF_LOOP_MAX seguidos sin haber
 * avanzado, el robot esta atrapado: FAULT frenado en vez de seguir.
 * Devuelve true si entro en FAULT. */
static bool cliff_count(NavContext_t* ctx, NavOutput_t* out, uint32_t now)
{
    if (ctx->cliff_streak < UINT8_MAX) ctx->cliff_streak++;
    if (ctx->cliff_streak >= NAV_CLIFF_LOOP_MAX) {
        enter_state(ctx, NAV_STATE_FAULT, now);
        ctx->fault_latched = true;
        out->events |= NAV_EVT_FAULT;
        set_brake(out);
        return true;
    }
    return false;
}

/* Repite el retroceso del borde (si no es un bucle) */
static void cliff_repeat(NavContext_t* ctx, NavOutput_t* out, uint32_t now)
{
    if (!cliff_count(ctx, out, now)) {
        enter_phase(ctx, MPHASE_SETTLE_BACK, now);
    }
}

/* Rutina evasiva: frenar -> pausa -> retroceder -> pausa -> girar al lado mas
 * libre -> pausa -> seguir. Si al girar no se libera el camino, vuelve a
 * retroceder (con pausa previa). */
static void evade_step(NavContext_t* ctx, const NavInput_t* in, NavOutput_t* out)
{
    const uint32_t now = in->now_ms;
    const uint32_t t = elapsed(now, ctx->phase_enter_ms);

    switch (ctx->phase) {
        case MPHASE_BRAKE:
            set_brake(out);
            if (t >= NAV_BRAKE_MS) enter_phase(ctx, MPHASE_SETTLE_BACK, now);
            break;
        case MPHASE_SETTLE_BACK:
            set_brake(out);
            if (t >= NAV_SETTLE_MS) enter_phase(ctx, MPHASE_BACKUP, now);
            break;
        case MPHASE_BACKUP:
            out->linear_mms = NAV_BACK_SPEED_MMS;
            if (t >= NAV_EVADE_BACK_MS) enter_phase(ctx, MPHASE_SETTLE_TURN, now);
            break;
        case MPHASE_SETTLE_TURN:
            set_brake(out);
            if (t >= NAV_SETTLE_MS) {
                /* Decide el lado con el robot quieto y lecturas estables */
                ctx->turn_dir = evade_dir(ctx, in);
                enter_phase(ctx, MPHASE_TURN, now);
            }
            break;
        case MPHASE_TURN:
            if (t >= NAV_EVADE_TURN_MIN_MS && front_clear(in)) {
                enter_phase(ctx, MPHASE_SETTLE_FWD, now);
                set_brake(out);
            } else if (t >= NAV_EVADE_TURN_MAX_MS) {
                enter_phase(ctx, MPHASE_SETTLE_BACK, now);
                set_brake(out);
            } else {
                out->angular_rads = (float)ctx->turn_dir * NAV_TURN_RAD_S;
            }
            break;
        case MPHASE_SETTLE_FWD:
        default:
            set_brake(out);
            if (t >= NAV_SETTLE_MS) {
                enter_state(ctx, NAV_STATE_FORWARD, now);
                out->brake = false;
                out->linear_mms = NAV_FWD_SPEED_MMS;
            }
            break;
    }
}

/* Recuperacion del borde: frenar -> pausa -> retroceder -> pausa -> girar
 * alejandose -> pausa -> FORWARD. Si al terminar el giro sigue viendo borde,
 * vuelve a retroceder (con pausa previa). */
static void cliff_step(NavContext_t* ctx, const NavInput_t* in, NavOutput_t* out)
{
    const uint32_t now = in->now_ms;
    const uint32_t t = elapsed(now, ctx->phase_enter_ms);

    switch (ctx->phase) {
        case MPHASE_BRAKE:
            set_brake(out);
            if (t >= NAV_BRAKE_MS) enter_phase(ctx, MPHASE_SETTLE_BACK, now);
            break;
        case MPHASE_SETTLE_BACK:
            set_brake(out);
            if (t >= NAV_SETTLE_MS) enter_phase(ctx, MPHASE_BACKUP, now);
            break;
        case MPHASE_BACKUP:
            out->linear_mms = NAV_BACK_SPEED_MMS;
            if (t >= NAV_BACKUP_MS) enter_phase(ctx, MPHASE_SETTLE_TURN, now);
            break;
        case MPHASE_SETTLE_TURN:
            set_brake(out);
            if (t >= NAV_SETTLE_MS) enter_phase(ctx, MPHASE_TURN, now);
            break;
        case MPHASE_TURN:
            if (t >= NAV_CLIFF_TURN_MS) {
                set_brake(out);
                if (!in->cliff_left && !in->cliff_right) {
                    enter_phase(ctx, MPHASE_SETTLE_FWD, now);
                } else {
                    /* Sigue viendo borde: retroceder otra vez */
                    cliff_repeat(ctx, out, now);
                }
            } else {
                out->angular_rads = (float)ctx->turn_dir * NAV_TURN_RAD_S;
            }
            break;
        case MPHASE_SETTLE_FWD:
        default:
            set_brake(out);
            if (t >= NAV_SETTLE_MS) {
                enter_state(ctx, NAV_STATE_FORWARD, now);
                out->brake = false;
                out->linear_mms = NAV_FWD_SPEED_MMS;
            }
            break;
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
        ctx->corner_enter_ms = in->now_ms;
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
    ctx->corner_enter_ms = 0;
    ctx->last_sensor_ms = 0;
    ctx->cliff_hits = 0;
    ctx->cliff_streak = 0;
    ctx->fault_latched = false;
}

NavOutput_t Nav_Step(NavContext_t* ctx, const NavInput_t* in)
{
    NavOutput_t out = { NAV_STATE_IDLE, 0.0f, 0.0f, false, 0u, false };
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

    /* --- Confirmacion del borde: cuenta lecturas nuevas seguidas --- */
    if (in->sensor_ms != ctx->last_sensor_ms) {
        ctx->last_sensor_ms = in->sensor_ms;
        if (in->cliff_left || in->cliff_right) {
            if (ctx->cliff_hits < UINT8_MAX) ctx->cliff_hits++;
        } else {
            ctx->cliff_hits = 0;
        }
    }

    /* --- Comandos --- */
    if (in->command == NAV_CMD_STOP) {
        enter_state(ctx, NAV_STATE_IDLE, now);
        ctx->fault_latched = false;
        ctx->cliff_streak = 0;
    } else if (in->command == NAV_CMD_START && ctx->state == NAV_STATE_IDLE) {
        enter_state(ctx, NAV_STATE_FORWARD, now);
    }

    /* --- Falla resuelta: vuelve a andar solo (salvo el bucle de retroceso) --- */
    if (ctx->state == NAV_STATE_FAULT && !ctx->fault_latched && !fault_condition(in)) {
        enter_state(ctx, NAV_STATE_FORWARD, now);
    }

    /* --- Fallas: tienen prioridad sobre cualquier maniobra --- */
    if (ctx->state != NAV_STATE_IDLE && ctx->state != NAV_STATE_FAULT && fault_condition(in)) {
        enter_state(ctx, NAV_STATE_FAULT, now);
        out.events |= NAV_EVT_FAULT;
    }

    /* --- Precipicio: prioridad sobre el obstaculo --- */
    if ((ctx->state == NAV_STATE_FORWARD || ctx->state == NAV_STATE_AVOID_OBSTACLE)
        && (in->cliff_left || in->cliff_right) && ctx->cliff_hits >= NAV_CLIFF_CONFIRM
        && !cliff_count(ctx, &out, now)) {
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
            /* Avanzo lo suficiente: ya no es un bucle de retroceso */
            if (elapsed(now, ctx->state_enter_ms) >= NAV_CLIFF_LOOP_RESET_MS) {
                ctx->cliff_streak = 0;
            }
            if (ctx->corner && elapsed(now, ctx->corner_enter_ms) >= NAV_CORNER_MAX_MS) {
                /* Esquina que no termina: rutina evasiva */
                enter_state(ctx, NAV_STATE_AVOID_OBSTACLE, now);
                ctx->corner = false;
                set_brake(&out);
            } else if (front_blocked(in)) {
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
            cliff_step(ctx, in, &out);
            break;

        case NAV_STATE_FAULT:
        default:
            set_brake(&out);
            break;
    }

    out.state = ctx->state;
    out.fault_latched = ctx->fault_latched;
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
