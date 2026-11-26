/*
 * mid_nav.c
 *
 *  Created on: Nov 2, 2026
 *      Author: Grupo 2
 *
 *  Maquina de estados de la navegacion autonoma (sin HAL). Recorrido por tramos:
 *
 *  IDLE           -> SCAN            : comando START (mide antes de moverse)
 *  FORWARD        -> SCAN            : frente a menos de NAV_OBSTACLE_MM, lateral a NAV_SIDE_STOP_MM o menos, o termino el tramo (NAV_STEP_MS)
 *  SCAN           -> FORWARD         : frente con NAV_CLEAR_MM libres
 *  SCAN (gira 90 grados en el lugar) -> FORWARD : sigue por el lateral que tenia NAV_SIDE_FREE_MM libres
 *  SCAN (retrocede)        -> SCAN    : ningun lado libre, vuelve a medir
 *  SCAN           -> FAULT           : NAV_SCAN_BACKUP_MAX retrocesos o NAV_SCAN_STUCK_MAX maniobras sin avanzar
 *  FORWARD/SCAN   -> CLIFF_RECOVER   : algun TCRT5000 ve precipicio en NAV_CLIFF_CONFIRM lecturas seguidas
 *  CLIFF_RECOVER  -> SCAN            : retrocede, gira 90 grados en el lugar y ambos TCRT5000 ven suelo
 *  CLIFF_RECOVER  -> FAULT           : NAV_CLIFF_LOOP_MAX retrocesos seguidos sin avanzar (bucle)
 *  activo         -> FAULT           : frontal sin eco N veces, foto vieja o sin MPU6050
 *  cualquiera     -> IDLE            : comando STOP
 *
 *  Los giros terminan por angulo (rumbo del MPU6050), con un tope de tiempo
 *  de seguridad. No hay giros "buscando" espacio libre: la decision se toma
 *  una sola vez, con el robot quieto.
 */

#include "mid_nav.h"
#include <stddef.h>
#include <string.h>

/* Sub-fases de CLIFF_RECOVER:
 * frenar -> pausa -> retroceder -> pausa -> girar -> pausa -> SCAN */
enum {
    MPHASE_BRAKE = 0,
    MPHASE_SETTLE_BACK,   // pausa antes de retroceder
    MPHASE_BACKUP,
    MPHASE_SETTLE_TURN,   // pausa antes de girar
    MPHASE_TURN,
    MPHASE_SETTLE_FWD     // pausa antes de volver a medir
};

/* Sub-fases de SCAN: quieto -> medir -> (girar o retroceder -> quieto ...) */
enum {
    SPHASE_SETTLE = 0,
    SPHASE_COLLECT,
    SPHASE_TURN,
    SPHASE_BACKUP,
    SPHASE_TURN_SETTLE,  // pausa tras el giro de 90 grados, luego avanza por ese lado
    SPHASE_CLOSE_BACKUP, // frente pegado: retroceso corto antes de girar
    SPHASE_CLOSE_SETTLE  // pausa tras ese retroceso, luego gira
};

/* Indices de NavContext_t.scan_samples */
enum { SCAN_FRONT = 0, SCAN_LEFT, SCAN_RIGHT, SCAN_SENSORS };

static void enter_state(NavContext_t* ctx, NavState_t s, uint32_t now)
{
    ctx->state = s;
    ctx->state_enter_ms = now;
    if (s != NAV_STATE_SCAN) {
        ctx->close_backup_done = false;   /* termino la maniobra */
    }
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

/* Lectura a NAV_SIDE_STOP_MM o menos (TOO_CLOSE incluido) */
static bool range_close(NavRange_t r, uint32_t mm)
{
    return (r == NAV_RANGE_TOO_CLOSE) || ((r == NAV_RANGE_OK) && (mm <= NAV_SIDE_STOP_MM));
}

/* Cuenta lecturas propias seguidas que cumplen la condicion. NO_ECHO en la
 * foto = no fue el turno de ese sensor: no suma ni reinicia. */
static void count_hit(uint8_t* hits, NavRange_t r, bool cond)
{
    if (r == NAV_RANGE_NO_ECHO) {
        return;
    }
    if (!cond) {
        *hits = 0;
    } else if (*hits < UINT8_MAX) {
        (*hits)++;
    }
}

/* Lateral bloqueado: NAV_OBSTACLE_CONFIRM lecturas seguidas a NAV_SIDE_STOP_MM
 * o menos, o NAV_SIDE_BAD_MAX lecturas seguidas sin eco o demasiado cerca */
static bool left_blocked(const NavContext_t* ctx, const NavInput_t* in)
{
    return ctx->left_close_hits >= NAV_OBSTACLE_CONFIRM || in->left_bad_count >= NAV_SIDE_BAD_MAX;
}

static bool right_blocked(const NavContext_t* ctx, const NavInput_t* in)
{
    return ctx->right_close_hits >= NAV_OBSTACLE_CONFIRM || in->right_bad_count >= NAV_SIDE_BAD_MAX;
}

/* Frente pegado confirmado (para cortar un giro) */
static bool front_too_close(const NavContext_t* ctx)
{
    return ctx->front_close_hits >= NAV_OBSTACLE_CONFIRM;
}

/* true si el tramo recto llego a su tope de tiempo (NAV_STEP_MS = 0: sin tope) */
static bool step_done(const NavContext_t* ctx, uint32_t now)
{
#if NAV_STEP_MS > 0
    return elapsed(now, ctx->state_enter_ms) >= NAV_STEP_MS;
#else
    (void)ctx;
    (void)now;
    return false;
#endif
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

/* Robot atrapado: FAULT frenado que no se recupera solo */
static void latch_fault(NavContext_t* ctx, NavOutput_t* out, uint32_t now)
{
    enter_state(ctx, NAV_STATE_FAULT, now);
    ctx->fault_latched = true;
    out->events |= NAV_EVT_FAULT;
    set_brake(out);
}

/* Cuenta un retroceso del borde. Con NAV_CLIFF_LOOP_MAX seguidos sin haber
 * avanzado, el robot esta atrapado: FAULT frenado en vez de seguir.
 * Devuelve true si entro en FAULT. */
static bool cliff_count(NavContext_t* ctx, NavOutput_t* out, uint32_t now)
{
    if (ctx->cliff_streak < UINT8_MAX) {
        ctx->cliff_streak++;
    }
    if (ctx->cliff_streak >= NAV_CLIFF_LOOP_MAX) {
        latch_fault(ctx, out, now);
        return true;
    }
    return false;
}

/* Estado con el que se empieza a recorrer: siempre midiendo */
static NavState_t run_state(void)
{
    return NAV_STATE_SCAN;
}

/* Empieza a recorrer (START o falla resuelta): el rumbo actual pasa a ser "recto" */
static void begin_run(NavContext_t* ctx, const NavInput_t* in, uint32_t now)
{
    enter_state(ctx, run_state(), now);
    ctx->heading_target_mdeg = in->heading_mdeg;
}

/* Empieza un giro: guarda el rumbo de partida (el robot esta quieto) */
static void begin_turn(NavContext_t* ctx, const NavInput_t* in, uint8_t phase,
                       int8_t dir, uint32_t now)
{
    ctx->turn_dir = dir;
    ctx->turn_start_mdeg = in->heading_mdeg;
    enter_phase(ctx, phase, now);
}

/* true si el giro termino: por angulo (heading + = izquierda, turn_dir +1 =
 * izquierda) o por el tope de tiempo de seguridad */
static bool turn_done(const NavContext_t* ctx, const NavInput_t* in, uint32_t t_ms)
{
    if (t_ms >= NAV_TURN_TIMEOUT_MS) {
        return true;
    }
    const int32_t turned = (in->heading_mdeg - ctx->turn_start_mdeg) * (int32_t)ctx->turn_dir;
    return turned >= (NAV_TURN_ANGLE_MDEG - NAV_TURN_COAST_MDEG);
}

/* Guarda una lectura ordenada. Sin eco no aporta: task_sensors publica un
 * solo ultrasonido valido por foto y marca los demas como NO_ECHO. */
static void scan_add(NavContext_t* ctx, uint8_t sensor, NavRange_t r, uint32_t mm)
{
    if (r == NAV_RANGE_NO_ECHO || ctx->scan_n[sensor] >= NAV_SCAN_SAMPLES) {
        return;
    }
    const uint16_t value = (uint16_t)free_space(r, mm);
    uint8_t k = ctx->scan_n[sensor]++;
    for (; k > 0u && ctx->scan_samples[sensor][k - 1u] > value; k--) {
        ctx->scan_samples[sensor][k] = ctx->scan_samples[sensor][k - 1u];
    }
    ctx->scan_samples[sensor][k] = value;
}

static bool scan_complete(const NavContext_t* ctx)
{
    for (uint8_t i = 0; i < SCAN_SENSORS; i++) {
        if (ctx->scan_n[i] < NAV_SCAN_SAMPLES) {
            return false;
        }
    }
    return true;
}

/* Mediana del espacio libre. Sin lecturas, el frente cuenta como bloqueado
 * (no avanza a ciegas) y los lados siguen NAV_NO_ECHO_AS_UNKNOWN. */
static uint32_t scan_median(const NavContext_t* ctx, uint8_t sensor)
{
    const uint8_t n = ctx->scan_n[sensor];
    if (n == 0u) {
        return (sensor == SCAN_FRONT) ? 0u : free_space(NAV_RANGE_NO_ECHO, 0u);
    }
    return ctx->scan_samples[sensor][(n - 1u) / 2u];
}

/* Correccion de rumbo en el tramo recto: proporcional al desvio respecto del
 * rumbo de referencia (+ = girar a la izquierda). */
static float heading_hold(const NavContext_t* ctx, const NavInput_t* in)
{
    if (!NAV_HEADING_HOLD || !in->imu_ok) {
        return 0.0f;
    }
    const int32_t err_mdeg = ctx->heading_target_mdeg - in->heading_mdeg;
    if (err_mdeg > -NAV_HEADING_DEADBAND_MDEG && err_mdeg < NAV_HEADING_DEADBAND_MDEG) {
        return 0.0f;
    }
    float w = NAV_HEADING_KP * (float)err_mdeg * (0.0174533f / 1000.0f);
    if (w > NAV_HEADING_MAX_RAD_S) {
        w = NAV_HEADING_MAX_RAD_S;
    }
    if (w < -NAV_HEADING_MAX_RAD_S) {
        w = -NAV_HEADING_MAX_RAD_S;
    }
    return w;
}

/* Decide una sola vez con las medianas: frente libre -> tramo nuevo (sin
 * cambiar el rumbo de referencia); si no, girar 90 grados a la derecha si
 * tiene NAV_SIDE_FREE_MM libres, si no a la izquierda; todo bloqueado ->
 * atras, hasta NAV_SCAN_BACKUP_MAX veces seguidas (luego FAULT). */
static void scan_decide(NavContext_t* ctx, const NavInput_t* in, NavOutput_t* out, uint32_t now)
{
    const uint32_t front = scan_median(ctx, SCAN_FRONT);
    const uint32_t left = scan_median(ctx, SCAN_LEFT);
    const uint32_t right = scan_median(ctx, SCAN_RIGHT);

    if (front >= NAV_CLEAR_MM) {
        ctx->backup_streak = 0;
        enter_state(ctx, NAV_STATE_FORWARD, now);
        return;
    }
    if (ctx->scan_streak < UINT8_MAX) {
        ctx->scan_streak++;
    }
    if (ctx->scan_streak >= NAV_SCAN_STUCK_MAX) {
        latch_fault(ctx, out, now);
    } else if (right >= NAV_SIDE_FREE_MM || left >= NAV_SIDE_FREE_MM) {
        const int8_t dir = (right >= NAV_SIDE_FREE_MM) ? -1 : 1;   /* prefiere la derecha */
        ctx->backup_streak = 0;
        if (front <= NAV_FRONT_BACKUP_MM && !ctx->close_backup_done) {
            /* Pegado a la pared: sin lugar para girar en el lugar.
             * Retrocede un poco UNA vez y despues gira hacia ese lado. */
            ctx->close_backup_done = true;
            ctx->turn_dir = dir;
            enter_phase(ctx, SPHASE_CLOSE_BACKUP, now);
        } else {
            begin_turn(ctx, in, SPHASE_TURN, dir, now);
        }
    } else if (ctx->backup_streak >= NAV_SCAN_BACKUP_MAX) {
        latch_fault(ctx, out, now);                  /* sin paso tras los retrocesos */
    } else {
        ctx->backup_streak++;
        enter_phase(ctx, SPHASE_BACKUP, now);
    }
}

/* Escaneo: frenar y esperar quieto -> medir -> decidir. Tras retroceder
 * vuelve a medir antes de moverse otra vez. */
static void scan_step(NavContext_t* ctx, const NavInput_t* in, NavOutput_t* out, bool fresh)
{
    const uint32_t now = in->now_ms;
    const uint32_t t = elapsed(now, ctx->phase_enter_ms);

    switch (ctx->phase) {
        case SPHASE_SETTLE:
            set_brake(out);
            if (t >= (NAV_BRAKE_MS + NAV_SETTLE_MS)) {
                memset(ctx->scan_n, 0, sizeof(ctx->scan_n));
                enter_phase(ctx, SPHASE_COLLECT, now);
            }
            break;
        case SPHASE_COLLECT:
            set_brake(out);
            if (fresh) {
                scan_add(ctx, SCAN_FRONT, in->front, in->front_mm);
                scan_add(ctx, SCAN_LEFT, in->left, in->left_mm);
                scan_add(ctx, SCAN_RIGHT, in->right, in->right_mm);
            }
            if (scan_complete(ctx) || t >= NAV_SCAN_MAX_MS) {
                scan_decide(ctx, in, out, now);
            }
            break;
        case SPHASE_TURN:
            /* Tras el retroceso corto no se corta por el frente: gira y sale
             * (si no, volvia a medir, retroceder y girar sin fin) */
            if ((front_too_close(ctx) && !ctx->close_backup_done)
                || (ctx->turn_dir > 0 && left_blocked(ctx, in))
                || (ctx->turn_dir < 0 && right_blocked(ctx, in))) {
                /* Algo pegado mientras gira: cortar y volver a medir */
                set_brake(out);
                enter_phase(ctx, SPHASE_SETTLE, now);
            } else if (turn_done(ctx, in, t)) {
                set_brake(out);
                enter_phase(ctx, SPHASE_TURN_SETTLE, now);
            } else {
                /* En el lugar: ruedas en sentidos opuestos */
                out->angular_rads = (float)ctx->turn_dir * NAV_TURN_SPOT_RAD_S;
            }
            break;
        case SPHASE_TURN_SETTLE:
            /* Ya se midio que ese lado tiene NAV_SIDE_FREE_MM libres: tras
             * quedar quieto, sigue de largo por ahi con el rumbo nuevo como
             * "recto". Si el frente nuevo esta cerca, FORWARD vuelve a frenar. */
            set_brake(out);
            if (t >= NAV_SETTLE_MS) {
                enter_state(ctx, NAV_STATE_FORWARD, now);
                ctx->heading_target_mdeg = in->heading_mdeg;
                out->brake = false;
                out->linear_mms = NAV_FWD_SPEED_MMS;
            }
            break;
        case SPHASE_CLOSE_BACKUP:
            if (t >= NAV_SCAN_CLOSE_BACK_MS) {
                set_brake(out);
                enter_phase(ctx, SPHASE_CLOSE_SETTLE, now);
            } else {
                out->linear_mms = NAV_BACK_SPEED_MMS;
            }
            break;
        case SPHASE_CLOSE_SETTLE:
            set_brake(out);
            if (t >= NAV_SETTLE_MS) {
                ctx->front_close_hits = 0;   /* la lectura vieja era de antes de retroceder */
                begin_turn(ctx, in, SPHASE_TURN, ctx->turn_dir, now);
            }
            break;
        case SPHASE_BACKUP:
        default:
            if (t >= NAV_SCAN_BACK_MS) {
                set_brake(out);
                enter_phase(ctx, SPHASE_SETTLE, now);
            } else {
                out->linear_mms = NAV_BACK_SPEED_MMS;
            }
            break;
    }
}

/* Repite el retroceso del borde (si no es un bucle) */
static void cliff_repeat(NavContext_t* ctx, NavOutput_t* out, uint32_t now)
{
    if (!cliff_count(ctx, out, now)) {
        enter_phase(ctx, MPHASE_SETTLE_BACK, now);
    }
}

/* Recuperacion del borde: frenar -> pausa -> retroceder lo minimo -> pausa ->
 * girar 90 grados en el lugar alejandose -> pausa -> SCAN. Gira en el lugar
 * y no en pivote: pivotando, la rueda externa avanzaria hacia el borde.
 * Si al terminar el giro sigue viendo borde, vuelve a retroceder. */
static void cliff_step(NavContext_t* ctx, const NavInput_t* in, NavOutput_t* out)
{
    const uint32_t now = in->now_ms;
    const uint32_t t = elapsed(now, ctx->phase_enter_ms);

    switch (ctx->phase) {
        case MPHASE_BRAKE:
            set_brake(out);
            if (t >= NAV_BRAKE_MS) {
                enter_phase(ctx, MPHASE_SETTLE_BACK, now);
            }
            break;
        case MPHASE_SETTLE_BACK:
            set_brake(out);
            if (t >= NAV_SETTLE_MS) {
                enter_phase(ctx, MPHASE_BACKUP, now);
            }
            break;
        case MPHASE_BACKUP:
            out->linear_mms = NAV_BACK_SPEED_MMS;
            if (t >= NAV_CLIFF_BACKUP_MS) {
                enter_phase(ctx, MPHASE_SETTLE_TURN, now);
            }
            break;
        case MPHASE_SETTLE_TURN:
            set_brake(out);
            if (t >= NAV_SETTLE_MS) {
                begin_turn(ctx, in, MPHASE_TURN, ctx->turn_dir, now);
            }
            break;
        case MPHASE_TURN:
            if (turn_done(ctx, in, t)) {
                set_brake(out);
                if (!in->cliff_left && !in->cliff_right) {
                    enter_phase(ctx, MPHASE_SETTLE_FWD, now);
                } else {
                    /* Sigue viendo borde: retroceder otra vez */
                    cliff_repeat(ctx, out, now);
                }
            } else {
                out->angular_rads = (float)ctx->turn_dir * NAV_TURN_SPOT_RAD_S;
            }
            break;
        case MPHASE_SETTLE_FWD:
        default:
            set_brake(out);
            if (t >= NAV_SETTLE_MS) {
                enter_state(ctx, run_state(), now);
                ctx->heading_target_mdeg = in->heading_mdeg;
            }
            break;
    }
}

void Nav_Init(NavContext_t* ctx)
{
    if (!ctx) {
        return;
    }
    ctx->state = NAV_STATE_IDLE;
    ctx->state_enter_ms = 0;
    ctx->phase = 0;
    ctx->phase_enter_ms = 0;
    ctx->turn_dir = 1;
    ctx->turn_start_mdeg = 0;
    ctx->last_alert_ms = 0;
    ctx->alert_armed = false;
    ctx->last_sensor_ms = 0;
    ctx->cliff_hits = 0;
    ctx->cliff_streak = 0;
    ctx->fault_latched = false;
    memset(ctx->scan_n, 0, sizeof(ctx->scan_n));
    ctx->scan_streak = 0;
    ctx->backup_streak = 0;
    ctx->heading_target_mdeg = 0;
    ctx->close_backup_done = false;
    ctx->front_block_hits = 0;
    ctx->front_close_hits = 0;
    ctx->left_close_hits = 0;
    ctx->right_close_hits = 0;
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
    if (gz < 0) {
        gz = -gz;
    }
    if (in->imu_ok && !in->snapshot_stale && gz > NAV_GYRO_ALERT_DPS) {
        if (!ctx->alert_armed || elapsed(now, ctx->last_alert_ms) >= NAV_ALERT_HOLDOFF_MS) {
            out.events |= NAV_EVT_ALERT;
            ctx->last_alert_ms = now;
            ctx->alert_armed = true;
        }
    }

    /* --- Confirmacion del borde: cuenta lecturas nuevas seguidas --- */
    const bool fresh = (in->sensor_ms != ctx->last_sensor_ms);
    if (fresh) {
        ctx->last_sensor_ms = in->sensor_ms;
        if (in->cliff_left || in->cliff_right) {
            if (ctx->cliff_hits < UINT8_MAX) {
                ctx->cliff_hits++;
            }
        } else {
            ctx->cliff_hits = 0;
        }
        /* Filtro de obstaculos: lecturas propias seguidas */
        count_hit(&ctx->front_block_hits, in->front, front_blocked(in));
        count_hit(&ctx->front_close_hits, in->front, range_close(in->front, in->front_mm));
        count_hit(&ctx->left_close_hits, in->left, range_close(in->left, in->left_mm));
        count_hit(&ctx->right_close_hits, in->right, range_close(in->right, in->right_mm));
    }

    /* --- Comandos --- */
    if (in->command == NAV_CMD_STOP) {
        enter_state(ctx, NAV_STATE_IDLE, now);
        ctx->fault_latched = false;
        ctx->cliff_streak = 0;
        ctx->scan_streak = 0;
        ctx->backup_streak = 0;
    } else if (in->command == NAV_CMD_START && ctx->state == NAV_STATE_IDLE) {
        begin_run(ctx, in, now);
    }

    /* --- Falla resuelta: vuelve a andar solo (salvo el bucle de retroceso) --- */
    if (ctx->state == NAV_STATE_FAULT && !ctx->fault_latched && !fault_condition(in)) {
        begin_run(ctx, in, now);
    }

    /* --- Fallas: tienen prioridad sobre cualquier maniobra --- */
    if (ctx->state != NAV_STATE_IDLE && ctx->state != NAV_STATE_FAULT && fault_condition(in)) {
        enter_state(ctx, NAV_STATE_FAULT, now);
        out.events |= NAV_EVT_FAULT;
    }

    /* --- Precipicio: prioridad sobre el obstaculo --- */
    if ((ctx->state == NAV_STATE_FORWARD || ctx->state == NAV_STATE_SCAN)
        && (in->cliff_left || in->cliff_right) && ctx->cliff_hits >= NAV_CLIFF_CONFIRM
        && !cliff_count(ctx, &out, now)) {
        enter_state(ctx, NAV_STATE_CLIFF_RECOVER, now);
        /* Girar alejandose del lado que vio el borde; con los dos, a la derecha */
        ctx->turn_dir = in->cliff_left ? -1 : 1;
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
                ctx->scan_streak = 0;
                ctx->backup_streak = 0;
            }
            /* Tramo recto: frena si aparece algo de frente, un lateral a
             * NAV_SIDE_STOP_MM o menos, o al cumplir el tope */
            if (ctx->front_block_hits >= NAV_OBSTACLE_CONFIRM
                || left_blocked(ctx, in) || right_blocked(ctx, in)
                || step_done(ctx, now)) {
                enter_state(ctx, NAV_STATE_SCAN, now);
                set_brake(&out);
            } else {
                out.linear_mms = NAV_FWD_SPEED_MMS;
                out.angular_rads = heading_hold(ctx, in);
            }
            break;

        case NAV_STATE_CLIFF_RECOVER:
            cliff_step(ctx, in, &out);
            break;

        case NAV_STATE_SCAN:
            scan_step(ctx, in, &out, fresh);
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
        case NAV_STATE_CLIFF_RECOVER:  return "CLIFF";
        case NAV_STATE_FAULT:          return "FAULT";
        case NAV_STATE_SCAN:           return "SCAN";
        default:                       return "?";
    }
}
