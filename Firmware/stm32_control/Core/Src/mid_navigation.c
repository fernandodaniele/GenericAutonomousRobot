#include "mid_navigation.h"
#include "task.h"
#include "task_control_robot.h"
#include "task_motor_control.h"
#include "task_sensores.h"
#include "task_safety.h"
#include "mid_motion.h"
#include "mid_odometry.h"
#include "odometry_config.h"
#include "mid_local_exploration.h"
#include <limits.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static struct {
    NavigationState_t state;
    bool active;
    uint8_t profile;
    bool motion_hold;
    NavigationState_t settle_target;
    bool emission_ready;
    uint32_t session;
    TickType_t started_tick;
    TickType_t reverse_started_tick;
    TickType_t decision_tick;
    NavigationFinishReason_t finish_reason;
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
    TickType_t turn_started_tick;
    bool reverse_applied;
    bool turn_used;
#endif
#if NAV_AUTO2_ENABLED
    TickType_t phase_started_tick;
    bool phase_complete;
    bool reverse_completed;
#endif
} navigation = { .state = NAV_DETENIDO, .profile=NAV_AUTO2_ENABLED ? 2U : NAV_AUTO1_ODOMETRY ? 1U : 0U };

bool Navigation_TurnEnabled(void) {
#if NAV_AUTO2_ENABLED
    return navigation.profile == NAV_PROFILE_TURN || navigation.profile == NAV_PROFILE_EVASION || navigation.profile == NAV_PROFILE_CONTINUOUS || navigation.profile==NAV_PROFILE_EXPLORE || NAV_EXPERIMENTAL_TURN_ENABLED;
#else
    return NAV_EXPERIMENTAL_TURN_ENABLED;
#endif
}
static uint32_t Navigation_SessionMaxMs(void) {
    return navigation.profile==NAV_PROFILE_EXPLORE ? UINT32_MAX : navigation.profile == NAV_PROFILE_CONTINUOUS ? NAV_AUTO4_SESSION_MAX_MS :
        navigation.profile == NAV_PROFILE_TURN ? NAV_AUTO3_ISOLATED_MAX_MS :
        navigation.profile == NAV_PROFILE_EVASION ? NAV_AUTO3_SESSION_MAX_MS : NAV_SESSION_MAX_MS;
}
static uint32_t Navigation_TurnMaxMs(void) {
    return navigation.profile >= NAV_PROFILE_TURN ? NAV_AUTO3_TURN_MAX_MS : NAV_TURN_MAX_MS;
}
#if NAV_AUTO2_ENABLED
static struct {
    LocalEvidence evidence;
    uint32_t key,challenge,segments,turns,retreats,evaluated,repeated,critical_faults;
    TickType_t link_tick,link_issued_tick,yaw_tick,changed_front_tick,changed_rear_tick;
    float scan_deg,last_yaw,goal_yaw,turn_target_deg;
    bool yaw_ready,right,ready,new_information,retreated;
    uint32_t link_query,link_ack;bool link_issued,link_ready,stop_received;
    uint8_t action,block,blocked_reason,inhibited;
    float failure_x,failure_y,failure_yaw;
    uint32_t front_sequence;TickType_t front_tick;float front_yaw,front_distance;uint8_t front_confirmations;bool front_seen;
    int selected;
    uint16_t blocked_front,blocked_rear,change_front,change_rear;
    uint8_t changes;
} explore;
static void Navigation_ExploreYaw(const OdometrySnapshot *o){
    if(!isfinite(o->theta_rad))return;
    if(explore.yaw_ready && ((TickType_t)(o->tick-explore.yaw_tick)==0U || (TickType_t)(o->tick-explore.yaw_tick)>=(TickType_t)0x80000000U))return;
    if(explore.yaw_ready && navigation.state==NAV_GIRAR_ACOTADO)
        explore.scan_deg=fminf(200.0f,explore.scan_deg+fabsf(Local_Difference(o->theta_rad-explore.last_yaw))*57.2957795131f);
    explore.last_yaw=o->theta_rad;explore.yaw_tick=o->tick;explore.yaw_ready=true;
}
static bool Navigation_IsExplore(void){return navigation.profile==NAV_PROFILE_EXPLORE;}
static bool Navigation_SessionLive(TickType_t now){
    return Navigation_IsExplore() ? explore.key!=0U :
        (TickType_t)(now-navigation.started_tick)<pdMS_TO_TICKS(Navigation_SessionMaxMs());
}
static bool Navigation_LinkLive(TickType_t now){
    return !Navigation_IsExplore() || (TickType_t)(now-explore.link_tick)<pdMS_TO_TICKS(LOCAL_LINK_TIMEOUT_MS);
}
static bool Navigation_TurnRight(void){return Navigation_IsExplore()?explore.right:!NAV_TURN_LEFT;}
#endif
void Navigation_TurnTargets(int32_t *left,int32_t *right){
    taskENTER_CRITICAL();
#if NAV_AUTO2_ENABLED
    bool clockwise=Navigation_TurnRight();
#else
    bool clockwise=!NAV_TURN_LEFT;
#endif
    *left=clockwise?MOTION_TURN_CPS:-MOTION_TURN_CPS;*right=-*left;
    taskEXIT_CRITICAL();
}
static NavigationDiagnostics_t navigation_diagnostics;
static TickType_t diagnostic_phase_tick;
static bool diagnostic_terminal;
/* Una copia de respuesta, propiedad exclusiva de TaskComms. */
static NavigationDiagnostics_t diagnostic_reply;
static uint32_t diagnostic_reply_query;
static TickType_t diagnostic_reply_tick;

#if NAV_AUTO2_ENABLED
static bool Navigation_IsContinuous(void) { return navigation.profile==NAV_PROFILE_CONTINUOUS; }
static NavigationStabilization_t *Navigation_WaitDiagnostics(NavigationDiagnostics_t *d) {
    return Navigation_IsContinuous() && navigation.settle_target==NAV_AVANZAR ? &d->continuous.resume_wait :
        navigation.settle_target==NAV_GIRAR_ACOTADO ? &d->turn_wait : &d->reverse_wait;
}
static struct {
    bool ready;
    TickType_t tick;
    int64_t left,right;
    double path_mm,forward_mm;
    float origin_x,origin_y,checkpoint_net;
    struct {float x,y,theta;} recovery[NAV_AUTO4_MAX_EVASIONS];
    uint8_t recovery_count;
} navigation_continuous;
#endif

static uint32_t Navigation_DiagnosticMs(TickType_t ticks) {
    uint64_t ms = (uint64_t)ticks * 1000U / configTICK_RATE_HZ;
    return ms > UINT32_MAX ? UINT32_MAX : (uint32_t)ms;
}
/* Helpers llamados bajo el lock existente: solo copias/calculos acotados.
 * Ningun campo diagnostico se consulta para autorizar o cancelar movimiento. */
static void Navigation_DiagnosticPhase(NavigationState_t phase, TickType_t now) {
    if (!(navigation_diagnostics.flags & NAV_DIAG_SESSION)) { return; }
    if (diagnostic_terminal || navigation_diagnostics.last_phase != phase) {
        diagnostic_phase_tick = now;
    }
    diagnostic_terminal = false;
    navigation_diagnostics.state = navigation_diagnostics.last_phase = phase;
    navigation_diagnostics.reason = NAV_FINISH_NONE;
    navigation_diagnostics.observed_tick = now;
}
static void Navigation_DiagnosticSensors(const SensoresDatos_t *s, bool available, TickType_t now) {
    if (!(navigation_diagnostics.flags & NAV_DIAG_SESSION) || diagnostic_terminal) { return; }
    NavigationDiagnostics_t *d = &navigation_diagnostics;
    d->decision_tick = d->observed_tick = now;
    d->sensor_flags = available ? 1U : 0U;
    d->front_mm = d->rear_mm = 0U;
    d->snapshot_age_ms = d->front_age_ms = d->rear_age_ms = 0U;
    if (!available) { return; }
    d->snapshot_age_ms = Navigation_DiagnosticMs((TickType_t)(now - s->timestamp));
    if (s->ultrasonido_valido && isfinite(s->distancia_m) && s->distancia_m >= 0.02f && s->distancia_m <= 4.0f) {
        d->sensor_flags |= 2U;
        d->front_mm = (uint16_t)(s->distancia_m * 1000.0f + 0.5f);
        d->front_age_ms = Navigation_DiagnosticMs((TickType_t)(now - s->timestamp_ultrasonido_frontal));
    }
    if (s->ultrasonido_trasero_valido && isfinite(s->distancia_trasera_m) && s->distancia_trasera_m >= 0.02f && s->distancia_trasera_m <= 4.0f) {
        d->sensor_flags |= 4U;
        d->rear_mm = (uint16_t)(s->distancia_trasera_m * 1000.0f + 0.5f);
        d->rear_age_ms = Navigation_DiagnosticMs((TickType_t)(now - s->timestamp_ultrasonido_trasero));
    }
    if (s->hay_caida) { d->sensor_flags |= 8U; }
}

#if NAV_AUTO1_ODOMETRY
static struct {
    bool ready;
    int64_t left, right, last_left, last_right;
    TickType_t reference_tick, last_tick;
    double left_mm_per_count, right_mm_per_count, progress_mm;
#if NAV_AUTO2_ENABLED
    TickType_t progress_tick;
    double checkpoint_mm;
#endif
} navigation_odometry;

#if NAV_AUTO2_ENABLED
static struct {
    bool ready;
    TickType_t started_tick, last_tick, window_tick;
    int64_t last_left, last_right, left_min, left_max, right_min, right_max;
    uint16_t observations, window_observations;
} navigation_settle;
#endif

static double Navigation_WheelToleranceMm(double left_mm,double right_mm) {
    /* A small mechanical asymmetry is not itself invalid encoder data.
     * Only LOCAL turns with bilateral progress get the 2-mm center budget.
     * Keep the original 3-mm startup guard if either wheel has not progressed;
     * validity, signs, timestamps and the cumulative phase reference stay strict. */
    if(navigation.profile==NAV_PROFILE_EXPLORE && navigation.state==NAV_GIRAR_ACOTADO &&
       left_mm>=NAV_AUTO3_MAX_WHEEL_DIFFERENCE_MM && right_mm>=NAV_AUTO3_MAX_WHEEL_DIFFERENCE_MM)
        return LOCAL_TURN_MAX_WHEEL_DIFFERENCE_MM;
    return navigation.profile>=NAV_PROFILE_TURN && navigation.state==NAV_GIRAR_ACOTADO ?
        NAV_AUTO3_MAX_WHEEL_DIFFERENCE_MM : NAV_AUTO1_MAX_WHEEL_DIFFERENCE_MM;
}

static bool Navigation_SubtractCount(int64_t count, int64_t reference, int64_t *delta) {
    if ((reference > 0 && count < INT64_MIN + reference) ||
        (reference < 0 && count > INT64_MAX + reference)) {
        return false;
    }
    *delta = count - reference;
    return true;
}

static bool Navigation_OdometryFresh(const OdometrySnapshot *s, TickType_t now) {
    return s->state == 2U &&
        (s->calibration == ODOM_GEOMETRY_PROVISIONAL || s->calibration == ODOM_GEOMETRY_CALIBRATED) &&
        (TickType_t)(now - s->tick) <= pdMS_TO_TICKS(ODOM_MAX_INTERVAL_MS) &&
        isfinite(s->x_m) && isfinite(s->y_m) && isfinite(s->theta_rad) &&
        isfinite(s->increment_m) && isfinite(s->increment_rad) &&
        fabsf(s->x_m) <= ODOM_POSITION_LIMIT_M && fabsf(s->y_m) <= ODOM_POSITION_LIMIT_M;
}

/* Clasifica el rechazo ya decidido por las condiciones originales. No altera
 * su orden, retorno ni referencia. Las restas diagnosticas tambien se protegen. */
static void Navigation_DiagnosticOdometry(const OdometrySnapshot *s, TickType_t now, int left_sign, int right_sign) {
    if (!(navigation_diagnostics.flags & NAV_DIAG_SESSION) || diagnostic_terminal) { return; }
    NavigationDiagnostics_t *d = &navigation_diagnostics;
    d->observed_tick = now;
    d->deltas_available = navigation_odometry.ready &&
        Navigation_SubtractCount(s->left_count, navigation_odometry.left, &d->left_delta) &&
        Navigation_SubtractCount(s->right_count, navigation_odometry.right, &d->right_delta);
    if (s->state != 2U ||
        (s->calibration != ODOM_GEOMETRY_PROVISIONAL && s->calibration != ODOM_GEOMETRY_CALIBRATED)) {
        d->odometry_error = NAV_ODOM_UNAVAILABLE;
    } else if ((TickType_t)(now - s->tick) > pdMS_TO_TICKS(ODOM_MAX_INTERVAL_MS)) {
        d->odometry_error = NAV_ODOM_STALE;
    } else if (!isfinite(s->x_m) || !isfinite(s->y_m) || !isfinite(s->theta_rad) ||
               !isfinite(s->increment_m) || !isfinite(s->increment_rad)) {
        d->odometry_error = NAV_ODOM_NONFINITE;
    } else if (fabsf(s->x_m)>ODOM_POSITION_LIMIT_M || fabsf(s->y_m)>ODOM_POSITION_LIMIT_M) {
        d->odometry_error = NAV_ODOM_RANGE;
    } else if (!navigation_odometry.ready) {
        d->odometry_error = NAV_ODOM_UNAVAILABLE;
    } else if ((TickType_t)(now - s->tick) > (TickType_t)(now - navigation_odometry.last_tick) ||
        (s->tick == navigation_odometry.last_tick &&
         (s->left_count != navigation_odometry.last_left || s->right_count != navigation_odometry.last_right))) {
        d->odometry_error = NAV_ODOM_TIMESTAMP;
    } else if ((left_sign > 0 ? s->left_count < navigation_odometry.last_left : s->left_count > navigation_odometry.last_left) ||
               (right_sign > 0 ? s->right_count < navigation_odometry.last_right : s->right_count > navigation_odometry.last_right)) {
        d->odometry_error = NAV_ODOM_DIRECTION;
    } else if (!d->deltas_available) {
        d->odometry_error = NAV_ODOM_OVERFLOW;
    } else if (!isfinite(s->x_m) || !isfinite(s->y_m) || !isfinite(s->theta_rad) ||
               !isfinite(s->increment_m) || !isfinite(s->increment_rad) ||
               !(navigation_odometry.left_mm_per_count > 0) || !(navigation_odometry.right_mm_per_count > 0)) {
        d->odometry_error = NAV_ODOM_NONFINITE;
    } else {
        double l = d->left_delta * navigation_odometry.left_mm_per_count * left_sign;
        double r = d->right_delta * navigation_odometry.right_mm_per_count * right_sign;
        d->odometry_error = !isfinite(l) || !isfinite(r) ? NAV_ODOM_NONFINITE :
            l<0 || r<0 ? NAV_ODOM_DIRECTION :
            l>2.0*ODOM_POSITION_LIMIT_M*1000.0 || r>2.0*ODOM_POSITION_LIMIT_M*1000.0 ? NAV_ODOM_RANGE :
            fabs(l-r)>Navigation_WheelToleranceMm(l,r) ? NAV_ODOM_DISAGREEMENT : NAV_ODOM_RANGE;
    }
}

/* Calculo acotado. Una muestra repetida solo es coherente si conserva conteos.
 * El consumidor no necesita ejecutarse en cada periodo del productor (10 ms). */
static NavigationFinishReason_t Navigation_OdometryOutcome(
        const OdometrySnapshot *s, TickType_t now, double *progress) {
    int64_t left, right;
    int left_sign = 1, right_sign = 1;
#if NAV_AUTO2_ENABLED
    if (navigation.state == NAV_RETROCEDER_ACOTADO) {
        left_sign = right_sign = -1;
    } else if (navigation.state == NAV_GIRAR_ACOTADO) {
        left_sign = Navigation_TurnRight() ? 1 : -1;
        right_sign = -left_sign;
    }
#endif
    if (!navigation_odometry.ready || !Navigation_OdometryFresh(s, now) ||
        (TickType_t)(now - s->tick) > (TickType_t)(now - navigation_odometry.last_tick) ||
        (s->tick == navigation_odometry.last_tick &&
         (s->left_count != navigation_odometry.last_left || s->right_count != navigation_odometry.last_right)) ||
        (left_sign > 0 ? s->left_count < navigation_odometry.last_left : s->left_count > navigation_odometry.last_left) ||
        (right_sign > 0 ? s->right_count < navigation_odometry.last_right : s->right_count > navigation_odometry.last_right) ||
        !Navigation_SubtractCount(s->left_count, navigation_odometry.left, &left) ||
        !Navigation_SubtractCount(s->right_count, navigation_odometry.right, &right)) {
        Navigation_DiagnosticOdometry(s, now, left_sign, right_sign);
        return NAV_FINISH_ODOMETRY;
    }
    double left_mm = left * navigation_odometry.left_mm_per_count * left_sign;
    double right_mm = right * navigation_odometry.right_mm_per_count * right_sign;
    /* Se admiten conteos absolutos negativos, pero no desplazamiento contrario
     * al signo esperado en esta fase. La extension de coordenadas existente acota
     * saltos numericos; no supone una velocidad fisica maxima del motor. */
    if (left_mm < 0 || right_mm < 0 || !isfinite(left_mm) || !isfinite(right_mm) ||
        !(navigation_odometry.left_mm_per_count > 0) || !(navigation_odometry.right_mm_per_count > 0) ||
        left_mm > 2.0 * ODOM_POSITION_LIMIT_M * 1000.0 ||
        right_mm > 2.0 * ODOM_POSITION_LIMIT_M * 1000.0 ||
        fabs(left_mm - right_mm) > Navigation_WheelToleranceMm(left_mm,right_mm)) {
        Navigation_DiagnosticOdometry(s, now, left_sign, right_sign);
        return NAV_FINISH_ODOMETRY;
    }
    *progress = (left_mm + right_mm) * 0.5;
    if (!diagnostic_terminal && (navigation_diagnostics.flags & NAV_DIAG_SESSION)) {
        navigation_diagnostics.observed_tick = now;
        if (navigation.state == NAV_RETROCEDER_ACOTADO) {
            navigation_diagnostics.reverse_um = (int32_t)(*progress * 1000.0);
            navigation_diagnostics.flags |= NAV_DIAG_REVERSE_PROGRESS;
        } else if (navigation.state == NAV_GIRAR_ACOTADO) {
            navigation_diagnostics.turn_mdeg = (uint32_t)(*progress * 2.0 / (ODOM_TRACK_M * 1000.0) * (180000.0 / 3.14159265358979323846) + 0.5);
            navigation_diagnostics.turn_flags |= 16U;
        } else if (navigation.state == NAV_AVANZAR || navigation.state == NAV_ESPERAR_DATOS) {
            navigation_diagnostics.advance_um = (int32_t)(*progress * 1000.0);
            navigation_diagnostics.flags |= NAV_DIAG_ADVANCE_PROGRESS;
        }
    }
    double target_mm = NAV_AUTO1_TARGET_MM;
#if NAV_AUTO2_ENABLED
    if(Navigation_IsExplore())target_mm=LOCAL_SEGMENT_MM;
#endif
#if NAV_AUTO2_ENABLED
    if (navigation.state == NAV_RETROCEDER_ACOTADO) {
        target_mm = NAV_AUTO2_REVERSE_TARGET_MM;
    } else if (navigation.state == NAV_GIRAR_ACOTADO) {
        target_mm = (Navigation_IsExplore()?explore.turn_target_deg:NAV_AUTO2_TURN_TARGET_DEG) * (3.14159265358979323846 / 180.0) * ODOM_TRACK_M * 500.0;
    }
    if (*progress >= target_mm) {
        return NAV_FINISH_TARGET;
    }
    if (*progress >= navigation_odometry.checkpoint_mm + NAV_AUTO2_MIN_PROGRESS_MM) {
        navigation_odometry.checkpoint_mm = *progress;
        navigation_odometry.progress_tick = s->tick;
    }
    if ((TickType_t)(now - navigation_odometry.progress_tick) >= pdMS_TO_TICKS(NAV_AUTO2_PROGRESS_WINDOW_MS)) {
        return NAV_FINISH_NO_PROGRESS;
    }
#endif
    return *progress >= target_mm ? NAV_FINISH_TARGET : NAV_FINISH_NONE;
}

#if NAV_AUTO2_ENABLED
static uint32_t Navigation_Micrometers(double mm) {
    return mm>=UINT32_MAX/1000.0 ? UINT32_MAX : (uint32_t)(mm*1000.0);
}
/* One observation per producer tick, including coast/settle and turn travel.
 * No sum of increment_m, no reset or correction of the global pose. */
static NavigationFinishReason_t Navigation_ContinuousObserve(const OdometrySnapshot *o,TickType_t now) {
    if(!Navigation_IsContinuous()) return NAV_FINISH_NONE;
    if(!Navigation_OdometryFresh(o,now)) {
        Navigation_DiagnosticOdometry(o,now,
            navigation.state==NAV_RETROCEDER_ACOTADO || (navigation.state==NAV_GIRAR_ACOTADO && NAV_TURN_LEFT) ? -1 : 1,
            navigation.state==NAV_RETROCEDER_ACOTADO || (navigation.state==NAV_GIRAR_ACOTADO && !NAV_TURN_LEFT) ? -1 : 1);
        return NAV_FINISH_ODOMETRY;
    }
    if(!navigation_continuous.ready) {
        if((TickType_t)(now-o->tick)>=(TickType_t)(now-navigation.started_tick))return NAV_FINISH_NONE;
        navigation_continuous.ready=true;navigation_continuous.tick=o->tick;
        navigation_continuous.left=o->left_count;navigation_continuous.right=o->right_count;
        navigation_continuous.origin_x=o->x_m;navigation_continuous.origin_y=o->y_m;
        return NAV_FINISH_NONE;
    }
    int64_t dl,dr;
    if((TickType_t)(now-o->tick)>(TickType_t)(now-navigation_continuous.tick) ||
       (o->tick==navigation_continuous.tick && (o->left_count!=navigation_continuous.left || o->right_count!=navigation_continuous.right))) {
        navigation_diagnostics.odometry_error=NAV_ODOM_TIMESTAMP;return NAV_FINISH_ODOMETRY;
    }
    if(o->tick==navigation_continuous.tick)return NAV_FINISH_NONE;
    if(!Navigation_SubtractCount(o->left_count,navigation_continuous.left,&dl) ||
       !Navigation_SubtractCount(o->right_count,navigation_continuous.right,&dr)) {
        navigation_diagnostics.odometry_error=NAV_ODOM_OVERFLOW;return NAV_FINISH_ODOMETRY;
    }
    double l=(double)dl*1000.0*3.14159265358979323846*ODOM_LEFT_DIAMETER_M/ODOM_LEFT_WHEEL_CPR;
    double r=(double)dr*1000.0*3.14159265358979323846*ODOM_RIGHT_DIAMETER_M/ODOM_RIGHT_WHEEL_CPR;
    if(!isfinite(l)||!isfinite(r)||fabs(l)>2.0*ODOM_POSITION_LIMIT_M*1000.0||fabs(r)>2.0*ODOM_POSITION_LIMIT_M*1000.0) {
        navigation_diagnostics.odometry_error=NAV_ODOM_RANGE;return NAV_FINISH_ODOMETRY;
    }
    navigation_continuous.path_mm+=(fabs(l)+fabs(r))*0.5;
    if(navigation.state==NAV_AVANZAR && l>0 && r>0)navigation_continuous.forward_mm+=(l+r)*0.5;
    NavigationContinuousDiagnostics_t *d=&navigation_diagnostics.continuous;
    d->path_um=Navigation_Micrometers(navigation_continuous.path_mm);
    d->forward_um=Navigation_Micrometers(navigation_continuous.forward_mm);
    d->useful_um=Navigation_Micrometers(1000.0*hypotf(o->x_m-navigation_continuous.origin_x,o->y_m-navigation_continuous.origin_y));
    navigation_continuous.tick=o->tick;navigation_continuous.left=o->left_count;navigation_continuous.right=o->right_count;
    return navigation_continuous.path_mm>=NAV_AUTO4_MAX_PATH_MM ? NAV_FINISH_LIMIT_REACHED : NAV_FINISH_NONE;
}
static bool Navigation_RecoveryAllowed(const OdometrySnapshot *o) {
    NavigationContinuousDiagnostics_t *d=&navigation_diagnostics.continuous;
    if(d->attempts>=NAV_AUTO4_MAX_EVASIONS || navigation_continuous.recovery_count>=NAV_AUTO4_MAX_EVASIONS){d->block=6U;return false;}
    float net=d->useful_um*0.001f;
    if(d->attempts && net<navigation_continuous.checkpoint_net+NAV_AUTO4_MIN_USEFUL_MM){d->block=4U;return false;}
    for(unsigned i=0;i<navigation_continuous.recovery_count;++i) {
        float distance=1000.0f*hypotf(o->x_m-navigation_continuous.recovery[i].x,o->y_m-navigation_continuous.recovery[i].y);
        float angle=fabsf(remainderf(o->theta_rad-navigation_continuous.recovery[i].theta,6.28318530718f));
        if(distance<=NAV_AUTO4_REVISIT_MM && angle<=NAV_AUTO4_REVISIT_DEG*(3.14159265358979323846f/180.0f)){d->block=5U;return false;}
    }
    unsigned n=navigation_continuous.recovery_count++;
    navigation_continuous.recovery[n].x=o->x_m;navigation_continuous.recovery[n].y=o->y_m;
    navigation_continuous.recovery[n].theta=o->theta_rad;
    navigation_continuous.checkpoint_net=net;++d->attempts;d->block=1U;
    navigation.turn_used=false;
    return true;
}
#endif

/* Referencia coherente de la fase; nunca reinicia la odometria global. */
static void Navigation_RecordReference(const OdometrySnapshot *s) {
    navigation_odometry.left = navigation_odometry.last_left = s->left_count;
    navigation_odometry.right = navigation_odometry.last_right = s->right_count;
    navigation_odometry.reference_tick = navigation_odometry.last_tick = s->tick;
    navigation_odometry.left_mm_per_count =
        1000.0 * 3.14159265358979323846 * ODOM_LEFT_DIAMETER_M / ODOM_LEFT_WHEEL_CPR;
    navigation_odometry.right_mm_per_count =
        1000.0 * 3.14159265358979323846 * ODOM_RIGHT_DIAMETER_M / ODOM_RIGHT_WHEEL_CPR;
    if (navigation.state == NAV_GIRAR_ACOTADO) {
        NavigationStabilization_t *w=&navigation_diagnostics.turn_wait;
        w->reverse_reference_available=true;w->reverse_reference_tick=s->tick;
        w->reverse_reference_left=s->left_count;w->reverse_reference_right=s->right_count;
    }
    navigation_odometry.progress_mm = 0;
    navigation_odometry.ready = true;
#if NAV_AUTO2_ENABLED
    navigation_odometry.checkpoint_mm = 0;
    navigation_odometry.progress_tick = s->tick;
    if (navigation.state == NAV_RETROCEDER_ACOTADO) {
        navigation_diagnostics.reverse_reference_available = true;
        navigation_diagnostics.reverse_reference_tick = s->tick;
        navigation_diagnostics.reverse_reference_left = s->left_count;
        navigation_diagnostics.reverse_reference_right = s->right_count;
    }
#endif
}
#endif

/* Bajo la seccion critica de navegacion; conserva la primera causa terminal. */
static void Navigation_StopFor(NavigationFinishReason_t reason) {
    if (navigation.finish_reason == NAV_FINISH_NONE) {
        navigation.finish_reason = reason;
    }
    TaskMotorControl_Stop(); /* Transaccion existente: generacion, permiso, PI y PWM. */
}

#if NAV_AUTO2_ENABLED
static bool Navigation_DeadlineExpired(TickType_t now) {
    /* Una fase completada ya tiene permiso revocado y salidas cero en el
     * backend. Puede evaluarse en el siguiente ciclo sin prolongar actuacion. */
    return !Navigation_SessionLive(now) || !Navigation_LinkLive(now) ||
        (navigation.profile != NAV_PROFILE_TURN && (navigation.state == NAV_ESPERAR_DATOS || navigation.state == NAV_AVANZAR) &&
         (TickType_t)(now - ((Navigation_IsContinuous() || Navigation_IsExplore()) ? navigation.phase_started_tick : navigation.started_tick)) >=
             pdMS_TO_TICKS((Navigation_IsContinuous() || Navigation_IsExplore()) ? NAV_AUTO4_ADVANCE_MAX_MS : NAV_AUTO2_ADVANCE_MAX_MS)) ||
        (navigation.state == NAV_DETENER_Y_ESPERAR &&
         (TickType_t)(now - navigation.phase_started_tick) >= pdMS_TO_TICKS(NAV_SETTLE_TIMEOUT_MS)) ||
        (navigation.state == NAV_RETROCEDER_ACOTADO && !navigation.phase_complete &&
         (TickType_t)(now - navigation.phase_started_tick) >= pdMS_TO_TICKS(NAV_REVERSE_MAX_MS)) ||
        (navigation.state == NAV_GIRAR_ACOTADO && !navigation.phase_complete &&
         (TickType_t)(now - navigation.phase_started_tick) >= pdMS_TO_TICKS(Navigation_TurnMaxMs()));
}
#endif

void Navigation_Begin(TickType_t entered_tick) {
    Navigation_BeginProfile(entered_tick, NAV_AUTO2_ENABLED ? 2U : NAV_AUTO1_ODOMETRY ? 1U : 0U);
}
void Navigation_BeginProfile(TickType_t entered_tick, uint8_t profile) {
    taskENTER_CRITICAL();
    navigation.profile=profile;
    navigation.motion_hold=false;
    navigation.settle_target=NAV_RETROCEDER_ACOTADO;
    uint32_t diagnostic_id = navigation_diagnostics.session_id + 1U;
    memset(&navigation_diagnostics, 0, sizeof navigation_diagnostics);
    navigation_diagnostics.session_id = diagnostic_id;
    navigation_diagnostics.profile=profile;
    navigation_diagnostics.turn_left=NAV_TURN_LEFT;
    navigation_diagnostics.turn_target_mdeg=NAV_AUTO2_TURN_TARGET_DEG*1000U;
    navigation_diagnostics.started_tick = entered_tick;
    navigation_diagnostics.flags = NAV_DIAG_SESSION;
    diagnostic_phase_tick = entered_tick;
    Navigation_DiagnosticPhase(NAV_ESPERAR_DATOS, entered_tick);
    ++navigation.session;
    navigation.started_tick = entered_tick;
    navigation.reverse_started_tick = 0U;
    navigation.finish_reason = NAV_FINISH_NONE;
#if NAV_AUTO1_ODOMETRY
    memset(&navigation_odometry, 0, sizeof navigation_odometry);
#endif
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
    navigation.turn_started_tick = 0U;
    navigation.reverse_applied = false;
    navigation.turn_used = false;
#endif
#if NAV_AUTO2_ENABLED
    navigation.phase_started_tick = entered_tick;
    navigation.phase_complete = false;
    navigation.reverse_completed = false;
    memset(&navigation_settle, 0, sizeof navigation_settle);
    memset(&navigation_continuous,0,sizeof navigation_continuous);
#endif
#if NAV_AUTO2_ENABLED
    if(profile==NAV_PROFILE_EXPLORE)memset(&explore,0,sizeof explore);
#endif
    navigation.state = NAV_ESPERAR_DATOS;
    navigation.active = true;
    navigation.emission_ready = false;
    taskEXIT_CRITICAL();
}

void Navigation_BeginExplore(TickType_t tick,uint32_t key){
    Navigation_BeginProfile(tick,NAV_PROFILE_EXPLORE);
#if NAV_AUTO2_ENABLED
    taskENTER_CRITICAL();explore.key=key;explore.link_tick=tick;explore.selected=-1;taskEXIT_CRITICAL();
#else
    (void)key;Navigation_Cancel();
#endif
}
/* Compatibility rejection only: periodic permits no longer exist. */
bool Navigation_ExploreRenew(uint32_t key,uint32_t sequence,uint32_t permit,TickType_t issued){
    (void)key;(void)sequence;(void)permit;(void)issued;return false;
}
bool Navigation_ExploreLinkQuery(uint32_t key,uint32_t query,uint32_t acknowledged,TickType_t issued){
#if NAV_AUTO2_ENABLED
    TickType_t now=xTaskGetTickCount();taskENTER_CRITICAL();
    bool fresh=!explore.link_issued || (query-explore.link_query && query-explore.link_query<0x80000000U);
    bool ok=navigation.active && Navigation_IsExplore() && key && key==explore.key && query && fresh &&
        Navigation_LinkLive(now) && !TaskSafety_IsStopActive() && !TaskControlRobot_IsStopPending() &&
        TaskControlRobot_GetState()==ROBOT_STATE_AUTONOMOUS && (TickType_t)(now-issued)<pdMS_TO_TICKS(300U);
    if(ok){
        if(explore.link_issued && acknowledged==explore.link_query &&
           (TickType_t)(now-explore.link_issued_tick)<pdMS_TO_TICKS(LOCAL_LINK_REPLY_MS)){
            explore.link_tick=now;explore.link_ready=true;explore.link_ack=acknowledged;
        }
        explore.link_issued=true;explore.link_query=query;explore.link_issued_tick=now;
    }
    taskEXIT_CRITICAL();return ok;
#else
    (void)key;(void)query;(void)acknowledged;(void)issued;return false;
#endif
}
void Navigation_Cancel(void) {
    taskENTER_CRITICAL();
#if NAV_AUTO2_ENABLED
    if(navigation_diagnostics.profile==NAV_PROFILE_EXPLORE && TaskControlRobot_IsStopPending())explore.stop_received=true;
#endif
    if (navigation.active && navigation.finish_reason == NAV_FINISH_NONE) {
        navigation.finish_reason = TaskSafety_IsStopActive() ? NAV_FINISH_SAFETY :
            navigation.profile!=NAV_PROFILE_EXPLORE && (TickType_t)(xTaskGetTickCount() - navigation.started_tick) >= pdMS_TO_TICKS(Navigation_SessionMaxMs()) ?
            NAV_FINISH_TIMEOUT : NAV_FINISH_CANCELLED;
#if NAV_AUTO2_ENABLED
        if (!TaskSafety_IsStopActive() && TaskControlRobot_IsStopPending()) {
            navigation.finish_reason = NAV_FINISH_OPERATOR_STOP;
        }
#endif
    }
    if (navigation.active && !diagnostic_terminal && (navigation_diagnostics.flags & NAV_DIAG_SESSION)) {
        TickType_t now = xTaskGetTickCount();
#if NAV_AUTO2_ENABLED
        if (navigation.state == NAV_DETENER_Y_ESPERAR) {
            NavigationStabilization_t *w=Navigation_WaitDiagnostics(&navigation_diagnostics);
            w->settle_elapsed_ms = Navigation_DiagnosticMs((TickType_t)(now-navigation_settle.started_tick));
            if (w->settle_state == 1U) {
                w->settle_state = navigation.finish_reason == NAV_FINISH_TIMEOUT &&
                    (TickType_t)(now-navigation_settle.started_tick) >= pdMS_TO_TICKS(NAV_SETTLE_TIMEOUT_MS) ? 3U : 4U;
            }
        }
#endif
#if NAV_AUTO2_ENABLED || NAV_EXPERIMENTAL_TURN_ENABLED
        if (navigation.state==NAV_GIRAR_ACOTADO) navigation_diagnostics.turn_elapsed_ms=Navigation_DiagnosticMs((TickType_t)(now-navigation.turn_started_tick));
#endif
#if NAV_AUTO2_ENABLED
        if(navigation.profile==NAV_PROFILE_EXPLORE)explore.critical_faults|=TaskSafety_GetFaults();
#endif
        if(navigation.profile==NAV_PROFILE_CONTINUOUS) {
            navigation_diagnostics.continuous.critical_faults=TaskSafety_GetFaults();
            navigation_diagnostics.continuous.permissions=0;
        }
        navigation_diagnostics.state = NAV_DETENIDO;
        navigation_diagnostics.reason = navigation.finish_reason;
        navigation_diagnostics.observed_tick = now;
        navigation_diagnostics.elapsed_ms = Navigation_DiagnosticMs((TickType_t)(now - navigation.started_tick));
        navigation_diagnostics.phase_elapsed_ms = Navigation_DiagnosticMs((TickType_t)(now - diagnostic_phase_tick));
        diagnostic_terminal = true;
    }
    ++navigation.session;
    navigation.active = false;
    navigation.emission_ready = false;
    navigation.motion_hold=false;
    navigation.state = NAV_DETENIDO;
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
    navigation.reverse_applied = false;
#endif
    taskEXIT_CRITICAL();
}

NavigationState_t Navigation_GetState(void) {
    taskENTER_CRITICAL();
    NavigationState_t state = navigation.state;
    taskEXIT_CRITICAL();
    return state;
}

NavigationFinishReason_t Navigation_GetFinishReason(void) {
    taskENTER_CRITICAL();
    NavigationFinishReason_t reason = navigation.finish_reason;
    taskEXIT_CRITICAL();
    return reason;
}

void Navigation_GetDiagnostics(NavigationDiagnostics_t *out) {
    if (out == NULL) { return; }
    taskENTER_CRITICAL();
    *out = navigation_diagnostics;
    out->profile = navigation.profile;
    out->state = navigation.state;
    if (navigation.active) {
        TickType_t now = xTaskGetTickCount();
        if (navigation.state==NAV_GIRAR_ACOTADO) out->turn_elapsed_ms=Navigation_DiagnosticMs((TickType_t)(now-diagnostic_phase_tick));
        out->elapsed_ms = Navigation_DiagnosticMs((TickType_t)(now - navigation.started_tick));
        out->phase_elapsed_ms = Navigation_DiagnosticMs((TickType_t)(now - diagnostic_phase_tick));
#if NAV_AUTO2_ENABLED
        if (navigation.state == NAV_DETENER_Y_ESPERAR) {
            NavigationStabilization_t *w=Navigation_WaitDiagnostics(out);
            w->settle_elapsed_ms = Navigation_DiagnosticMs((TickType_t)(now-navigation_settle.started_tick));
        }
#endif
    }
    taskEXIT_CRITICAL();
}

static const char *Navigation_CountText(char text[22], int64_t value) {
    uint64_t magnitude = value < 0 ? (uint64_t)(-(value+1))+1U : (uint64_t)value;
    unsigned n=21U; text[n]='\0';
    do { text[--n]=(char)('0'+magnitude%10U); magnitude/=10U; } while (magnitude);
    if (value<0) { text[--n]='-'; }
    return text+n;
}

int Navigation_FormatDiagnostics(char *out, size_t capacity, uint32_t query) {
    if (!out || !capacity || !query) { return -1; }
    /* Propiedad de TaskComms, igual que sus otras respuestas diagnosticas. */
    NavigationDiagnostics_t *d = &diagnostic_reply;
    Navigation_GetDiagnostics(d);
    TickType_t now=xTaskGetTickCount();
    char left[22], right[22];
    int n=snprintf(out,capacity,
        "NAV,1,%lu,%u,%lu,%lu,%u,%u,%u,%u,%lu,%lu,%ld,%ld,%u,%u,%u,%lu,%lu,%lu,%u,%s,%s,%lu,%lu\n",
        (unsigned long)query,d->profile,(unsigned long)d->session_id,(unsigned long)d->started_tick,
        (unsigned)d->state,(unsigned)d->last_phase,(unsigned)d->reason,d->flags,
        (unsigned long)d->elapsed_ms,(unsigned long)d->phase_elapsed_ms,(long)d->advance_um,(long)d->reverse_um,
        d->front_mm,d->rear_mm,d->sensor_flags,(unsigned long)d->snapshot_age_ms,
        (unsigned long)d->front_age_ms,(unsigned long)d->rear_age_ms,(unsigned)d->odometry_error,
        d->deltas_available ? Navigation_CountText(left,d->left_delta) : "-",
        d->deltas_available ? Navigation_CountText(right,d->right_delta) : "-",
        (unsigned long)Navigation_DiagnosticMs((TickType_t)(now-d->decision_tick)),
        (unsigned long)Navigation_DiagnosticMs((TickType_t)(now-d->observed_tick)));
    diagnostic_reply_query = n>0 && (size_t)n<capacity && n<=255 ? query : 0U;
    diagnostic_reply_tick = now;
    return n>0 && (size_t)n<capacity && n<=255 ? n : -1;
}

int Navigation_FormatStabilization(char *out, size_t capacity, uint32_t query) {
    if (!out || !capacity || !query || diagnostic_reply_query!=query) { return -1; }
    const NavigationDiagnostics_t *d = &diagnostic_reply;
    TickType_t now=xTaskGetTickCount();
    if ((TickType_t)(now-diagnostic_reply_tick)>=pdMS_TO_TICKS(1000U)) { return -1; }
    char left[22], right[22], tick[12]="-", entry[12]="-";
    if (d->reverse_reference_available) {
        (void)snprintf(tick,sizeof tick,"%lu",(unsigned long)d->reverse_reference_tick);
    }
    if (d->reverse_entered) {
        (void)snprintf(entry,sizeof entry,"%lu",(unsigned long)d->reverse_entry_tick);
    }
    int n=snprintf(out,capacity,"NAVW,1,%lu,%lu,%lu,%u,%lu,%u,%u,%lu,%lu,%lu,%s,%s,%s,%s,%lu\n",
        (unsigned long)query,(unsigned long)d->session_id,(unsigned long)d->started_tick,d->settle_state,
        (unsigned long)d->settle_elapsed_ms,d->settle_observations,d->settle_window_observations,
        (unsigned long)d->settle_window_ms,(unsigned long)d->settle_left_span,(unsigned long)d->settle_right_span,
        tick,d->reverse_reference_available ? Navigation_CountText(left,d->reverse_reference_left) : "-",
        d->reverse_reference_available ? Navigation_CountText(right,d->reverse_reference_right) : "-",entry,
        (unsigned long)Navigation_DiagnosticMs((TickType_t)(now-diagnostic_reply_tick)));
    return n>0 && (size_t)n<capacity && n<=255 ? n : -1;
}

/* Third bounded packet from the exact NAV snapshot, never a new acquisition. */
int Navigation_FormatTurn(char *out,size_t capacity,uint32_t query) {
    if (!out || !capacity || !query || query!=diagnostic_reply_query ||
        (TickType_t)(xTaskGetTickCount()-diagnostic_reply_tick)>=pdMS_TO_TICKS(1000U)) return -1;
    const NavigationDiagnostics_t *d=&diagnostic_reply;
    const NavigationStabilization_t *w=&d->turn_wait;
    char left[22],right[22],tick[12]="-",entry[12]="-";
    if(w->reverse_reference_available) (void)snprintf(tick,sizeof tick,"%lu",(unsigned long)w->reverse_reference_tick);
    if(w->reverse_entered) (void)snprintf(entry,sizeof entry,"%lu",(unsigned long)w->reverse_entry_tick);
    int n=snprintf(out,capacity,"NAVT,1,%lu,%lu,%lu,%u,%lu,%lu,%u,%lu,%u,%u,%lu,%lu,%lu,%s,%s,%s,%s,%lu,%u,%u\n",
        (unsigned long)query,(unsigned long)d->session_id,(unsigned long)d->started_tick,d->turn_flags,
        (unsigned long)d->turn_mdeg,(unsigned long)d->turn_elapsed_ms,w->settle_state,
        (unsigned long)w->settle_elapsed_ms,w->settle_observations,w->settle_window_observations,
        (unsigned long)w->settle_window_ms,(unsigned long)w->settle_left_span,(unsigned long)w->settle_right_span,
        tick,w->reverse_reference_available?Navigation_CountText(left,w->reverse_reference_left):"-",
        w->reverse_reference_available?Navigation_CountText(right,w->reverse_reference_right):"-",entry,
        (unsigned long)Navigation_DiagnosticMs((TickType_t)(xTaskGetTickCount()-diagnostic_reply_tick)),
        (unsigned)(d->flags?d->turn_target_mdeg:NAV_AUTO2_TURN_TARGET_DEG*1000U),(unsigned)(d->flags?d->turn_left:NAV_TURN_LEFT));
    return n>0 && (size_t)n<capacity && n<=255 ? n : -1;
}

int Navigation_FormatContinuous(char *out,size_t capacity,uint32_t query) {
    if(!out || !capacity || !query || query!=diagnostic_reply_query ||
        (TickType_t)(xTaskGetTickCount()-diagnostic_reply_tick)>=pdMS_TO_TICKS(1000U))return -1;
    const NavigationDiagnostics_t *s=&diagnostic_reply;
    const NavigationContinuousDiagnostics_t *d=&s->continuous;
    const NavigationStabilization_t *w=&d->resume_wait;
    int n=snprintf(out,capacity,"NAVC,1,%lu,%lu,%lu,%u,%u,%u,%u,%lu,%lu,%lu,%u,%u,%u,%lu,%u,%u,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u\n",
        (unsigned long)query,(unsigned long)s->session_id,(unsigned long)s->started_tick,
        d->segments,d->completed_segments,d->attempts,d->evasions,(unsigned long)d->path_um,
        (unsigned long)d->useful_um,(unsigned long)d->forward_um,d->block,d->permissions,w->settle_state,
        (unsigned long)w->settle_elapsed_ms,w->settle_observations,w->settle_window_observations,
        (unsigned long)w->settle_window_ms,(unsigned long)w->settle_left_span,(unsigned long)w->settle_right_span,
        (unsigned long)d->critical_faults,(unsigned long)Navigation_DiagnosticMs((TickType_t)(xTaskGetTickCount()-diagnostic_reply_tick)),
        (unsigned)NAV_AUTO4_SESSION_MAX_MS,(unsigned)NAV_AUTO4_MAX_SEGMENTS,(unsigned)NAV_AUTO4_MAX_EVASIONS,(unsigned)NAV_AUTO4_MAX_PATH_MM,(unsigned)d->refresh_pending);
    return n>0 && (size_t)n<capacity && n<=255 ? n : -1;
}

void Navigation_CancelSession(uint32_t session) {
    taskENTER_CRITICAL();
    if (navigation.active && navigation.session == session) Navigation_Cancel();
    taskEXIT_CRITICAL();
}

#if NAV_AUTO2_ENABLED
static void Navigation_GateSensors(const SensoresDatos_t *data,bool available,uint32_t session) {
    TickType_t now=xTaskGetTickCount();
    uint32_t faults=available ? TaskSafety_EvaluateSnapshot(data,now) : SAFETY_FAULT_SENSOR;
    /* Same physical range as TaskSensores; a corrupt/nonfinite value is not clearance. */
    if(available && (!isfinite(data->distancia_m) || !isfinite(data->distancia_trasera_m) ||
        data->distancia_m<0.02f || data->distancia_m>4.0f || data->distancia_trasera_m<0.02f || data->distancia_trasera_m>4.0f))
        faults|=SAFETY_FAULT_SENSOR;
    taskENTER_CRITICAL();
    bool current=navigation.active && (Navigation_IsContinuous() || Navigation_IsExplore()) && navigation.session==session;
    if(current) {
        NavigationContinuousDiagnostics_t *d=&navigation_diagnostics.continuous;
        d->permissions=faults ? 0U : (data->distancia_m>(float)NAV_FRONT_STOP_MM/1000.0f ? 1U : 0U) |
            (data->distancia_trasera_m>(float)NAV_REAR_REVERSE_MIN_MM/1000.0f ? 2U : 0U);
        if(d->permissions==3U)d->permissions|=4U; /* Supervised only, no lateral measurement. */
        bool front=!faults && data->distancia_m<=(float)NAV_FRONT_STOP_MM/1000.0f;
        bool rear=!faults && data->distancia_trasera_m<=(float)NAV_REAR_REVERSE_MIN_MM/1000.0f;
        if((navigation.state==NAV_AVANZAR && front) || (navigation.state==NAV_RETROCEDER_ACOTADO && rear) ||
           (navigation.state==NAV_GIRAR_ACOTADO && (front||rear))) {
            navigation.motion_hold=true;navigation.emission_ready=false;
            d->block=navigation.state==NAV_AVANZAR?1U:navigation.state==NAV_RETROCEDER_ACOTADO?2U:3U;
        }
        if(navigation.motion_hold)d->permissions &= (uint8_t)~(navigation.state==NAV_AVANZAR?1U:navigation.state==NAV_RETROCEDER_ACOTADO?2U:4U);
    }
    taskEXIT_CRITICAL();
    /* Never mask critical bits, and never publish logger events under an outer lock. */
    if(current && faults)TaskSafety_ReportFault((SafetyFault_t)faults);
}
#endif
void Navigation_ObserveMotionSensors(void) {
#if NAV_AUTO2_ENABLED
    taskENTER_CRITICAL();
    bool observe=navigation.active && (Navigation_IsContinuous() || Navigation_IsExplore());uint32_t session=navigation.session;
    taskEXIT_CRITICAL();
    if(!observe)return;
    SensoresDatos_t data;bool available=TaskSensores_GetData(&data)==pdTRUE;
    Navigation_GateSensors(&data,available,session);
#endif
}

bool Navigation_IsCommandCurrent(uint32_t session, TickType_t now) {
    taskENTER_CRITICAL();
#if NAV_AUTO2_ENABLED
    if (navigation.active && navigation.session == session && Navigation_DeadlineExpired(now)) {
        if (navigation.finish_reason == NAV_FINISH_NONE) {
            if(Navigation_IsExplore() && Navigation_SessionLive(now) && Navigation_LinkLive(now) && navigation.state!=NAV_DETENER_Y_ESPERAR){
                navigation.motion_hold=true;navigation.emission_ready=false;explore.block=9U;
            }else {
                bool lost=Navigation_IsExplore() && !Navigation_LinkLive(now);
                if(lost)explore.block=12U;
                navigation.finish_reason=lost?NAV_FINISH_INTERNAL_LINK_LOST:NAV_FINISH_TIMEOUT;
            }
        }
    }
#endif
    bool current = navigation.active &&
        (navigation.state == NAV_AVANZAR || navigation.state == NAV_RETROCEDER_ACOTADO
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
         || (Navigation_TurnEnabled() && navigation.state == NAV_GIRAR_ACOTADO)
#endif
        ) &&
        navigation.session == session &&
#if NAV_AUTO2_ENABLED
        Navigation_SessionLive(now) && Navigation_LinkLive(now) && (!Navigation_IsExplore() || explore.link_ready) &&
#else
        (TickType_t)(now - navigation.started_tick) < pdMS_TO_TICKS(Navigation_SessionMaxMs()) &&
#endif
        (navigation.state != NAV_RETROCEDER_ACOTADO ||
         (TickType_t)(now - navigation.reverse_started_tick) < pdMS_TO_TICKS(NAV_REVERSE_MAX_MS))
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
        && (navigation.state != NAV_GIRAR_ACOTADO ||
            (TickType_t)(now - navigation.turn_started_tick) < pdMS_TO_TICKS(Navigation_TurnMaxMs()))
#endif
        ;
#if NAV_AUTO1_ODOMETRY
    current = current &&
#if NAV_AUTO2_ENABLED
        !navigation.phase_complete && !navigation.motion_hold &&
#else
        navigation.state == NAV_AVANZAR &&
#endif
        navigation_odometry.ready &&
        navigation.finish_reason == NAV_FINISH_NONE;
    if (current) {
        OdometrySnapshot s;
        double progress;
        Odometry_GetSnapshot(&s); /* Copia breve, sin colas ni esperas. */
        NavigationFinishReason_t outcome = NAV_FINISH_NONE;
#if NAV_AUTO2_ENABLED
        if(Navigation_IsExplore() && Navigation_OdometryFresh(&s,now)){
            Navigation_ExploreYaw(&s);
            if(navigation.state==NAV_GIRAR_ACOTADO && explore.scan_deg>=LOCAL_MAX_SCAN_DEG){
                navigation.motion_hold=true;navigation.emission_ready=false;explore.block=5U;current=false;
            }
        }
        outcome=Navigation_ContinuousObserve(&s,now);
#endif
        if(outcome==NAV_FINISH_NONE) outcome = Navigation_OdometryOutcome(&s, now, &progress);
        if (outcome != NAV_FINISH_NONE) {
#if NAV_AUTO2_ENABLED
            if (outcome == NAV_FINISH_TARGET && (navigation.state != NAV_AVANZAR || Navigation_IsContinuous() || Navigation_IsExplore())) {
                navigation.phase_complete = true;
            } else
#endif
            {
#if NAV_AUTO2_ENABLED
                if(Navigation_IsExplore() && outcome==NAV_FINISH_NO_PROGRESS){navigation.motion_hold=true;explore.block=10U;}
                else
#endif
                navigation.finish_reason = outcome;
            }
            navigation.emission_ready = false;
            if (outcome == NAV_FINISH_TARGET) {
                navigation_odometry.progress_mm = progress;
            }
            current = false;
        }
    }
#endif
    taskEXIT_CRITICAL();
    return current;
}

bool Navigation_ConsumeEmission(uint32_t session, TickType_t issued_tick, TickType_t now,
                                NavigationState_t phase) {
    taskENTER_CRITICAL();
    bool allowed = Navigation_IsCommandCurrent(session, now) &&
        navigation.emission_ready && issued_tick == navigation.decision_tick &&
        phase == navigation.state;
    if (allowed) navigation.emission_ready = false;
    taskEXIT_CRITICAL();
    return allowed;
}

#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
void Navigation_CommandApplied(uint32_t session, TickType_t now) {
    taskENTER_CRITICAL();
    if (navigation.state == NAV_RETROCEDER_ACOTADO && Navigation_IsCommandCurrent(session, now)) {
        navigation.reverse_applied = true;
        navigation_diagnostics.flags |= NAV_DIAG_REVERSE_APPLIED;
    }
    if (navigation.state==NAV_GIRAR_ACOTADO && Navigation_IsCommandCurrent(session,now)) navigation_diagnostics.turn_flags |= 4U;
    taskEXIT_CRITICAL();
}

#if !NAV_AUTO2_ENABLED
/* Requiere seccion critica. Se reserva la ventana completa de 250 ms, incluida
 * la siguiente iteracion sin comprimir el cronograma. Nunca renueva started_tick. */
static bool Navigation_TurnReady(uint32_t session, TickType_t now) {
    TickType_t age = (TickType_t)(now - navigation.started_tick);
    return !NAV_AUTO1_ODOMETRY && navigation.active && navigation.session == session &&
        navigation.state == NAV_RETROCEDER_ACOTADO && navigation.reverse_applied &&
        !navigation.turn_used &&
        (TickType_t)(now - navigation.reverse_started_tick) >= pdMS_TO_TICKS(NAV_REVERSE_MAX_MS) &&
        age < pdMS_TO_TICKS(Navigation_SessionMaxMs()) &&
        pdMS_TO_TICKS(Navigation_SessionMaxMs()) - age >= pdMS_TO_TICKS(Navigation_TurnMaxMs());
}
#endif

bool Navigation_CompleteCommand(uint32_t session, TickType_t now) {
    taskENTER_CRITICAL();
#if NAV_AUTO2_ENABLED
    bool complete = navigation.active && navigation.session == session && (navigation.phase_complete || ((Navigation_IsContinuous() || Navigation_IsExplore()) && navigation.motion_hold)) &&
        navigation.finish_reason == NAV_FINISH_NONE &&
        (navigation.state == NAV_RETROCEDER_ACOTADO || ((Navigation_IsContinuous() || Navigation_IsExplore()) && navigation.state==NAV_AVANZAR) ||
         (Navigation_TurnEnabled() && navigation.state == NAV_GIRAR_ACOTADO)) &&
        Navigation_SessionLive(now) && Navigation_LinkLive(now);
#else
    bool complete = Navigation_TurnReady(session, now);
#endif
    if (complete) navigation.emission_ready = false;
    taskEXIT_CRITICAL();
    return complete;
}
#endif

#if NAV_AUTO2_ENABLED
/* Toda transicion revoca generacion/sesion anterior y escribe cero. El plazo
 * global no cambia. Otra iteracion tomara una referencia posterior al cero. */
static void Navigation_StartPhase(NavigationState_t phase, TickType_t now) {
    if(Navigation_IsContinuous() && phase==NAV_AVANZAR && navigation_diagnostics.continuous.segments>=NAV_AUTO4_MAX_SEGMENTS) {
        Navigation_StopFor(NAV_FINISH_LIMIT_REACHED);return;
    }
    TaskMotorControl_Stop();
    navigation.active = true;
    navigation.state = phase;
    navigation.finish_reason = NAV_FINISH_NONE;
    navigation.phase_started_tick = now;
    navigation.phase_complete = false;
    navigation.motion_hold=false;
    if(Navigation_IsContinuous()) navigation_diagnostics.continuous.refresh_pending=0;
    navigation_odometry.ready = false;
    Navigation_DiagnosticPhase(phase, now);
    if (phase == NAV_RETROCEDER_ACOTADO) {
        if(Navigation_IsContinuous()) {
            navigation.reverse_completed=false;navigation_diagnostics.reverse_um=0;
            navigation_diagnostics.reverse_reference_available=false;
            navigation_diagnostics.flags &= (uint16_t)~(NAV_DIAG_REVERSE_REQUESTED|NAV_DIAG_REVERSE_PUBLISHED|NAV_DIAG_REVERSE_APPLIED|NAV_DIAG_REVERSE_COMPLETED|NAV_DIAG_REVERSE_PROGRESS);
        }
        navigation_diagnostics.flags |= NAV_DIAG_REVERSE_AUTHORIZED;
        navigation_diagnostics.reverse_entered = true;
        navigation_diagnostics.reverse_entry_tick = now;
        navigation.reverse_started_tick = now;
    } else if (phase == NAV_DETENER_Y_ESPERAR) {
        memset(&navigation_settle, 0, sizeof navigation_settle);
        navigation_settle.started_tick = now;
        NavigationStabilization_t *w=Navigation_WaitDiagnostics(&navigation_diagnostics);
        memset(w,0,sizeof *w);w->settle_state=1U;
        if((Navigation_IsContinuous() || Navigation_IsExplore()) && navigation.settle_target==NAV_GIRAR_ACOTADO){navigation_diagnostics.turn_flags=0;navigation_diagnostics.turn_mdeg=0;}
    } else if (phase==NAV_AVANZAR) {
        if(Navigation_IsContinuous())++navigation_diagnostics.continuous.segments;
    } else if(phase==NAV_GIRAR_ACOTADO) {
        navigation_diagnostics.turn_left=Navigation_IsExplore()?!explore.right:NAV_TURN_LEFT;
        navigation_diagnostics.turn_target_mdeg=(uint32_t)((Navigation_IsExplore()?explore.turn_target_deg:NAV_AUTO2_TURN_TARGET_DEG)*1000.0f);
        if(Navigation_IsContinuous() || Navigation_IsExplore()) {navigation.turn_used=false;navigation_diagnostics.turn_flags=0;navigation_diagnostics.turn_mdeg=0;}
        navigation.turn_started_tick = now;
        navigation.turn_used = true;
        navigation_diagnostics.turn_wait.reverse_entered=true;
        navigation_diagnostics.turn_wait.reverse_entry_tick=now;
    }
}

/* Solo muestras nuevas posteriores al cero. No evalua signo: puede haber
 * inercia de una o ambas ruedas. Rango, no diferencia neta, para cada ventana. */
static void Navigation_SettleStep(const OdometrySnapshot *s, TickType_t now, bool fresh_clearance) {
    NavigationStabilization_t *d = Navigation_WaitDiagnostics(&navigation_diagnostics);
    if (!navigation_settle.ready) {
        if ((TickType_t)(now-s->tick)>=(TickType_t)(now-navigation_settle.started_tick)) { return; }
    } else {
        if ((TickType_t)(now-s->tick)>(TickType_t)(now-navigation_settle.last_tick) ||
            (s->tick==navigation_settle.last_tick &&
             (s->left_count!=navigation_settle.last_left || s->right_count!=navigation_settle.last_right))) {
            navigation_diagnostics.odometry_error = NAV_ODOM_TIMESTAMP;
            Navigation_StopFor(NAV_FINISH_ODOMETRY); return;
        }
        if (s->tick==navigation_settle.last_tick) { return; }
    }
    if (navigation_settle.observations<UINT16_MAX) { ++navigation_settle.observations; }
    int64_t lmin=navigation_settle.left_min, lmax=navigation_settle.left_max;
    int64_t rmin=navigation_settle.right_min, rmax=navigation_settle.right_max;
    if (s->left_count<lmin) { lmin=s->left_count; }
    if (s->left_count>lmax) { lmax=s->left_count; }
    if (s->right_count<rmin) { rmin=s->right_count; }
    if (s->right_count>rmax) { rmax=s->right_count; }
    int64_t lspan=0,rspan=0;
    if (navigation_settle.ready && (!Navigation_SubtractCount(lmax,lmin,&lspan) ||
        !Navigation_SubtractCount(rmax,rmin,&rspan))) {
        navigation_diagnostics.odometry_error=NAV_ODOM_OVERFLOW;
        Navigation_StopFor(NAV_FINISH_ODOMETRY); return;
    }
    if (!navigation_settle.ready || lspan>NAV_SETTLE_MAX_SPAN_COUNTS || rspan>NAV_SETTLE_MAX_SPAN_COUNTS) {
        navigation_settle.window_tick=s->tick;
        lmin=lmax=s->left_count;rmin=rmax=s->right_count;
        navigation_settle.window_observations=1U;
        lspan=rspan=0;
    } else if (navigation_settle.window_observations<UINT16_MAX) { ++navigation_settle.window_observations; }
    navigation_settle.ready=true;
    navigation_settle.left_min=lmin;navigation_settle.left_max=lmax;
    navigation_settle.right_min=rmin;navigation_settle.right_max=rmax;
    navigation_settle.last_tick=s->tick;
    navigation_settle.last_left=s->left_count;navigation_settle.last_right=s->right_count;
    d->settle_elapsed_ms=Navigation_DiagnosticMs((TickType_t)(now-navigation_settle.started_tick));
    d->settle_window_ms=Navigation_DiagnosticMs((TickType_t)(s->tick-navigation_settle.window_tick));
    d->settle_left_span=(uint32_t)lspan;d->settle_right_span=(uint32_t)rspan;
    d->settle_observations=navigation_settle.observations;
    d->settle_window_observations=navigation_settle.window_observations;
    if (fresh_clearance && navigation_settle.window_observations>=NAV_SETTLE_MIN_OBSERVATIONS &&
        (TickType_t)(s->tick-navigation_settle.window_tick)>=pdMS_TO_TICKS(NAV_SETTLE_WINDOW_MS)) {
        /* Solo cambia de fase. La proxima iteracion exigira una muestra
         * posterior a esta confirmacion antes de tomar la nueva referencia. */
        d->settle_state=2U;
        Navigation_StartPhase(navigation.settle_target,now);
    }
}

static void Navigation_Auto2PhaseFinished(const SensoresDatos_t *data, TickType_t now) {
    if (navigation.state == NAV_AVANZAR) {
        if(Navigation_IsContinuous()) {
            ++navigation_diagnostics.continuous.completed_segments;
            if(navigation_diagnostics.continuous.segments>=NAV_AUTO4_MAX_SEGMENTS) Navigation_StopFor(NAV_FINISH_LIMIT_REACHED);
            else {navigation.settle_target=NAV_AVANZAR;Navigation_StartPhase(NAV_DETENER_Y_ESPERAR,now);}
        } else Navigation_StopFor(NAV_FINISH_TARGET);
    } else if (navigation.state == NAV_RETROCEDER_ACOTADO) {
        navigation.reverse_completed = true;
        navigation_diagnostics.flags |= NAV_DIAG_REVERSE_COMPLETED;
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
        if (Navigation_TurnEnabled() && navigation.reverse_applied && !navigation.turn_used &&
            data->distancia_m > (float)NAV_FRONT_STOP_MM / 1000.0f &&
            data->distancia_trasera_m > (float)NAV_REAR_REVERSE_MIN_MM / 1000.0f &&
            pdMS_TO_TICKS(Navigation_SessionMaxMs()) - (TickType_t)(now - navigation.started_tick) >=
                pdMS_TO_TICKS(Navigation_TurnMaxMs() + (navigation.profile>=NAV_PROFILE_TURN ? NAV_SETTLE_TIMEOUT_MS : 0U) + 20U)) {
            if (navigation.profile>=NAV_PROFILE_TURN) {
                navigation.settle_target=NAV_GIRAR_ACOTADO;
                Navigation_StartPhase(NAV_DETENER_Y_ESPERAR,now);
            } else Navigation_StartPhase(NAV_GIRAR_ACOTADO, now);
            return;
        }
#else
        (void)data; (void)now;
#endif
        /* Sin cobertura lateral no hay autorizacion operativa del giro.
         * reverse_completed conserva que la separacion si se completo. */
        Navigation_StopFor(Navigation_IsContinuous() ? NAV_FINISH_BLOCKED_NO_SAFE_PATH : NAV_FINISH_EVASION_NOT_AUTHORIZED);
    } else {
        navigation_diagnostics.turn_flags |= 8U;
        if(Navigation_IsContinuous()) {
            ++navigation_diagnostics.continuous.evasions;
            navigation.settle_target=NAV_AVANZAR;
            Navigation_StartPhase(NAV_DETENER_Y_ESPERAR,now);
        } else Navigation_StopFor(navigation.profile==NAV_PROFILE_TURN ? NAV_FINISH_TARGET : NAV_FINISH_EVASION_COMPLETED);
    }
}

/* Ejecuta la misma maquina de navegacion bajo su lock. true habilita una sola
 * emision por la ruta compartida de Motion, fuera de la seccion critica. */
static bool Navigation_Auto2StepLocked(const SensoresDatos_t *data, const OdometrySnapshot *odometry,
                                      bool available, TickType_t now) {
    if (navigation.state != NAV_ESPERAR_DATOS && navigation.state != NAV_AVANZAR &&
        navigation.state != NAV_RETROCEDER_ACOTADO && navigation.state != NAV_GIRAR_ACOTADO &&
        navigation.state != NAV_DETENER_Y_ESPERAR) {
        Navigation_StopFor(NAV_FINISH_CANCELLED); return false;
    }
    if (TaskSafety_IsStopActive()) {
        Navigation_StopFor(NAV_FINISH_SAFETY); return false;
    }
    if (TaskControlRobot_IsStopPending() || TaskControlRobot_GetState() != ROBOT_STATE_AUTONOMOUS) {
        Navigation_StopFor(TaskControlRobot_IsStopPending() ? NAV_FINISH_OPERATOR_STOP : NAV_FINISH_CANCELLED);
        return false;
    }
    if (Navigation_DeadlineExpired(now)) {
        Navigation_StopFor(NAV_FINISH_TIMEOUT); return false;
    }
    uint32_t faults = available ? TaskSafety_EvaluateSnapshot(data, now) : SAFETY_FAULT_SENSOR;
    bool finite = available && isfinite(data->distancia_m) && isfinite(data->distancia_trasera_m);
    bool front_current = available && data->ultrasonido_valido && isfinite(data->distancia_m) &&
        (TickType_t)(now - data->timestamp) <= pdMS_TO_TICKS(TASK_SAFETY_SENSOR_TIMEOUT_MS) &&
        (TickType_t)(now - data->timestamp_ultrasonido_frontal) <= pdMS_TO_TICKS(TASK_SAFETY_ULTRASOUND_MAX_AGE_MS);
    bool obstacle = (front_current && data->distancia_m <= (float)NAV_FRONT_STOP_MM / 1000.0f) ||
        (Navigation_IsContinuous() && navigation.motion_hold && navigation.state==NAV_AVANZAR);
    if (obstacle) {
        navigation_diagnostics.flags |= NAV_DIAG_OBSTACLE;
        if (navigation.state == NAV_AVANZAR) { navigation_diagnostics.flags |= NAV_DIAG_REVERSE_CONSIDERED; }
    }
    if (navigation.state == NAV_ESPERAR_DATOS && obstacle) {
        if(Navigation_IsContinuous()) navigation_diagnostics.continuous.block=7U;
        Navigation_StopFor(Navigation_IsContinuous() ? NAV_FINISH_BLOCKED_NO_SAFE_PATH : NAV_FINISH_OBSTACLE); return false;
    }
    if (faults != SAFETY_FAULT_NONE || !finite) {
        if (navigation.state != NAV_ESPERAR_DATOS || (faults & ~SAFETY_FAULT_SENSOR) != 0U) {
            Navigation_StopFor(navigation.state == NAV_AVANZAR && obstacle ?
                NAV_FINISH_EVASION_NOT_AUTHORIZED : NAV_FINISH_SENSORS);
        }
        return false;
    }
    if(Navigation_IsContinuous() && navigation.state==NAV_RETROCEDER_ACOTADO && navigation.motion_hold) {
        navigation_diagnostics.continuous.block=2U;Navigation_StopFor(NAV_FINISH_BLOCKED_NO_SAFE_PATH);return false;
    }
    if ((navigation.state == NAV_RETROCEDER_ACOTADO || (navigation.state == NAV_DETENER_Y_ESPERAR &&
        !(Navigation_IsContinuous() && navigation.settle_target==NAV_AVANZAR))) &&
        data->distancia_trasera_m <= (float)NAV_REAR_REVERSE_MIN_MM / 1000.0f) {
        if(Navigation_IsContinuous()) navigation_diagnostics.continuous.block=2U;
        Navigation_StopFor(Navigation_IsContinuous() ? NAV_FINISH_BLOCKED_NO_SAFE_PATH : NAV_FINISH_EVASION_NOT_AUTHORIZED); return false;
    }
    if ((navigation.state == NAV_GIRAR_ACOTADO || (navigation.state==NAV_DETENER_Y_ESPERAR && navigation.settle_target==NAV_GIRAR_ACOTADO)) &&
        (!Navigation_TurnEnabled() || ((Navigation_IsContinuous() || Navigation_IsExplore()) && navigation.motion_hold) || data->distancia_m <= (float)NAV_FRONT_STOP_MM / 1000.0f ||
         data->distancia_trasera_m <= (float)NAV_REAR_REVERSE_MIN_MM / 1000.0f)) {
        if(Navigation_IsContinuous()) navigation_diagnostics.continuous.block=3U;
        Navigation_StopFor(Navigation_IsContinuous() ? NAV_FINISH_BLOCKED_NO_SAFE_PATH : NAV_FINISH_EVASION_NOT_AUTHORIZED); return false;
    }
    if (!Navigation_OdometryFresh(odometry, now)) {
        Navigation_DiagnosticOdometry(odometry, now,
            navigation.state == NAV_RETROCEDER_ACOTADO || (navigation.state == NAV_GIRAR_ACOTADO && NAV_TURN_LEFT) ? -1 : 1,
            navigation.state == NAV_RETROCEDER_ACOTADO || (navigation.state == NAV_GIRAR_ACOTADO && !NAV_TURN_LEFT) ? -1 : 1);
        Navigation_StopFor(NAV_FINISH_ODOMETRY); return false;
    }
    if(Navigation_IsContinuous()) {
        NavigationFinishReason_t limit=Navigation_ContinuousObserve(odometry,now);
        if(limit!=NAV_FINISH_NONE){Navigation_StopFor(limit);return false;}
    }
    if (navigation.profile==NAV_PROFILE_TURN && navigation.state==NAV_ESPERAR_DATOS) {
        navigation.settle_target=NAV_GIRAR_ACOTADO;
        Navigation_StartPhase(NAV_DETENER_Y_ESPERAR,now);return false;
    }
    if(Navigation_IsContinuous() && navigation.state==NAV_DETENER_Y_ESPERAR && navigation.settle_target==NAV_AVANZAR && obstacle) {
        navigation_diagnostics.continuous.block=1U;Navigation_StopFor(NAV_FINISH_BLOCKED_NO_SAFE_PATH);return false;
    }
    if (navigation.state == NAV_DETENER_Y_ESPERAR) {
        bool fresh_clearance=!Navigation_IsContinuous() ||
            ((TickType_t)(now-data->timestamp_ultrasonido_frontal)<(TickType_t)(now-navigation_settle.started_tick) &&
             (TickType_t)(now-data->timestamp_ultrasonido_trasero)<(TickType_t)(now-navigation_settle.started_tick));
        if(Navigation_IsContinuous()) navigation_diagnostics.continuous.refresh_pending=!fresh_clearance;
        Navigation_SettleStep(odometry,now,fresh_clearance);
        return false; /* Sin emision en la espera ni al confirmar estabilidad. */
    }
    if (!navigation_odometry.ready) {
        if ((TickType_t)(now - odometry->tick) >= (TickType_t)(now - navigation.phase_started_tick)) {
            return false;
        }
        Navigation_RecordReference(odometry);
    }
    NavigationFinishReason_t outcome = Navigation_OdometryOutcome(odometry, now, &navigation_odometry.progress_mm);
    if (outcome != NAV_FINISH_NONE && outcome != NAV_FINISH_TARGET) {
        Navigation_StopFor(outcome); return false;
    }
    if ((outcome == NAV_FINISH_TARGET || navigation.phase_complete) &&
        !(Navigation_IsContinuous() && navigation.state==NAV_AVANZAR && obstacle)) {
        Navigation_Auto2PhaseFinished(data, now); return false;
    }
    navigation_odometry.last_tick = odometry->tick;
    navigation_odometry.last_left = odometry->left_count;
    navigation_odometry.last_right = odometry->right_count;
    if (navigation.state == NAV_AVANZAR && obstacle) {
        if(Navigation_IsContinuous() && !Navigation_RecoveryAllowed(odometry)) {
            Navigation_StopFor(NAV_FINISH_BLOCKED_NO_SAFE_PATH);return false;
        }
        if (data->distancia_trasera_m <= (float)NAV_REAR_REVERSE_MIN_MM / 1000.0f) {
            if(Navigation_IsContinuous()) navigation_diagnostics.continuous.block=2U;
            Navigation_StopFor(Navigation_IsContinuous() ? NAV_FINISH_BLOCKED_NO_SAFE_PATH : NAV_FINISH_EVASION_NOT_AUTHORIZED);
        } else if (pdMS_TO_TICKS(Navigation_SessionMaxMs()) - (TickType_t)(now - navigation.started_tick) <
                   pdMS_TO_TICKS(NAV_REVERSE_MAX_MS + 20U)) {
            Navigation_StopFor(NAV_FINISH_TIMEOUT);
        } else {
            navigation.settle_target=NAV_RETROCEDER_ACOTADO;
            Navigation_StartPhase(NAV_DETENER_Y_ESPERAR, now);
        }
        return false;
    }
    return true;
}
#endif


#if NAV_AUTO2_ENABLED
static void Navigation_ExploreObserve(void){
    OdometryRay rays[2];Odometry_GetRay(0,&rays[0]);Odometry_GetRay(1,&rays[1]);
    OdometrySnapshot o;Odometry_GetSnapshot(&o);TickType_t now=xTaskGetTickCount();
    taskENTER_CRITICAL();
    if(navigation.active && Navigation_IsExplore()){
        if(!explore.evidence.origin_set || hypotf(o.x_m-explore.evidence.x,o.y_m-explore.evidence.y)>0.030f)
            Local_Reset(&explore.evidence,o.x_m,o.y_m);
        for(unsigned i=0;i<2U;++i)(void)Local_Observe(&explore.evidence,i,&rays[i],now);
        OdometryRay *f=&rays[0];
        if(!explore.front_seen || (f->sequence-explore.front_sequence && f->sequence-explore.front_sequence<0x80000000U)){
            bool usable=f->state==3U && isfinite(f->distance_m) && f->distance_m>=(float)LOCAL_CLEAR_MM/1000.0f && f->distance_m<=4.0f &&
                isfinite(f->yaw_rad) && fabsf(Local_Difference(f->yaw_rad-o.theta_rad))<LOCAL_FRONT_BEARING_RAD &&
                (TickType_t)(now-f->tick)<=pdMS_TO_TICKS(200U);
            bool consecutive=explore.front_seen && f->tick!=explore.front_tick &&
                (TickType_t)(f->tick-explore.front_tick)<=pdMS_TO_TICKS(200U) &&
                fabsf(f->distance_m-explore.front_distance)<=LOCAL_CHANGE_TOLERANCE_MM*.001f &&
                fabsf(Local_Difference(f->yaw_rad-explore.front_yaw))<LOCAL_FRONT_BEARING_RAD;
            explore.front_confirmations=usable?(consecutive && explore.front_confirmations?LOCAL_FRONT_CONFIRMATIONS:1U):0U;
            explore.front_seen=true;explore.front_sequence=f->sequence;explore.front_tick=f->tick;explore.front_yaw=f->yaw_rad;
            explore.front_distance=f->distance_m;
        }
        Local_Expire(&explore.evidence,now);
        Navigation_ExploreYaw(&o);
    }
    taskEXIT_CRITICAL();
}
/* Failed movement directions are remembered at this pose, not retried just
 * because the session lease was renewed. A measured new pose/orientation or
 * confirmed environment change permits a new evaluation. */
static void Navigation_ExploreFailed(const OdometrySnapshot *o){
    uint8_t bit=navigation.state==NAV_AVANZAR?1U:navigation.state==NAV_RETROCEDER_ACOTADO?8U:explore.right?4U:2U;
    explore.inhibited|=bit;explore.failure_x=o->x_m;explore.failure_y=o->y_m;explore.failure_yaw=o->theta_rad;
}
static void Navigation_ExploreWait(uint8_t block,TickType_t now){
    explore.block=block;explore.action=0;
    navigation.settle_target=NAV_EXPLORAR;Navigation_StartPhase(NAV_DETENER_Y_ESPERAR,now);
}
static void Navigation_ExplorePlan(unsigned action,int selected,uint8_t block){
    if(explore.action!=action || explore.selected!=selected){
        if(++explore.challenge==0U)++explore.challenge;
    }
    explore.action=(uint8_t)action;explore.selected=selected;explore.block=block;
}
static bool Navigation_ExploreStep(const SensoresDatos_t *data,const OdometrySnapshot *o,bool available,TickType_t now){
    if(TaskSafety_IsStopActive()){Navigation_StopFor(NAV_FINISH_SAFETY);return false;}
    if(TaskControlRobot_IsStopPending() || TaskControlRobot_GetState()!=ROBOT_STATE_AUTONOMOUS){
        Navigation_StopFor(TaskControlRobot_IsStopPending()?NAV_FINISH_OPERATOR_STOP:NAV_FINISH_CANCELLED);return false;
    }
    if(!Navigation_LinkLive(now)){explore.block=12U;Navigation_StopFor(NAV_FINISH_INTERNAL_LINK_LOST);return false;}
    if(!available || TaskSafety_EvaluateSnapshot(data,now)){Navigation_StopFor(NAV_FINISH_SENSORS);return false;}
    if(!Navigation_OdometryFresh(o,now)){Navigation_DiagnosticOdometry(o,now,1,1);Navigation_StopFor(NAV_FINISH_ODOMETRY);return false;}
    if(navigation.state==NAV_ESPERAR_DATOS){Navigation_ExploreWait(1U,now);return false;}
    if(navigation.state==NAV_DETENER_Y_ESPERAR){
        if((TickType_t)(now-navigation.phase_started_tick)>=pdMS_TO_TICKS(NAV_SETTLE_TIMEOUT_MS)){Navigation_StopFor(NAV_FINISH_TIMEOUT);return false;}
        bool fresh=(TickType_t)(now-data->timestamp_ultrasonido_frontal)<(TickType_t)(now-navigation_settle.started_tick) &&
                   (TickType_t)(now-data->timestamp_ultrasonido_trasero)<(TickType_t)(now-navigation_settle.started_tick);
        Navigation_SettleStep(o,now,fresh);return false;
    }
    if(navigation.motion_hold && (explore.block==9U || explore.block==10U || explore.block==5U)){
        if(explore.block!=5U)Navigation_ExploreFailed(o);
        Navigation_ExploreWait(explore.block,now);return false;
    }
    bool front=data->distancia_m>(float)NAV_FRONT_STOP_MM/1000.0f;
    bool rear=data->distancia_trasera_m>(float)NAV_REAR_REVERSE_MIN_MM/1000.0f;
    if(navigation.state==NAV_EXPLORAR){
        /* Initial communication must be proven before the first movement.
         * This never grants or renews navigation permission. */
        if(!explore.link_ready){explore.action=0;explore.block=1U;return false;}
        if(explore.inhibited && (hypotf(o->x_m-explore.failure_x,o->y_m-explore.failure_y)>=0.015f ||
            fabsf(Local_Difference(o->theta_rad-explore.failure_yaw))>=0.3490658504f))explore.inhibited=0;
        uint16_t f=(uint16_t)(data->distancia_m*1000.0f),r=(uint16_t)(data->distancia_trasera_m*1000.0f);
        if(explore.block==6U || explore.block==7U || explore.block==8U){
            if(explore.block!=8U)explore.blocked_reason=explore.block;
            bool changed=abs((int)f-explore.blocked_front )>=(int)LOCAL_CHANGE_MM || abs((int)r-explore.blocked_rear)>=(int)LOCAL_CHANGE_MM;
            if(!changed){explore.changes=0;explore.block=explore.blocked_reason;}
            else if(data->timestamp_ultrasonido_frontal!=explore.changed_front_tick && data->timestamp_ultrasonido_trasero!=explore.changed_rear_tick){
                bool consistent=explore.changes && abs((int)f-explore.change_front)<(int)LOCAL_CHANGE_TOLERANCE_MM && abs((int)r-explore.change_rear)<(int)LOCAL_CHANGE_TOLERANCE_MM;
                explore.changes=consistent?(uint8_t)(explore.changes+1U):1U;
                explore.change_front=f;explore.change_rear=r;
                explore.changed_front_tick=data->timestamp_ultrasonido_frontal;explore.changed_rear_tick=data->timestamp_ultrasonido_trasero;
                if(explore.changes>=LOCAL_CHANGE_OBSERVATIONS){
                    /* Reconsider only this position after three new coherent
                     * capture pairs, not a fluctuating or repeated sample. */
                    unsigned count=0;
                    for(unsigned v=0;v<explore.evidence.visit_count;++v)
                        if(hypotf(o->x_m-explore.evidence.visits[v].x,o->y_m-explore.evidence.visits[v].y)>=LOCAL_HISTORY_RADIUS_M)
                            explore.evidence.visits[count++]=explore.evidence.visits[v];
                    explore.evidence.visit_count=(uint8_t)count;explore.evidence.visit_next=(uint8_t)(count%LOCAL_VISITS);
                    explore.scan_deg=0;explore.retreated=false;explore.block=0;explore.changes=0;explore.inhibited=0;
                }else explore.block=8U; /* Valid captures, change still unconfirmed. */
            }
            if(explore.block==6U || explore.block==7U || explore.block==8U)return false;
        }
        int selected=Local_Select(&explore.evidence,o->x_m,o->y_m,o->theta_rad,&explore.evaluated,&explore.repeated);
        float delta=selected<0 ? 0 : Local_Difference(selected*0.1745329252f-o->theta_rad);
        unsigned action=0;uint8_t reason=1;
        float yaw=Local_Difference(o->theta_rad);if(yaw<0)yaw+=6.2831853072f;
        unsigned forward_sector=(unsigned)floorf(yaw/0.1745329252f+0.5f)%LOCAL_SECTORS;
        bool frontal=data->distancia_m>=(float)LOCAL_CLEAR_MM/1000.0f && !(explore.inhibited&1U) &&
            !Local_Visited(&explore.evidence,o->x_m,o->y_m,o->theta_rad);
        bool confirmed=explore.front_confirmations>=LOCAL_FRONT_CONFIRMATIONS &&
            (TickType_t)(now-explore.front_tick)<=pdMS_TO_TICKS(200U) &&
            fabsf(Local_Difference(explore.front_yaw-o->theta_rad))<LOCAL_FRONT_BEARING_RAD;
        /* A clear frontal reading waits for a second independent capture,
         * rather than initiating an unnecessary scan. No single echo moves. */
        if(frontal && !confirmed){Navigation_ExplorePlan(0U,-1,1U);return false;}
        if(frontal && confirmed){selected=(int)forward_sector;action=1U;reason=0;}
        else if(selected>=0 && fabsf(delta)<0.0872664626f && front && !(explore.inhibited&1U)){action=1U;reason=0;}
        else if(front && rear && explore.scan_deg+2.0f+LOCAL_TURN_RESERVE_DEG<=LOCAL_MAX_SCAN_DEG && (explore.inhibited&6U)!=6U){
            bool right=selected>=0 && delta<=0;
            if(explore.inhibited&(right?4U:2U))right=!right;
            action=right?4U:selected<0?2U:3U;reason=4U;
        }else if(rear && !(explore.inhibited&8U) && Local_RetreatAllowed(&explore.evidence,o->x_m,o->y_m,o->theta_rad)){action=5U;reason=front?4U:2U;}
        else {reason=(explore.repeated || (rear && !Local_RetreatAllowed(&explore.evidence,o->x_m,o->y_m,o->theta_rad)))?7U:6U;if(explore.block!=6U && explore.block!=7U){explore.blocked_front=f;explore.blocked_rear=r;explore.changes=0;}}
        Navigation_ExplorePlan(action,selected,reason);
        if(!action)return false;
        /* Initial operator consent scopes this supervised session. Navigation
         * chooses maneuvers; the lease is not consumed per decision. Every
         * phase still settles, captures a new reference and uses the existing
         * one-use Motion/MotorControl emission permission. */
        if(action==1U){
            if(!Local_Visit(&explore.evidence,o->x_m,o->y_m,o->theta_rad)){explore.block=7;return false;}
            if(explore.segments<UINT32_MAX)++explore.segments;
            /* The just-completed stop window can serve the immediately next
             * straight segment, only while both encoder counts remain stable.
             * StartPhase still zeros/revokes; a later producer sample supplies
             * the new motion reference. Long waits always stabilize again. */
            int64_t dl,dr;
            bool settled=navigation_settle.ready && navigation_settle.window_observations>=NAV_SETTLE_MIN_OBSERVATIONS &&
                (TickType_t)(navigation_settle.last_tick-navigation_settle.window_tick)>=pdMS_TO_TICKS(NAV_SETTLE_WINDOW_MS) &&
                (TickType_t)(now-navigation_settle.last_tick)<=pdMS_TO_TICKS(NAV_SETTLE_TIMEOUT_MS) &&
                Navigation_SubtractCount(o->left_count,navigation_settle.last_left,&dl) && dl>=-1 && dl<=1 &&
                Navigation_SubtractCount(o->right_count,navigation_settle.last_right,&dr) && dr>=-1 && dr<=1;
            navigation.settle_target=NAV_AVANZAR;Navigation_StartPhase(settled?NAV_AVANZAR:NAV_DETENER_Y_ESPERAR,now);
        }else if(action==5U){
            if(!Local_RetreatVisit(&explore.evidence,o->x_m,o->y_m,o->theta_rad)){explore.block=7;return false;}
            explore.retreated=true;if(explore.retreats<UINT32_MAX)++explore.retreats;
            navigation.settle_target=NAV_RETROCEDER_ACOTADO;Navigation_StartPhase(NAV_DETENER_Y_ESPERAR,now);
        }else{
            explore.right=action==4U;
            explore.turn_target_deg=fminf(LOCAL_TURN_STEP_DEG,LOCAL_MAX_SCAN_DEG-explore.scan_deg-LOCAL_TURN_RESERVE_DEG);
            explore.goal_yaw=o->theta_rad+(explore.right?-1.0f:1.0f)*explore.turn_target_deg*0.01745329252f;
            if(explore.turns<UINT32_MAX)++explore.turns;
            navigation.settle_target=NAV_GIRAR_ACOTADO;Navigation_StartPhase(NAV_DETENER_Y_ESPERAR,now);
        }
        return false; /* New producer sample/reference required before emission. */
    }
    if((navigation.state==NAV_AVANZAR && (!front || navigation.motion_hold)) ||
       (navigation.state==NAV_RETROCEDER_ACOTADO && (!rear || navigation.motion_hold)) ||
       (navigation.state==NAV_GIRAR_ACOTADO && (!front || !rear || navigation.motion_hold || explore.scan_deg>LOCAL_MAX_SCAN_DEG))){
        if(navigation.state==NAV_AVANZAR)navigation_diagnostics.flags|=NAV_DIAG_OBSTACLE;
        Navigation_ExploreWait(navigation.state==NAV_RETROCEDER_ACOTADO?3U:2U,now);return false;
    }
    if(Navigation_DeadlineExpired(now)){
        /* A motion timeout ends this attempt, not the entire supervised search. */
        if(navigation.state==NAV_GIRAR_ACOTADO || navigation.state==NAV_AVANZAR)
            (void)Local_Visit(&explore.evidence,o->x_m,o->y_m,o->theta_rad);
        Navigation_ExploreFailed(o);Navigation_ExploreWait(9U,now);return false;
    }
    if(!navigation_odometry.ready){
        if((TickType_t)(now-o->tick)>=(TickType_t)(now-navigation.phase_started_tick))return false;
        Navigation_RecordReference(o);
    }
    NavigationFinishReason_t outcome=Navigation_OdometryOutcome(o,now,&navigation_odometry.progress_mm);
    if(outcome==NAV_FINISH_NO_PROGRESS){Navigation_ExploreFailed(o);Navigation_ExploreWait(10U,now);return false;}
    if(outcome!=NAV_FINISH_NONE && outcome!=NAV_FINISH_TARGET){Navigation_StopFor(outcome);return false;}
    if(outcome==NAV_FINISH_TARGET || navigation.phase_complete){
        if(navigation.state==NAV_AVANZAR){explore.scan_deg=0;explore.retreated=false;}
        if(navigation.state==NAV_RETROCEDER_ACOTADO){navigation_diagnostics.flags|=NAV_DIAG_REVERSE_COMPLETED;Local_Reset(&explore.evidence,o->x_m,o->y_m);explore.scan_deg=0;}
        if(navigation.state==NAV_GIRAR_ACOTADO)navigation_diagnostics.turn_flags|=8U;
        Navigation_ExploreWait(0U,now);return false;
    }
    navigation_odometry.last_tick=o->tick;navigation_odometry.last_left=o->left_count;navigation_odometry.last_right=o->right_count;
    return true;
}
#endif
int Navigation_FormatExplore(char *out,size_t cap,uint32_t query,unsigned part){
#if NAV_AUTO2_ENABLED
    static struct {uint32_t session,start,challenge,block,segments,turns,retreats,scan,selected,evaluated,repeated,lease,sequence,faults,key,link_age,link_ack;
        uint16_t front,rear,observations;uint32_t settle_ms;uint8_t settle,state,action,available,link_ready,reason,stop;char sectors[37],ranges[109];} reply;
    static uint32_t q;static TickType_t captured;
    TickType_t now=xTaskGetTickCount();if(!query || !out || !cap || part>1U)return -1;
    if(!part){
        taskENTER_CRITICAL();memset(&reply,0,sizeof reply);reply.available=navigation_diagnostics.profile==NAV_PROFILE_EXPLORE;
        if(reply.available){
            reply.session=navigation_diagnostics.session_id;reply.start=navigation_diagnostics.started_tick;reply.state=(uint8_t)navigation.state;
            reply.challenge=explore.challenge;reply.action=explore.action;reply.block=explore.block;reply.segments=explore.segments;
            reply.turns=explore.turns;reply.retreats=explore.retreats;reply.scan=(uint32_t)(explore.scan_deg*1000.0f);
            reply.selected=(uint32_t)(explore.selected+1);reply.evaluated=explore.evaluated;reply.repeated=explore.repeated;
            reply.lease=0; /* No expiring navigation authorization. */
            reply.link_age=Navigation_DiagnosticMs((TickType_t)(now-explore.link_tick));reply.link_ack=explore.link_ack;
            reply.link_ready=explore.link_ready;reply.reason=navigation_diagnostics.reason;reply.stop=explore.stop_received;
            reply.settle=navigation_diagnostics.reverse_wait.settle_state;reply.observations=navigation_diagnostics.reverse_wait.settle_observations;reply.settle_ms=navigation_diagnostics.reverse_wait.settle_elapsed_ms;
            reply.sequence=0; /* Reserved former renewal sequence. */reply.front=navigation_diagnostics.front_mm;reply.rear=navigation_diagnostics.rear_mm;
            reply.key=explore.key;reply.faults=explore.critical_faults|TaskSafety_GetFaults();Local_Expire(&explore.evidence,now);
        }
        for(unsigned i=0;i<36U;++i){reply.sectors[i]=(char)('0'+(reply.available?explore.evidence.state[i]:0));
            unsigned cm=reply.available && explore.evidence.state[i]?explore.evidence.mm[i]/10U:0;
            static const char hex[]="0123456789abcdef";reply.ranges[3*i]=hex[(cm>>8)&15];reply.ranges[3*i+1]=hex[(cm>>4)&15];reply.ranges[3*i+2]=hex[cm&15];}
        q=query;captured=now;taskEXIT_CRITICAL();
    }else if(q!=query || (TickType_t)(now-captured)>=pdMS_TO_TICKS(1000U))return -1;
    int n=part?snprintf(out,cap,"EXPS,1,%lu,%lu,%lu,%s,%s\n",(unsigned long)q,(unsigned long)reply.session,(unsigned long)reply.start,reply.sectors,reply.ranges):
        snprintf(out,cap,"EXPL,1,%lu,%lu,%lu,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%lu,%u,%u,%lu,%lu,%lu,%lu,%u,%u,%u\n",
        (unsigned long)q,(unsigned long)reply.session,(unsigned long)reply.start,reply.state,reply.action,(unsigned long)reply.challenge,
        (unsigned long)reply.block,(unsigned long)reply.segments,(unsigned long)reply.turns,(unsigned long)reply.retreats,(unsigned long)reply.scan,
        (unsigned long)reply.selected,(unsigned long)reply.evaluated,(unsigned long)reply.repeated,(unsigned long)reply.lease,
        (unsigned long)reply.sequence,reply.front,reply.rear,reply.available,(unsigned long)reply.faults,reply.settle,reply.observations,(unsigned long)reply.settle_ms,(unsigned long)reply.key,(unsigned long)reply.link_age,(unsigned long)reply.link_ack,reply.link_ready,reply.reason,reply.stop);
    return n>0 && n<=255 && (size_t)n<cap?n:-1;
#else
    (void)out;(void)cap;(void)query;(void)part;return -1;
#endif
}

void Navigation_Step(void) {
    if (!TaskControlRobot_IsCurrentTask()) return;
    taskENTER_CRITICAL();
    bool active = navigation.active;
    uint32_t session = navigation.session;
    taskEXIT_CRITICAL();
    if (!active) return;

    /* La copia no bloqueante queda fuera de toda seccion critica. */
    SensoresDatos_t data;
    bool available = TaskSensores_GetData(&data) == pdTRUE;
#if NAV_AUTO2_ENABLED
    Navigation_GateSensors(&data,available,session); /* Same critical policy, outside the outer lock. */
#endif
#if NAV_AUTO1_ODOMETRY
    OdometrySnapshot odometry;
    Odometry_GetSnapshot(&odometry);
#if NAV_AUTO2_ENABLED
    if(navigation.profile==NAV_PROFILE_EXPLORE)Navigation_ExploreObserve();
#endif
#endif
    taskENTER_CRITICAL();
    TickType_t now = xTaskGetTickCount();
    if (!navigation.active || navigation.session != session) {
        taskEXIT_CRITICAL();
        return; /* STOP/Safety pudo cancelar durante la copia. */
    }
    Navigation_DiagnosticSensors(&data, available, now);
#if NAV_AUTO2_ENABLED
    if (!(Navigation_IsExplore()?Navigation_ExploreStep(&data,&odometry,available,now):Navigation_Auto2StepLocked(&data, &odometry, available, now))) {
        taskEXIT_CRITICAL();
        return;
    }
#else
    if (TaskControlRobot_GetState() != ROBOT_STATE_AUTONOMOUS ||
        TaskControlRobot_IsStopPending() || TaskSafety_IsStopActive() ||
        (TickType_t)(now - navigation.started_tick) >= pdMS_TO_TICKS(Navigation_SessionMaxMs()) ||
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
        (navigation.state == NAV_GIRAR_ACOTADO &&
         (TickType_t)(now - navigation.turn_started_tick) >= pdMS_TO_TICKS(Navigation_TurnMaxMs()))) {
#else
        (navigation.state == NAV_RETROCEDER_ACOTADO &&
         (TickType_t)(now - navigation.reverse_started_tick) >= pdMS_TO_TICKS(NAV_REVERSE_MAX_MS))) {
#endif
        Navigation_StopFor(TaskSafety_IsStopActive() ? NAV_FINISH_SAFETY :
            (TickType_t)(now-navigation.started_tick)>=pdMS_TO_TICKS(Navigation_SessionMaxMs()) ?
            NAV_FINISH_TIMEOUT : NAV_FINISH_CANCELLED);
        taskEXIT_CRITICAL();
        return;
    }
    uint32_t faults = available ? TaskSafety_EvaluateSnapshot(&data, now) : SAFETY_FAULT_SENSOR;
    bool finite = available && isfinite(data.distancia_m) && isfinite(data.distancia_trasera_m);
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
    if (navigation.state == NAV_RETROCEDER_ACOTADO &&
        (TickType_t)(now - navigation.reverse_started_tick) >= pdMS_TO_TICKS(NAV_REVERSE_MAX_MS)) {
        bool turn = Navigation_TurnReady(session, now) && faults == SAFETY_FAULT_NONE && finite &&
            data.distancia_trasera_m > (float)NAV_REAR_REVERSE_MIN_MM / 1000.0f;
        TaskMotorControl_Stop();
        if (turn) {
            navigation.active = true;
            navigation.state = NAV_GIRAR_ACOTADO;
            navigation.turn_started_tick = now;
            navigation.turn_used = true;
            navigation.finish_reason = NAV_FINISH_NONE;
        }
        taskEXIT_CRITICAL();
        return; /* Cero antes del giro; otra iteracion revalida sensores. */
    }
#endif
    /* Un obstaculo frontal vigente termina incluso la espera si aun falta el
     * trasero. No convertir una distancia frontal antigua en un dato vigente.
     */
    bool front_current = available && data.ultrasonido_valido != 0U &&
        (TickType_t)(now - data.timestamp) <= pdMS_TO_TICKS(TASK_SAFETY_SENSOR_TIMEOUT_MS) &&
        (TickType_t)(now - data.timestamp_ultrasonido_frontal) <=
            pdMS_TO_TICKS(TASK_SAFETY_ULTRASOUND_MAX_AGE_MS);
    if ((navigation.state == NAV_ESPERAR_DATOS || navigation.state == NAV_AVANZAR) && front_current &&
        data.distancia_m <= (float)NAV_FRONT_STOP_MM / 1000.0f) {
        navigation_diagnostics.flags |= NAV_DIAG_OBSTACLE;
        if (navigation.state == NAV_AVANZAR) { navigation_diagnostics.flags |= NAV_DIAG_REVERSE_CONSIDERED; }
        bool reverse_allowed = !NAV_AUTO1_ODOMETRY && navigation.state == NAV_AVANZAR &&
            faults == SAFETY_FAULT_NONE && finite &&
            data.distancia_trasera_m > (float)NAV_REAR_REVERSE_MIN_MM / 1000.0f;
        /* Revoca primero objetivo positivo, integral, generacion y permiso.
         * Solo esta transicion interna habilita otra fase bajo la misma
         * seccion critica. Cancel conserva started_tick: no renueva los 2 s.
         */
        Navigation_StopFor(NAV_FINISH_OBSTACLE);
        if (reverse_allowed) {
            navigation_diagnostics.flags |= NAV_DIAG_REVERSE_AUTHORIZED;
            navigation.active = true;
            navigation.state = NAV_RETROCEDER_ACOTADO;
            navigation.reverse_started_tick = now;
            navigation.finish_reason = NAV_FINISH_NONE;
        }
        taskEXIT_CRITICAL();
        return; /* Sin emision en este ciclo; revalidar en la proxima iteracion. */
    }
    if (faults != SAFETY_FAULT_NONE || !finite) {
        if (navigation.state != NAV_ESPERAR_DATOS ||
            (faults & (SAFETY_FAULT_OBSTACLE | SAFETY_FAULT_CLIFF)) != 0U) {
            Navigation_StopFor(NAV_FINISH_SENSORS);
        } /* Sin datos al iniciar: permanece esperando, con objetivos cero. */
        taskEXIT_CRITICAL();
        return;
    }
    if (navigation.state == NAV_RETROCEDER_ACOTADO &&
        data.distancia_trasera_m <= (float)NAV_REAR_REVERSE_MIN_MM / 1000.0f) {
        TaskMotorControl_Stop();
        taskEXIT_CRITICAL();
        return;
    }
#if NAV_AUTO1_ODOMETRY
    if (navigation.state != NAV_ESPERAR_DATOS && navigation.state != NAV_AVANZAR) {
        Navigation_StopFor(NAV_FINISH_CANCELLED);
        taskEXIT_CRITICAL();
        return;
    }
    if (!Navigation_OdometryFresh(&odometry, now)) {
        Navigation_DiagnosticOdometry(&odometry, now, 1, 1);
        Navigation_StopFor(NAV_FINISH_ODOMETRY);
        taskEXIT_CRITICAL();
        return;
    }
    if (!navigation_odometry.ready) {
        /* Exigir muestra estrictamente posterior a AUTO: no se puede probar
         * el orden dentro de un mismo tick. La espera consume parte de los 2 s. */
        if ((TickType_t)(now - odometry.tick) >= (TickType_t)(now - navigation.started_tick)) {
            taskEXIT_CRITICAL();
            return;
        }
        Navigation_RecordReference(&odometry);
    }
    NavigationFinishReason_t outcome = Navigation_OdometryOutcome(&odometry, now, &navigation_odometry.progress_mm);
    if (outcome != NAV_FINISH_NONE) {
        Navigation_StopFor(outcome);
        taskEXIT_CRITICAL();
        return;
    }
    navigation_odometry.last_tick = odometry.tick;
    navigation_odometry.last_left = odometry.left_count;
    navigation_odometry.last_right = odometry.right_count;
#endif
#endif /* Perfil AUTO-1/B7 conservado. */
    if ((TickType_t)(now - navigation.started_tick) == 0U ||
        (navigation.state == NAV_RETROCEDER_ACOTADO &&
         (TickType_t)(now - navigation.reverse_started_tick) == 0U)
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
        || (navigation.state == NAV_GIRAR_ACOTADO &&
            (TickType_t)(now - navigation.turn_started_tick) == 0U)
#endif
        ) {
        taskEXIT_CRITICAL();
        return; /* Igual tick que AUTO o parada de avance: no inventar orden. */
    }
    bool reversing = navigation.state == NAV_RETROCEDER_ACOTADO;
    bool turning = Navigation_TurnEnabled() && navigation.state == NAV_GIRAR_ACOTADO;
    if (!reversing && !turning) {
#if NAV_AUTO2_ENABLED
        if(Navigation_IsContinuous() && navigation.state==NAV_ESPERAR_DATOS) {
            navigation_diagnostics.continuous.segments=1U;navigation.phase_started_tick=now;
        }
#endif
        navigation.state = NAV_AVANZAR;
    }
    Navigation_DiagnosticPhase(navigation.state, now);
    uint32_t diagnostic_run_id = navigation_diagnostics.session_id;
    if (reversing) { navigation_diagnostics.flags |= NAV_DIAG_REVERSE_REQUESTED; }
    if (turning) navigation_diagnostics.turn_flags |= 1U;
    navigation.decision_tick = now;
    navigation.emission_ready = true;
    taskEXIT_CRITICAL();

    bool emitted;
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
    if (turning) emitted = Motion_NavigationTurn(now, session);
    else
#endif
    emitted = reversing ? Motion_NavigationReverse(now, session) :
                          Motion_NavigationAdvance(now, session);
    if (turning && emitted) {
        taskENTER_CRITICAL();
        if(navigation_diagnostics.session_id==diagnostic_run_id) navigation_diagnostics.turn_flags |= 2U;
        taskEXIT_CRITICAL();
    }
    if (reversing && emitted) {
        taskENTER_CRITICAL();
        if (navigation_diagnostics.session_id == diagnostic_run_id) {
            navigation_diagnostics.flags |= NAV_DIAG_REVERSE_PUBLISHED;
        }
        taskEXIT_CRITICAL();
    }
    if (!emitted) {
        taskENTER_CRITICAL();
        if (navigation.active && navigation.session == session) {
            Navigation_StopFor(navigation.profile!=NAV_PROFILE_EXPLORE && (TickType_t)(xTaskGetTickCount() - navigation.started_tick) >= pdMS_TO_TICKS(Navigation_SessionMaxMs()) ?
                NAV_FINISH_TIMEOUT : NAV_FINISH_EMISSION);
        }
        taskEXIT_CRITICAL();
    }
}
