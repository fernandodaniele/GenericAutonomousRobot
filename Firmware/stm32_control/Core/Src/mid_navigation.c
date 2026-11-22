#include "mid_navigation.h"
#include "task.h"
#include "task_control_robot.h"
#include "task_motor_control.h"
#include "task_sensores.h"
#include "task_safety.h"
#include "mid_motion.h"
#include "mid_odometry.h"
#include "odometry_config.h"
#include <limits.h>
#include <math.h>
#include <string.h>

static struct {
    NavigationState_t state;
    bool active;
    bool emission_ready;
    uint32_t session;
    TickType_t started_tick;
    TickType_t reverse_started_tick;
    TickType_t decision_tick;
    NavigationFinishReason_t finish_reason;
#if NAV_EXPERIMENTAL_TURN_ENABLED
    TickType_t turn_started_tick;
    bool reverse_applied;
    bool turn_used;
#endif
} navigation = { .state = NAV_DETENIDO };

#if NAV_AUTO1_ODOMETRY
static struct {
    bool ready;
    int64_t left, right, last_left, last_right;
    TickType_t reference_tick, last_tick;
    double left_mm_per_count, right_mm_per_count, progress_mm;
} navigation_odometry;

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

/* Calculo acotado. Una muestra repetida solo es coherente si conserva conteos.
 * El consumidor no necesita ejecutarse en cada periodo del productor (10 ms). */
static NavigationFinishReason_t Navigation_OdometryOutcome(
        const OdometrySnapshot *s, TickType_t now, double *progress) {
    int64_t left, right;
    if (!navigation_odometry.ready || !Navigation_OdometryFresh(s, now) ||
        (TickType_t)(now - s->tick) > (TickType_t)(now - navigation_odometry.last_tick) ||
        (s->tick == navigation_odometry.last_tick &&
         (s->left_count != navigation_odometry.last_left || s->right_count != navigation_odometry.last_right)) ||
        s->left_count < navigation_odometry.last_left || s->right_count < navigation_odometry.last_right ||
        !Navigation_SubtractCount(s->left_count, navigation_odometry.left, &left) ||
        !Navigation_SubtractCount(s->right_count, navigation_odometry.right, &right)) {
        return NAV_FINISH_ODOMETRY;
    }
    double left_mm = left * navigation_odometry.left_mm_per_count;
    double right_mm = right * navigation_odometry.right_mm_per_count;
    /* Se admiten conteos absolutos negativos, pero no desplazamiento negativo
     * en esta sesion de avance. La extension de coordenadas existente acota
     * saltos numericos; no supone una velocidad fisica maxima del motor. */
    if (left < 0 || right < 0 || !isfinite(left_mm) || !isfinite(right_mm) ||
        !(navigation_odometry.left_mm_per_count > 0) || !(navigation_odometry.right_mm_per_count > 0) ||
        left_mm > 2.0 * ODOM_POSITION_LIMIT_M * 1000.0 ||
        right_mm > 2.0 * ODOM_POSITION_LIMIT_M * 1000.0 ||
        fabs(left_mm - right_mm) > NAV_AUTO1_MAX_WHEEL_DIFFERENCE_MM) {
        return NAV_FINISH_ODOMETRY;
    }
    *progress = (left_mm + right_mm) * 0.5;
    return *progress >= NAV_AUTO1_TARGET_MM ? NAV_FINISH_TARGET : NAV_FINISH_NONE;
}
#endif

/* Bajo la seccion critica de navegacion; conserva la primera causa terminal. */
static void Navigation_StopFor(NavigationFinishReason_t reason) {
    if (navigation.finish_reason == NAV_FINISH_NONE) {
        navigation.finish_reason = reason;
    }
    TaskMotorControl_Stop(); /* Transaccion existente: generacion, permiso, PI y PWM. */
}

void Navigation_Begin(TickType_t entered_tick) {
    taskENTER_CRITICAL();
    ++navigation.session;
    navigation.started_tick = entered_tick;
    navigation.reverse_started_tick = 0U;
    navigation.finish_reason = NAV_FINISH_NONE;
#if NAV_AUTO1_ODOMETRY
    memset(&navigation_odometry, 0, sizeof navigation_odometry);
#endif
#if NAV_EXPERIMENTAL_TURN_ENABLED
    navigation.turn_started_tick = 0U;
    navigation.reverse_applied = false;
    navigation.turn_used = false;
#endif
    navigation.state = NAV_ESPERAR_DATOS;
    navigation.active = true;
    navigation.emission_ready = false;
    taskEXIT_CRITICAL();
}

void Navigation_Cancel(void) {
    taskENTER_CRITICAL();
    if (navigation.active && navigation.finish_reason == NAV_FINISH_NONE) {
        navigation.finish_reason = TaskSafety_IsStopActive() ? NAV_FINISH_SAFETY :
            (TickType_t)(xTaskGetTickCount() - navigation.started_tick) >= pdMS_TO_TICKS(NAV_SESSION_MAX_MS) ?
            NAV_FINISH_TIMEOUT : NAV_FINISH_CANCELLED;
    }
    ++navigation.session;
    navigation.active = false;
    navigation.emission_ready = false;
    navigation.state = NAV_DETENIDO;
#if NAV_EXPERIMENTAL_TURN_ENABLED
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

void Navigation_CancelSession(uint32_t session) {
    taskENTER_CRITICAL();
    if (navigation.active && navigation.session == session) Navigation_Cancel();
    taskEXIT_CRITICAL();
}

bool Navigation_IsCommandCurrent(uint32_t session, TickType_t now) {
    taskENTER_CRITICAL();
    bool current = navigation.active &&
        (navigation.state == NAV_AVANZAR || navigation.state == NAV_RETROCEDER_ACOTADO
#if NAV_EXPERIMENTAL_TURN_ENABLED
         || navigation.state == NAV_GIRAR_ACOTADO
#endif
        ) &&
        navigation.session == session &&
        (TickType_t)(now - navigation.started_tick) < pdMS_TO_TICKS(NAV_SESSION_MAX_MS) &&
        (navigation.state != NAV_RETROCEDER_ACOTADO ||
         (TickType_t)(now - navigation.reverse_started_tick) < pdMS_TO_TICKS(NAV_REVERSE_MAX_MS))
#if NAV_EXPERIMENTAL_TURN_ENABLED
        && (navigation.state != NAV_GIRAR_ACOTADO ||
            (TickType_t)(now - navigation.turn_started_tick) < pdMS_TO_TICKS(NAV_TURN_MAX_MS))
#endif
        ;
#if NAV_AUTO1_ODOMETRY
    current = current && navigation.state == NAV_AVANZAR && navigation_odometry.ready &&
        navigation.finish_reason == NAV_FINISH_NONE;
    if (current) {
        OdometrySnapshot s;
        double progress;
        Odometry_GetSnapshot(&s); /* Copia breve, sin colas ni esperas. */
        NavigationFinishReason_t outcome = Navigation_OdometryOutcome(&s, now, &progress);
        if (outcome != NAV_FINISH_NONE) {
            navigation.finish_reason = outcome;
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

#if NAV_EXPERIMENTAL_TURN_ENABLED
void Navigation_CommandApplied(uint32_t session, TickType_t now) {
    taskENTER_CRITICAL();
    if (navigation.state == NAV_RETROCEDER_ACOTADO && Navigation_IsCommandCurrent(session, now)) {
        navigation.reverse_applied = true;
    }
    taskEXIT_CRITICAL();
}

/* Requiere seccion critica. Se reserva la ventana completa de 250 ms, incluida
 * la siguiente iteracion sin comprimir el cronograma. Nunca renueva started_tick. */
static bool Navigation_TurnReady(uint32_t session, TickType_t now) {
    TickType_t age = (TickType_t)(now - navigation.started_tick);
    return !NAV_AUTO1_ODOMETRY && navigation.active && navigation.session == session &&
        navigation.state == NAV_RETROCEDER_ACOTADO && navigation.reverse_applied &&
        !navigation.turn_used &&
        (TickType_t)(now - navigation.reverse_started_tick) >= pdMS_TO_TICKS(NAV_REVERSE_MAX_MS) &&
        age < pdMS_TO_TICKS(NAV_SESSION_MAX_MS) &&
        pdMS_TO_TICKS(NAV_SESSION_MAX_MS) - age >= pdMS_TO_TICKS(NAV_TURN_MAX_MS);
}

bool Navigation_CompleteCommand(uint32_t session, TickType_t now) {
    taskENTER_CRITICAL();
    bool complete = Navigation_TurnReady(session, now);
    if (complete) navigation.emission_ready = false;
    taskEXIT_CRITICAL();
    return complete;
}
#endif

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
#if NAV_AUTO1_ODOMETRY
    OdometrySnapshot odometry;
    Odometry_GetSnapshot(&odometry);
#endif
    taskENTER_CRITICAL();
    TickType_t now = xTaskGetTickCount();
    if (!navigation.active || navigation.session != session) {
        taskEXIT_CRITICAL();
        return; /* STOP/Safety pudo cancelar durante la copia. */
    }
    if (TaskControlRobot_GetState() != ROBOT_STATE_AUTONOMOUS ||
        TaskControlRobot_IsStopPending() || TaskSafety_IsStopActive() ||
        (TickType_t)(now - navigation.started_tick) >= pdMS_TO_TICKS(NAV_SESSION_MAX_MS) ||
#if NAV_EXPERIMENTAL_TURN_ENABLED
        (navigation.state == NAV_GIRAR_ACOTADO &&
         (TickType_t)(now - navigation.turn_started_tick) >= pdMS_TO_TICKS(NAV_TURN_MAX_MS))) {
#else
        (navigation.state == NAV_RETROCEDER_ACOTADO &&
         (TickType_t)(now - navigation.reverse_started_tick) >= pdMS_TO_TICKS(NAV_REVERSE_MAX_MS))) {
#endif
        Navigation_StopFor(TaskSafety_IsStopActive() ? NAV_FINISH_SAFETY :
            (TickType_t)(now-navigation.started_tick)>=pdMS_TO_TICKS(NAV_SESSION_MAX_MS) ?
            NAV_FINISH_TIMEOUT : NAV_FINISH_CANCELLED);
        taskEXIT_CRITICAL();
        return;
    }
    uint32_t faults = available ? TaskSafety_EvaluateSnapshot(&data, now) : SAFETY_FAULT_SENSOR;
    bool finite = available && isfinite(data.distancia_m) && isfinite(data.distancia_trasera_m);
#if NAV_EXPERIMENTAL_TURN_ENABLED
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
        bool reverse_allowed = !NAV_AUTO1_ODOMETRY && navigation.state == NAV_AVANZAR &&
            faults == SAFETY_FAULT_NONE && finite &&
            data.distancia_trasera_m > (float)NAV_REAR_REVERSE_MIN_MM / 1000.0f;
        /* Revoca primero objetivo positivo, integral, generacion y permiso.
         * Solo esta transicion interna habilita otra fase bajo la misma
         * seccion critica. Cancel conserva started_tick: no renueva los 2 s.
         */
        Navigation_StopFor(NAV_FINISH_OBSTACLE);
        if (reverse_allowed) {
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
        navigation_odometry.left = navigation_odometry.last_left = odometry.left_count;
        navigation_odometry.right = navigation_odometry.last_right = odometry.right_count;
        navigation_odometry.reference_tick = navigation_odometry.last_tick = odometry.tick;
        navigation_odometry.left_mm_per_count =
            1000.0 * 3.14159265358979323846 * ODOM_LEFT_DIAMETER_M / ODOM_LEFT_WHEEL_CPR;
        navigation_odometry.right_mm_per_count =
            1000.0 * 3.14159265358979323846 * ODOM_RIGHT_DIAMETER_M / ODOM_RIGHT_WHEEL_CPR;
        navigation_odometry.ready = true;
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
    if ((TickType_t)(now - navigation.started_tick) == 0U ||
        (navigation.state == NAV_RETROCEDER_ACOTADO &&
         (TickType_t)(now - navigation.reverse_started_tick) == 0U)
#if NAV_EXPERIMENTAL_TURN_ENABLED
        || (navigation.state == NAV_GIRAR_ACOTADO &&
            (TickType_t)(now - navigation.turn_started_tick) == 0U)
#endif
        ) {
        taskEXIT_CRITICAL();
        return; /* Igual tick que AUTO o parada de avance: no inventar orden. */
    }
    bool reversing = navigation.state == NAV_RETROCEDER_ACOTADO;
    bool turning = NAV_EXPERIMENTAL_TURN_ENABLED && navigation.state == NAV_GIRAR_ACOTADO;
    if (!reversing && !turning) navigation.state = NAV_AVANZAR;
    navigation.decision_tick = now;
    navigation.emission_ready = true;
    taskEXIT_CRITICAL();

    bool emitted;
#if NAV_EXPERIMENTAL_TURN_ENABLED
    if (turning) emitted = Motion_NavigationTurn(now, session);
    else
#endif
    emitted = reversing ? Motion_NavigationReverse(now, session) :
                          Motion_NavigationAdvance(now, session);
    if (!emitted) {
        taskENTER_CRITICAL();
        if (navigation.active && navigation.session == session) {
            Navigation_StopFor((TickType_t)(xTaskGetTickCount() - navigation.started_tick) >= pdMS_TO_TICKS(NAV_SESSION_MAX_MS) ?
                NAV_FINISH_TIMEOUT : NAV_FINISH_EMISSION);
        }
        taskEXIT_CRITICAL();
    }
}
