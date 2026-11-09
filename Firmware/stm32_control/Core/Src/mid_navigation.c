#include "mid_navigation.h"
#include "task.h"
#include "task_control_robot.h"
#include "task_motor_control.h"
#include "task_sensores.h"
#include "task_safety.h"
#include "mid_motion.h"
#include <math.h>

static struct {
    NavigationState_t state;
    bool active;
    bool emission_ready;
    uint32_t session;
    TickType_t started_tick;
    TickType_t reverse_started_tick;
    TickType_t decision_tick;
#if NAV_EXPERIMENTAL_TURN_ENABLED
    TickType_t turn_started_tick;
    bool reverse_applied;
    bool turn_used;
#endif
} navigation = { .state = NAV_DETENIDO };

void Navigation_Begin(TickType_t entered_tick) {
    taskENTER_CRITICAL();
    ++navigation.session;
    navigation.started_tick = entered_tick;
    navigation.reverse_started_tick = 0U;
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
    return navigation.active && navigation.session == session &&
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
        TaskMotorControl_Stop();
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
        bool reverse_allowed = navigation.state == NAV_AVANZAR &&
            faults == SAFETY_FAULT_NONE && finite &&
            data.distancia_trasera_m > (float)NAV_REAR_REVERSE_MIN_MM / 1000.0f;
        /* Revoca primero objetivo positivo, integral, generacion y permiso.
         * Solo esta transicion interna habilita otra fase bajo la misma
         * seccion critica. Cancel conserva started_tick: no renueva los 2 s.
         */
        TaskMotorControl_Stop();
        if (reverse_allowed) {
            navigation.active = true;
            navigation.state = NAV_RETROCEDER_ACOTADO;
            navigation.reverse_started_tick = now;
        }
        taskEXIT_CRITICAL();
        return; /* Sin emision en este ciclo; revalidar en la proxima iteracion. */
    }
    if (faults != SAFETY_FAULT_NONE || !finite) {
        if (navigation.state != NAV_ESPERAR_DATOS ||
            (faults & (SAFETY_FAULT_OBSTACLE | SAFETY_FAULT_CLIFF)) != 0U) {
            TaskMotorControl_Stop();
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
        if (navigation.active && navigation.session == session) TaskMotorControl_Stop();
        taskEXIT_CRITICAL();
    }
}
