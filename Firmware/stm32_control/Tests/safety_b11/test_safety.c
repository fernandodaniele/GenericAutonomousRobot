/* Real production sources, deterministic RTOS/HAL/encoder/sensor substitutes.
 * No copy of the safety/control algorithms and no claim of physical validation.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <math.h>
#include "main.h"
#include "task.h"
#include "queue.h"
#include "task_sensores.h"
#include "task_safety.h"
#include "task_control_robot.h"

static TickType_t clock_tick;
static SensoresDatos_t input;
static int queue_available, refresh_snapshot;
static int critical_depth, scheduler_depth, cycles, delay_calls;
static int peek_calls, suspended_peeks, suspend_calls, resume_calls;
static int peek_action, suspend_action, overwrite_action, receive_action;
static int fault_at_inner_exit, fault_at_next_outer_exit, pending_fault;
static int mode_at_second_delay;
static unsigned tests, unsafe_pwm_writes, transaction_writes, max_transaction_writes;
static unsigned event_count;
static RobotControlEvent_t events[8];
static jmp_buf finish;

#include "../../Core/Src/task_safety.c"
#include "../../Core/Src/task_control_robot.c"
#include "../../Core/Src/drv_motor.c"
#include "../../Core/Src/task_motor_control.c"

static TaskMotorControlTarget_t queued_target;
static int target_pending;
static TIM_HandleTypeDef pwm;
static GPIO_TypeDef pins;
static MotorHandle_t left_motor, right_motor;
static EncoderHandle_t left_encoder, right_encoder;
static int32_t left_encoder_delta, right_encoder_delta;
static unsigned left_encoder_reads, right_encoder_reads, pi_calls;

/* Host-only function-entry instrumentation: counts the real PI, not a wrapper. */
void __attribute__((no_instrument_function))
__cyg_profile_func_enter(void *function, void *caller) {
    (void)caller;
    if (function == (void *)MotorPI_Calculate) ++pi_calls;
}
void __attribute__((no_instrument_function))
__cyg_profile_func_exit(void *function, void *caller) {
    (void)function; (void)caller;
}

#define CHECK(value) do { if (!(value)) { \
    fprintf(stderr, "FAIL line %u: %s\n", (unsigned)__LINE__, #value); exit(1); \
} } while (0)
#define PASS(label) do { CHECK(critical_depth == 0 && scheduler_depth == 0); \
    CHECK(unsafe_pwm_writes == 0U); CHECK(max_transaction_writes <= 6U); \
    printf("PASS %02u %s\n", ++tests, label); } while (0)

static void deliver_fault(void) {
    if (pending_fault == 1 && critical_depth == 0 && scheduler_depth == 0) {
        pending_fault = 0;
        TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    }
}
void test_enter(void) {
    if (critical_depth == 0) transaction_writes = 0U;
    ++critical_depth;
}
void test_exit(void) {
    CHECK(critical_depth > 0);
    --critical_depth;
    // Models a higher-priority task made ready after the nested flag read.
    if (critical_depth == 1 && fault_at_inner_exit) {
        if (--fault_at_inner_exit == 0) pending_fault = 1;
    }
    if (critical_depth == 0) {
        if (transaction_writes > max_transaction_writes)
            max_transaction_writes = transaction_writes;
        if (fault_at_next_outer_exit) {
            fault_at_next_outer_exit = 0; pending_fault = 1;
        }
        deliver_fault();
    }
}
TickType_t xTaskGetTickCount(void) { return clock_tick; }
void vTaskDelayUntil(TickType_t *last, TickType_t period) {
    CHECK(critical_depth == 0 && scheduler_depth == 0);
    if (++delay_calls > cycles) longjmp(finish, 1);
    *last += period; clock_tick = *last;
    if (mode_at_second_delay && delay_calls == 2) {
        RobotControlEvent_t mode = mode_at_second_delay == 1
                                  ? ROBOT_EVENT_START_MANUAL : ROBOT_EVENT_START_AUTO;
        mode_at_second_delay = 0;
        TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
        CHECK(pwm.compare[0] == 0U && pwm.compare[1] == 0U);
        TaskControlRobot_ProcessEvent(mode);
    }
}
void vTaskSuspendAll(void) {
    CHECK(critical_depth == 0 && scheduler_depth == 0);
    if (suspend_action == 1) {
        suspend_action = 0;
        TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    }
    ++scheduler_depth; ++suspend_calls;
}
BaseType_t xTaskResumeAll(void) {
    CHECK(critical_depth == 0 && scheduler_depth == 1);
    --scheduler_depth; ++resume_calls; deliver_fault(); return pdFALSE;
}
BaseType_t xTaskCreate(void (*entry)(void *), const char *name, unsigned stack,
                      void *arg, UBaseType_t priority, TaskHandle_t *out) {
    (void)entry; (void)name; (void)stack; (void)arg; (void)priority;
    if (out != NULL) *out = (void *)1;
    return pdPASS;
}
void Error_Handler(void) { fprintf(stderr, "Unexpected Error_Handler\n"); exit(1); }
BaseType_t TaskSensores_GetData(SensoresDatos_t *out) {
    CHECK(critical_depth == 0);
    ++peek_calls;
    if (scheduler_depth) { ++suspended_peeks; CHECK(peek_action == 0); }
    if (!queue_available) return pdFALSE;
    if (refresh_snapshot) input.timestamp = clock_tick;
    *out = input;
    // Publication/fault between the first copy and the final scheduler lock.
    int action = peek_action; peek_action = 0;
    if (action == 1) input.hay_caida = 1U;
    if (action == 2) input.ultrasonido_valido = 0U;
    if (action == 3) input.distancia_trasera_m = 0.10f;
    if (action == 4) TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    if (action == 5) queue_available = 0;
    if (action == 6) input.ultrasonido_trasero_valido = 0U;
    if (scheduler_depth && pending_fault == 2) pending_fault = 1;
    return pdTRUE;
}
QueueHandle_t xQueueCreate(UBaseType_t count, UBaseType_t size) {
    CHECK(critical_depth == 0);
    if (count == 1U) { CHECK(size == sizeof(TaskMotorControlTarget_t)); return (void *)2; }
    CHECK(count == 8U && size == sizeof(RobotControlEvent_t)); return (void *)1;
}
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait) {
    CHECK(critical_depth == 0 && wait == 0U && queue == (void *)1);
    if (event_count == 8U) return pdFALSE;
    events[event_count++] = *(const RobotControlEvent_t *)item; return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait) {
    CHECK(critical_depth == 0 && scheduler_depth == 0 && wait == 0U);
    if (queue == (void *)2) {
        if (!target_pending) return pdFALSE;
        *(TaskMotorControlTarget_t *)out = queued_target; target_pending = 0;
        int action = receive_action; receive_action = 0;
        if (action) {
            TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
            TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
        }
        return pdTRUE;
    }
    CHECK(queue == (void *)1);
    if (!event_count) return pdFALSE;
    *(RobotControlEvent_t *)out = events[0];
    --event_count; memmove(events, events + 1, event_count * sizeof(*events));
    return pdTRUE;
}
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item) {
    CHECK(critical_depth == 0 && scheduler_depth == 0 && queue == (void *)2);
    int action = overwrite_action; overwrite_action = 0;
    if (action == 1) {
        TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
        TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
    }
    if (action == 2) TaskMotorControl_SetTargetSpeeds(0, 0);
    queued_target = *(const TaskMotorControlTarget_t *)item;
    target_pending = 1; return pdPASS;
}
BaseType_t xQueueReset(QueueHandle_t queue) {
    CHECK(critical_depth == 0 && queue == (void *)1);
    event_count = 0; return pdPASS;
}
void vQueueDelete(QueueHandle_t queue) { CHECK(critical_depth == 0); (void)queue; }
void HAL_GPIO_WritePin(GPIO_TypeDef *gpio, uint16_t pin, GPIO_PinState value) {
    if (critical_depth) ++transaction_writes;
    if (value == GPIO_PIN_SET) gpio->pins |= pin;
    else gpio->pins &= (uint16_t)~pin;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    CHECK(critical_depth == 0); (void)timer; (void)channel; return HAL_OK;
}
void test_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value) {
    CHECK(channel < 4U);
    if (critical_depth) ++transaction_writes;
    if (value != 0U) {
        CHECK(critical_depth > 0);
        if (safety_stop_active) ++unsafe_pwm_writes;
    }
    timer->compare[channel] = value;
}
int32_t Encoder_ReadDelta(EncoderHandle_t *encoder) {
    CHECK(critical_depth == 0);
    if (encoder == &left_encoder) { ++left_encoder_reads; return left_encoder_delta; }
    CHECK(encoder == &right_encoder);
    ++right_encoder_reads; return right_encoder_delta;
}

static void setup(void) {
    CHECK(critical_depth == 0 && scheduler_depth == 0);
    clock_tick = 1000U;
    input = (SensoresDatos_t){0};
    input.timestamp = clock_tick;
    input.distancia_m = input.distancia_trasera_m = 0.30f;
    input.ultrasonido_valido = input.ultrasonido_trasero_valido = 1U;
    input.timestamp_ultrasonido_frontal = clock_tick - 20U;
    input.timestamp_ultrasonido_trasero = clock_tick - 90U;
    queue_available = 1; refresh_snapshot = 0;
    cycles = delay_calls = peek_calls = suspended_peeks = suspend_calls = resume_calls = 0;
    peek_action = suspend_action = overwrite_action = receive_action = 0;
    fault_at_inner_exit = fault_at_next_outer_exit = pending_fault = mode_at_second_delay = 0;
    unsafe_pwm_writes = transaction_writes = max_transaction_writes = event_count = 0U;
    target_pending = 0;
    left_encoder_delta = right_encoder_delta = 0;
    left_encoder_reads = right_encoder_reads = pi_calls = 0U;
    safety_stop_active = false; safety_faults = SAFETY_FAULT_NONE;
    safety_fault_revision = 0U; safety_fault_tick = 0U; safety_has_fault = false;
    safety_task_handle = NULL; robot_state = ROBOT_STATE_MANUAL; event_queue = (void *)1;
    command_generation = 0U;
    memset(&task_context, 0, sizeof(task_context));
    memset(&pwm, 0, sizeof(pwm)); memset(&pins, 0, sizeof(pins));
    left_motor = (MotorHandle_t){ .htim = &pwm, .channel = 0U,
        .port_a = &pins, .pin_a = 1U, .port_b = &pins, .pin_b = 2U };
    right_motor = (MotorHandle_t){ .htim = &pwm, .channel = 1U,
        .port_a = &pins, .pin_a = 4U, .port_b = &pins, .pin_b = 8U };
    TaskMotorControl_Create(&left_encoder, &right_encoder, &left_motor, &right_motor);
}
static void run_safety(int count) {
    cycles = count; delay_calls = 0;
    if (setjmp(finish) == 0) TaskSafety_Run(NULL);
}
static void run_motor(int count) {
    cycles = count; delay_calls = 0;
    if (setjmp(finish) == 0) TaskMotorControl_Run(&task_context);
}
static void run_fsm(int count) {
    cycles = count; delay_calls = 0;
    if (setjmp(finish) == 0) TaskControlRobot_Run(NULL);
}
static void recovery(void) {
    setup(); TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    clock_tick += 100U;
    input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = clock_tick - 70U;
    input.timestamp_ultrasonido_trasero = clock_tick - 10U;
}
static void expect_zero(void) {
    CHECK(pwm.compare[0] == 0U && pwm.compare[1] == 0U);
    CHECK(task_context.left_target_cps == 0 && task_context.right_target_cps == 0);
    CHECK(task_context.left_integral == 0.0f && task_context.right_integral == 0.0f);
    CHECK(task_context.left_pwm_command == 0 && task_context.right_pwm_command == 0);
}
static void residual_motion(void) {
    left_encoder_delta = 2;
    right_encoder_delta = -3;
}
static void expect_residual_speed(void) {
    CHECK(task_context.left_speed_cps == 200 && task_context.right_speed_cps == -300);
    CHECK(left_encoder_reads >= 2U && right_encoder_reads >= 2U);
}

int main(void) {
    CHECK(TaskSafety_IsStopActive() && robot_state == ROBOT_STATE_IDLE);
    PASS("startup blocked");
    setup(); TaskSafety_Create(); CHECK(TaskSafety_IsStopActive());
    PASS("Safety creation preserves startup block");
    setup(); queue_available = 0; safety_stop_active = true;
    CHECK(!TaskSafety_RequestReset()); PASS("reset needs a published snapshot");
    setup(); queue_available = 0; safety_stop_active = true; run_safety(5);
    CHECK(TaskSafety_GetFaults() == SAFETY_FAULT_NONE); PASS("startup grace through 100 ms");
    setup(); queue_available = 0; run_safety(6);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_SENSOR); expect_zero();
    PASS("missing queue after startup faults and stops");
    setup(); run_safety(1); CHECK(!TaskSafety_IsStopActive()); PASS("both current and safe");
    setup(); input.timestamp = clock_tick - 80U; run_safety(1);
    CHECK(!TaskSafety_IsStopActive()); PASS("snapshot exact age 100 accepted");
    setup(); input.timestamp = clock_tick - 81U; run_safety(1);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_SENSOR); PASS("snapshot age 101 rejected");
    setup(); input.timestamp_ultrasonido_frontal = clock_tick - 181U; run_safety(1);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_SENSOR); PASS("fresh snapshot old front rejected");
    setup(); input.timestamp_ultrasonido_trasero = clock_tick - 181U; run_safety(1);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_SENSOR); PASS("fresh snapshot old rear rejected");
    setup(); input.timestamp_ultrasonido_frontal = clock_tick - 180U;
    input.timestamp_ultrasonido_trasero = clock_tick - 180U; run_safety(1);
    CHECK(!TaskSafety_IsStopActive()); PASS("both exact age 200 accepted");
    setup(); input.ultrasonido_valido = 0U; run_safety(1);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_SENSOR); PASS("invalid front stops");
    setup(); input.ultrasonido_trasero_valido = 0U; run_safety(1);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_SENSOR); PASS("invalid rear stops");
    setup(); input.distancia_m = 0.20f; run_safety(1);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_OBSTACLE); PASS("front exact 0.20 stops");
    setup(); input.distancia_m = 0.19f; run_safety(1);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_OBSTACLE); PASS("front below 0.20 stops");
    setup(); input.distancia_trasera_m = 0.20f; run_safety(1);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_OBSTACLE); PASS("rear exact 0.20 stops");
    setup(); input.distancia_trasera_m = 0.19f; run_safety(1);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_OBSTACLE); PASS("rear below 0.20 stops");
    setup(); input.distancia_m = input.distancia_trasera_m = 0.2001f; run_safety(1);
    CHECK(!TaskSafety_IsStopActive()); PASS("both strictly above 0.20 accepted");
    setup(); input.hay_caida = 1U; run_safety(1);
    CHECK(TaskSafety_GetFaults() & SAFETY_FAULT_CLIFF); PASS("IR cliff stops");
    setup(); input.distancia_trasera_m = NAN; run_safety(1);
    CHECK(TaskSafety_IsStopActive()); PASS("NaN cannot satisfy safe distance");
    setup(); clock_tick = 20U; input.timestamp = UINT32_MAX - 29U;
    input.timestamp_ultrasonido_frontal = UINT32_MAX - 99U;
    input.timestamp_ultrasonido_trasero = UINT32_MAX - 149U;
    CHECK(TaskSafety_RequestReset()); PASS("snapshot and sensor ages cross tick wrap");
    setup(); clock_tick = 20U; input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = UINT32_MAX - 180U;
    CHECK(!TaskSafety_RequestReset()); PASS("sensor age 201 crossing wrap rejected");
    setup(); clock_tick = 20U; input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = input.timestamp_ultrasonido_trasero = UINT32_MAX - 179U;
    CHECK(TaskSafety_RequestReset()); PASS("sensor age exactly 200 crossing wrap accepted");
    setup(); clock_tick = 20U; input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = input.timestamp_ultrasonido_trasero = 0U;
    safety_stop_active = true; CHECK(TaskSafety_RequestReset());
    PASS("legitimate timestamp zero at startup accepted without fault");
    setup(); clock_tick = 0U; input.timestamp = 0U;
    input.timestamp_ultrasonido_frontal = input.timestamp_ultrasonido_trasero = 0U;
    run_safety(1); CHECK(!TaskSafety_IsStopActive());
    PASS("supervision accepts valid acquisition tick zero");
    setup(); safety_stop_active = true; input.ultrasonido_valido = 0U;
    CHECK(!TaskSafety_RequestReset()); CHECK(TaskSafety_IsStopActive());
    PASS("unsafe reset remains blocked");
    setup(); input.ultrasonido_trasero_valido = 0U;
    CHECK(!TaskSafety_RequestReset()); PASS("reset rejects invalid rear");
    setup(); input.timestamp = clock_tick - 101U;
    CHECK(!TaskSafety_RequestReset()); PASS("reset rejects stale snapshot");
    setup(); input.timestamp_ultrasonido_frontal = clock_tick - 201U;
    CHECK(!TaskSafety_RequestReset()); PASS("reset rejects old front despite fresh snapshot");
    setup(); input.timestamp_ultrasonido_trasero = clock_tick - 201U;
    CHECK(!TaskSafety_RequestReset()); PASS("reset rejects old rear despite fresh snapshot");
    setup(); input.timestamp_ultrasonido_frontal = clock_tick - 200U;
    input.timestamp_ultrasonido_trasero = clock_tick - 200U;
    CHECK(TaskSafety_RequestReset()); PASS("reset accepts exact individual age 200 without fault");
    setup(); input.distancia_m = 0.20f;
    CHECK(!TaskSafety_RequestReset()); PASS("reset rejects front at obstacle limit");
    setup(); input.distancia_trasera_m = 0.20f;
    CHECK(!TaskSafety_RequestReset()); PASS("reset rejects rear at obstacle limit");
    setup(); input.hay_caida = 1U;
    CHECK(!TaskSafety_RequestReset()); PASS("reset rejects cliff");
    setup(); TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    CHECK(!TaskSafety_RequestReset()); PASS("pre-fault captures cannot rearm");
    recovery(); input.timestamp_ultrasonido_trasero = safety_fault_tick - 1U;
    CHECK(!TaskSafety_RequestReset()); PASS("one pre-fault capture prevents rearm");
    recovery(); CHECK(TaskSafety_RequestReset()); CHECK(!TaskSafety_IsStopActive());
    CHECK(suspended_peeks == 1 && suspend_calls == 1 && resume_calls == 1);
    expect_zero(); PASS("new safe captures rearm with one final locked snapshot");
    recovery(); peek_action = 4; CHECK(!TaskSafety_RequestReset());
    CHECK(TaskSafety_IsStopActive()); PASS("concurrent ReportFault prevents rearm");
    recovery(); peek_action = 1; CHECK(!TaskSafety_RequestReset());
    CHECK(peek_calls == 2); PASS("new cliff publication during reset rejected");
    recovery(); peek_action = 2; CHECK(!TaskSafety_RequestReset());
    PASS("new invalid front publication during reset rejected");
    recovery(); peek_action = 3; CHECK(!TaskSafety_RequestReset());
    PASS("new rear obstacle publication during reset rejected");
    recovery(); peek_action = 6; CHECK(!TaskSafety_RequestReset());
    PASS("new invalid rear publication during reset rejected");
    recovery(); peek_action = 5; CHECK(!TaskSafety_RequestReset());
    CHECK(resume_calls == 1); PASS("lost final snapshot resumes scheduler and rejects reset");
    recovery(); suspend_action = 1; CHECK(!TaskSafety_RequestReset());
    PASS("fault just before scheduler suspension preserved");
    recovery(); pending_fault = 2; CHECK(!TaskSafety_RequestReset());
    CHECK(TaskSafety_IsStopActive()); expect_zero();
    PASS("fault task released on resume blocks before reset returns");
    setup(); TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    input.timestamp_ultrasonido_frontal = input.timestamp_ultrasonido_trasero = clock_tick;
    CHECK(!TaskSafety_RequestReset()); PASS("capture and fault in same tick rejected");
    setup(); input.timestamp_ultrasonido_frontal = input.timestamp_ultrasonido_trasero = clock_tick;
    TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    CHECK(!TaskSafety_RequestReset()); PASS("capture before fault within same tick rejected");
    setup(); TaskSafety_ReportFault(SAFETY_FAULT_SENSOR); ++clock_tick;
    input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = input.timestamp_ultrasonido_trasero = clock_tick;
    CHECK(TaskSafety_RequestReset()); PASS("strictly later capture tick accepted");
    setup(); clock_tick = UINT32_MAX - 20U; TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    clock_tick = 10U; input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = 0U; input.timestamp_ultrasonido_trasero = 1U;
    CHECK(TaskSafety_RequestReset()); PASS("post-fault capture zero after wrap accepted");
    setup(); clock_tick = 10U; TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = UINT32_MAX - 5U;
    input.timestamp_ultrasonido_trasero = 9U;
    CHECK(!TaskSafety_RequestReset()); PASS("pre-fault capture crossing wrap rejected");
    recovery(); TaskSafety_ReportFault(SAFETY_FAULT_EMERGENCY);
    clock_tick += 10U; input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = input.timestamp_ultrasonido_trasero = clock_tick;
    CHECK(!TaskSafety_RequestReset()); PASS("emergency remains non-resettable");
    recovery(); robot_state = ROBOT_STATE_SAFETY_STOP;
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_RESET);
    CHECK(robot_state == ROBOT_STATE_IDLE && event_count == 0U);
    expect_zero(); PASS("FSM reset stays IDLE clears events and never moves");
    setup(); safety_stop_active = true; robot_state = ROBOT_STATE_SAFETY_STOP;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_AUTO);
    CHECK(robot_state == ROBOT_STATE_SAFETY_STOP); expect_zero();
    PASS("safety stop rejects both mode enable events");
    recovery(); robot_state = ROBOT_STATE_SAFETY_STOP;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_RESET);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
    TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_STOP);
    CHECK(robot_state == ROBOT_STATE_MANUAL && pwm.compare[0] > 0U);
    PASS("obsolete safety event cannot erase a new post-reset command");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100);
    TaskSafety_ReportFault(SAFETY_FAULT_SENSOR); clock_tick += 100U;
    input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = clock_tick - 70U;
    input.timestamp_ultrasonido_trasero = clock_tick - 10U;
    robot_state = ROBOT_STATE_SAFETY_STOP;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_RESET);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL); run_motor(1);
    expect_zero(); PASS("pending pre-fault command cannot survive reset and MANUAL");
    setup(); input.ultrasonido_valido = 0U; run_safety(1);
    clock_tick += 100U; input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = input.timestamp_ultrasonido_trasero = clock_tick;
    input.ultrasonido_valido = 1U; run_safety(1);
    CHECK(TaskSafety_IsStopActive()); PASS("healthy recovery never automatically rearms");

    setup(); TaskMotorControl_SetTargetSpeeds(100, 100);
    CHECK(queued_target.issued_tick == clock_tick); run_motor(1);
    CHECK(pwm.compare[0] > 0U && pwm.compare[1] > 0U);
    PASS("fresh command timestamped at emission applies PI");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); clock_tick += 489U; run_motor(1);
    CHECK(pwm.compare[0] > 0U); PASS("command age 499 at consumption accepted");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); clock_tick += 490U; run_motor(1);
    expect_zero(); PASS("command age exactly 500 at consumption rejected");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); clock_tick += 1000U; run_motor(1);
    expect_zero(); PASS("old queue command cannot become fresh on consumption");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(50);
    expect_zero(); PASS("active command expires 500 after emission not dequeue");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100);
    TaskMotorControlTarget_t old = queued_target; run_motor(1);
    clock_tick = 1490U; (void)xQueueOverwrite(target_speed_queue, &old); run_motor(1);
    expect_zero(); PASS("reenqueue does not extend original 500 ms lifetime");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1);
    TaskMotorControl_Stop(); expect_zero();
    PASS("stop immediately clears PWM targets and integrals");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL); run_motor(1);
    CHECK(robot_state == ROBOT_STATE_MANUAL); expect_zero();
    PASS("same-tick STOP MANUAL discards queued previous command");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_AUTO); run_motor(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); expect_zero();
    PASS("same-tick STOP AUTO discards queued previous command");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
    CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("STOP clears live PI and PWM before next motor iteration");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100);
    mode_at_second_delay = 1; run_motor(2); expect_zero();
    PASS("STOP MANUAL between motor iterations clears already active targets");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100);
    mode_at_second_delay = 2; run_motor(2); expect_zero();
    PASS("STOP AUTO between motor iterations clears already active targets");
    setup(); overwrite_action = 1; TaskMotorControl_SetTargetSpeeds(100, 100);
    run_motor(1); expect_zero(); PASS("STOP between emission and enqueue rejects old epoch");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); receive_action = 1;
    run_motor(1); expect_zero(); PASS("STOP after dequeue before acceptance rejects old epoch");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
    TickType_t same_tick = clock_tick;
    TaskMotorControl_SetTargetSpeeds(200, 200);
    CHECK(queued_target.issued_tick == same_tick); run_motor(1);
    CHECK(task_context.left_target_cps == 200 && pwm.compare[0] > 0U);
    PASS("new same-tick post-STOP command is accepted with new epoch");
    setup(); command_generation = UINT32_MAX - 1U;
    TaskMotorControl_SetTargetSpeeds(100, 100); TaskMotorControl_Stop();
    CHECK(command_generation == 0U); run_motor(1); expect_zero();
    PASS("epoch rollover rejects previous epoch");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100);
    TaskMotorControl_SetTargetSpeeds(200, 200); run_motor(1);
    CHECK(task_context.left_target_cps == 200 && pwm.compare[0] > 0U);
    PASS("multiple emissions within one tick keep latest command");
    setup(); overwrite_action = 2; TaskMotorControl_SetTargetSpeeds(100, 100);
    run_motor(1); expect_zero();
    PASS("late old enqueue cannot override newer same-tick zero command");
    setup(); clock_tick = UINT32_MAX - 100U;
    TaskMotorControl_SetTargetSpeeds(100, 100); clock_tick = 20U; run_motor(1);
    CHECK(pwm.compare[0] > 0U); PASS("fresh command crosses tick wrap");
    setup(); clock_tick = UINT32_MAX - 400U;
    TaskMotorControl_SetTargetSpeeds(100, 100); clock_tick = 100U; run_motor(1);
    expect_zero(); PASS("expired command crossing tick wrap rejected");
    setup(); safety_stop_active = true; TaskMotorControl_SetTargetSpeeds(100, 100);
    CHECK(!target_pending); run_motor(1); expect_zero(); PASS("blocked emission rejected");
    setup(); robot_state = ROBOT_STATE_IDLE; TaskMotorControl_SetTargetSpeeds(100, 100);
    CHECK(!target_pending); run_motor(1); expect_zero(); PASS("IDLE emission rejected");
    setup(); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    TaskMotorControl_SetTargetSpeeds(100, 100); run_fsm(1); run_motor(1);
    expect_zero(); PASS("real FSM queue drains STOP MANUAL without restoring motion");
    setup(); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_AUTO));
    TaskMotorControl_SetTargetSpeeds(100, 100); run_fsm(1); run_motor(1);
    expect_zero(); PASS("real FSM queue drains STOP AUTO without restoring motion");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1);
    for (unsigned i = 0U; i < 8U; ++i)
        CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    input.ultrasonido_trasero_valido = 0U; run_safety(1);
    CHECK(TaskSafety_IsStopActive()); expect_zero();
    PASS("full FSM queue cannot delay safety PWM zero");

    setup(); fault_at_next_outer_exit = 1; Motor_SetSpeed(&left_motor, 100);
    CHECK(TaskSafety_IsStopActive()); expect_zero();
    PASS("fault at driver transaction exit immediately clears PWM");
    setup(); fault_at_inner_exit = 1; Motor_SetSpeed(&left_motor, 100);
    CHECK(TaskSafety_IsStopActive()); expect_zero();
    PASS("preemption after flag copy defers until PWM transaction closes");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); fault_at_inner_exit = 2;
    run_motor(1); CHECK(TaskSafety_IsStopActive()); expect_zero();
    PASS("real motor task PI and both outputs serialize with fault activation");
    setup(); Motor_SetSpeed(&left_motor, 100); Motor_SetSpeed(&right_motor, -100);
    TaskSafety_ReportFault(SAFETY_FAULT_SENSOR); expect_zero();
    Motor_SetSpeed(&left_motor, 100); Motor_SetSpeed(&right_motor, -100);
    expect_zero(); PASS("latched block rejects both PWM signs after immediate stop");
    setup(); TaskMotorControl_Stop(); expect_zero(); TaskMotorControl_Stop(); expect_zero();
    CHECK(critical_depth == 0); PASS("nested repeated stops are balanced and nonblocking");
    setup(); Motor_SetSpeed(&left_motor, 0); CHECK(pwm.compare[0] == 0U);
    CHECK(critical_depth == 0); PASS("zero speed driver early return releases critical section");
    setup(); Motor_Init(&left_motor); CHECK(pwm.compare[0] == 0U);
    PASS("motor initialization remains PWM zero");

    setup(); residual_motion(); run_motor(3);
    expect_zero(); expect_residual_speed(); CHECK(pi_calls == 0U);
    CHECK(left_encoder_reads == 4U && right_encoder_reads == 4U);
    PASS("residual speed without command is sampled but never invokes PI");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1);
    residual_motion(); TaskSafety_ReportFault(SAFETY_FAULT_SENSOR); pi_calls = 0U;
    run_motor(3); expect_zero(); expect_residual_speed(); CHECK(pi_calls == 0U);
    PASS("active block keeps PI and PWM zero despite residual speed");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1);
    residual_motion(); robot_state = ROBOT_STATE_IDLE; pi_calls = 0U;
    run_motor(3); expect_zero(); expect_residual_speed(); CHECK(pi_calls == 0U);
    PASS("IDLE clears previous PI and never integrates residual speed");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1);
    residual_motion(); TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP); pi_calls = 0U;
    run_motor(3); expect_zero(); expect_residual_speed(); CHECK(pi_calls == 0U);
    PASS("STOP keeps PI cleared on subsequent residual-motion cycles");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1);
    residual_motion(); clock_tick = 1490U; pi_calls = 0U;
    run_motor(3); expect_zero(); expect_residual_speed(); CHECK(pi_calls == 0U);
    PASS("active command timeout clears PI and skips residual integration");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); residual_motion();
    clock_tick = 1490U; run_motor(3);
    expect_zero(); expect_residual_speed(); CHECK(pi_calls == 0U);
    PASS("expired queued command cannot integrate residual speed");
    setup(); residual_motion(); TaskMotorControl_SetTargetSpeeds(0, 0);
    task_context.left_integral = 7.0f; task_context.right_integral = -9.0f;
    run_motor(3); expect_zero(); expect_residual_speed(); CHECK(pi_calls == 0U);
    PASS("explicit zero targets clear old integrals without invoking PI");
    setup(); residual_motion(); TaskMotorControl_SetTargetSpeeds(0, 100);
    task_context.left_integral = 7.0f; run_motor(3); expect_residual_speed();
    CHECK(task_context.left_integral == 0.0f && task_context.left_pwm_command == 0);
    CHECK(pwm.compare[0] == 0U && pwm.compare[1] > 0U);
    CHECK(task_context.right_integral > 0.0f && pi_calls == 3U);
    PASS("left individual stop skips PI while right motor remains regulated");
    setup(); residual_motion(); TaskMotorControl_SetTargetSpeeds(100, 0);
    task_context.right_integral = -9.0f; run_motor(3); expect_residual_speed();
    CHECK(task_context.right_integral == 0.0f && task_context.right_pwm_command == 0);
    CHECK(pwm.compare[1] == 0U && pwm.compare[0] > 0U);
    CHECK(task_context.left_integral < 0.0f && pi_calls == 3U);
    PASS("right individual stop skips PI while left motor remains regulated");
    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(3);
    CHECK(task_context.left_integral > 0.0f && task_context.right_integral > 0.0f);
    residual_motion(); TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL); pi_calls = 0U;
    run_motor(2); expect_zero(); expect_residual_speed(); CHECK(pi_calls == 0U);
    TaskMotorControl_SetTargetSpeeds(300, 100); run_motor(1);
    CHECK(fabsf(task_context.left_integral - 0.1f) < 1e-6f);
    CHECK(fabsf(task_context.right_integral - 0.4f) < 1e-6f);
    CHECK(pi_calls == 2U && pwm.compare[0] > 0U && pwm.compare[1] > 0U);
    PASS("new post-STOP command starts PI with only the new cycle contribution");
    setup(); residual_motion(); TaskMotorControl_SetTargetSpeeds(0, 100); run_motor(2);
    CHECK(task_context.left_integral == 0.0f);
    CHECK(fabsf(task_context.right_integral - 0.8f) < 1e-6f);
    pi_calls = 0U; TaskMotorControl_SetTargetSpeeds(300, 100); run_motor(1);
    CHECK(fabsf(task_context.left_integral - 0.1f) < 1e-6f);
    CHECK(fabsf(task_context.right_integral - 1.2f) < 1e-6f);
    CHECK(pi_calls == 2U);
    PASS("individual restart starts clean and preserves the other active PI");

    printf("RESULT %u tests passed; queues/waits outside critical sections; "
           "at most 6 GPIO/CCR writes per transaction. Hardware timing pending.\n", tests);
    return 0;
}
