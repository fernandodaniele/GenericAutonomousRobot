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

static uint32_t test_get_autoreload(TIM_HandleTypeDef *timer);
#define __HAL_TIM_GET_AUTORELOAD(timer) test_get_autoreload(timer)

#include "task_control_robot.h"

static TickType_t clock_tick;
static TaskHandle_t current_task;
#define CONTROL_TASK_ID ((TaskHandle_t)10)
#define MOTOR_TASK_ID ((TaskHandle_t)11)
static SensoresDatos_t input;
static int queue_available, refresh_snapshot;
static int critical_depth, scheduler_depth, cycles, delay_calls;
static int peek_calls, suspended_peeks, suspend_calls, resume_calls;
static int peek_action, suspend_action, overwrite_action, receive_action;
static int event_send_action;
static int fault_at_inner_exit, fault_at_next_outer_exit, pending_fault;
static int mode_at_second_delay;
static TickType_t advance_tick_on_motor_receive;
static TickType_t advance_tick_before_navigation_emit;
static unsigned tests, unsafe_pwm_writes, transaction_writes, max_transaction_writes;
static unsigned event_count;
static jmp_buf finish;

#include "../../Core/Src/task_safety.c"
#include "../../Core/Src/task_control_robot.c"
#include "../../Core/Src/drv_motor.c"
#include "../../Core/Src/task_motor_control.c"
#include "../../Core/Src/mid_motion.c"
#include "../../Core/Src/drv_uart.c"
#include "../../Core/Src/task_comms.c"
#include "../../Core/Src/mid_navigation.c"
#include "../../Core/Src/mid_odometry.c"
#include "../../Core/Src/mid_mapping.c"
static TaskIMUSnapshot_t imu_snapshot;
static bool imu_snapshot_available;
static bool imu_cal_accepted;
static uint32_t imu_cal_number;
static TickType_t imu_cal_issued;
bool TaskIMU_GetSnapshot(TaskIMUSnapshot_t *out) {
    if (!imu_snapshot_available) return false;
    *out = imu_snapshot;
    return true;
}
bool TaskIMU_RequestCalibration(uint32_t request, TickType_t issued) {
    imu_cal_issued=issued;imu_cal_number=request;
    return imu_cal_accepted; /* Actual task admission is tested in imu_1. */
}
static TaskControlRobotEventMessage_t events[8];

/* Real encoder driver under distinct host-only names; task injection remains
 * available for deterministic PI/race tests. No copied delta algorithm. */
#define Encoder_Init ActualEncoder_Init
#define Encoder_Start ActualEncoder_Start
#define Encoder_Stop ActualEncoder_Stop
#define Encoder_ReadDelta ActualEncoder_ReadDelta
#include "../../Core/Src/drv_encoder.c"
#undef Encoder_Init
#undef Encoder_Start
#undef Encoder_Stop
#undef Encoder_ReadDelta

static TaskMotorControlTarget_t queued_target;
static int target_pending;
static TIM_HandleTypeDef pwm;
static GPIO_TypeDef pins;
static MotorHandle_t left_motor, right_motor;
static EncoderHandle_t left_encoder, right_encoder;
static int32_t left_encoder_delta, right_encoder_delta;
static unsigned left_encoder_reads, right_encoder_reads, pi_calls;
static uint32_t mock_pwm_arr;
static unsigned autoreload_reads;
/* Approved motor checkpoint: 40% of the normal TIM4 period (8400 counts). */
enum { TEST_PWM_LIMIT = 3360, TEST_SHORT_PERIOD_LIMIT = 800 };
static char uart_response[192];
static unsigned uart_rearms;
static void run_motor(int count);

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
        if (navigation.emission_ready && advance_tick_before_navigation_emit != 0U) {
            clock_tick += advance_tick_before_navigation_emit;
            advance_tick_before_navigation_emit = 0U;
        }
        if (transaction_writes > max_transaction_writes)
            max_transaction_writes = transaction_writes;
        if (fault_at_next_outer_exit) {
            fault_at_next_outer_exit = 0; pending_fault = 1;
        }
        deliver_fault();
    }
}
TickType_t xTaskGetTickCount(void) { return clock_tick; }
TickType_t xTaskGetTickCountFromISR(void) { return clock_tick; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return current_task; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t size) {
    (void)uart; CHECK(byte != NULL && size == 1U); ++uart_rearms; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *bytes,
                                   uint16_t size, uint32_t timeout) {
    (void)uart; CHECK(critical_depth == 0 && scheduler_depth == 0);
    CHECK(size < sizeof(uart_response) && timeout == 50U);
    memcpy(uart_response, bytes, size); uart_response[size] = '\0'; return HAL_OK;
}
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
    if (out != NULL) *out = strcmp(name, "ControlRobot") == 0 ? CONTROL_TASK_ID : (void *)1;
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
    if (action == 7) TaskMotorControl_Stop();
    if (scheduler_depth && pending_fault == 2) pending_fault = 1;
    return pdTRUE;
}
QueueHandle_t xQueueCreate(UBaseType_t count, UBaseType_t size) {
    CHECK(critical_depth == 0);
    if (count == 1U) { CHECK(size == sizeof(TaskMotorControlTarget_t)); return (void *)2; }
    CHECK(count == 8U && size == sizeof(TaskControlRobotEventMessage_t)); return (void *)1;
}
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait) {
    CHECK(critical_depth == 0 && wait == 0U && queue == (void *)1);
    if (((const TaskControlRobotEventMessage_t *)item)->event == ROBOT_EVENT_START_AUTO &&
        event_send_action != 0) {
        int action = event_send_action; event_send_action = 0;
        if (action == 1) CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
        if (action == 2) CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET));
        if (action == 3) TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    }
    if (event_count == 8U) return pdFALSE;
    events[event_count++] = *(const TaskControlRobotEventMessage_t *)item; return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait) {
    CHECK(critical_depth == 0 && scheduler_depth == 0 && wait == 0U);
    if (queue == (void *)2) {
        clock_tick += advance_tick_on_motor_receive;
        advance_tick_on_motor_receive = 0U;
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
    *(TaskControlRobotEventMessage_t *)out = events[0];
    --event_count; memmove(events, events + 1, event_count * sizeof(*events));
    return pdTRUE;
}
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item) {
    CHECK(critical_depth == 0 && queue == (void *)2);
    CHECK(scheduler_depth == 0);
    int action = overwrite_action; overwrite_action = 0;
    if (action == 1) {
        TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
        TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
    }
    if (action == 2) TaskMotorControl_SetTargetSpeeds(0, 0);
    if (action == 3) CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
    if (action == 4) TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    if (action == 6) run_motor(1); /* Higher-priority motor task before overwrite. */
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
static uint32_t test_get_autoreload(TIM_HandleTypeDef *timer) {
    CHECK(timer == &pwm);
    ++autoreload_reads;
    return mock_pwm_arr;
}
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channels) {
    (void)timer; CHECK(channels == TIM_CHANNEL_ALL); return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *timer, uint32_t channels) {
    (void)timer; CHECK(channels == TIM_CHANNEL_ALL); return HAL_OK;
}
int32_t Encoder_ReadDelta(EncoderHandle_t *encoder) {
    CHECK(critical_depth == 0);
    if (encoder == &left_encoder) { ++left_encoder_reads; return left_encoder_delta; }
    CHECK(encoder == &right_encoder);
    ++right_encoder_reads; return right_encoder_delta;
}

static void setup(void) {
    imu_snapshot_available = false;
    imu_cal_accepted=false;imu_cal_number=0U;
    CHECK(critical_depth == 0 && scheduler_depth == 0);
    clock_tick = 1000U;
    advance_tick_on_motor_receive = 0U;
    advance_tick_before_navigation_emit = 0U;
    input = (SensoresDatos_t){0};
    input.timestamp = clock_tick;
    input.distancia_m = input.distancia_trasera_m = 0.30f;
    input.ultrasonido_valido = input.ultrasonido_trasero_valido = 1U;
    input.timestamp_ultrasonido_frontal = clock_tick - 20U;
    input.timestamp_ultrasonido_trasero = clock_tick - 90U;
    queue_available = 1; refresh_snapshot = 0;
    cycles = delay_calls = peek_calls = suspended_peeks = suspend_calls = resume_calls = 0;
    peek_action = suspend_action = overwrite_action = receive_action = 0;
    event_send_action = 0;
    fault_at_inner_exit = fault_at_next_outer_exit = pending_fault = mode_at_second_delay = 0;
    unsafe_pwm_writes = transaction_writes = max_transaction_writes = event_count = 0U;
    target_pending = 0;
    left_encoder_delta = right_encoder_delta = 0;
    left_encoder_reads = right_encoder_reads = pi_calls = 0U;
    mock_pwm_arr = 8399U; autoreload_reads = 0U;
    safety_stop_active = false; safety_faults = SAFETY_FAULT_NONE;
    safety_fault_revision = 0U; safety_fault_tick = 0U; safety_has_fault = false;
    safety_task_handle = NULL; robot_state = ROBOT_STATE_MANUAL; event_queue = (void *)1;
    command_generation = 0U;
    has_stop = false; last_stop_tick = 0U;
    stop_pending = false; mode_entry_tick = clock_tick - 1U;
    auto_request_epoch = 0U; auto_barrier_tick = clock_tick - 1U;
    current_task = (TaskHandle_t)12; control_task_handle = CONTROL_TASK_ID;
    navigation.active = false; navigation.state = NAV_DETENIDO;
    navigation.emission_ready = false; navigation.session = 0U;
#if NAV_EXPERIMENTAL_TURN_ENABLED
    navigation.reverse_applied = false; navigation.turn_used = false;
    navigation.turn_started_tick = 0U;
#endif
    uart_response[0] = '\0'; uart_rearms = 0U;
    memset(&task_context, 0, sizeof(task_context));
    memset(&pwm, 0, sizeof(pwm)); memset(&pins, 0, sizeof(pins));
    left_motor = (MotorHandle_t){ .htim = &pwm, .channel = 0U,
        .port_a = &pins, .pin_a = 1U, .port_b = &pins, .pin_b = 2U };
    right_motor = (MotorHandle_t){ .htim = &pwm, .channel = 1U,
        /* main.c: port_a=BIN1/PD3, port_b=BIN2/PD2. */
        .port_a = &pins, .pin_a = 8U, .port_b = &pins, .pin_b = 4U };
    TaskMotorControl_Create(&left_encoder, &right_encoder, &left_motor, &right_motor);
}
static void run_safety(int count) {
    cycles = count; delay_calls = 0;
    if (setjmp(finish) == 0) TaskSafety_Run(NULL);
}
static void run_motor(int count) {
    TaskHandle_t previous = current_task; current_task = MOTOR_TASK_ID;
    cycles = count; delay_calls = 0;
    if (setjmp(finish) == 0) TaskMotorControl_Run(&task_context);
    current_task = previous;
}
static void run_fsm(int count) {
    TaskHandle_t previous = current_task; current_task = CONTROL_TASK_ID;
    cycles = count; delay_calls = 0;
    if (setjmp(finish) == 0) TaskControlRobot_Run(NULL);
    current_task = previous;
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

static void test_motor_encoder_regressions(void) {
    setup();
    CHECK(MOTOR_PWM_MAX_PERCENT == 40U);
    CHECK(MOTOR_PI_KP == 2.20f && MOTOR_PI_KI == 0.10f && MOTOR_PI_DT_S == 0.010f);
    PASS("motor checkpoint uses approved 40 percent ceiling and PI gains without changing control period");

    setup(); CHECK(Motor_GetPwmLimit(&left_motor) == TEST_PWM_LIMIT && Motor_GetPwmLimit(&right_motor) == TEST_PWM_LIMIT);
    Motor_SetSpeed(&left_motor, INT16_MAX); Motor_SetSpeed(&right_motor, INT16_MIN);
    CHECK(left_motor.current_speed == TEST_PWM_LIMIT && right_motor.current_speed == -TEST_PWM_LIMIT);
    CHECK(pwm.compare[0] == TEST_PWM_LIMIT && pwm.compare[1] == TEST_PWM_LIMIT);
    CHECK((pins.pins & 3U) == 1U && (pins.pins & 12U) == 4U);
    PASS("permanent driver cap is symmetric and preserves electrical sign");

    setup(); mock_pwm_arr = 1999U; Motor_SetSpeed(&left_motor, INT16_MAX); Motor_SetSpeed(&right_motor, INT16_MIN);
    CHECK(pwm.compare[0] == TEST_SHORT_PERIOD_LIMIT && pwm.compare[1] == TEST_SHORT_PERIOD_LIMIT);
    PASS("permanent driver limit follows actual ARR for both motors");

    setup(); mock_pwm_arr = 0U; Motor_SetSpeed(&left_motor, 800); Motor_SetSpeed(&right_motor, -800);
    CHECK(pwm.compare[0] == 0U && pwm.compare[1] == 0U);
    PASS("zero effective range disables actuation without affecting zero path");

    setup(); mock_pwm_arr = UINT32_MAX;
    CHECK(Motor_GetPwmLimit(&left_motor) == MOTOR_MAX_SPEED);
    Motor_SetSpeed(&left_motor, INT16_MAX); CHECK(pwm.compare[0] == MOTOR_MAX_SPEED);
    PASS("period arithmetic cannot overflow or exceed driver representation");

    setup(); Motor_SetSpeed(&left_motor, 0); Motor_Stop(&right_motor);
    CHECK(autoreload_reads == 0U); expect_zero();
    PASS("zero and Stop bypass PWM limit computation");

    setup(); safety_stop_active = true; Motor_SetSpeed(&left_motor, INT16_MAX);
    CHECK(autoreload_reads == 0U); expect_zero();
    PASS("Safety block reaches immediate zero before any limit calculation");

    setup(); TaskMotorControl_SetTargetSpeeds(100000, -100000); run_motor(10);
    CHECK(pwm.compare[0] == TEST_PWM_LIMIT && pwm.compare[1] == TEST_PWM_LIMIT);
    CHECK(task_context.left_integral == 0.0f && task_context.right_integral == 0.0f);
    PASS("real PI and driver share saturation without accumulating integral");

    setup(); mock_pwm_arr = 1999U; TaskMotorControl_SetTargetSpeeds(1700, -1700); run_motor(2);
    CHECK(pwm.compare[0] == TEST_SHORT_PERIOD_LIMIT && pwm.compare[1] == TEST_SHORT_PERIOD_LIMIT);
    CHECK(task_context.left_integral == 0.0f && task_context.right_integral == 0.0f);
    PASS("antiwindup follows effective driver ceiling for a smaller ARR");

    setup(); float integral = 500.0f;
    CHECK(MotorPI_Calculate(100, 100, &integral, 200U) == 200 && integral == 200.0f);
    integral = -500.0f;
    CHECK(MotorPI_Calculate(-100, -100, &integral, 200U) == -200 && integral == -200.0f);
    PASS("shrinking PWM limit clamps a previously accumulated integral");

    setup(); integral = 840.0f;
    CHECK(MotorPI_Calculate(100, 200, &integral, 840U) < 840 && integral < 840.0f);
    integral = -840.0f;
    CHECK(MotorPI_Calculate(-100, -200, &integral, 840U) > -840 && integral > -840.0f);
    PASS("integral can recover from saturation with opposing error");

    setup(); integral = 500.0f;
    CHECK(MotorPI_Calculate(100, 0, &integral, 0U) == 0 && integral == 0.0f);
    integral = 500.0f;
    CHECK(MotorPI_Calculate(0, 300, &integral, 840U) == 0 && integral == 0.0f);
    PASS("zero target or zero ceiling never integrates residual movement");

    setup(); integral = 0.0f;
    int16_t correct = MotorPI_Calculate(1700, 500, &integral, TEST_PWM_LIMIT);
    integral = 0.0f;
    int16_t reversed = MotorPI_Calculate(1700, -500, &integral, TEST_PWM_LIMIT);
    CHECK(correct > 0 && correct < reversed && reversed == TEST_PWM_LIMIT);
    PASS("demonstrates negative measured sign increases positive target error");

    setup(); CHECK(TaskMotorControl_DeltaToCps(1) == 100 && TaskMotorControl_DeltaToCps(-1) == -100);
    CHECK(TaskMotorControl_DeltaToCps(INT32_MAX) == INT32_MAX);
    CHECK(TaskMotorControl_DeltaToCps(INT32_MIN) == INT32_MIN);
    left_encoder_delta = INT32_MAX; right_encoder_delta = INT32_MIN; run_motor(1);
    CHECK(task_context.left_speed_cps == INT32_MAX && task_context.right_speed_cps == INT32_MIN);
    expect_zero(); PASS("cps conversion preserves sign without signed multiplication overflow");

    TIM_HandleTypeDef timer = {0}; EncoderHandle_t encoder;
    setup(); timer.Init.Period = UINT16_MAX; timer.counter = 65530U;
    ActualEncoder_Init(&encoder, &timer); timer.counter = 4U;
    CHECK(ActualEncoder_ReadDelta(&encoder) == 10);
    CHECK(timer.counter == 4U && encoder.last_count == 4U);
    CHECK(ActualEncoder_ReadDelta(&encoder) == 0);
    PASS("real TIM1 encoder delta wraps forward and never resets CNT");

    timer.counter = 65530U;
    CHECK(ActualEncoder_ReadDelta(&encoder) == -10 && timer.counter == 65530U);
    PASS("real TIM1 encoder delta wraps backward with signed result");

    setup(); timer.Init.Period = UINT32_MAX; timer.counter = UINT32_MAX - 5U;
    ActualEncoder_Init(&encoder, &timer); timer.counter = 4U;
    CHECK(ActualEncoder_ReadDelta(&encoder) == 10 && timer.counter == 4U);
    timer.counter = UINT32_MAX - 5U;
    CHECK(ActualEncoder_ReadDelta(&encoder) == -10);
    PASS("real TIM2 driver handles 32 bit wrap in both directions");

    setup(); timer.counter = 123U; ActualEncoder_Init(&encoder, &timer);
    timer.counter = 456U; CHECK(ActualEncoder_Start(&encoder) == HAL_OK);
    CHECK(timer.counter == 456U && encoder.last_count == 456U);
    CHECK(ActualEncoder_ReadDelta(&encoder) == 0 && ActualEncoder_Stop(&encoder) == HAL_OK);
    PASS("encoder start updates reference without zeroing hardware counter");
}

/* Coverage retained from the removed fixtures, through the public API only. */
static void test_public_velocity_control(void) {
    for (unsigned selected = 0U; selected < 4U; ++selected) {
        setup();
        bool left = selected < 2U;
        int32_t target = selected % 2U == 0U ? 1700 : -1700;
        TaskMotorControl_SetTargetSpeeds(left ? target : 0, left ? 0 : target);
        run_motor(1);
        CHECK(pi_calls == 1U);
        CHECK(pwm.compare[left ? 0 : 1] == TEST_PWM_LIMIT && pwm.compare[left ? 1 : 0] == 0U);
        CHECK((left ? left_motor.current_speed : right_motor.current_speed) == (target > 0 ? TEST_PWM_LIMIT : -TEST_PWM_LIMIT));
        if (left) left_encoder_delta = target > 0 ? 5 : -5;
        else right_encoder_delta = target > 0 ? 5 : -5;
        run_motor(1);
        int32_t measured = left ? task_context.left_speed_cps : task_context.right_speed_cps;
        int16_t applied = left ? left_motor.current_speed : right_motor.current_speed;
        CHECK(measured == (target > 0 ? 500 : -500));
        CHECK((target > 0 && applied > 0 && applied < TEST_PWM_LIMIT) ||
              (target < 0 && applied < 0 && applied > -TEST_PWM_LIMIT));
        CHECK(pwm.compare[left ? 1 : 0] == 0U && pi_calls == 2U);
        PASS("public PI individually controls either motor in either direction with signed feedback");
    }

    setup(); clock_tick = 0U; TaskMotorControl_SetTargetSpeeds(100, 0); run_motor(1);
    CHECK(task_context.left_target_cps == 100 && pwm.compare[0] > 0U && pwm.compare[1] == 0U);
    CHECK(task_context.command_tick == 0U && pi_calls == 1U);
    PASS("public velocity command emitted at legitimate tick zero is accepted");

    setup(); clock_tick = 0U; TaskMotorControl_SetTargetSpeeds(100, 0);
    target_pending = 0; run_motor(1); expect_zero(); CHECK(pi_calls == 0U);
    PASS("unconsumed tick zero metadata cannot generate motor output");

    setup(); TaskMotorControl_SetTargetSpeeds(100, 100); clock_tick += 489U;
    advance_tick_on_motor_receive = 1U; run_motor(1);
    CHECK((TickType_t)(clock_tick - queued_target.issued_tick) == 500U);
    expect_zero(); CHECK(pi_calls == 0U);
    PASS("deadline crossed while consuming a public command prevents stale PI actuation");
}

static void uart_feed(DrvUart_t *uart, const char *frame) {
    while (*frame) {
        uart->rx_byte = (uint8_t)*frame++;
        DrvUart_RxCallback(uart);
    }
}
static bool uart_consume(DrvUart_t *uart) {
    char frame[UART_RX_BUFFER_SIZE];
    TickType_t tick;
    if (!DrvUart_GetMessageTimed(uart, frame, sizeof(frame), &tick)) return false;
    TaskComms_ProcessMessage(uart, frame, tick);
    return true;
}

static void test_imu_protocol(void) {
    setup(); DrvUart_t uart; UART_HandleTypeDef peripheral = {0}; DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "IMU Q=1\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response, "IMU,1,1,0,0,0,-,-,-,-,-,-,-\n") == 0);
    PASS("IMU read-only query exposes initialization without invented values");
    imu_snapshot_available = true;
    imu_snapshot = (TaskIMUSnapshot_t){ .state=IMU_VALID, .has_sample=true, .sample_tick=0U,
        .data={.Accel_X_RAW=-32768,.Accel_Y_RAW=0,.Accel_Z_RAW=16384,.Gyro_X_RAW=131,.Gyro_Y_RAW=-131,.Gyro_Z_RAW=32767} };
    clock_tick=0U; uart_feed(&uart,"IMU Q=4294967295\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response,"IMU,1,4294967295,2,0,0,0,-32768,0,16384,131,-131,32767\n")==0);
    CHECK(!target_pending); expect_zero();
    PASS("IMU raw signed axes query correlation and legitimate zero tick do not emit motion");
    imu_snapshot.sample_tick=UINT32_MAX-9U; clock_tick=10U;
    uart_feed(&uart,"IMU\n"); CHECK(uart_consume(&uart)); CHECK(strstr(uart_response,",2,0,0,20,"));
    PASS("IMU sample age uses unsigned FreeRTOS tick wrap");
    imu_snapshot.state=IMU_READ_ERROR; imu_snapshot.hal_status=HAL_TIMEOUT; imu_snapshot.i2c_error=32;
    uart_feed(&uart,"IMU Q=2\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response,"IMU,1,2,4,3,32,20,-,-,-,-,-,-\n")==0);
    PASS("IMU failed read withdraws axes while preserving last-success age and HAL error");
    const char *bad[]={"IMU Q=0\n","IMU Q=-1\n","IMU Q=4294967296\n","IMU Q=2extra\n"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) {
        uart_feed(&uart,bad[i]); CHECK(uart_consume(&uart)); CHECK(strcmp(uart_response,"ERR_UNKNOWN\n")==0);
    }
    PASS("IMU malformed query numbers cannot reach acquisition or motion");
    uart_feed(&uart,"STOP\nIMU Q=4\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response,"STOP_QUEUED\n")==0); expect_zero();
    setup(); DrvUart_Init(&uart,&peripheral); uart_feed(&uart,"IMU Q=4\nDETENER\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response,"MOVE_ACCEPTED\n")==0); expect_zero();
    PASS("STOP and DETENER retain mailbox priority over read-only IMU telemetry");
    uart_feed(&uart,"IMU Q=4\nSENSORS Q=5\n"); CHECK(uart_consume(&uart));
    CHECK(strncmp(uart_response,"SENSORS,1,5,",12U)==0);
    uart_feed(&uart,"IMU Q=4\nMAPPING Q=6 S=0\n"); CHECK(uart_consume(&uart));
    CHECK(strncmp(uart_response,"POSE,2,6,",9U)==0);
    PASS("existing SENSORS and MAPPING outrank IMU without changing mailbox priority classes");
    setup();DrvUart_Init(&uart,&peripheral);imu_snapshot_available=true;
    imu_snapshot=(TaskIMUSnapshot_t){.state=IMU_VALID,.has_sample=true,.sample_tick=clock_tick,
        .data={.Accel_Z_RAW=16384,.Gyro_X_RAW=131,.Gyro_Y_RAW=-262,.Gyro_Z_RAW=66},
        .calibration={.state=IMU_CAL_VALID,.samples=250,.request=7,.completed_tick=clock_tick,.sum={32750,-65500,16500}}};
    uart_feed(&uart,"IMU2 Q=8\n");CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response,"IMU,2,8,2,0,0,0,0,0,16384,131,-262,66,2,0,250,7,0,32750,-65500,16500\n")==0);
    uart_feed(&uart,"IMU Q=9\n");CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response,"IMU,1,9,2,0,0,0,0,0,16384,131,-262,66\n")==0);
    PASS("IMU-2 v2 adds measured calibration sums and age while legacy IMU response remains byte-compatible");
    imu_cal_accepted=true;uart_feed(&uart,"IMUCAL N=17\n");CHECK(uart_consume(&uart));
    CHECK(imu_cal_number==17U && imu_cal_issued==clock_tick && strcmp(uart_response,"IMUCAL,1,17,1\n")==0);expect_zero();
    imu_cal_accepted=false;uart_feed(&uart,"IMUCAL N=18\n");CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response,"IMUCAL,1,18,0\n")==0);expect_zero();
    uart_feed(&uart,"IMUCAL N=0\n");CHECK(uart_consume(&uart));CHECK(strcmp(uart_response,"ERR_UNKNOWN\n")==0);
    uart_feed(&uart,"IMUCAL N=4294967296\n");CHECK(uart_consume(&uart));CHECK(strcmp(uart_response,"ERR_UNKNOWN\n")==0);
    PASS("IMU-2 explicit bounded calibration command reports admission, rejects invalid IDs and never emits motor targets");
    clock_tick=UINT32_MAX;imu_snapshot.sample_tick=imu_snapshot.calibration.completed_tick=0U;
    imu_snapshot.data.Accel_X_RAW=imu_snapshot.data.Accel_Y_RAW=imu_snapshot.data.Accel_Z_RAW=INT16_MIN;
    imu_snapshot.data.Gyro_X_RAW=imu_snapshot.data.Gyro_Y_RAW=imu_snapshot.data.Gyro_Z_RAW=INT16_MIN;
    uart_feed(&uart,"IMU2 Q=4294967295\n");CHECK(uart_consume(&uart));
    CHECK(strlen(uart_response)<192U && strstr(uart_response,",4294967295,-32768,-32768,-32768,") && uart_response[strlen(uart_response)-1U]=='\n');
    PASS("IMU-2 worst signed axes and full 32-bit ages fit the bounded UART response without truncation");
}
static void test_motion_commands(void) {
    static const char *frames[] = {
        "AVANZAR\n", "RETROCEDER\n", "GIRAR_IZQUIERDA\n", "GIRAR_DERECHA\n"
    };
    static const int32_t expected[][2] = {
        {1600, 1600}, {-1600, -1600}, {-1200, 1200}, {1200, -1200}
    };
    DrvUart_t uart;
    UART_HandleTypeDef peripheral = {0};
    for (unsigned i = 0; i < 4U; ++i) {
        setup(); DrvUart_Init(&uart, &peripheral); DrvUart_StartReceive(&uart);
        uart_feed(&uart, frames[i]); CHECK(uart_consume(&uart));
        CHECK(strcmp(uart_response, "MOVE_ACCEPTED\n") == 0);
        CHECK(queued_target.left_speed_cps == expected[i][0]);
        CHECK(queued_target.right_speed_cps == expected[i][1]);
        CHECK(queued_target.issued_tick == 1000U);
        run_motor(1);
        CHECK(task_context.left_target_cps == expected[i][0]);
        CHECK(task_context.right_target_cps == expected[i][1]);
        CHECK((left_motor.current_speed > 0) == (expected[i][0] > 0));
        CHECK((right_motor.current_speed > 0) == (expected[i][1] > 0));
        CHECK(pwm.compare[0] > 0 && pwm.compare[0] <= TEST_PWM_LIMIT);
        CHECK(pwm.compare[1] > 0 && pwm.compare[1] <= TEST_PWM_LIMIT);
        CHECK((pins.pins & 3U) == (expected[i][0] > 0 ? 1U : 2U));
        CHECK((pins.pins & 12U) == (expected[i][1] > 0 ? 8U : 4U));
        CHECK(pwm.compare[0] == (i < 2U ? TEST_PWM_LIMIT : 2641U));
        CHECK(pwm.compare[1] == (i < 2U ? TEST_PWM_LIMIT : 2641U));
        PASS("B6 UART mapping exercises real motion layer PI and limited driver");
    }
    setup(); DrvUart_Init(&uart, &peripheral);
    CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    run_motor(1); uart_feed(&uart, "DETENER\n"); CHECK(uart_consume(&uart));
    expect_zero(); CHECK(robot_state == ROBOT_STATE_MANUAL);
    run_motor(3); expect_zero();
    PASS("B6 DETENER immediately clears objectives integrals and PWM without leaving MANUAL");

    const RobotControlState_t rejected[] = { ROBOT_STATE_IDLE, ROBOT_STATE_SAFETY_STOP, ROBOT_STATE_AUTONOMOUS };
    for (unsigned i = 0U; i < 3U; ++i) {
        setup(); robot_state = rejected[i]; DrvUart_Init(&uart, &peripheral);
        uart_feed(&uart, "AVANZAR\n"); CHECK(uart_consume(&uart));
        CHECK(strcmp(uart_response, "MOVE_REJECTED\n") == 0 && !target_pending);
        run_motor(1); expect_zero();
        PASS("B6 manual UART rejects non-MANUAL state without emitting a target");
    }
    setup(); safety_stop_active = true;
    CHECK(!Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    CHECK(!Motion_Execute(MOTION_HALT, ROBOT_STATE_MANUAL, clock_tick));
    CHECK(!target_pending); run_motor(1); expect_zero();
    PASS("B6 Safety block rejects motion regardless of MANUAL state");
    setup(); CHECK(!Motion_Execute((MotionCommand_t)99, ROBOT_STATE_MANUAL, clock_tick));
    CHECK(!Motion_Execute(MOTION_FORWARD, ROBOT_STATE_IDLE, clock_tick));
    CHECK(!target_pending && command_generation == 0U);
    PASS("B6 invalid order or requested mode cannot publish a target");

    setup(); mode_entry_tick = clock_tick - 1000U;
    CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick - 499U));
    CHECK(queued_target.issued_tick == clock_tick - 499U);
    run_motor(1); expect_zero();
    PASS("B6 age 499 admitted but deadline before consumption prevents PWM");
    setup(); mode_entry_tick = 0U;
    CHECK(!Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick - 500U));
    CHECK(!target_pending); PASS("B6 age exactly 500 is rejected at publication");
    setup(); CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    uint32_t generation = command_generation;
    run_motor(49); CHECK(pwm.compare[0] > 0U);
    run_motor(1); expect_zero(); run_motor(10); expect_zero();
    CHECK(command_generation == generation && !target_pending);
    PASS("B6 single emission expires at 500 ms and never renews automatically");

    setup(); CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    run_motor(1); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); expect_zero();
    CHECK(TaskControlRobot_IsStopPending());
    CHECK(!Motion_Execute(MOTION_REVERSE, ROBOT_STATE_MANUAL, clock_tick));
    run_motor(1); expect_zero(); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE && !TaskControlRobot_IsStopPending());
    PASS("B6 accepted STOP zeroes PWM immediately and blocks publication before FSM drain");
    setup(); CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    run_motor(1);
    for (unsigned i = 0U; i < 8U; ++i) CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); expect_zero();
    CHECK(!Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    run_fsm(1); CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B6 full event queue cannot prevent STOP or its eventual IDLE transition");

    setup(); TickType_t old = clock_tick;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
    CHECK(!Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, old));
    CHECK(!target_pending); run_motor(1); expect_zero();
    CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    run_motor(1); CHECK(pwm.compare[0] > 0U);
    PASS("B6 same-tick STOP MANUAL rejects old or ambiguous order and admits later fresh order");
    setup(); mode_entry_tick = clock_tick;
    CHECK(!Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    ++clock_tick;
    CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    PASS("B6 mode entry and order in same tick conservatively require a fresh later tick");
    recovery(); TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_STOP);
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE);
    TickType_t before_manual = clock_tick;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
    CHECK(!Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, before_manual));
    run_motor(1); expect_zero();
    CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    run_motor(1); CHECK(pwm.compare[0] > 0U);
    PASS("B6 explicit recovery and MANUAL do not revive a pre-transition order");

    setup(); overwrite_action = 1;
    CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    CHECK(queued_target.generation != command_generation); run_motor(1); expect_zero();
    PASS("B6 STOP MANUAL between admission and enqueue cannot execute old generation");
    setup(); fault_at_next_outer_exit = 1;
    CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    CHECK(TaskSafety_IsStopActive()); run_motor(1); expect_zero();
    PASS("B6 concurrent Safety after admission invalidates late queued command");
    setup(); CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, clock_tick));
    ++clock_tick; CHECK(Motion_Execute(MOTION_HALT, ROBOT_STATE_MANUAL, clock_tick));
    run_motor(1); expect_zero();
    PASS("B6 DETENER invalidates an unconsumed movement generation");
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVAN"); ++clock_tick;
    CHECK(Motion_Execute(MOTION_HALT, ROBOT_STATE_MANUAL, clock_tick));
    ++clock_tick; uart_feed(&uart, "ZAR\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response, "MOVE_REJECTED\n") == 0 && !target_pending);
    run_motor(1); expect_zero();
    PASS("B6 partial frame older than DETENER cannot restart motion in unchanged MANUAL");
    setup(); clock_tick = 0U; mode_entry_tick = UINT32_MAX - 2U;
    has_stop = true; last_stop_tick = UINT32_MAX - 3U;
    CHECK(Motion_Execute(MOTION_FORWARD, ROBOT_STATE_MANUAL, 0U));
    run_motor(1); CHECK(pwm.compare[0] > 0U);
    clock_tick = 500U; run_motor(1); expect_zero();
    PASS("B6 modular ordering and expiry accept legitimate tick zero across wrap");
    setup(); robot_state = ROBOT_STATE_AUTONOMOUS;
    CHECK(!Motion_Execute(MOTION_TURN_LEFT, ROBOT_STATE_AUTONOMOUS, clock_tick));
    CHECK(!target_pending); run_motor(1); expect_zero();
    PASS("B7.2 reusable API remains available but AUTO cannot emit movement yet");

    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVANZAR\nSTOP\nRETROCEDER\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response, "STOP_QUEUED\n") == 0 && !target_pending);
    CHECK(!uart_consume(&uart)); run_fsm(1); expect_zero();
    PASS("B6 STOP replaces pending motion and cannot be replaced by subsequent motion");
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVANZAR\nDETENER\nRETROCEDER\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response, "MOVE_ACCEPTED\n") == 0 && has_stop && !target_pending);
    PASS("B6 DETENER replaces pending ordinary command and discards following motion");
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "DETENER\nSTOP\n"); CHECK(uart_consume(&uart));
    CHECK(TaskControlRobot_IsStopPending()); PASS("B6 STOP takes precedence over DETENER");
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVANZAR\nRETROCEDER\n"); CHECK(uart_consume(&uart));
    CHECK(queued_target.left_speed_cps == 1600 && !uart_consume(&uart));
    PASS("B6 bounded mailbox does not accumulate or renew a burst of movements");
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVANZAR\r\n"); CHECK(uart_consume(&uart));
    CHECK(queued_target.left_speed_cps == 1600);
    PASS("B6 CRLF produces a complete recognized command");
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVAN"); clock_tick += 501U; uart_feed(&uart, "ZAR\n");
    CHECK(uart_consume(&uart) && strcmp(uart_response, "MOVE_REJECTED\n") == 0);
    CHECK(!target_pending); PASS("B6 partial frame is dated at first byte and cannot gain freshness at LF");
    setup(); robot_state = ROBOT_STATE_IDLE; DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVAN"); ++clock_tick;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL); ++clock_tick;
    uart_feed(&uart, "ZAR\n"); CHECK(uart_consume(&uart)); CHECK(!target_pending);
    PASS("B6 frame started before MANUAL cannot be admitted after mode change");
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVANZAR\n"); clock_tick += 500U;
    CHECK(uart_consume(&uart) && !target_pending);
    PASS("B6 complete frame held in mailbox for 500 ms is not rejuvenated on consumption");
    setup(); DrvUart_Init(&uart, &peripheral);
    for (unsigned i = 0; i < 70U; ++i) uart_feed(&uart, "X");
    uart_feed(&uart, "AVANZAR\n"); CHECK(!uart_consume(&uart)); CHECK(!target_pending);
    uart_feed(&uart, "AVANZAR\n"); CHECK(uart_consume(&uart) && target_pending);
    PASS("B6 oversized frame discarded through LF without executing a valid suffix");
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVANZAR"); uart.rx_byte = 0U; DrvUart_RxCallback(&uart);
    uart_feed(&uart, "\n"); CHECK(!uart_consume(&uart));
    uart.rx_byte = 0U; DrvUart_RxCallback(&uart);
    uart_feed(&uart, "AVANZAR\n"); CHECK(uart_consume(&uart) && target_pending);
    PASS("B6 embedded NUL invalidates frame while leading boot NUL remains tolerated");
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVANZAR\rX\n"); CHECK(!uart_consume(&uart));
    uart_feed(&uart, "AVANZAR\t\n"); CHECK(!uart_consume(&uart));
    CHECK(!target_pending); PASS("B6 malformed CR and embedded control bytes cannot actuate");
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "AVANZAR\n"); char small[4]; TickType_t tick = 0U;
    CHECK(!DrvUart_GetMessageTimed(&uart, small, sizeof(small), &tick));
    CHECK(small[0] == '\0' && !DrvUart_IsMessageReady(&uart));
    CHECK(!DrvUart_GetMessageTimed(NULL, small, sizeof(small), &tick));
    PASS("B6 bounded copy cannot truncate received text into another command");
    static const char *invalid[] = {"avanzar\n", "AVANZAR 1700\n", "GIRO\n", " AVANZAR\n", "\n"};
    for (unsigned i = 0; i < 5U; ++i) {
        setup(); DrvUart_Init(&uart, &peripheral); uart_feed(&uart, invalid[i]);
        bool got = uart_consume(&uart);
        CHECK(i == 4U ? !got : got && strcmp(uart_response, "ERR_UNKNOWN\n") == 0);
        CHECK(!target_pending && command_generation == 0U);
        PASS("B6 malformed or unknown command emits no target");
    }
    setup(); DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "PING\n"); CHECK(uart_consume(&uart)); CHECK(strcmp(uart_response, "PONG\n") == 0);
    uart_feed(&uart, "STATUS\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response, "STATE=2 FAULTS=0 STOP=0\n") == 0);
    uart_feed(&uart, "RESET\n"); CHECK(uart_consume(&uart)); CHECK(strcmp(uart_response, "RESET_QUEUED\n") == 0);
    uart_feed(&uart, "MANUAL\n"); CHECK(uart_consume(&uart)); CHECK(strcmp(uart_response, "MANUAL_QUEUED\n") == 0);
    PASS("B6 legacy PING STATUS RESET MANUAL responses remain compatible");
}

/* B6.5: real UART/FSM/Safety/motor paths. Only the sensor publication is
 * substituted: a transient cliff must remain latched independently of STOP.
 */
static void b65_sensor_snapshot(uint8_t cliff) {
    input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = clock_tick - 1U;
    input.timestamp_ultrasonido_trasero = clock_tick - 1U;
    input.hay_caida = cliff;
}
static void b65_uart_command(DrvUart_t *uart, const char *frame, const char *response) {
    uart_feed(uart, frame);
    CHECK(uart_consume(uart));
    CHECK(strcmp(uart_response, response) == 0);
}
static void b65_start_manual(DrvUart_t *uart, UART_HandleTypeDef *peripheral) {
    setup(); safety_stop_active = true; robot_state = ROBOT_STATE_SAFETY_STOP;
    DrvUart_Init(uart, peripheral);
    b65_uart_command(uart, "RESET\n", "RESET_QUEUED\n"); run_fsm(1);
    b65_uart_command(uart, "STATUS\n", "STATE=0 FAULTS=0 STOP=0\n");
    b65_uart_command(uart, "MANUAL\n", "MANUAL_QUEUED\n"); run_fsm(1);
    b65_uart_command(uart, "STATUS\n", "STATE=2 FAULTS=0 STOP=0\n");
    ++clock_tick; /* A fresh command strictly after the mode transition. */
}
static void test_stop_b65(void) {
    DrvUart_t uart;
    UART_HandleTypeDef peripheral = {0};
    b65_start_manual(&uart, &peripheral);
    const char *frames[] = {
        "AVANZAR\n", "RETROCEDER\n", "GIRAR_IZQUIERDA\n", "GIRAR_DERECHA\n", "DETENER\n"
    };
    for (unsigned i = 0U; i < 5U; ++i) {
        ++clock_tick;
        b65_uart_command(&uart, frames[i], "MOVE_ACCEPTED\n"); run_motor(1);
    }
    b65_uart_command(&uart, "STOP\n", "STOP_QUEUED\n"); expect_zero();
    CHECK(TaskSafety_GetFaults() == SAFETY_FAULT_NONE && !TaskSafety_IsStopActive());
    run_fsm(1); b65_sensor_snapshot(0U); run_safety(1); run_fsm(1);
    b65_uart_command(&uart, "STATUS\n", "STATE=0 FAULTS=0 STOP=0\n"); expect_zero();
    PASS("B6.5 RESET MANUAL five accepted orders STOP ends IDLE without creating faults");

    for (unsigned safety_first = 0U; safety_first < 2U; ++safety_first) {
        b65_start_manual(&uart, &peripheral);
        b65_uart_command(&uart, "AVANZAR\n", "MOVE_ACCEPTED\n"); run_motor(1);
        CHECK(pwm.compare[0] > 0U && pwm.compare[1] > 0U);
        TickType_t old_origin = clock_tick;
        /* Leave an old motion in the actual motor queue across STOP/reset. */
        b65_uart_command(&uart, "RETROCEDER\n", "MOVE_ACCEPTED\n");
        CHECK(target_pending);
        if (safety_first) { b65_sensor_snapshot(1U); run_safety(1); }
        b65_uart_command(&uart, "STOP\n", "STOP_QUEUED\n"); expect_zero();
        CHECK(TaskSafety_GetFaults() == (safety_first ? SAFETY_FAULT_CLIFF : SAFETY_FAULT_NONE));
        if (!safety_first) { b65_sensor_snapshot(1U); run_safety(1); }
        CHECK(TaskSafety_GetFaults() == 1U);
        run_fsm(1);
        b65_uart_command(&uart, "STATUS\n", "STATE=3 FAULTS=1 STOP=1\n"); expect_zero();
        PASS("B6.5 cliff before or after STOP produces exact observed latched status");

        uint32_t revision = safety_fault_revision;
        clock_tick += 81000U;
        b65_sensor_snapshot(0U); run_safety(1); run_fsm(1); run_motor(1);
        CHECK(safety_fault_revision == revision); /* No new fault is needed. */
        b65_uart_command(&uart, "STATUS\n", "STATE=3 FAULTS=1 STOP=1\n"); expect_zero();
        PASS("B6.5 transient cliff remains latched beyond 80 seconds with fresh safe sensors");

        b65_uart_command(&uart, "MANUAL\n", "MANUAL_QUEUED\n"); run_fsm(1);
        CHECK(robot_state == ROBOT_STATE_SAFETY_STOP); expect_zero();
        b65_sensor_snapshot(0U);
        b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); run_fsm(1); run_motor(1);
        b65_uart_command(&uart, "STATUS\n", "STATE=0 FAULTS=0 STOP=0\n"); expect_zero();
        b65_uart_command(&uart, "MANUAL\n", "MANUAL_QUEUED\n"); run_fsm(1); run_motor(1);
        CHECK(robot_state == ROBOT_STATE_MANUAL); expect_zero();
        CHECK(!Motion_Execute(MOTION_REVERSE, ROBOT_STATE_MANUAL, old_origin));
        expect_zero();
        b65_uart_command(&uart, "AVANZAR\n", "MOVE_ACCEPTED\n"); run_motor(1);
        CHECK(pwm.compare[0] > 0U && pwm.compare[1] > 0U);
        b65_uart_command(&uart, "STOP\n", "STOP_QUEUED\n"); run_fsm(1); expect_zero();
        PASS("B6.5 reset needs new safe captures and MANUAL cannot restore pre-STOP motion");
    }
}

#include "test_log_producers.c"
#include "test_sensor_telemetry.c"

static void b72_idle(DrvUart_t *uart, UART_HandleTypeDef *peripheral) {
    setup(); robot_state = ROBOT_STATE_IDLE; DrvUart_Init(uart, peripheral);
}
static void test_auto_b72(void) {
    DrvUart_t uart;
    UART_HandleTypeDef peripheral = {0};
    b72_idle(&uart, &peripheral);
    b65_uart_command(&uart, "AUTO\n", "AUTO_QUEUED\n");
    CHECK(robot_state == ROBOT_STATE_IDLE); run_fsm(1);
    b65_uart_command(&uart, "STATUS\n", "STATE=1 FAULTS=0 STOP=0\n"); expect_zero();
    PASS("B7.2 AUTO UART enters AUTONOMOUS only when its event is consumed");

    const char *manual[] = {"AVANZAR\n", "RETROCEDER\n", "GIRAR_IZQUIERDA\n", "GIRAR_DERECHA\n", "DETENER\n"};
    for (unsigned i = 0U; i < 5U; ++i) {
        b65_uart_command(&uart, manual[i], "MOVE_REJECTED\n"); run_motor(1); expect_zero();
        CHECK(robot_state == ROBOT_STATE_AUTONOMOUS && !target_pending);
        PASS("B7.2 manual UART command is rejected in AUTO without a target");
    }
    uint32_t generation = command_generation;
    TaskMotorControl_SetTargetSpeeds(1600, -1600);
    CHECK(!Motion_Execute(MOTION_FORWARD, ROBOT_STATE_AUTONOMOUS, clock_tick));
    CHECK(!target_pending); residual_motion(); run_motor(100); run_fsm(50); expect_zero();
    CHECK(command_generation == generation && pi_calls == 0U);
    expect_residual_speed();
    PASS("B7.2 AUTO blocks both motor APIs and never renews or calculates nonzero actuation");
    b65_uart_command(&uart, "MANUAL\n", "MANUAL_QUEUED\n"); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); expect_zero();
    b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); expect_zero();
    PASS("B7.2 MANUAL and RESET preserve their existing no-op semantics in healthy AUTO");
    b65_uart_command(&uart, "STOP\n", "STOP_QUEUED\n"); expect_zero(); run_fsm(1);
    b65_uart_command(&uart, "STATUS\n", "STATE=0 FAULTS=0 STOP=0\n");
    PASS("B7.2 AUTO STOP returns to IDLE without inventing a Safety fault");

    setup(); DrvUart_Init(&uart, &peripheral);
    b65_uart_command(&uart, "AUTO\n", "AUTO_REJECTED\n"); CHECK(event_count == 0U);
    CHECK(robot_state == ROBOT_STATE_MANUAL); expect_zero();
    PASS("B7.2 AUTO rejected from MANUAL before enqueuing");
    TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1);
    generation = command_generation;
    b65_uart_command(&uart, "AUTO\n", "AUTO_REJECTED\n");
    CHECK(command_generation == generation && task_context.left_target_cps == 100);
    CHECK(pwm.compare[0] > 0U); TaskMotorControl_Stop();
    PASS("B7.2 rejected AUTO leaves an existing MANUAL command semantics intact");
    b72_idle(&uart, &peripheral); safety_stop_active = true; robot_state = ROBOT_STATE_SAFETY_STOP;
    b65_uart_command(&uart, "AUTO\n", "AUTO_REJECTED\n");
    CHECK(event_count == 0U && robot_state == ROBOT_STATE_SAFETY_STOP); expect_zero();
    PASS("B7.2 AUTO rejected from SAFETY_STOP");
    b72_idle(&uart, &peripheral); safety_stop_active = true;
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick)); CHECK(event_count == 0U);
    PASS("B7.2 blocked Safety prevents AUTO even if the FSM still says IDLE");
    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); expect_zero();
    PASS("B7.2 pending STOP prevents AUTO admission");

    b72_idle(&uart, &peripheral); mode_entry_tick = clock_tick;
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick)); ++clock_tick;
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); expect_zero();
    PASS("B7.2 same-tick IDLE entry is ambiguous but a later AUTO is accepted");
    b72_idle(&uart, &peripheral); TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP);
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick)); ++clock_tick;
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); expect_zero();
    PASS("B7.2 same-tick STOP AUTO rejected and later fresh AUTO works");
    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET));
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); expect_zero();
    PASS("B7.2 same-tick RESET AUTO rejected and request after reset processing works");

    b72_idle(&uart, &peripheral); clock_tick += 500U;
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick - 500U)); CHECK(event_count == 0U);
    PASS("B7.2 AUTO exactly 500 ms old is rejected at admission");
    b72_idle(&uart, &peripheral);
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick + 1U)); CHECK(event_count == 0U);
    PASS("B7.2 future timestamp cannot become a fresh AUTO request");
    b72_idle(&uart, &peripheral); clock_tick += 499U;
    CHECK(TaskControlRobot_RequestAutoAt(1000U)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 AUTO 499 ms old cannot regain freshness while queued");
    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
    clock_tick = 1480U; run_fsm(1); CHECK(clock_tick == 1500U);
    CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 AUTO exactly 500 ms old is rejected at consumption");
    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
    clock_tick = 1479U; run_fsm(1); CHECK(clock_tick == 1499U);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); expect_zero();
    PASS("B7.2 AUTO age 499 at consumption is accepted without movement");

    b72_idle(&uart, &peripheral); uart_feed(&uart, "AU");
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1); ++clock_tick;
    uart_feed(&uart, "TO\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response, "AUTO_REJECTED\n") == 0 && event_count == 0U);
    PASS("B7.2 partial AUTO frame started before STOP cannot activate afterward");
    b72_idle(&uart, &peripheral); uart_feed(&uart, "AU");
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET)); run_fsm(1); ++clock_tick;
    uart_feed(&uart, "TO\n"); CHECK(uart_consume(&uart));
    CHECK(strcmp(uart_response, "AUTO_REJECTED\n") == 0 && event_count == 0U);
    PASS("B7.2 partial AUTO frame started before RESET cannot activate afterward");
    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 queued AUTO ahead of STOP invalidated even in the same tick");
    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 queued AUTO ahead of RESET invalidated even in the same tick");
    for (int action = 1; action <= 3; ++action) {
        b72_idle(&uart, &peripheral); event_send_action = action;
        CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
        if (action == 3) run_safety(1);
        run_fsm(1); CHECK(robot_state != ROBOT_STATE_AUTONOMOUS); expect_zero();
        PASS("B7.2 STOP RESET or Safety between AUTO admission and enqueue cancels activation");
    }
    b72_idle(&uart, &peripheral);
    for (unsigned i = 0U; i < 8U; ++i) CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    b65_uart_command(&uart, "AUTO\n", "AUTO_REJECTED\n");
    CHECK(event_count == 8U && robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 full existing event queue rejects AUTO without changing mode or outputs");
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); expect_zero(); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE && !stop_pending);
    PASS("B7.2 full queue cannot prevent STOP cancellation or IDLE fallback");
    b72_idle(&uart, &peripheral);
    for (unsigned i = 0U; i < 8U; ++i) CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 full queue of old AUTO requests is invalidated by STOP");
    b72_idle(&uart, &peripheral);
    for (unsigned i = 0U; i < 8U; ++i) CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 RESET with full queue still invalidates all older AUTO requests");
    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
    fault_at_next_outer_exit = 1; run_fsm(1);
    CHECK(TaskSafety_IsStopActive()); expect_zero(); run_safety(1); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_SAFETY_STOP); expect_zero();
    PASS("B7.2 fault made ready during AUTO transition cannot enable PWM");
    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
    TaskControlRobotEventMessage_t old_auto = events[0];
    TaskSafety_ReportFault(SAFETY_FAULT_SENSOR); run_safety(1); run_fsm(1);
    clock_tick += 100U; b65_sensor_snapshot(0U);
    b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE);
    CHECK(xQueueSend(event_queue, &old_auto, 0U) == pdTRUE); run_fsm(1);
    CHECK((TickType_t)(clock_tick - old_auto.issued_tick) < 500U);
    CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 late AUTO preceding a real fault and reset is rejected before its time deadline");

    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_AUTO)); run_fsm(1);
    TaskSafety_ReportFault(SAFETY_FAULT_CLIFF); run_safety(1); run_fsm(1);
    b65_uart_command(&uart, "STOP\n", "STOP_QUEUED\n"); run_fsm(1);
    b65_uart_command(&uart, "STATUS\n", "STATE=3 FAULTS=1 STOP=1\n"); expect_zero();
    clock_tick += 100U; b65_sensor_snapshot(0U);
    b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); run_fsm(1); run_motor(1);
    b65_uart_command(&uart, "STATUS\n", "STATE=0 FAULTS=0 STOP=0\n"); expect_zero();
    run_fsm(5); CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 Safety STOP stays latched and explicit reset never restores AUTO");
    ++clock_tick; b65_uart_command(&uart, "AUTO\n", "AUTO_QUEUED\n"); run_fsm(1); expect_zero();
    b65_uart_command(&uart, "STOP\n", "STOP_QUEUED\n"); run_fsm(1);
    b65_uart_command(&uart, "MANUAL\n", "MANUAL_QUEUED\n"); run_fsm(1); ++clock_tick;
    b65_uart_command(&uart, "AVANZAR\n", "MOVE_ACCEPTED\n"); run_motor(1);
    CHECK(robot_state == ROBOT_STATE_MANUAL && pwm.compare[0] > 0U);
    TaskMotorControl_Stop();
    PASS("B7.2 AUTO STOP MANUAL requires a fresh explicit manual movement command");

    b72_idle(&uart, &peripheral); clock_tick = 0U;
    mode_entry_tick = UINT32_MAX - 2U; auto_barrier_tick = UINT32_MAX - 1U;
    CHECK(TaskControlRobot_RequestAutoAt(0U)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); expect_zero();
    PASS("B7.2 legitimate AUTO tick zero works across tick wrap");
    b72_idle(&uart, &peripheral); auto_request_epoch = UINT32_MAX;
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 epoch wrap cannot revive an AUTO request preceding STOP");
    b72_idle(&uart, &peripheral); event_queue = NULL;
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick));
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_START_AUTO)); expect_zero();
    PASS("B7.2 absent event queue rejects AUTO safely");
    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_MANUAL); expect_zero();
    PASS("B7.2 mode changed while queued prevents AUTO at consumption");
    b72_idle(&uart, &peripheral); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET));
    ++clock_tick; CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE); expect_zero();
    PASS("B7.2 reset processing invalidates AUTO already waiting behind reset");
    b72_idle(&uart, &peripheral); event_queue = NULL; TaskControlRobot_Create();
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick)); ++clock_tick;
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); expect_zero();
    PASS("B7.2 real task creation keeps one existing eight-entry queue with dated events");
}

#include "test_navigation.c"
#include "test_navigation_reverse.c"
#include "test_navigation_turn.c"

/* W6: coexistencia real de FSM/navegacion, datalogger y telemetria W5. */
static void test_w6_integration(void) {
    DrvUart_t uart; UART_HandleTypeDef peripheral = {0};
    producer_setup(); robot_state = ROBOT_STATE_IDLE; DrvUart_Init(&uart, &peripheral);
    b73_fresh();
    b65_uart_command(&uart, "AUTO\n", "AUTO_QUEUED\n");
    CHECK(!count_code(110U)); run_fsm(1);
    check_event(110U, 0U, 1U, ROBOT_EVENT_START_AUTO);
    CHECK(count_code(110U) == 1U); expect_zero();
    PASS("W6 actual queued AUTO transition publishes logger event outside every critical section");
    ++clock_tick; b73_fresh(); run_fsm(1); run_motor(1);
    CHECK(Navigation_GetState() == NAV_AVANZAR && pwm.compare[0] > 0U);
    b65_uart_command(&uart, "STATUS\n", "STATE=1 FAULTS=0 STOP=0\n");
    input.resultado_frontal_disponible = input.resultado_trasero_disponible = 1U;
    input.timestamp_resultado_frontal = input.timestamp_resultado_trasero = clock_tick;
    input.timestamp = input.timestamp_ir = clock_tick;
    b65_uart_command(&uart, "SENSORS Q=42\n", "SENSORS,1,42,1,0,1,500,0,1,500,0,15,0,0\n");
    CHECK(Navigation_GetState() == NAV_AVANZAR && task_context.left_target_cps == 1600);
    PASS("W6 STATUS and SENSORS preserve real AUTO state and independent telemetry without changing its targets");
    clear_log(); b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); expect_zero();
    CHECK(!count_code(300U)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS && Navigation_GetState() == NAV_DETENIDO);
    CHECK(!count_code(110U) && !count_code(210U)); expect_zero();
    PASS("W6 RESET cancels navigation without inventing STOP log, successful rearm or automatic restart");
    b65_uart_command(&uart, "STOP\n", "STOP_QUEUED\n"); run_fsm(1);
    check_event(300U, 1U, 0U, 0U); check_event(110U, 1U, 0U, ROBOT_EVENT_STOP);
    CHECK(count_code(300U) == 1U); expect_zero();
    PASS("W6 AUTO STOP preserves one request log and one effective IDLE transition");

    producer_setup(); robot_state = ROBOT_STATE_IDLE; DrvUart_Init(&uart, &peripheral);
    uart_feed(&uart, "SENSORS\nAUTO\nSTATUS\n");
    CHECK(uart_consume(&uart) && !strcmp(uart_response, "AUTO_QUEUED\n"));
    CHECK(!uart_consume(&uart)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS);
    PASS("W6 AUTO mode request outranks monitoring in UART mailbox, preserving STOP/DETENER priority");
    producer_setup(); robot_state = ROBOT_STATE_IDLE; logger_capacity = 0U;
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS && attempted_count == 1U && !logged_count);
    TaskSafety_ReportFault(SAFETY_FAULT_CLIFF); run_safety(1); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_SAFETY_STOP); expect_zero();
    PASS("W6 saturated logger cannot prevent AUTO entry or cliff cancellation and Safety latch");
    producer_logging_enabled = false;
}

static void test_w7_integration(void) {
    setup();left_encoder_delta=-13;right_encoder_delta=24;run_motor(2);
    OdometrySnapshot od;Odometry_GetSnapshot(&od);
    CHECK(od.left_count==-26&&od.right_count==48&&od.state==2U&&od.calibration==ODOM_GEOMETRY_PROVISIONAL);
    CHECK(left_encoder_reads==3U&&right_encoder_reads==3U);
    CHECK(task_context.left_speed_cps==-1300&&task_context.right_speed_cps==2400);
    PASS("W7 real MotorControl feeds both deltas to odometry once, after PWM, without changing speed conversion");
    TaskMotorControl_Stop();Odometry_GetSnapshot(&od);CHECK(od.left_count==-26&&od.right_count==48);
    CHECK(task_context.left_integral==0&&task_context.right_integral==0);expect_zero();
    PASS("W7 STOP preserves measured odometry counts while clearing PI targets and PWM");
    DrvUart_t uart;UART_HandleTypeDef peripheral={0};DrvUart_Init(&uart,&peripheral);
    uart_feed(&uart,"MAPPING Q=24 S=0\n");CHECK(uart_consume(&uart));
    CHECK(strncmp(uart_response,"POSE,2,24,",10U)==0&&strstr(uart_response,",2,1,-26,48,"));
    CHECK(!target_pending&&robot_state==ROBOT_STATE_MANUAL);expect_zero();
    PASS("W7 actual TaskComms reads a consistent provisional pose without issuing movement or changing STATUS/SENSORS");
    uart_feed(&uart,"MAPPING Q=25 S=1\nSTATUS\n");CHECK(uart_consume(&uart)&&strncmp(uart_response,"STATE=",6U)==0);
    uart_feed(&uart,"MAPPING Q=26 S=0\nSTOP\n");CHECK(uart_consume(&uart)&&strcmp(uart_response,"STOP_QUEUED\n")==0);
    run_fsm(1);expect_zero();CHECK(robot_state==ROBOT_STATE_IDLE);
    PASS("W7 UART telemetry cannot displace STATUS or STOP and leaves no reactivation target");
    uart_feed(&uart,"MAPPING Q=27 S=0\nSENSORS Q=28\n");
    CHECK(uart_consume(&uart)&&strncmp(uart_response,"SENSORS,1,28,",13U)==0);
    uart_feed(&uart,"SENSORS Q=29\nMAPPING Q=30 S=0\n");
    CHECK(uart_consume(&uart)&&strncmp(uart_response,"SENSORS,1,29,",13U)==0);
    PASS("W7 SENSORS outranks mapping in either burst order without changing operational priority classes");
}

int run_permanent_regressions(void) {

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
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_START_AUTO));
    TaskMotorControl_SetTargetSpeeds(100, 100); run_fsm(1); run_motor(1);
    expect_zero(); CHECK(robot_state == ROBOT_STATE_IDLE);
    PASS("real FSM queue STOP rejects AUTO while pending without restoring motion");
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

    test_motor_encoder_regressions();
    test_public_velocity_control();
    test_motion_commands();
    test_imu_protocol();
    test_stop_b65();
    test_log_producers();
    test_sensor_telemetry();
    test_auto_b72();
    test_navigation_b73();
    test_navigation_b75();
    test_w6_integration();
    test_w7_integration();
    return 0;
}

int main(void) {
#if NAV_EXPERIMENTAL_TURN_ENABLED
    test_turn_enabled_b77();
#else
    (void)run_permanent_regressions();
    test_turn_disabled_b77();
#endif
    printf("RESULT %u tests passed; TURN=%u LEFT=%u; queues/waits outside critical sections; "
           "at most 6 GPIO/CCR writes per transaction. Hardware timing pending.\n",
           tests, (unsigned)NAV_EXPERIMENTAL_TURN_ENABLED, (unsigned)NAV_TURN_LEFT);

    return 0;
}
