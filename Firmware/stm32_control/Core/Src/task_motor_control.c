#include "task_motor_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "task_safety.h"
#include "task_control_robot.h"
#include "mid_navigation.h"
#include "mid_motion.h"
#include "mid_odometry.h"

#define MOTOR_CONTROL_PERIOD_MS 10U
#define MOTOR_CONTROL_STACK_SIZE 256U
#define MOTOR_CONTROL_PRIORITY (tskIDLE_PRIORITY + 5U)
#define MOTOR_CONTROL_COMMAND_TIMEOUT_MS 500U

// Parametros iniciales para pruebas de software.
// No representan una calibracion de los motores reales.
#define MOTOR_PI_KP 2.20f
#define MOTOR_PI_KI 0.10f
#define MOTOR_PI_DT_S 0.010f

typedef struct {
    int32_t left_speed_cps;
    int32_t right_speed_cps;
    TickType_t issued_tick;
    uint32_t generation;
    RobotControlState_t mode;
    uint32_t navigation_session;
} TaskMotorControlTarget_t;

typedef struct {
    EncoderHandle_t *encoder_left;
    EncoderHandle_t *encoder_right;
    MotorHandle_t *motor_left;
    MotorHandle_t *motor_right;
    int32_t left_speed_cps;
    int32_t right_speed_cps;
    int32_t left_target_cps;
    int32_t right_target_cps;

    float left_integral;
    float right_integral;

    int16_t left_pwm_command;
    int16_t right_pwm_command;
    TickType_t command_tick;
    uint32_t command_generation;
    RobotControlState_t command_mode;
    uint32_t navigation_session;
} TaskMotorControlContext_t;

static TaskMotorControlContext_t task_context;
static QueueHandle_t target_speed_queue = NULL;
// Orden total de emisiones/paradas, incluso si comparten tick o se encolan tarde.
static uint32_t command_generation = 0U;
static TickType_t last_stop_tick = 0U;
static bool has_stop = false;

/* Con scheduler activo requiere seccion critica. Conserva velocidades medidas. */
static void TaskMotorControl_ClearTargets(TaskMotorControlContext_t *context) {
    context->left_target_cps = 0;
    context->right_target_cps = 0;
    context->left_integral = 0.0f;
    context->right_integral = 0.0f;
    context->left_pwm_command = 0;
    context->right_pwm_command = 0;
    context->command_tick = 0U;
    context->command_generation = command_generation;
    context->command_mode = ROBOT_STATE_IDLE;
    context->navigation_session = 0U;
}

void TaskMotorControl_Stop(void) {
    taskENTER_CRITICAL();
    Navigation_Cancel();
    last_stop_tick = xTaskGetTickCount();
    has_stop = true;
    ++command_generation;
    TaskMotorControl_ClearTargets(&task_context);
    // Los handles se registran antes de iniciar las tareas; tolerar aun no creado.
    if (task_context.motor_left != NULL) {
        Motor_Stop(task_context.motor_left);
    }
    if (task_context.motor_right != NULL) {
        Motor_Stop(task_context.motor_right);
    }
    taskEXIT_CRITICAL();
}

static int16_t MotorPI_Calculate(int32_t target_cps,
                                 int32_t measured_cps,
                                 float *integral, uint16_t pwm_limit)
{
    if (target_cps == 0 || pwm_limit == 0U) {
        *integral = 0.0f;
        return 0;
    }
    float limit = (float)pwm_limit;
    /* Coherente tambien si el limite se redujo desde el ciclo anterior. */
    if (*integral > limit) *integral = limit;
    if (*integral < -limit) *integral = -limit;

    float error = (float)target_cps - (float)measured_cps;

    float proposed_integral =
        *integral + MOTOR_PI_KI * MOTOR_PI_DT_S * error;

    // Limita el termino integral.
    if (proposed_integral > limit) {
        proposed_integral = limit;
    } else if (proposed_integral < -limit) {
        proposed_integral = -limit;
    }

    float proposed_output =
        MOTOR_PI_KP * error + proposed_integral;

    // Anti-windup por integracion condicional:
    // evita aumentar la saturacion, pero permite recuperarse.
    if ((proposed_output <= limit && proposed_output >= -limit) ||
        (proposed_output > limit && error < 0.0f) ||
        (proposed_output < -limit && error > 0.0f)) {

        *integral = proposed_integral;
    }

    float output = MOTOR_PI_KP * error + *integral;

    // Saturacion del PWM.
    if (output > limit) {
        output = limit;
    } else if (output < -limit) {
        output = -limit;
    }

    return (int16_t)output;
}

static int32_t TaskMotorControl_DeltaToCps(int32_t delta) {
    int64_t cps = (int64_t)delta * (1000U / MOTOR_CONTROL_PERIOD_MS);
    if (cps > INT32_MAX) return INT32_MAX;
    if (cps < INT32_MIN) return INT32_MIN;
    return (int32_t)cps;
}

static void TaskMotorControl_Run(void *parameters) {
    TaskMotorControlContext_t *context = (TaskMotorControlContext_t *)parameters;
    Motor_Stop(context->motor_left);
    Motor_Stop(context->motor_right);
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)Encoder_ReadDelta(context->encoder_left);
    (void)Encoder_ReadDelta(context->encoder_right);
    Odometry_Init(xTaskGetTickCount());

    for (;;) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));

        Navigation_ObserveMotionSensors(); /* AUTO-4 only; outside the PI/PWM lock. */
        TaskMotorControlTarget_t target;
        BaseType_t received = xQueueReceive(target_speed_queue, &target, 0U);
        int32_t left_delta = Encoder_ReadDelta(context->encoder_left);
        int32_t right_delta = Encoder_ReadDelta(context->encoder_right);
        TickType_t encoder_tick = xTaskGetTickCount();

        /* Serializa objetivos, PI y salida con STOP/ReportFault. Solo calculos
         * acotados y escrituras GPIO/CCR: sin colas, encoder, esperas ni retardos.
         */
        taskENTER_CRITICAL();
        TickType_t now = xTaskGetTickCount();
        RobotControlState_t state = TaskControlRobot_GetState();
        bool enabled = !TaskSafety_IsStopActive() &&
                       !TaskControlRobot_IsStopPending() &&
                       (state == ROBOT_STATE_MANUAL || state == ROBOT_STATE_AUTONOMOUS);
        context->left_speed_cps = TaskMotorControl_DeltaToCps(left_delta);
        context->right_speed_cps = TaskMotorControl_DeltaToCps(right_delta);

        /* Tambien termina una autorizacion vencida recibida antes de haber
         * aplicado su primer objetivo. Un mensaje de otra fase no la cancela.
         */
        if (received == pdTRUE && target.mode == ROBOT_STATE_AUTONOMOUS &&
            target.generation == command_generation &&
            (!Navigation_IsCommandCurrent(target.navigation_session, now) ||
             (TickType_t)(now - target.issued_tick) >= pdMS_TO_TICKS(MOTOR_CONTROL_COMMAND_TIMEOUT_MS))) {
            bool complete = false;
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
            complete = enabled && state == ROBOT_STATE_AUTONOMOUS &&
                (TickType_t)(now - target.issued_tick) < pdMS_TO_TICKS(MOTOR_CONTROL_COMMAND_TIMEOUT_MS) &&
                Navigation_CompleteCommand(target.navigation_session, now);
#endif
            if (!complete) Navigation_CancelSession(target.navigation_session);
        }

        if (enabled && received == pdTRUE &&
            target.mode == state &&
            (state == ROBOT_STATE_MANUAL ||
             Navigation_IsCommandCurrent(target.navigation_session, now)) &&
            target.generation == command_generation &&
            (TickType_t)(now - target.issued_tick) <
            pdMS_TO_TICKS(MOTOR_CONTROL_COMMAND_TIMEOUT_MS)) {
            context->left_target_cps = target.left_speed_cps;
            context->right_target_cps = target.right_speed_cps;
            context->command_tick = target.issued_tick;
            context->command_generation = target.generation;
            context->command_mode = target.mode;
            context->navigation_session = target.navigation_session;
        }

        if (!enabled || context->command_mode != state ||
            (state == ROBOT_STATE_AUTONOMOUS &&
             !Navigation_IsCommandCurrent(context->navigation_session, now)) ||
            context->command_generation != command_generation ||
            (TickType_t)(now - context->command_tick) >=
            pdMS_TO_TICKS(MOTOR_CONTROL_COMMAND_TIMEOUT_MS)) {
            /* Un timeout/cambio de permiso termina AUTO. El desfase transitorio
             * de generacion durante una renovacion no cancela otra sesion.
             * Solo estado; la ruta normal de abajo escribe ambos PWM cero.
             */
            if (context->command_mode == ROBOT_STATE_AUTONOMOUS &&
                (!enabled || state != ROBOT_STATE_AUTONOMOUS ||
                 !Navigation_IsCommandCurrent(context->navigation_session, now) ||
                 (TickType_t)(now - context->command_tick) >=
                 pdMS_TO_TICKS(MOTOR_CONTROL_COMMAND_TIMEOUT_MS))) {
                bool complete = false;
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
                complete = enabled && state == ROBOT_STATE_AUTONOMOUS &&
                    (TickType_t)(now - context->command_tick) < pdMS_TO_TICKS(MOTOR_CONTROL_COMMAND_TIMEOUT_MS) &&
                    Navigation_CompleteCommand(context->navigation_session, now);
#endif
                if (!complete) Navigation_CancelSession(context->navigation_session);
            }
            TaskMotorControl_ClearTargets(context);
        }

        // Un motor sin objetivo conserva PI/PWM en cero aunque siga girando.
        if (context->left_target_cps == 0) {
            context->left_integral = 0.0f;
            context->left_pwm_command = 0;
            Motor_Stop(context->motor_left);
        } else {
            context->left_pwm_command = MotorPI_Calculate(
                context->left_target_cps, context->left_speed_cps, &context->left_integral,
                Motor_GetPwmLimit(context->motor_left));
            Motor_SetSpeed(context->motor_left, context->left_pwm_command);
        }
        if (context->right_target_cps == 0) {
            context->right_integral = 0.0f;
            context->right_pwm_command = 0;
            Motor_Stop(context->motor_right);
        } else {
            context->right_pwm_command = MotorPI_Calculate(
                context->right_target_cps, context->right_speed_cps, &context->right_integral,
                Motor_GetPwmLimit(context->motor_right));
            Motor_SetSpeed(context->motor_right, context->right_pwm_command);
        }
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
        if (context->command_mode == ROBOT_STATE_AUTONOMOUS) {
            Navigation_CommandApplied(context->navigation_session, now);
        }
#endif
        taskEXIT_CRITICAL();
        /* Observador W7: reutiliza las lecturas, despues de aplicar PWM y sin
         * extender la transaccion de seguridad. Ninguna E/S ni nuevo encoder. */
        Odometry_Update(left_delta, right_delta, encoder_tick);
    }
}

void TaskMotorControl_Create(EncoderHandle_t *encoder_left,
                             EncoderHandle_t *encoder_right,
                             MotorHandle_t *motor_left,
                             MotorHandle_t *motor_right) {
    // Verifica que los drivers recibidos sean válidos.
    if ((encoder_left == NULL) || (encoder_right == NULL) ||
        (motor_left == NULL) || (motor_right == NULL)) {
        Error_Handler();
        return;
    }
    // Creacion previa al scheduler, sin consignas habilitadas.
    TaskMotorControl_ClearTargets(&task_context);

    // Crea una cola de un elemento para conservar la última orden.
    target_speed_queue = xQueueCreate(1U, sizeof(TaskMotorControlTarget_t));

    if (target_speed_queue == NULL) {
        Error_Handler();
        return;
    }
    task_context.encoder_left = encoder_left;
    task_context.encoder_right = encoder_right;
    task_context.motor_left = motor_left;
    task_context.motor_right = motor_right;

    BaseType_t result = xTaskCreate(
        TaskMotorControl_Run,
        "MotorControl",
        MOTOR_CONTROL_STACK_SIZE,
        &task_context,
        MOTOR_CONTROL_PRIORITY,
        NULL
    );

    if (result != pdPASS) {
        Error_Handler();
    }
}

static bool TaskMotorControl_Publish(int32_t left_speed_cps, int32_t right_speed_cps,
                                     bool dated, TickType_t issued_tick,
                                     RobotControlState_t required_mode,
                                     bool from_navigation, uint32_t navigation_session) {
    TaskMotorControlTarget_t target;
    // Admision y epoca atomicas. La API clasica fecha esta llamada; la variante
    // fechada conserva el origen UART y comprueba las barreras de parada/modo.
    taskENTER_CRITICAL();
    RobotControlState_t state = TaskControlRobot_GetState();
    TickType_t now = xTaskGetTickCount();

    // La API manual no puede seleccionar AUTO mediante un parametro.
    NavigationState_t phase = NAV_DETENIDO;
    if (left_speed_cps == MOTION_DRIVE_CPS && right_speed_cps == MOTION_DRIVE_CPS) phase = NAV_AVANZAR;
    if (left_speed_cps == -MOTION_DRIVE_CPS && right_speed_cps == -MOTION_DRIVE_CPS) phase = NAV_RETROCEDER_ACOTADO;
#if NAV_EXPERIMENTAL_TURN_ENABLED || NAV_AUTO2_ENABLED
    int32_t turn_left,turn_right;Navigation_TurnTargets(&turn_left,&turn_right);
    if (left_speed_cps == turn_left && right_speed_cps == turn_right) phase = NAV_GIRAR_ACOTADO;
#endif
    bool authorized_source = from_navigation ?
        (dated && required_mode == ROBOT_STATE_AUTONOMOUS &&
         state == ROBOT_STATE_AUTONOMOUS && TaskControlRobot_IsCurrentTask() &&
         phase != NAV_DETENIDO) :
        state == ROBOT_STATE_MANUAL;
    if (TaskSafety_IsStopActive() || TaskControlRobot_IsStopPending() ||
        !authorized_source || target_speed_queue == NULL) {
        taskEXIT_CRITICAL();
        return false;
    }
    if (dated && ((TickType_t)(now - issued_tick) >= pdMS_TO_TICKS(MOTOR_CONTROL_COMMAND_TIMEOUT_MS) ||
        (has_stop && (TickType_t)(now - issued_tick) >= (TickType_t)(now - last_stop_tick)) ||
        !TaskControlRobot_AcceptsMotionAt(required_mode, issued_tick))) {
        taskEXIT_CRITICAL();
        return false;
    }
    if (dated && left_speed_cps == 0 && right_speed_cps == 0) {
        TaskMotorControl_Stop();
        taskEXIT_CRITICAL();
        return true;
    }
    if (from_navigation &&
        !Navigation_ConsumeEmission(navigation_session, issued_tick, now, phase)) {
        taskEXIT_CRITICAL();
        return false;
    }
    target.left_speed_cps = left_speed_cps;
    target.right_speed_cps = right_speed_cps;
    target.issued_tick = dated ? issued_tick : now;
    target.generation = ++command_generation;
    target.mode = state;
    target.navigation_session = from_navigation ? navigation_session : 0U;
    taskEXIT_CRITICAL();

    // Sin colas dentro de la seccion critica. Si hubo STOP, se rechazara la epoca.
    return xQueueOverwrite(target_speed_queue, &target) == pdPASS;
}

void TaskMotorControl_SetTargetSpeeds(int32_t left_speed_cps, int32_t right_speed_cps) {
    (void)TaskMotorControl_Publish(left_speed_cps, right_speed_cps, false, 0U,
                                 ROBOT_STATE_IDLE, false, 0U);
}

bool TaskMotorControl_TrySetTargetSpeedsAt(int32_t left_speed_cps, int32_t right_speed_cps,
                                         TickType_t issued_tick, RobotControlState_t required_mode) {
    return TaskMotorControl_Publish(left_speed_cps, right_speed_cps, true, issued_tick,
                                    required_mode, false, 0U);
}

bool TaskMotorControl_TrySetNavigationSpeedsAt(int32_t left_speed_cps, int32_t right_speed_cps,
                                             TickType_t issued_tick, uint32_t session) {
    return TaskMotorControl_Publish(left_speed_cps, right_speed_cps, true, issued_tick,
                                    ROBOT_STATE_AUTONOMOUS, true, session);
}
