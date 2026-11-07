#include "task_motor_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "task_safety.h"
#include "task_control_robot.h"

#define MOTOR_CONTROL_PERIOD_MS 10U
#define MOTOR_CONTROL_STACK_SIZE 256U
#define MOTOR_CONTROL_PRIORITY (tskIDLE_PRIORITY + 5U)
#define MOTOR_CONTROL_COMMAND_TIMEOUT_MS 500U

// Parametros iniciales para pruebas de software.
// No representan una calibracion de los motores reales.
#define MOTOR_PI_KP 0.50f
#define MOTOR_PI_KI 0.10f
#define MOTOR_PI_DT_S 0.010f

typedef struct {
    int32_t left_speed_cps;
    int32_t right_speed_cps;
    TickType_t issued_tick;
    uint32_t generation;
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
} TaskMotorControlContext_t;

static TaskMotorControlContext_t task_context;
static QueueHandle_t target_speed_queue = NULL;
// Orden total de emisiones/paradas, incluso si comparten tick o se encolan tarde.
static uint32_t command_generation = 0U;

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
}

void TaskMotorControl_Stop(void) {
    taskENTER_CRITICAL();
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

    for (;;) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));

        TaskMotorControlTarget_t target;
        BaseType_t received = xQueueReceive(target_speed_queue, &target, 0U);
        int32_t left_delta = Encoder_ReadDelta(context->encoder_left);
        int32_t right_delta = Encoder_ReadDelta(context->encoder_right);

        /* Serializa objetivos, PI y salida con STOP/ReportFault. Solo calculos
         * acotados y escrituras GPIO/CCR: sin colas, encoder, esperas ni retardos.
         */
        taskENTER_CRITICAL();
        TickType_t now = xTaskGetTickCount();
        RobotControlState_t state = TaskControlRobot_GetState();
        bool enabled = !TaskSafety_IsStopActive() &&
                       (state == ROBOT_STATE_MANUAL || state == ROBOT_STATE_AUTONOMOUS);
        context->left_speed_cps = TaskMotorControl_DeltaToCps(left_delta);
        context->right_speed_cps = TaskMotorControl_DeltaToCps(right_delta);

        if (enabled && received == pdTRUE &&
            target.generation == command_generation &&
            (TickType_t)(now - target.issued_tick) <
            pdMS_TO_TICKS(MOTOR_CONTROL_COMMAND_TIMEOUT_MS)) {
            context->left_target_cps = target.left_speed_cps;
            context->right_target_cps = target.right_speed_cps;
            context->command_tick = target.issued_tick;
            context->command_generation = target.generation;
        }

        if (!enabled || context->command_generation != command_generation ||
            (TickType_t)(now - context->command_tick) >=
            pdMS_TO_TICKS(MOTOR_CONTROL_COMMAND_TIMEOUT_MS)) {
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
        taskEXIT_CRITICAL();
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

void TaskMotorControl_SetTargetSpeeds(int32_t left_speed_cps,
                                      int32_t right_speed_cps) {
    TaskMotorControlTarget_t target;
    // Este es el punto de emision real: no existen productores intermedios.
    // Tick y epoca se capturan juntos, incluso si STOP ocurre en el mismo tick.
    taskENTER_CRITICAL();
    RobotControlState_t state = TaskControlRobot_GetState();

    // Rechaza ordenes si existe una parada de seguridad
    // o si el robot no esta en un modo de movimiento.
    if (TaskSafety_IsStopActive() ||
        (state != ROBOT_STATE_MANUAL &&
        state != ROBOT_STATE_AUTONOMOUS) || target_speed_queue == NULL) {
        taskEXIT_CRITICAL();
        return;
    }
    target.left_speed_cps = left_speed_cps;
    target.right_speed_cps = right_speed_cps;
    target.issued_tick = xTaskGetTickCount();
    target.generation = ++command_generation;
    taskEXIT_CRITICAL();

    // Sin colas dentro de la seccion critica. Si hubo STOP, se rechazara la epoca.
    (void)xQueueOverwrite(target_speed_queue, &target);
}
