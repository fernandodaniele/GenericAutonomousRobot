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

// Limite provisional: aproximadamente 10% del PWM disponible.
#define MOTOR_PI_PWM_LIMIT 840.0f

typedef struct {
    int32_t left_speed_cps;
    int32_t right_speed_cps;
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
} TaskMotorControlContext_t;

static TaskMotorControlContext_t task_context;
static QueueHandle_t target_speed_queue = NULL;

static int16_t MotorPI_Calculate(int32_t target_cps,
                                 int32_t measured_cps,
                                 float *integral)
{
    if (target_cps == 0) {
        *integral = 0.0f;
        return 0;
    }

    float error = (float)target_cps - (float)measured_cps;

    float proposed_integral =
        *integral + MOTOR_PI_KI * MOTOR_PI_DT_S * error;

    // Limita el termino integral.
    if (proposed_integral > MOTOR_PI_PWM_LIMIT) {
        proposed_integral = MOTOR_PI_PWM_LIMIT;
    } else if (proposed_integral < -MOTOR_PI_PWM_LIMIT) {
        proposed_integral = -MOTOR_PI_PWM_LIMIT;
    }

    float proposed_output =
        MOTOR_PI_KP * error + proposed_integral;

    // Anti-windup por integracion condicional:
    // evita aumentar la saturacion, pero permite recuperarse.
    if ((proposed_output <= MOTOR_PI_PWM_LIMIT &&
         proposed_output >= -MOTOR_PI_PWM_LIMIT) ||
        (proposed_output > MOTOR_PI_PWM_LIMIT && error < 0.0f) ||
        (proposed_output < -MOTOR_PI_PWM_LIMIT && error > 0.0f)) {

        *integral = proposed_integral;
    }

    float output = MOTOR_PI_KP * error + *integral;

    // Saturacion del PWM.
    if (output > MOTOR_PI_PWM_LIMIT) {
        output = MOTOR_PI_PWM_LIMIT;
    } else if (output < -MOTOR_PI_PWM_LIMIT) {
        output = -MOTOR_PI_PWM_LIMIT;
    }

    return (int16_t)output;
}

static void TaskMotorControl_Run(void *parameters) {
    TaskMotorControlContext_t *context = (TaskMotorControlContext_t *)parameters;

    // Garantiza que ambos motores comiencen detenidos.
    Motor_Stop(context->motor_left);
    Motor_Stop(context->motor_right);

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_command_tick = xTaskGetTickCount();

    // Descarta las cuentas acumuladas antes del inicio de la tarea.
    (void)Encoder_ReadDelta(context->encoder_left);
    (void)Encoder_ReadDelta(context->encoder_right);

    for (;;) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));

        // Recibe la última velocidad objetivo sin bloquear la tarea.
        TaskMotorControlTarget_t target;

        if (xQueueReceive(target_speed_queue, &target, 0U) == pdTRUE) {
            context->left_target_cps = target.left_speed_cps;
            context->right_target_cps = target.right_speed_cps;
            last_command_tick = xTaskGetTickCount();
        }

        // Invalida los objetivos si no llegan nuevas órdenes.
        if ((xTaskGetTickCount() - last_command_tick) >=
            pdMS_TO_TICKS(MOTOR_CONTROL_COMMAND_TIMEOUT_MS)) {
            context->left_target_cps = 0;
            context->right_target_cps = 0;
        }

        // Solo se permite movimiento en modo manual o autonomo.
        RobotControlState_t state = TaskControlRobot_GetState();

        if (TaskSafety_IsStopActive() ||
            (state != ROBOT_STATE_MANUAL &&
            state != ROBOT_STATE_AUTONOMOUS)) {

            // Descarta los objetivos de velocidad.
            context->left_target_cps = 0;
            context->right_target_cps = 0;

            // Elimina cualquier orden pendiente.
            (void)xQueueReset(target_speed_queue);

            // Detiene ambos motores.
            Motor_Stop(context->motor_left);
            Motor_Stop(context->motor_right);
        }

        // Calcula la velocidad a partir de las cuentas del período.
        int32_t left_delta = Encoder_ReadDelta(context->encoder_left);
        int32_t right_delta = Encoder_ReadDelta(context->encoder_right);

        context->left_speed_cps = left_delta * (int32_t)(1000U / MOTOR_CONTROL_PERIOD_MS);
        context->right_speed_cps = right_delta * (int32_t)(1000U / MOTOR_CONTROL_PERIOD_MS);

        // Calcula los comandos PWM mediante el controlador PI.
        RobotControlState_t pi_state = TaskControlRobot_GetState();

        if (!TaskSafety_IsStopActive() &&
            (pi_state == ROBOT_STATE_MANUAL ||
            pi_state == ROBOT_STATE_AUTONOMOUS)) {

            context->left_pwm_command = MotorPI_Calculate(
                context->left_target_cps,
                context->left_speed_cps,
                &context->left_integral
            );

            context->right_pwm_command = MotorPI_Calculate(
                context->right_target_cps,
                context->right_speed_cps,
                &context->right_integral
            );

        } else {

            context->left_integral = 0.0f;
            context->right_integral = 0.0f;

            context->left_pwm_command = 0;
            context->right_pwm_command = 0;
        }

        // Verifica nuevamente la habilitacion antes de aplicar PWM.
        RobotControlState_t output_state = TaskControlRobot_GetState();

        if (TaskSafety_IsStopActive() ||
            (output_state != ROBOT_STATE_MANUAL &&
            output_state != ROBOT_STATE_AUTONOMOUS)) {

            // Reinicia el controlador.
            context->left_integral = 0.0f;
            context->right_integral = 0.0f;

            context->left_pwm_command = 0;
            context->right_pwm_command = 0;

            // Detencion de ambos motores.
            Motor_Stop(context->motor_left);
            Motor_Stop(context->motor_right);

        } else {

            // Motor izquierdo.
            if (context->left_target_cps == 0) {
                Motor_Stop(context->motor_left);
            } else {
                Motor_SetSpeed(
                    context->motor_left,
                    context->left_pwm_command
                );
            }

            // Motor derecho.
            if (context->right_target_cps == 0) {
                Motor_Stop(context->motor_right);
            } else {
                Motor_SetSpeed(
                    context->motor_right,
                    context->right_pwm_command
                );
            }
        }
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
    // Inicializa las velocidades objetivo en cero.
    task_context.left_target_cps = 0;
    task_context.right_target_cps = 0;

    task_context.left_integral = 0.0f;
    task_context.right_integral = 0.0f;

    task_context.left_pwm_command = 0;
    task_context.right_pwm_command = 0;

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
    RobotControlState_t state = TaskControlRobot_GetState();

    // Rechaza ordenes si existe una parada de seguridad
    // o si el robot no esta en un modo de movimiento.
    if (TaskSafety_IsStopActive() ||
        (state != ROBOT_STATE_MANUAL &&
        state != ROBOT_STATE_AUTONOMOUS)) {
        return;
    }
    TaskMotorControlTarget_t target = {
        .left_speed_cps = left_speed_cps,
        .right_speed_cps = right_speed_cps
    };

    // Reemplaza la orden anterior por la más reciente.
    if (target_speed_queue != NULL) {
        (void)xQueueOverwrite(target_speed_queue, &target);
    }
}
