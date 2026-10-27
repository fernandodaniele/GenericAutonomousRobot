#include "task_motor_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "task_safety.h"

#define MOTOR_CONTROL_PERIOD_MS 10U
#define MOTOR_CONTROL_STACK_SIZE 256U
#define MOTOR_CONTROL_PRIORITY (tskIDLE_PRIORITY + 5U)
#define MOTOR_CONTROL_COMMAND_TIMEOUT_MS 500U

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
} TaskMotorControlContext_t;

static TaskMotorControlContext_t task_context;
static QueueHandle_t target_speed_queue = NULL;

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

        // Inhibe el movimiento mientras exista una parada de seguridad.
        if (TaskSafety_IsStopActive()) {

            // Descarta los objetivos de velocidad.
            context->left_target_cps = 0;
            context->right_target_cps = 0;

            // Elimina cualquier orden pendiente.
            (void)xQueueReset(target_speed_queue);

            // Ordena detener ambos motores.
            Motor_Stop(context->motor_left);
            Motor_Stop(context->motor_right);
        }

        // Calcula la velocidad a partir de las cuentas del período.
        int32_t left_delta = Encoder_ReadDelta(context->encoder_left);
        int32_t right_delta = Encoder_ReadDelta(context->encoder_right);

        context->left_speed_cps = left_delta * (int32_t)(1000U / MOTOR_CONTROL_PERIOD_MS);
        context->right_speed_cps = right_delta * (int32_t)(1000U / MOTOR_CONTROL_PERIOD_MS);
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
    // Rechaza nuevas ordenes durante una parada de seguridad.
    if (TaskSafety_IsStopActive()) {
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
