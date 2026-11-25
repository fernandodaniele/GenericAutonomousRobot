#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "drv_TCRT5000.h"
#include "drv_ultrasound.h"
#include "task_sensores.h"
#include "mid_odometry.h"

#define SENSORES_PERIODO_BASE_MS  10U
#define SENSORES_PERIODO_IR_MS    20U
#define SENSORES_SEPARACION_US_MS 70U
#define SENSORES_TIMEOUT_US_MS   50U
#define SENSORES_CICLO_PASOS     10U /* 100 ms para el contador de lecturas IR */
#define SENSORES_PRIORIDAD        (tskIDLE_PRIORITY + 4U)
#define SENSORES_STACK_WORDS      320U

typedef enum {
    ULTRASOUND_FRONT = 0,
    ULTRASOUND_REAR
} UltrasoundSensor_t;

typedef enum {
    ULTRASOUND_IDLE = 0,
    ULTRASOUND_WAITING,
    ULTRASOUND_STOPPING
} UltrasoundState_t;

static QueueHandle_t sensores_queue = NULL;

static void TaskSensores_SetUltrasoundResult(SensoresDatos_t *data,
                                             UltrasoundSensor_t sensor,
                                             float distance,
                                             TickType_t trigger_tick,
                                             uint8_t valid) {
    float *published_distance = (sensor == ULTRASOUND_FRONT)
                                ? &data->distancia_m : &data->distancia_trasera_m;
    uint8_t *published_valid = (sensor == ULTRASOUND_FRONT)
                              ? &data->ultrasonido_valido : &data->ultrasonido_trasero_valido;
    TickType_t *published_tick = (sensor == ULTRASOUND_FRONT)
                                ? &data->timestamp_ultrasonido_frontal
                                : &data->timestamp_ultrasonido_trasero;

    *published_distance = valid ? distance : 0.0f;
    if (sensor == ULTRASOUND_FRONT) {
        data->resultado_frontal_disponible = 1U;
        data->timestamp_resultado_frontal = trigger_tick;
    } else {
        data->resultado_trasero_disponible = 1U;
        data->timestamp_resultado_trasero = trigger_tick;
    }
    *published_valid = valid;
    if (valid != 0U) {
        *published_tick = trigger_tick;
    }
    Odometry_Observe((unsigned)sensor, distance, trigger_tick, valid != 0U);
}

static void vTaskSensores(void *pvParameters)
{
    (void)pvParameters;

    SensoresDatos_t datos = {0};
    TickType_t ultimo_despertar = xTaskGetTickCount();
    TickType_t acquisition_tick = 0U;
    TickType_t last_attempt_end = 0U;
    UltrasoundSensor_t active_sensor = ULTRASOUND_FRONT;
    UltrasoundSensor_t next_sensor = ULTRASOUND_FRONT;
    UltrasoundState_t acquisition_state = ULTRASOUND_IDLE;
    uint8_t has_attempt = 0U;
    uint8_t paso = 0U;

    TCRT_Init();

    for (;;)
    {
        /* Ejecutar la lectura IR en los instantes 0, 20, 40, ... ms. */
        if ((paso % (SENSORES_PERIODO_IR_MS / SENSORES_PERIODO_BASE_MS)) == 0U)
        {
            datos.ir = TCRT_LeerTodos();
            datos.timestamp_ir = xTaskGetTickCount();
            datos.hay_caida =
            (datos.ir.del_izq  == TCRT_NO_DETECTA) ||
            (datos.ir.del_der  == TCRT_NO_DETECTA) ||
            (datos.ir.tras_izq == TCRT_NO_DETECTA) ||
            (datos.ir.tras_der == TCRT_NO_DETECTA);
        }

        if (acquisition_state == ULTRASOUND_WAITING) {
            TickType_t elapsed = (TickType_t)(xTaskGetTickCount() - acquisition_tick);
            uint8_t ready = (active_sensor == ULTRASOUND_FRONT)
                            ? HCSR04_IsReady() : HCSR04_Rear_IsReady();

            /* Una captura observada despues del limite no rejuvenece el dato. */
            if ((ready != 0U) ||
                (elapsed >= pdMS_TO_TICKS(SENSORES_TIMEOUT_US_MS))) {
                float distance = 0.0f;
                if ((ready != 0U) &&
                    (elapsed <= pdMS_TO_TICKS(SENSORES_TIMEOUT_US_MS))) {
                    distance = (active_sensor == ULTRASOUND_FRONT)
                               ? HCSR04_GetDistance() : HCSR04_Rear_GetDistance();
                }

                HAL_StatusTypeDef stopped = (active_sensor == ULTRASOUND_FRONT)
                                            ? HCSR04_Cancel() : HCSR04_Rear_Cancel();
                uint8_t valid = (stopped == HAL_OK) &&
                                (distance >= 0.02f) && (distance <= 4.0f);
                TaskSensores_SetUltrasoundResult(&datos, active_sensor,
                                                 distance, acquisition_tick, valid);
                acquisition_state = (stopped == HAL_OK)
                                    ? ULTRASOUND_IDLE : ULTRASOUND_STOPPING;
            }
        } else if (acquisition_state == ULTRASOUND_STOPPING) {
            /* No habilitar el otro sensor mientras el stream siga sin cerrarse. */
            HAL_StatusTypeDef stopped = (active_sensor == ULTRASOUND_FRONT)
                                        ? HCSR04_Cancel() : HCSR04_Rear_Cancel();
            if (stopped == HAL_OK) {
                acquisition_state = ULTRASOUND_IDLE;
            }
        }

        if ((acquisition_state == ULTRASOUND_IDLE) &&
            ((has_attempt == 0U) ||
             ((TickType_t)(xTaskGetTickCount() - last_attempt_end) >=
              pdMS_TO_TICKS(SENSORES_SEPARACION_US_MS)))) {
            active_sensor = next_sensor;
            acquisition_tick = xTaskGetTickCount();
            HAL_StatusTypeDef started = (active_sensor == ULTRASOUND_FRONT)
                                        ? HCSR04_Trigger() : HCSR04_Rear_Trigger();

            /* Medir la separación desde el fin del intento de disparo, sin
             * compensar retrasos con disparos acumulados. */
            last_attempt_end = xTaskGetTickCount();
            has_attempt = 1U;
            next_sensor = (active_sensor == ULTRASOUND_FRONT)
                          ? ULTRASOUND_REAR : ULTRASOUND_FRONT;
            if (started == HAL_OK) {
                acquisition_state = ULTRASOUND_WAITING;
            } else {
                TaskSensores_SetUltrasoundResult(&datos, active_sensor,
                                                 0.0f, acquisition_tick, 0U);
                acquisition_state = ULTRASOUND_STOPPING;
            }
        }

        /* Publicar una copia consistente para el resto de las tareas. */
        datos.timestamp = xTaskGetTickCount();
        (void)xQueueOverwrite(sensores_queue, &datos);

        paso = (uint8_t)((paso + 1U) %
                        SENSORES_CICLO_PASOS);

        vTaskDelayUntil(&ultimo_despertar,
                        pdMS_TO_TICKS(SENSORES_PERIODO_BASE_MS));
    }
}

void TaskSensores_Create(void)
{
    sensores_queue = xQueueCreate(1U, sizeof(SensoresDatos_t));
    if (sensores_queue == NULL)
    {
        Error_Handler();
    }

    if (xTaskCreate(vTaskSensores,
                    "Sensores",
                    SENSORES_STACK_WORDS,
                    NULL,
                    SENSORES_PRIORIDAD,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }
}

BaseType_t TaskSensores_GetData(SensoresDatos_t *out)
{
    if ((out == NULL) || (sensores_queue == NULL))
    {
        return pdFALSE;
    }

    return xQueuePeek(sensores_queue, out, 0U);
}
