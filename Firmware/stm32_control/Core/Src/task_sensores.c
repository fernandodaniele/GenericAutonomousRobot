#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "drv_TCRT5000.h"
#include "drv_ultrasound.h"
#include "task_sensores.h"

#define SENSORES_PERIODO_BASE_MS  10U
#define SENSORES_PERIODO_IR_MS    20U
#define SENSORES_PERIODO_US_MS    50U
#define SENSORES_CICLO_PASOS     10U /* 100 ms: multiplo de 20 y 50 */
#define SENSORES_PRIORIDAD        (tskIDLE_PRIORITY + 4U)
#define SENSORES_STACK_WORDS      320U

static QueueHandle_t sensores_queue = NULL;

static void vTaskSensores(void *pvParameters)
{
    (void)pvParameters;

    SensoresDatos_t datos = {0};
    TickType_t ultimo_despertar = xTaskGetTickCount();
    uint8_t paso = 0U;
    uint8_t ultrasonido_pendiente = 0U;

    TCRT_Init();

    for (;;)
    {
        /* Ejecutar la lectura IR en los instantes 0, 20, 40, ... ms. */
        if ((paso % (SENSORES_PERIODO_IR_MS / SENSORES_PERIODO_BASE_MS)) == 0U)
        {
            datos.ir = TCRT_LeerTodos();
            datos.hay_caida =
            (datos.ir.del_izq  == TCRT_NO_DETECTA) ||
            (datos.ir.del_der  == TCRT_NO_DETECTA) ||
            (datos.ir.tras_izq == TCRT_NO_DETECTA) ||
            (datos.ir.tras_der == TCRT_NO_DETECTA);
        }

        /* Captura por DMA: consultar sin detener la tarea. */
        if (ultrasonido_pendiente && HCSR04_IsReady())
        {
            float distancia = HCSR04_GetDistance();
            ultrasonido_pendiente = 0U;

            if ((distancia >= 0.02f) && (distancia <= 4.0f))
            {
                datos.distancia_m = distancia;
                datos.ultrasonido_valido = 1U;
            }
            else
            {
                datos.distancia_m = 0.0f;
                datos.ultrasonido_valido = 0U;
            }
        }

        /* Disparar una medicion a los 0, 50, 100, ... ms. */
        if ((paso % (SENSORES_PERIODO_US_MS / SENSORES_PERIODO_BASE_MS)) == 0U)
        {
            if (ultrasonido_pendiente)
            {
                /* El eco anterior no llego en el intervalo esperado. */
                datos.distancia_m = 0.0f;
                datos.ultrasonido_valido = 0U;
            }

            HCSR04_Trigger();
            ultrasonido_pendiente = 1U;
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
