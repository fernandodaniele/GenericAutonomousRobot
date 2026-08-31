#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_ds1302.h"
#include "mid_log.h"

/* Periodo del heartbeat. */
#define TASK_DS1302_PERIOD_MS   1000U

static void vTaskDs1302(void *pvParameters)
{
  (void)pvParameters;

  /* Desfasar de task_example para repartir el uso del CDC en el tiempo. */
  vTaskDelay(pdMS_TO_TICKS(TASK_DS1302_PERIOD_MS / 2U));

  for (;;)
  {
    /* Latido visible: LED distinto al de task_example (LD4). */
    HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);

    /* El timestamp lo pone mid_log a partir del DS1302. */
    LOG_INFO("heartbeat, uptime=%lu s", (unsigned long)(HAL_GetTick() / 1000U));

    vTaskDelay(pdMS_TO_TICKS(TASK_DS1302_PERIOD_MS));
  }
}

void TaskDs1302_Create(void)
{
  /* 384 words: LOG_INFO usa vsnprintf (newlib-nano). */
  BaseType_t result = xTaskCreate(
      vTaskDs1302,
      "ds1302",
      384,
      NULL,
      tskIDLE_PRIORITY + 1,
      NULL);

  if (result != pdPASS)
  {
    Error_Handler();
  }
}
