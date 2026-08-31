#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_example.h"
#include "mid_log.h"

static void vTaskExample(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    LOG_INFO("hola desde task_example");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskExample_Create(void)
{
  /* 384 words: LOG_INFO usa vsnprintf (newlib-nano). */
  BaseType_t result = xTaskCreate(
      vTaskExample,
      "Example",
      384,
      NULL,
      tskIDLE_PRIORITY + 1,
      NULL);

  if (result != pdPASS)
  {
    Error_Handler();
  }
}
