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
  /* 512 words: LOG_INFO -> vsnprintf (newlib-nano) + FatFs (f_write/f_sync). */
  BaseType_t result = xTaskCreate(
      vTaskExample,
      "Example",
      512,
      NULL,
      tskIDLE_PRIORITY + 1,
      NULL);

  if (result != pdPASS)
  {
    Error_Handler();
  }
}
