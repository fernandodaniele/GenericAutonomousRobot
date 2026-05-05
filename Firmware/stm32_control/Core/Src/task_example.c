#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include "task_example.h"

static void vTaskExample(void *pvParameters)
{
  const uint8_t message[] = "Hola";

  (void) pvParameters;

  for (;;)
  {
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    CDC_Transmit_FS((uint8_t *)message, sizeof(message) - 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskExample_Create(void)
{
  BaseType_t result = xTaskCreate(
      vTaskExample,
      "Example",
      256,
      NULL,
      tskIDLE_PRIORITY + 1,
      NULL);

  if (result != pdPASS)
  {
    Error_Handler();
  }
}
