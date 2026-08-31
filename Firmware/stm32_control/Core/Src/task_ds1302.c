#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include "task_ds1302.h"
#include "ds1302.h"

#include <stdio.h>

/* Periodo de sondeo del RTC. */
#define TASK_DS1302_PERIOD_MS   1000U

static void vTaskDs1302(void *pvParameters)
{
  Ds1302_t *ds1302 = (Ds1302_t *)pvParameters;
  char line[48];

  /* Desfasar respecto de task_example: ambas usan CDC_Transmit_FS, que no es
   * reentrante. Mitigación para el bring-up; la serialización real llega en
   * Fase 3 con mid_log. */
  vTaskDelay(pdMS_TO_TICKS(TASK_DS1302_PERIOD_MS / 2U));

  for (;;)
  {
    /* Latido visible: LED distinto al de task_example (LD4). */
    HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);

    if (DS1302_UpdateDateTime(ds1302) == DS1302_OK)
    {
      const char *weekday = DS1302_GetWeekDay(ds1302);
      const char *month   = DS1302_GetMonth(ds1302);

      int n = snprintf(line, sizeof(line),
                       "[%s %04u-%s-%02u %02u:%02u:%02u]\r\n",
                       (weekday != NULL) ? weekday : "?",
                       (unsigned)DS1302_GetYear(ds1302),
                       (month != NULL) ? month : "?",
                       (unsigned)DS1302_GetMonthDay(ds1302),
                       (unsigned)ds1302->data.date_time.hours,
                       (unsigned)ds1302->data.date_time.minutes,
                       (unsigned)ds1302->data.date_time.seconds);

      if (n > 0)
      {
        CDC_Transmit_FS((uint8_t *)line, (uint16_t)n);
      }
    }
    else
    {
      const char err[] = "[DS1302: error de lectura]\r\n";
      CDC_Transmit_FS((uint8_t *)err, sizeof(err) - 1U);
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_DS1302_PERIOD_MS));
  }
}

void TaskDs1302_Create(Ds1302_t *ds1302)
{
  /* 384 words: task_example usa 256, pero acá se suma snprintf (newlib-nano). */
  BaseType_t result = xTaskCreate(
      vTaskDs1302,
      "ds1302",
      384,
      ds1302,
      tskIDLE_PRIORITY + 1,
      NULL);

  if (result != pdPASS)
  {
    Error_Handler();
  }
}
