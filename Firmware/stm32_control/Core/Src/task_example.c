#include "task_sensors.h"
#include <stdio.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include "task_example.h"

static void format_range(char *out, size_t size, const SensorSnapshot_t *s, UltrasoundIndex_t i)
{
  switch (s->us_status[i]) {
    case ULTRASOUND_STATUS_OK:        snprintf(out, size, "%lu", (unsigned long)s->dist_mm[i]); break;
    case ULTRASOUND_STATUS_FAR:       snprintf(out, size, "LEJOS"); break;
    case ULTRASOUND_STATUS_TOO_CLOSE: snprintf(out, size, "<20"); break;
    default:                          snprintf(out, size, "SINECO"); break;
  }
}

static void vTaskExample(void *pvParameters)
{
  /* static: CDC_Transmit_FS sigue usando el buffer despues de retornar */
  static char line[80];
  SensorSnapshot_t snap;
  char f[8], l[8], r[8];
  uint8_t count = 0;

  (void) pvParameters;

  for (;;)
  {
    /* LD4 parpadea cada 1 s: el RTOS esta corriendo */
    if (++count >= 5u) {
      count = 0;
      HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    }

    /* Lecturas de ultrasonido por USB (si esta ocupado, se saltea esta linea) */
    if (TaskSensors_GetSnapshot(&snap)) {
      format_range(f, sizeof(f), &snap, ULTRASOUND_FRONT);
      format_range(l, sizeof(l), &snap, ULTRASOUND_LEFT);
      format_range(r, sizeof(r), &snap, ULTRASOUND_RIGHT);
      int n = snprintf(line, sizeof(line), "US mm: F=%s I=%s D=%s | TCRT I=%u%s D=%u%s\r\n", f, l, r,
                       snap.tcrt_left, snap.cliff_left ? " BORDE" : "",
                       snap.tcrt_right, snap.cliff_right ? " BORDE" : "");
      if (n > 0) {
        CDC_Transmit_FS((uint8_t *)line, (uint16_t)n);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
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
