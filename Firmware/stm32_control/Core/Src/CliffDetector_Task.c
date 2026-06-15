#include "CliffDetector_Task.h"
#include "drv_TCRT5000.h"
#include "task.h"

QueueHandle_t xQueue_SensorData;

static void vTask_Sensor_TCRT(void *pvParameters)
{
    TCRT_Data_t data;

    for(;;)
    {
        drv_tcrt5000_update();

        data.left      = drv_tcrt5000_get_left();
        data.right     = drv_tcrt5000_get_right();
        data.timestamp = xTaskGetTickCount();

        xQueueSend(xQueue_SensorData, &data, 0);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void Task_Sensor_TCRT_Init(void)
{
    drv_tcrt5000_init();

    xQueue_SensorData = xQueueCreate(10, sizeof(TCRT_Data_t));

    xTaskCreate(vTask_Sensor_TCRT,
                "TCRT",
                256,
                NULL,
                2,
                NULL);
}
