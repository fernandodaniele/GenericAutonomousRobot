
#include "FreeRTOS.h"
#include "task.h"
#include "TCRT_Task.h"
#include "drv_TCRT5000.h"

// Cola definida acá, declarada extern en el .h
QueueHandle_t xQueue_SensorData;

// Función interna — no se expone en el .h
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

    xTaskCreate(vTask_Sensor,
                "Sensor",
                256,
                NULL,
                2,
                NULL);
}
