#ifndef CliffDetector_Task
#define CliffDetector_Task

#include "FreeRTOS.h"
#include "queue.h"
#include "drv_TCRT5000.h"

typedef struct {
    tcrt_reading_t left;
    tcrt_reading_t right;
    uint32_t       timestamp;
} TCRT_Data_t;

extern QueueHandle_t xQueue_SensorData;

void Task_Sensor_TCRT_Init(void);

#endif
