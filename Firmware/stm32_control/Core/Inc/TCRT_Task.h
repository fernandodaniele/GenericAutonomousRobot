#ifndef TASK_SENSOR_TCRT
#define TASK_SENSOR_TCRT
#include "drv_TCRT5000.h"

#include "FreeRTOS.h"
#include "queue.h"

// Estructura del dato que produce esta tarea
typedef struct {
    tcrt_reading_t left;
    tcrt_reading_t right;
    uint32_t       timestamp;
} TCRT_Data_t;

// Cola compartida — se declara extern para que otras tareas la vean
extern QueueHandle_t xQueue_SensorData;

// Única función pública: inicializa y crea la tarea
void Task_Sensor_TCRT_Init(void);

#endif
