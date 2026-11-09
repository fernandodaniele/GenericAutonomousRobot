#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
TickType_t xTaskGetTickCountFromISR(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void vTaskDelayUntil(TickType_t *, TickType_t);
void vTaskSuspendAll(void);
BaseType_t xTaskResumeAll(void);
BaseType_t xTaskCreate(void (*)(void *), const char *, unsigned, void *, UBaseType_t, TaskHandle_t *);
#endif
