#ifndef W5_QUEUE_H
#define W5_QUEUE_H
#include "FreeRTOS.h"
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(UBaseType_t, UBaseType_t);
BaseType_t xQueueOverwrite(QueueHandle_t,const void *);
BaseType_t xQueuePeek(QueueHandle_t,void *,TickType_t);
#endif
