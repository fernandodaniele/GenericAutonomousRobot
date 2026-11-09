#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef void *TaskHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define tskIDLE_PRIORITY 0U
#define configTICK_RATE_HZ 1000U
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
void test_enter(void);
void test_exit(void);
#define taskENTER_CRITICAL() test_enter()
#define taskEXIT_CRITICAL() test_exit()
#endif
