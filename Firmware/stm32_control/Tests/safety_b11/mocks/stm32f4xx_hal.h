#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t compare[4]; } TIM_HandleTypeDef;
typedef struct { uint16_t pins; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
void test_compare(TIM_HandleTypeDef *, uint32_t, uint32_t);
#define __HAL_TIM_SET_COMPARE(timer, channel, value) test_compare(timer, channel, value)
#endif
