#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t ErrorCode; } I2C_HandleTypeDef;
#define I2C_MEMADD_SIZE_8BIT 1U
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *);
uint32_t HAL_GetTick(void);
typedef struct { unsigned unused; } UART_HandleTypeDef;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
typedef struct {
    uint32_t compare[4];
    struct { uint32_t Period; } Init;
    uint32_t counter;
} TIM_HandleTypeDef;
typedef struct { uint16_t pins; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
#define TIM_CHANNEL_ALL 0xFFFFU
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *, uint32_t);
#define __HAL_TIM_GET_COUNTER(timer) ((timer)->counter)
void test_compare(TIM_HandleTypeDef *, uint32_t, uint32_t);
#define __HAL_TIM_SET_COMPARE(timer, channel, value) test_compare(timer, channel, value)
#endif
