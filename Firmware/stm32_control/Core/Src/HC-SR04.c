/* Includes */
#include "HC-SR04.h"
#include "drv_ultrasound.h"
#include <stdio.h>

void HCSR04_Init(HCSR04_HandleTypeDef *dev) {
    if (!dev) return;
    if (dev->htim) {
        HAL_TIM_Base_Start(dev->htim);
    }
    if (dev->trig_port) {
        HAL_GPIO_WritePin(dev->trig_port, dev->trig_pin, GPIO_PIN_RESET);
    }
    dev->is_first_captured = 0;
    dev->distance_cm = 0;
}

void HCSR04_Trigger(HCSR04_HandleTypeDef *dev) {
    if (!dev || !dev->trig_port || !dev->htim) return;

    HAL_GPIO_WritePin(dev->trig_port, dev->trig_pin, GPIO_PIN_SET);
    uint32_t start = __HAL_TIM_GET_COUNTER(dev->htim);
    while ((__HAL_TIM_GET_COUNTER(dev->htim) - start) < 10) {
        __NOP();
    }
    HAL_GPIO_WritePin(dev->trig_port, dev->trig_pin, GPIO_PIN_RESET);
}

void HCSR04_IC_Callback(HCSR04_HandleTypeDef *dev) {
    if (!dev) return;
    if (dev->is_first_captured == 0) {
        dev->ic_val1 = HAL_TIM_ReadCapturedValue(dev->htim, dev->channel);
        dev->is_first_captured = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(dev->htim, dev->channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    } else if (dev->is_first_captured == 1) {
        dev->ic_val2 = HAL_TIM_ReadCapturedValue(dev->htim, dev->channel);
        __HAL_TIM_SET_COUNTER(dev->htim, 0);

        uint32_t diff = 0;
        if (dev->ic_val2 > dev->ic_val1) {
            diff = dev->ic_val2 - dev->ic_val1;
        } else {
            diff = (0xFFFF - dev->ic_val1) + dev->ic_val2;
        }

        dev->distance_cm = diff / 58;
        dev->is_first_captured = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(dev->htim, dev->channel, TIM_INPUTCHANNELPOLARITY_RISING);
        __HAL_TIM_DISABLE_IT(dev->htim, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
    }
}

uint32_t HCSR04_Read(HCSR04_HandleTypeDef *dev) {
    if (!dev) return 0;
    Ultrasound_t temp_sensor = {
        .trig_port = dev->trig_port,
        .trig_pin  = dev->trig_pin,
        .echo_port = dev->echo_port,
        .echo_pin  = dev->echo_pin,
        .timer     = dev->htim
    };
    uint32_t mm = Ultrasound_Read_mm(&temp_sensor);
    dev->distance_cm = mm / 10;
    return dev->distance_cm;
}

#ifdef STANDALONE_HCSR04_TEST
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

HCSR04_HandleTypeDef hc_sr04_test;
char test_msg[64];

int test_main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_USB_DEVICE_Init();

    hc_sr04_test.htim      = &htim2;
    hc_sr04_test.channel   = TIM_CHANNEL_2;
    hc_sr04_test.trig_port = TRIG_FRONT_GPIO_Port;
    hc_sr04_test.trig_pin  = TRIG_FRONT_Pin;
    hc_sr04_test.echo_port = ECHO_FRONT_GPIO_Port;
    hc_sr04_test.echo_pin  = ECHO_FRONT_Pin;
    HCSR04_Init(&hc_sr04_test);

    HAL_Delay(500);

    while (1) {
        uint32_t d = HCSR04_Read(&hc_sr04_test);
        int n = sprintf(test_msg, "Distancia: %lu cm\r\n", d);
        while (CDC_Transmit_FS((uint8_t*)test_msg, n) == USBD_BUSY) {
            HAL_Delay(1);
        }
        HAL_Delay(200);
    }
}
#endif
