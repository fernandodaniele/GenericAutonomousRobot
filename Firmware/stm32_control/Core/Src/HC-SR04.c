/* Includes */
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include "HC-SR04.h"

/* Variables */
HCSR04_HandleTypeDef hc_sr04;
char msg[64];

/* Prototipos */
void SystemClock_Config(void);

/* Callback de captura (TIM2) */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        HCSR04_IC_Callback(&hc_sr04);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();      // 1 MHz, IC en el canal del ECHO
    MX_USB_DEVICE_Init(); // Inicializa USB CDC

    /* HC-SR04: TRIG PB4, ECHO PB5 */
    hc_sr04.htim      = &htim2;
    hc_sr04.channel   = TIM_CHANNEL_2;
    hc_sr04.trig_port = GPIOB;
    hc_sr04.trig_pin  = GPIO_PIN_4;
    hc_sr04.echo_port = GPIOB;
    hc_sr04.echo_pin  = GPIO_PIN_5;

    HCSR04_Init(&hc_sr04);

    HAL_Delay(500); // Espera a que la PC enumere el USB

    while (1)
    {
        HCSR04_Trigger(&hc_sr04);
        HAL_Delay(60);

        uint32_t d = HCSR04_Read(&hc_sr04);

        int n = sprintf(msg, "Distancia: %lu cm\r\n", d);

        /* Enviar por USB CDC */
        while (CDC_Transmit_FS((uint8_t*)msg, n) == USBD_BUSY) {
            HAL_Delay(1);
        }

        HAL_Delay(200);
    }
}

/* Clock */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                 RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}
