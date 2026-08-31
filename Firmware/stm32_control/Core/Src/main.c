/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_example.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_motor.h"
#include "drv_ultrasound.h"
#include "mid_kinematics.h"
#include "ds1302.h"
#include "task_ds1302.h"
#include "mid_log.h"

#include "drv_uart.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAFE_DISTANCE 200	// Safety distance in mm
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


// Objects of our layers
MotorHandle_t motor_l, motor_r;
RobotCommand_t robot_cmd;
DrvUart_t esp_uart;
char uart_message[UART_RX_BUFFER_SIZE];
Ds1302_t ds1302;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void TaskExample_Create(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int main ()
{
	/* Reset peripherals, initialize Flash and Systick */
	  HAL_Init();
	  SystemClock_Config();

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();
	  MX_USART2_UART_Init();
	  MX_TIM2_Init();
	  MX_TIM4_Init();
	  MX_SPI2_Init();          /* tarjeta SD */
	  MX_FATFS_Init();         /* enlaza el driver USER (drv_sd_spi) a FatFs */
	  MX_USB_DEVICE_Init();

	  DrvUart_Init(&esp_uart, &huart2);
	  DrvUart_StartReceive(&esp_uart);
	  DrvUart_SendString(&esp_uart, "STM32 UART ready\n");

	  motor_l.htim = &htim4;
	  motor_l.channel = TIM_CHANNEL_2;
	  motor_l.port_a = GPIOD; motor_l.pin_a = GPIO_PIN_0;
	  motor_l.port_b = GPIOD; motor_l.pin_b = GPIO_PIN_1;
	  Motor_Init(&motor_l);

	  /* 2. Right Motor Driver Configuration */
	  motor_r.htim = &htim4;
	  motor_r.channel = TIM_CHANNEL_3;
	  motor_r.port_a = GPIOD; motor_r.pin_a = GPIO_PIN_2;
	  motor_r.port_b = GPIOD; motor_r.pin_b = GPIO_PIN_3;
	  Motor_Init(&motor_r);

	  /* 3. Ultrasound Driver Configuration (HC-SR04)
	   *    TRIG=PB4, ECHO=PA6 (TIM3_CH1 + DMA1_Stream4). El driver gestiona
	   *    TIM3 y el DMA internamente; la API nueva no recibe handle. */
	  HCSR04_Init();

	  /* 4. RTC DS1302 (bit-bang: CLK=PD8, DAT=PD9, RST=PD10; GPIO en MX_GPIO_Init) */
	  const Ds1302Config_t ds1302_cfg = {
	      .reset_pin = { .mcu_port = GPIOD, .pin = GPIO_PIN_10 }
	  };
	  DS1302_Init(&ds1302, &ds1302_cfg);
	  /* Bring-up: se fija una hora conocida en cada arranque para validar el driver.
	   * TODO(Fase 3): setear solo si el reloj no corre (bit CH) y tomar la hora real. */
	  DS1302_SetTime(&ds1302,
	                 0U,      /* formato 24 h            */
	                 12U,     /* horas                   */
	                 0U,      /* minutos                 */
	                 0U,      /* segundos                */
	                 0U,      /* am_pm (ignorado en 24h) */
	                 1U,      /* dia de semana (1 = Dom)  */
	                 1U,      /* dia del mes             */
	                 1U,      /* mes (1 = Ene)           */
	                 2026);   /* anio                    */

	  /* 5. Logger por USB CDC con timestamp del DS1302. */
	  Log_Init(&ds1302);

    TaskExample_Create();
    TaskDs1302_Create();

    vTaskStartScheduler();
    
    /* If all is well, the scheduler will now be running, and the following
     line will never be reached. If it does, there was insufficient FreeRTOS
     heap memory available for the idle task. */
    Error_Handler();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief Callback ejecutado cuando se completa la recepción UART por interrupción.
 * @param huart Puntero al handle UART que generó la interrupción.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        DrvUart_RxCallback(&esp_uart);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
