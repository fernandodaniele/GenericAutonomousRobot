/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_uart.h"
#include "task_sensors.h"
#include "task_nav.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void send_nav_event(DrvUart_t *uart, const NavEventMsg_t *ev);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* Navegacion autonoma (plan Pruebas_Grupo2, etapas 1 y 2) */
  TaskSensors_Create();
  TaskNav_Create();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  extern DrvUart_t esp_uart;
  static char message[UART_RX_BUFFER_SIZE];

  /* Infinite loop */
  for(;;)
  {
    /* Prueba de enlace con el ESP32: responde OK a PING.
     * Ademas acepta START / STOP para la navegacion autonoma. */
    if (DrvUart_IsMessageReady(&esp_uart)) {
        DrvUart_GetMessage(&esp_uart, message, sizeof(message));
        if (strcmp(message, "PING") == 0) {
            DrvUart_SendString(&esp_uart, "OK\n");
        } else if (strcmp(message, "START") == 0) {
            (void) TaskNav_SendCommand(NAV_CMD_START);
        } else if (strcmp(message, "STOP") == 0) {
            (void) TaskNav_SendCommand(NAV_CMD_STOP);
        }
    }

    /* Eventos de navegacion hacia el ESP32: EVT,<ms>,<texto> */
    NavEventMsg_t ev;
    while (TaskNav_GetEvent(&ev)) {
        send_nav_event(&esp_uart, &ev);
    }
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* Convierte un entero sin signo a texto decimal (sin printf: nano.specs). */
static char *u32_to_str(uint32_t v, char *end)
{
  *end = '\0';
  do {
    *--end = (char)('0' + (v % 10u));
    v /= 10u;
  } while (v != 0u);
  return end;
}

static void send_one_event(DrvUart_t *uart, uint32_t ms, const char *text)
{
  char num[11];
  DrvUart_SendString(uart, "EVT,");
  DrvUart_SendString(uart, u32_to_str(ms, &num[sizeof(num) - 1]));
  DrvUart_SendString(uart, ",");
  DrvUart_SendString(uart, text);
  DrvUart_SendString(uart, "\n");
}

static void send_nav_event(DrvUart_t *uart, const NavEventMsg_t *ev)
{
  if (ev->events & NAV_EVT_ALERT) send_one_event(uart, ev->timestamp_ms, "ALERTA");
  if (ev->events & NAV_EVT_CLIFF) send_one_event(uart, ev->timestamp_ms, "CLIFF");
  if (ev->events & NAV_EVT_FAULT) send_one_event(uart, ev->timestamp_ms, "FAULT");
}

/* USER CODE END Application */

