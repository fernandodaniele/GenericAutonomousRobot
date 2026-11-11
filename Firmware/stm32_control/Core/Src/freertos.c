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
static void send_map_line(DrvUart_t *uart, uint32_t ms);
static void send_tel_line(DrvUart_t *uart, uint32_t ms);

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

    /* Datos para armar el mapa en la PC, cada 200 ms mientras se mueve */
    static uint32_t last_map_ms = 0;
    uint32_t now_ms = (uint32_t)osKernelGetTickCount();
    if (now_ms - last_map_ms >= 200u) {
        last_map_ms = now_ms;
        send_map_line(&esp_uart, now_ms);
        send_tel_line(&esp_uart, now_ms);
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

static void send_int(DrvUart_t *uart, int32_t v)
{
  char num[11];
  DrvUart_SendString(uart, ",");
  if (v < 0) {
    DrvUart_SendString(uart, "-");
    v = -v;
  }
  DrvUart_SendString(uart, u32_to_str((uint32_t)v, &num[sizeof(num) - 1]));
}

/* 9999 = nada al alcance; 0 = sin lectura */
static int32_t map_range(const SensorSnapshot_t *s, UltrasoundIndex_t i)
{
  if (s->us_status[i] == ULTRASOUND_STATUS_FAR) return 9999;
  if (s->us_status[i] == ULTRASOUND_STATUS_NO_ECHO) return 0;
  return (int32_t)s->dist_mm[i];
}

/* MAP,<ms>,<x_mm>,<y_mm>,<theta_mrad>,<frente_mm>,<izq_mm>,<der_mm> */
static void send_map_line(DrvUart_t *uart, uint32_t ms)
{
  NavStatus_t st = TaskNav_GetStatus();
  SensorSnapshot_t snap;
  if (st.state == NAV_STATE_IDLE || st.state == NAV_STATE_FAULT || !TaskSensors_GetSnapshot(&snap)) {
    return;
  }
  DrvUart_SendString(uart, "MAP");
  send_int(uart, (int32_t)ms);
  send_int(uart, st.x_mm);
  send_int(uart, st.y_mm);
  send_int(uart, st.theta_mrad);
  send_int(uart, map_range(&snap, ULTRASOUND_FRONT));
  send_int(uart, map_range(&snap, ULTRASOUND_LEFT));
  send_int(uart, map_range(&snap, ULTRASOUND_RIGHT));
  DrvUart_SendString(uart, "\n");
}

static void send_range(DrvUart_t *uart, const SensorSnapshot_t *s, UltrasoundIndex_t i)
{
  const char *status;
  switch (s->us_last_status[i]) {
    case ULTRASOUND_STATUS_OK:        status = ",O"; break;
    case ULTRASOUND_STATUS_FAR:       status = ",F"; break;
    case ULTRASOUND_STATUS_TOO_CLOSE: status = ",C"; break;
    default:                          status = ",N"; break;
  }
  DrvUart_SendString(uart, status);
  send_int(uart, (int32_t)s->dist_mm[i]);
}

/* Telemetria para la pagina web del ESP32, siempre (tambien en IDLE y FAULT):
 * TEL,<ms>,<estado>,<pwm_izq>,<pwm_der>,<freno>,
 *     <est_frente>,<mm_frente>,<est_izq>,<mm_izq>,<est_der>,<mm_der>,
 *     <tcrt_izq>,<tcrt_der>,<borde_izq>,<borde_der>,<giro_z>,<imu_ok>,<rumbo_decimas_grado>,
 *     <diag_imu> (ver IMU_DIAG_* en task_sensors.h)
 * Estados de ultrasonido: O = valido, F = libre, C = pegado, N = sin eco */
static void send_tel_line(DrvUart_t *uart, uint32_t ms)
{
  NavStatus_t st = TaskNav_GetStatus();
  SensorSnapshot_t snap;
  if (!TaskSensors_GetSnapshot(&snap)) {
    return;
  }
  DrvUart_SendString(uart, "TEL");
  send_int(uart, (int32_t)ms);
  DrvUart_SendString(uart, ",");
  DrvUart_SendString(uart, Nav_StateName(st.state));
  send_int(uart, st.pwm_left);
  send_int(uart, st.pwm_right);
  send_int(uart, st.brake ? 1 : 0);
  send_range(uart, &snap, ULTRASOUND_FRONT);
  send_range(uart, &snap, ULTRASOUND_LEFT);
  send_range(uart, &snap, ULTRASOUND_RIGHT);
  send_int(uart, snap.tcrt_left);
  send_int(uart, snap.tcrt_right);
  send_int(uart, snap.cliff_left ? 1 : 0);
  send_int(uart, snap.cliff_right ? 1 : 0);
  send_int(uart, snap.gyro_z_dps);
  send_int(uart, snap.imu_ok ? 1 : 0);
  send_int(uart, snap.heading_mdeg / 100);
  send_int(uart, snap.imu_diag);
  DrvUart_SendString(uart, "\n");
}

static void send_nav_event(DrvUart_t *uart, const NavEventMsg_t *ev)
{
  if (ev->events & NAV_EVT_ALERT) send_one_event(uart, ev->timestamp_ms, "ALERTA");
  if (ev->events & NAV_EVT_CLIFF) send_one_event(uart, ev->timestamp_ms, "CLIFF");
  if (ev->events & NAV_EVT_FAULT) send_one_event(uart, ev->timestamp_ms, "FAULT");
}

/* USER CODE END Application */

