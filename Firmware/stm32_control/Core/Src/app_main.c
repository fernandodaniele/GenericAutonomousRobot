/**
 * @file    app_main.c
 * @brief   Punto de entrada de la aplicación (int main).
 *
 * STM32CubeMX (projectgenerator 4.7.0-B52) NO genera la función main() para
 * este proyecto: su Core/Src/main.c sale SIN main() en cada "Generate Code"
 * (probado 3 veces, incluso borrando main.c antes de regenerar).
 *
 * Por eso main() vive acá, en un archivo que CubeMX no conoce ni toca (se
 * compila vía GNUmakefile -> APP_C_SOURCES). El Core/Src/main.c de CubeMX
 * queda solo con SystemClock_Config / Error_Handler / assert_failed, que sí
 * regenera bien. Un "Generate Code" puede reescribir main.c sin romper nada.
 *
 * Ver fix-plan.md §5.
 */

#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

#include "fatfs.h"
#include "drv_motor.h"
#include "drv_ultrasound.h"
#include "mid_kinematics.h"
#include "ds1302.h"
#include "mid_log.h"
#include "drv_uart.h"
#include <string.h>

/* Generadas por CubeMX en main.c / freertos.c */
extern void SystemClock_Config(void);
extern void MX_FREERTOS_Init(void);

/* Objetos de las capas de la aplicación (las tareas se crean en freertos.c). */
MotorHandle_t motor_l, motor_r;
RobotCommand_t robot_cmd;
DrvUart_t esp_uart;
char uart_message[UART_RX_BUFFER_SIZE];
Ds1302_t ds1302;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* Periféricos configurados por CubeMX */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* MX_USB_DEVICE_Init() lo llama StartDefaultTask (freertos.c), como genera CubeMX. */

  /* FatFs: enlaza el driver USER (drv_sd_spi). No está en el .ioc, ver FATFS/README.md. */
  MX_FATFS_Init();

  /* 1. UART hacia el ESP32 */
  DrvUart_Init(&esp_uart, &huart2);
  DrvUart_StartReceive(&esp_uart);
  DrvUart_SendString(&esp_uart, "STM32 UART ready\n");

  /* 2. Motores (TB6612FNG, PWM por TIM4) */
  motor_l.htim = &htim4;
  motor_l.channel = TIM_CHANNEL_2;
  motor_l.port_a = GPIOD; motor_l.pin_a = GPIO_PIN_0;
  motor_l.port_b = GPIOD; motor_l.pin_b = GPIO_PIN_1;
  Motor_Init(&motor_l);

  motor_r.htim = &htim4;
  motor_r.channel = TIM_CHANNEL_3;
  motor_r.port_a = GPIOD; motor_r.pin_a = GPIO_PIN_2;
  motor_r.port_b = GPIOD; motor_r.pin_b = GPIO_PIN_3;
  Motor_Init(&motor_r);

  /* 3. Ultrasonido HC-SR04: TRIG=PB4, ECHO=PA6 (TIM3_CH1 + DMA1_Stream4).
   *    El driver gestiona TIM3 y el DMA internamente. */
  HCSR04_Init();

  /* 4. RTC DS1302 (bit-bang: CLK=PD8, DAT=PD9, RST=PD10; GPIO en MX_GPIO_Init) */
  const Ds1302Config_t ds1302_cfg = {
      .reset_pin = { .mcu_port = DS1302_RST_GPIO_Port, .pin = DS1302_RST_Pin }
  };
  DS1302_Init(&ds1302, &ds1302_cfg);
  /* Bring-up: hora fija en cada arranque para validar el driver.
   * TODO: setear solo si el reloj no corre (bit CH) y tomar la hora real. */
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

  /* 5. Logger por USB CDC + SD, con timestamp del DS1302. */
  Log_Init(&ds1302);

  /* Arranque de FreeRTOS (flujo nativo de CubeMX). */
  osKernelInitialize();
  MX_FREERTOS_Init();
  osKernelStart();

  /* No se llega acá: el scheduler tomó el control. */
  while (1)
  {
  }
}

/**
 * @brief Callback ejecutado cuando se completa la recepción UART por interrupción.
 * @param huart Puntero al handle UART que generó la interrupción.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    DrvUart_RxCallback(&esp_uart);
  }
}
