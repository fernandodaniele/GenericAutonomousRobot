#ifndef DRV_ULTRASOUND_H_
#define DRV_ULTRASOUND_H_

#include "stm32f4xx_hal.h"

/*
 * Driver para sensor de distancia ultrasónico HC-SR04 — STM32F407VG @ 168 MHz.
 *
 * Hardware:
 *   TRIG: PB4  — salida GPIO push-pull
 *   ECHO: PA6  — TIM3 CH1 (captura de entrada, ambos flancos, DMA, AF2)
 *
 * TIM3 (APB1, 84 MHz) configurado a 1 MHz con PSC=83.
 * DMA1 Stream4 Channel5 captura dos timestamps de 16 bits (flanco ascendente y descendente).
 * El pulso de TRIG (10 µs) se genera con el contador DWT — sin timer adicional.
 *
 * NOTA: HAL_TIM_IC_MspInit y HAL_TIM_IC_CaptureCallback están definidos en
 * drv_ultrasound.c. Si otros drivers usan TIM IC, unificar esos callbacks.
 */

/**
 * @brief Inicializa el GPIO de TRIG y TIM3 con DMA para captura del ECHO.
 */
void HCSR04_Init(void);

/**
 * @brief Emite un pulso de 10 µs en TRIG y arma la captura DMA del ECHO.
 */
void HCSR04_Trigger(void);

/**
 * @brief Indica si la captura DMA completó (flanco descendente recibido).
 * @return 1 si hay nueva medición disponible, 0 si aún está pendiente.
 */
uint8_t HCSR04_IsReady(void);

/**
 * @brief Calcula la distancia a partir de los timestamps capturados.
 * @return Distancia en metros. Consume el resultado (marca como no listo).
 */
float HCSR04_GetDistance(void);

#endif /* DRV_ULTRASOUND_H_ */
