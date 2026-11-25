#ifndef DRV_ULTRASOUND_H_
#define DRV_ULTRASOUND_H_

#include "stm32f4xx_hal.h"

/*
 * Driver de los dos HC-SR04 en STM32F407VG.
 * Frontal: TRIG PC9, ECHO PC6 (TIM3 CH1, DMA1 Stream 4, canal 5).
 * Trasero: TRIG PC5, ECHO PC8 (TIM3 CH3, DMA1 Stream 7, canal 5).
 * TIM3 a 1 MHz (PSC=83). Cada canal DMA guarda dos tiempos de 16 bits.
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
 * @brief Arma DMA y emite TRIG de 10 us; exige ECHO en bajo.
 * @return HAL_OK si se emitio TRIG; otro estado si no se inicio la adquisicion.
 * @note API de adquisicion para una unica tarea, no ISR. Cancelacion acotada
 *       por el timeout HAL; requiere que SysTick permanezca habilitado.
 */
HAL_StatusTypeDef HCSR04_Trigger(void);

/* Cancela CH1 y descarta su resultado. HAL_OK confirma el stream detenido. */
HAL_StatusTypeDef HCSR04_Cancel(void);

/**
 * @brief Indica si la captura DMA completó (flanco descendente recibido).
 * @return 1 si hay nueva medición disponible, 0 si aún está pendiente.
 */
uint8_t HCSR04_IsReady(void);

/**
 * @brief Calcula la distancia a partir de los timestamps capturados.
 * @return Distancia en metros; 0 si la captura no es utilizable.
 *         Consume el resultado (marca como no listo).
 */
float HCSR04_GetDistance(void);

/*
 * HC-SR04 trasero
 * TRIG: PC5
 * ECHO: PC8 - TIM3 CH3 + DMA1 Stream7
 */

/* Mismo contrato que HCSR04_Trigger(), aplicado al sensor trasero. */
HAL_StatusTypeDef HCSR04_Rear_Trigger(void);

/* Cancela CH3 y descarta su resultado. HAL_OK confirma el stream detenido. */
HAL_StatusTypeDef HCSR04_Rear_Cancel(void);

/* Indica si finalizo la captura DMA trasera. */
uint8_t HCSR04_Rear_IsReady(void);

/* Obtiene la distancia trasera en metros. */
float HCSR04_Rear_GetDistance(void);

#endif /* DRV_ULTRASOUND_H_ */
