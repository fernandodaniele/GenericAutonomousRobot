#ifndef DRV_ENCODER_H_
#define DRV_ENCODER_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t last_count;
} EncoderHandle_t;

/**
 * @brief Inicializa el estado del driver del encoder.
 * @param encoder Puntero a la estructura del encoder.
 * @param htim Puntero al timer previamente configurado.
 */
void Encoder_Init(EncoderHandle_t *encoder, TIM_HandleTypeDef *htim);

/**
 * @brief Inicia el conteo del encoder en ambos canales.
 * @return Estado de la operación HAL.
 */
HAL_StatusTypeDef Encoder_Start(EncoderHandle_t *encoder);

/**
 * @brief Detiene el conteo del encoder.
 * @return Estado de la operación HAL.
 */
HAL_StatusTypeDef Encoder_Stop(EncoderHandle_t *encoder);

/**
 * @brief Obtiene la variación de cuentas desde la última lectura.
 * @return Variación con signo, considerando el desbordamiento del contador.
 */
int32_t Encoder_ReadDelta(EncoderHandle_t *encoder);

#endif /* DRV_ENCODER_H_ */
