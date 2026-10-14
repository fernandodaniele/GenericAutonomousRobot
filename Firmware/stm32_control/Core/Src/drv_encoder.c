#include "drv_encoder.h"

void Encoder_Init(EncoderHandle_t *encoder, TIM_HandleTypeDef *htim) {
    encoder->htim = htim;
    encoder->last_count = __HAL_TIM_GET_COUNTER(htim);
}

HAL_StatusTypeDef Encoder_Start(EncoderHandle_t *encoder) {
    HAL_StatusTypeDef status;

    status = HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);

    if (status == HAL_OK) {
        encoder->last_count = __HAL_TIM_GET_COUNTER(encoder->htim);
    }

    return status;
}

HAL_StatusTypeDef Encoder_Stop(EncoderHandle_t *encoder) {
    return HAL_TIM_Encoder_Stop(encoder->htim, TIM_CHANNEL_ALL);
}

int32_t Encoder_ReadDelta(EncoderHandle_t *encoder) {
    uint32_t current_count;
    uint32_t count_delta;

    current_count = __HAL_TIM_GET_COUNTER(encoder->htim);
    count_delta = current_count - encoder->last_count;
    encoder->last_count = current_count;

    // TIM1 utiliza contador de 16 bits; TIM2 utiliza 32 bits.
    if (encoder->htim->Init.Period == UINT16_MAX) {
        return (int16_t)(uint16_t)count_delta;
    }

    return (int32_t)count_delta;
}
