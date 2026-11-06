#include "drv_ultrasound.h"

/* ── Retardo en microsegundos (DWT) ──────────────────────────────────────────
 * Usa el contador de ciclos DWT del Cortex-M4 — no requiere timer adicional.
 * SystemCoreClock = 168 MHz → 10 µs = 1680 ciclos.
 */
static void Delay_Us(uint32_t us) {
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < cycles);
}

/* ── Estado privado ──────────────────────────────────────────────────────────
 */
static TIM_HandleTypeDef htim3_echo;
static DMA_HandleTypeDef hdma_tim3_ic;
static DMA_HandleTypeDef hdma_tim3_ic_rear;

static volatile uint16_t captures[2] = {0, 0};
static volatile uint8_t  capture_done = 0;

/* Capturas DMA independientes del HC-SR04 trasero */
static volatile uint16_t rear_captures[2] = {0, 0};
static volatile uint8_t rear_capture_done = 0;

/* ── API pública ─────────────────────────────────────────────────────────────
 */

void HCSR04_Init(void) {
    /* Habilita el contador de ciclos DWT para retardos en microsegundos */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* Pin TRIG: PC9 como salida GPIO, reposo en LOW */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef trig = {0};
    trig.Pin   = GPIO_PIN_9;
    trig.Mode  = GPIO_MODE_OUTPUT_PP;
    trig.Pull  = GPIO_NOPULL;
    trig.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &trig);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

    /* Pin ECHO: PC6 como TIM3 CH1, captura en ambos flancos */
    TIM_IC_InitTypeDef ic = {0};

    htim3_echo.Instance               = TIM3;
    htim3_echo.Init.Prescaler         = 83;   /* APB1 84 MHz → tick de 1 MHz */
    htim3_echo.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3_echo.Init.Period            = 0xFFFF;
    htim3_echo.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&htim3_echo);

    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 4;

    HAL_TIM_IC_ConfigChannel(&htim3_echo, &ic, TIM_CHANNEL_1);

    /* HC-SR04 trasero: configurar TIM3 Channel 3 */
    HAL_TIM_IC_ConfigChannel(&htim3_echo, &ic, TIM_CHANNEL_3);
}

/*
 * Cierra solo el canal solicitado. HAL_OK confirma EN=0, aunque un estado
 * excepcional del handle pueda impedir un armado posterior. No reinicializa DMA.
 * Todas las API de adquisicion deben utilizarse desde una unica tarea, no ISR.
 */
static HAL_StatusTypeDef HCSR04_StopCapture(uint32_t channel,
                                           DMA_HandleTypeDef *dma,
                                           IRQn_Type irq,
                                           volatile uint8_t *ready) {
    uint32_t irq_enabled = NVIC_GetEnableIRQ(irq);
    HAL_StatusTypeDef status = HAL_OK;

    HAL_NVIC_DisableIRQ(irq);
    *ready = 0U;

    if (HAL_DMA_GetState(dma) == HAL_DMA_STATE_BUSY) {
        status = HAL_DMA_Abort(dma);
        if ((status != HAL_OK) &&
            (HAL_DMA_GetState(dma) == HAL_DMA_STATE_READY)) {
            status = HAL_OK;
        }
    }

    __HAL_TIM_DISABLE_DMA(&htim3_echo,
                         (channel == TIM_CHANNEL_1) ? TIM_DMA_CC1 : TIM_DMA_CC3);
    (void)HAL_TIM_IC_Stop(&htim3_echo, channel);
    __HAL_DMA_DISABLE_IT(dma, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(dma, DMA_IT_FE);

    if ((dma->Instance->CR & DMA_SxCR_EN) == 0U) {
        __HAL_TIM_CLEAR_FLAG(&htim3_echo,
                            (channel == TIM_CHANNEL_1)
                            ? (TIM_FLAG_CC1 | TIM_FLAG_CC1OF)
                            : (TIM_FLAG_CC3 | TIM_FLAG_CC3OF));
        __HAL_DMA_CLEAR_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma) |
                                 __HAL_DMA_GET_HT_FLAG_INDEX(dma) |
                                 __HAL_DMA_GET_TE_FLAG_INDEX(dma) |
                                 __HAL_DMA_GET_DME_FLAG_INDEX(dma) |
                                 __HAL_DMA_GET_FE_FLAG_INDEX(dma));
        NVIC_ClearPendingIRQ(irq);
    } else if (status == HAL_OK) {
        status = HAL_ERROR;
    }

    if (irq_enabled != 0U) {
        HAL_NVIC_EnableIRQ(irq);
    }
    return status;
}

static HAL_StatusTypeDef HCSR04_StartCapture(uint32_t channel,
                                            DMA_HandleTypeDef *dma,
                                            IRQn_Type irq,
                                            volatile uint16_t *buffer,
                                            volatile uint8_t *ready,
                                            uint16_t trig_pin,
                                            uint16_t echo_pin) {
    HAL_StatusTypeDef status = HCSR04_StopCapture(channel, dma, irq, ready);
    if (status != HAL_OK) {
        return status;
    }

    uint32_t irq_enabled = NVIC_GetEnableIRQ(irq);
    HAL_NVIC_DisableIRQ(irq);

    /* Un ECHO aun alto no puede iniciar una pareja subida/bajada nueva. */
    if ((HAL_DMA_GetState(dma) != HAL_DMA_STATE_READY) ||
        ((dma->Instance->CR & DMA_SxCR_EN) != 0U) ||
        (HAL_GPIO_ReadPin(GPIOC, echo_pin) != GPIO_PIN_RESET)) {
        status = HAL_BUSY;
    } else {
        buffer[0] = 0U;
        buffer[1] = 0U;
        status = HAL_TIM_IC_Start_DMA(&htim3_echo, channel,
                                      (uint32_t *)buffer, 2U);
        if (status == HAL_OK) {
            HAL_GPIO_WritePin(GPIOC, trig_pin, GPIO_PIN_SET);
            Delay_Us(10);
            HAL_GPIO_WritePin(GPIOC, trig_pin, GPIO_PIN_RESET);
        } else {
            /* El arranque HAL puede dejar el canal TIM ocupado al fallar. */
            (void)HCSR04_StopCapture(channel, dma, irq, ready);
        }
    }

    if (irq_enabled != 0U) {
        HAL_NVIC_EnableIRQ(irq);
    }
    return status;
}

/* Leer una pareja completa con su IRQ aislada; 0 m indica resultado inutilizable. */
static float HCSR04_ReadCapture(DMA_HandleTypeDef *dma, IRQn_Type irq,
                                volatile uint16_t *buffer,
                                volatile uint8_t *ready) {
    uint32_t irq_enabled = NVIC_GetEnableIRQ(irq);
    float distance = 0.0f;

    HAL_NVIC_DisableIRQ(irq);
    uint32_t error_flags = __HAL_DMA_GET_TE_FLAG_INDEX(dma) |
                           __HAL_DMA_GET_DME_FLAG_INDEX(dma) |
                           __HAL_DMA_GET_FE_FLAG_INDEX(dma);
    if ((*ready != 0U) && (HAL_DMA_GetState(dma) == HAL_DMA_STATE_READY) &&
        ((dma->Instance->CR & DMA_SxCR_EN) == 0U) &&
        (HAL_DMA_GetError(dma) == HAL_DMA_ERROR_NONE) &&
        (__HAL_DMA_GET_FLAG(dma, error_flags) == 0U)) {
        uint16_t rise = buffer[0];
        uint16_t fall = buffer[1];
        uint16_t width = (uint16_t)(fall - rise);
        distance = (float)width * 1e-6f * 340.0f / 2.0f;
    }
    *ready = 0U;

    if (irq_enabled != 0U) {
        HAL_NVIC_EnableIRQ(irq);
    }
    return distance;
}

HAL_StatusTypeDef HCSR04_Trigger(void) {
    return HCSR04_StartCapture(TIM_CHANNEL_1, &hdma_tim3_ic,
                               DMA1_Stream4_IRQn, captures, &capture_done,
                               GPIO_PIN_9, GPIO_PIN_6);
}

HAL_StatusTypeDef HCSR04_Rear_Trigger(void) {
    return HCSR04_StartCapture(TIM_CHANNEL_3, &hdma_tim3_ic_rear,
                               DMA1_Stream7_IRQn, rear_captures, &rear_capture_done,
                               GPIO_PIN_5, GPIO_PIN_8);
}

HAL_StatusTypeDef HCSR04_Cancel(void) {
    return HCSR04_StopCapture(TIM_CHANNEL_1, &hdma_tim3_ic,
                              DMA1_Stream4_IRQn, &capture_done);
}

HAL_StatusTypeDef HCSR04_Rear_Cancel(void) {
    return HCSR04_StopCapture(TIM_CHANNEL_3, &hdma_tim3_ic_rear,
                              DMA1_Stream7_IRQn, &rear_capture_done);
}

uint8_t HCSR04_IsReady(void) {
    return (capture_done != 0U) &&
           (HAL_DMA_GetError(&hdma_tim3_ic) == HAL_DMA_ERROR_NONE);
}

uint8_t HCSR04_Rear_IsReady(void) {
    return (rear_capture_done != 0U) &&
           (HAL_DMA_GetError(&hdma_tim3_ic_rear) == HAL_DMA_ERROR_NONE);
}

float HCSR04_GetDistance(void) {
    return HCSR04_ReadCapture(&hdma_tim3_ic, DMA1_Stream4_IRQn,
                              captures, &capture_done);
}

float HCSR04_Rear_GetDistance(void) {
    return HCSR04_ReadCapture(&hdma_tim3_ic_rear, DMA1_Stream7_IRQn,
                              rear_captures, &rear_capture_done);
}

/* ── Manejadores de interrupción ─────────────────────────────────────────────
 */

void DMA1_Stream4_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_tim3_ic); }
void DMA1_Stream7_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_tim3_ic_rear); }
void TIM3_IRQHandler(void)         { HAL_TIM_IRQHandler(&htim3_echo); }

/* ── Callbacks HAL ───────────────────────────────────────────────────────────
 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3)
    {
        return;
    }

    /* HC-SR04 frontal: TIM3 CH1 */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        capture_done = 1;
    }

    /* HC-SR04 trasero: TIM3 CH3 */
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        rear_capture_done = 1;
    }
}

/* ── MSP Init ────────────────────────────────────────────────────────────────
 */

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM3) return;

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef echo = {0};
    echo.Pin       = GPIO_PIN_6;
    echo.Mode      = GPIO_MODE_AF_PP;
    echo.Pull      = GPIO_NOPULL;
    echo.Speed     = GPIO_SPEED_FREQ_LOW;
    echo.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &echo);

    /* HC-SR04 trasero: PC8 como TIM3 CH3 (AF2) */
    echo.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOC, &echo);

    /* DMA1 Stream4 Channel5 ← TIM3 CH1 */
    hdma_tim3_ic.Instance                 = DMA1_Stream4;
    hdma_tim3_ic.Init.Channel             = DMA_CHANNEL_5;
    hdma_tim3_ic.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_tim3_ic.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim3_ic.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim3_ic.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim3_ic.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_tim3_ic.Init.Mode                = DMA_NORMAL;
    hdma_tim3_ic.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_tim3_ic.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_tim3_ic);

    __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC1], hdma_tim3_ic);

    /* DMA1 Stream7 Channel5: TIM3 CH3 - HC-SR04 trasero */
    hdma_tim3_ic_rear.Instance                 = DMA1_Stream7;
    hdma_tim3_ic_rear.Init.Channel             = DMA_CHANNEL_5;
    hdma_tim3_ic_rear.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_tim3_ic_rear.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim3_ic_rear.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim3_ic_rear.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim3_ic_rear.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_tim3_ic_rear.Init.Mode                = DMA_NORMAL;
    hdma_tim3_ic_rear.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_tim3_ic_rear.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdma_tim3_ic_rear);

    __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC3], hdma_tim3_ic_rear);

    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    /* Interrupcion DMA del HC-SR04 trasero */
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}
