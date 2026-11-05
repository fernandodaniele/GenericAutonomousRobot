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

void HCSR04_Trigger(void) {
    uint32_t dma_irq_enabled = NVIC_GetEnableIRQ(DMA1_Stream4_IRQn);

    /* Aisla el callback frontal sin bloquear SysTick ni la IRQ trasera. */
    HAL_NVIC_DisableIRQ(DMA1_Stream4_IRQn);

    /* Solo la CPU modifica este indicador; su IRQ ya esta inhibida. */
    capture_done = 0U;

    if (HAL_DMA_GetState(&hdma_tim3_ic) == HAL_DMA_STATE_BUSY) {
        if ((HAL_DMA_Abort(&hdma_tim3_ic) != HAL_OK) &&
            (HAL_DMA_GetState(&hdma_tim3_ic) != HAL_DMA_STATE_READY)) {
            goto stop_capture;
        }
    }

    /* No tocar el buffer si el DMA no esta listo y efectivamente detenido. */
    if ((HAL_DMA_GetState(&hdma_tim3_ic) != HAL_DMA_STATE_READY) ||
        ((hdma_tim3_ic.Instance->CR & DMA_SxCR_EN) != 0U)) {
        goto stop_capture;
    }

    __HAL_TIM_DISABLE_DMA(&htim3_echo, TIM_DMA_CC1);
    (void)HAL_TIM_IC_Stop(&htim3_echo, TIM_CHANNEL_1);

    captures[0] = 0U;
    captures[1] = 0U;
    __HAL_TIM_CLEAR_FLAG(&htim3_echo, TIM_FLAG_CC1 | TIM_FLAG_CC1OF);
    __HAL_DMA_CLEAR_FLAG(&hdma_tim3_ic,
                         DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4 |
                         DMA_FLAG_TEIF0_4 | DMA_FLAG_DMEIF0_4 |
                         DMA_FLAG_FEIF0_4);
    NVIC_ClearPendingIRQ(DMA1_Stream4_IRQn);

    if (HAL_TIM_IC_Start_DMA(&htim3_echo, TIM_CHANNEL_1,
                             (uint32_t *)captures, 2U) != HAL_OK) {
        goto stop_capture;
    }

    if (dma_irq_enabled != 0U) {
        HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    }

    /* Pulso de 10 µs en PC9 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    Delay_Us(10);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    return;

stop_capture:
    /* Inhibe solo CH1 y sus interrupciones DMA; no inicia otro aborto. */
    __HAL_TIM_DISABLE_DMA(&htim3_echo, TIM_DMA_CC1);
    (void)HAL_TIM_IC_Stop(&htim3_echo, TIM_CHANNEL_1);
    __HAL_DMA_DISABLE_IT(&hdma_tim3_ic, DMA_IT_TC | DMA_IT_HT |
                                      DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(&hdma_tim3_ic, DMA_IT_FE);

    if (dma_irq_enabled != 0U) {
        HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    }
}

/**
 * @brief Inicia una medicion del HC-SR04 trasero.
 *
 * ECHO: TIM3 CH3 + DMA1 Stream7.
 * TRIG: PC5.
 */
void HCSR04_Rear_Trigger(void)
{
    /* Detener una posible captura trasera anterior */
    HAL_TIM_IC_Stop_DMA(&htim3_echo, TIM_CHANNEL_3);

    /* Reiniciar estado y buffer del sensor trasero */
    rear_capture_done = 0U;
    rear_captures[0] = 0U;
    rear_captures[1] = 0U;

    /* Preparar DMA para capturar ambos flancos de ECHO */
    if (HAL_TIM_IC_Start_DMA(&htim3_echo,
                             TIM_CHANNEL_3,
                             (uint32_t *)rear_captures,
                             2U) != HAL_OK)
    {
        return;
    }

    /* Generar pulso TRIG de 10 us en PC5 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    Delay_Us(10);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}

uint8_t HCSR04_IsReady(void) { return capture_done; }

/**
 * @brief Indica si finalizo la captura DMA del HC-SR04 trasero.
 * @return 1 si la captura termino, 0 en caso contrario.
 */
uint8_t HCSR04_Rear_IsReady(void)
{
    return rear_capture_done;
}

float HCSR04_GetDistance(void) {
    capture_done = 0;

    /* Maneja el desbordamiento del contador de 16 bits */
    uint16_t diff = (captures[1] >= captures[0])
                  ? (captures[1] - captures[0])
                  : (uint16_t)(0xFFFFU - captures[0] + captures[1] + 1U);

    /* diff en µs (tick de 1 MHz); distancia = diff_us × 340 m/s / 2 */
    return (float)diff * 1e-6f * 340.0f / 2.0f;
}

/**
 * @brief Obtiene la distancia medida por el HC-SR04 trasero.
 * @return Distancia en metros.
 *
 * Debe llamarse unicamente cuando
 * HCSR04_Rear_IsReady() devuelve 1.
 */
float HCSR04_Rear_GetDistance(void)
{
    rear_capture_done = 0U;

    /* Capturas de subida y bajada del ECHO trasero */
    uint16_t rise = rear_captures[0];
    uint16_t fall = rear_captures[1];

    /* Manejar desbordamiento del contador TIM3 de 16 bits */
    uint16_t diff = (fall >= rise)
                  ? (fall - rise)
                  : (uint16_t)(0xFFFFU - rise + fall + 1U);

    /* TIM3: 1 tick = 1 us
     * Distancia = tiempo * velocidad_sonido / 2
     */
    return (float)diff * 1e-6f * 340.0f / 2.0f;
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
