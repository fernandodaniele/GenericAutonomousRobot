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

static volatile uint16_t captures[2] = {0, 0};
static volatile uint8_t  capture_done = 0;

/* ── API pública ─────────────────────────────────────────────────────────────
 */

void HCSR04_Init(void) {
    /* Habilita el contador de ciclos DWT para retardos en microsegundos */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* Pin TRIG: PB4 como salida GPIO, reposo en LOW */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef trig = {0};
    trig.Pin   = GPIO_PIN_4;
    trig.Mode  = GPIO_MODE_OUTPUT_PP;
    trig.Pull  = GPIO_NOPULL;
    trig.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &trig);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    /* Pin ECHO: PA6 como TIM3 CH1, captura en ambos flancos */
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
}

void HCSR04_Trigger(void) {
    capture_done = 0;
    captures[0]  = 0;
    captures[1]  = 0;

    /* Cancela cualquier captura pendiente de un disparo anterior sin respuesta */
    HAL_TIM_IC_Stop_DMA(&htim3_echo, TIM_CHANNEL_1);

    /* Pulso de 10 µs en PB4 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    Delay_Us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    /* Arma el DMA para capturar los timestamps de subida y bajada */
    HAL_TIM_IC_Start_DMA(&htim3_echo, TIM_CHANNEL_1, (uint32_t *)captures, 2);
}

uint8_t HCSR04_IsReady(void) { return capture_done; }

float HCSR04_GetDistance(void) {
    capture_done = 0;

    /* Maneja el desbordamiento del contador de 16 bits */
    uint16_t diff = (captures[1] >= captures[0])
                  ? (captures[1] - captures[0])
                  : (uint16_t)(0xFFFFU - captures[0] + captures[1] + 1U);

    /* diff en µs (tick de 1 MHz); distancia = diff_us × 340 m/s / 2 */
    return (float)diff * 1e-6f * 340.0f / 2.0f;
}

/* ── Manejadores de interrupción ─────────────────────────────────────────────
 */

void DMA1_Stream4_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_tim3_ic); }
void TIM3_IRQHandler(void)         { HAL_TIM_IRQHandler(&htim3_echo); }

/* ── Callbacks HAL ───────────────────────────────────────────────────────────
 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        capture_done = 1;
}

/* ── MSP Init ────────────────────────────────────────────────────────────────
 */

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM3) return;

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA6 = TIM3 CH1 = ECHO */
    GPIO_InitTypeDef echo = {0};
    echo.Pin       = GPIO_PIN_6;
    echo.Mode      = GPIO_MODE_AF_PP;
    echo.Pull      = GPIO_PULLDOWN;
    echo.Speed     = GPIO_SPEED_FREQ_LOW;
    echo.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &echo);

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

    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}
