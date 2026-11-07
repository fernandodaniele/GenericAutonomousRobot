/* Tests of the real driver with deterministic HAL/clock substitutes, not hardware. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "../../Core/Src/drv_ultrasound.c"

uint32_t SystemCoreClock = 168000000U;
TIM_TypeDef test_tim3;
DMA_Stream_TypeDef test_stream4, test_stream7;
GPIO_TypeDef test_gpioc;
CoreDebug_Type test_coredebug;
static DWT_Type dwt;
static uint32_t irq_enabled[8], trig_front, trig_rear, aborts, tests;
static HAL_StatusTypeDef start_result, abort_result;
static int abort_keeps_enabled;
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL line %u: %s\n", (unsigned)__LINE__, #x); exit(1); } } while (0)
#define PASS(label) do { printf("PASS %02u %s\n", ++tests, label); } while (0)
#define NEAR(a, b) CHECK(fabsf((a) - (b)) < 1e-6f)

DWT_Type *test_dwt(void) { dwt.CYCCNT += 168U; return &dwt; }
uint32_t NVIC_GetEnableIRQ(IRQn_Type irq) { return irq_enabled[irq]; }
void HAL_NVIC_DisableIRQ(IRQn_Type irq) { irq_enabled[irq] = 0U; }
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { irq_enabled[irq] = 1U; }
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t p, uint32_t s) { (void)irq; (void)p; (void)s; }
void NVIC_ClearPendingIRQ(IRQn_Type irq) { (void)irq; }
void HAL_GPIO_Init(GPIO_TypeDef *gpio, GPIO_InitTypeDef *init) { (void)gpio; (void)init; }
void HAL_GPIO_WritePin(GPIO_TypeDef *gpio, uint16_t pin, GPIO_PinState state) {
    if (state == GPIO_PIN_SET) {
        gpio->ODR |= pin;
        if (pin == GPIO_PIN_9) ++trig_front;
        if (pin == GPIO_PIN_5) ++trig_rear;
    } else gpio->ODR &= ~(uint32_t)pin;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *gpio, uint16_t pin) {
    return (gpio->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *timer) {
    HAL_TIM_IC_MspInit(timer); timer->Instance->PSC = timer->Init.Prescaler;
    timer->Instance->ARR = timer->Init.Period; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *timer, TIM_IC_InitTypeDef *ic, uint32_t ch) {
    CHECK(ic->ICPolarity == TIM_INPUTCHANNELPOLARITY_BOTHEDGE);
    CHECK(ic->ICPrescaler == TIM_ICPSC_DIV1 && ic->ICSelection == TIM_ICSELECTION_DIRECTTI);
    if (ch == TIM_CHANNEL_1) timer->Instance->CCMR1 = (ic->ICFilter << 4U) | 1U;
    else timer->Instance->CCMR2 = (ic->ICFilter << 4U) | 1U;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *timer, uint32_t ch, uint32_t *buffer, uint16_t len) {
    CHECK(len == 2U);
    CHECK((void *)buffer == (ch == TIM_CHANNEL_1 ? (void *)captures : (void *)rear_captures));
    if (start_result != HAL_OK) return start_result;
    DMA_HandleTypeDef *dma = timer->hdma[ch == TIM_CHANNEL_1 ? TIM_DMA_ID_CC1 : TIM_DMA_ID_CC3];
    CHECK(dma->State == HAL_DMA_STATE_READY);
    dma->State = HAL_DMA_STATE_BUSY; dma->Instance->CR |= DMA_SxCR_EN;
    dma->Instance->NDTR = len;
    timer->Instance->DIER |= ch == TIM_CHANNEL_1 ? TIM_DMA_CC1 : TIM_DMA_CC3;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *timer, uint32_t ch) {
    (void)timer; (void)ch; return HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *dma) {
    dma->State = HAL_DMA_STATE_READY; dma->ErrorCode = HAL_DMA_ERROR_NONE; return HAL_OK;
}
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *dma) { return dma->State; }
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *dma) { return dma->ErrorCode; }
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *dma) {
    ++aborts;
    if (!abort_keeps_enabled) dma->Instance->CR &= ~DMA_SxCR_EN;
    dma->State = abort_result == HAL_OK ? HAL_DMA_STATE_READY : HAL_DMA_STATE_TIMEOUT;
    return abort_result;
}
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *dma) { (void)dma; }
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *timer) { (void)timer; }

static void setup(void) {
    memset(&test_tim3, 0, sizeof(test_tim3));
    memset(&test_stream4, 0, sizeof(test_stream4)); memset(&test_stream7, 0, sizeof(test_stream7));
    memset(&test_gpioc, 0, sizeof(test_gpioc)); memset(&htim3_echo, 0, sizeof(htim3_echo));
    memset(&hdma_tim3_ic, 0, sizeof(hdma_tim3_ic)); memset(&hdma_tim3_ic_rear, 0, sizeof(hdma_tim3_ic_rear));
    captures[0] = captures[1] = rear_captures[0] = rear_captures[1] = 0U;
    capture_done = rear_capture_done = 0U;
    memset(irq_enabled, 0, sizeof(irq_enabled));
    trig_front = trig_rear = aborts = 0U;
    start_result = abort_result = HAL_OK; abort_keeps_enabled = 0;
    HCSR04_Init();
}
static void complete(uint32_t sensor, uint16_t first, uint16_t second) {
    DMA_HandleTypeDef *dma = sensor ? &hdma_tim3_ic_rear : &hdma_tim3_ic;
    volatile uint16_t *buffer = sensor ? rear_captures : captures;
    buffer[0] = first; buffer[1] = second;
    dma->Instance->CR &= ~DMA_SxCR_EN; dma->Instance->NDTR = 0U;
    dma->State = HAL_DMA_STATE_READY;
    htim3_echo.Channel = sensor ? HAL_TIM_ACTIVE_CHANNEL_3 : HAL_TIM_ACTIVE_CHANNEL_1;
    HAL_TIM_IC_CaptureCallback(&htim3_echo);
}
static float measure_front(uint16_t first, uint16_t second) {
    CHECK(HCSR04_Trigger() == HAL_OK); complete(0U, first, second);
    CHECK(HCSR04_IsReady()); float distance = HCSR04_GetDistance();
    CHECK(HCSR04_Cancel() == HAL_OK); return distance;
}

int main(void) {
    setup(); CHECK(test_tim3.PSC == 83U && test_tim3.ARR == 65535U);
    CHECK(hdma_tim3_ic.Init.PeriphDataAlignment == DMA_PDATAALIGN_HALFWORD);
    CHECK(hdma_tim3_ic_rear.Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD);
    PASS("unchanged TIM3 configuration and halfword DMA");
    setup(); NEAR(measure_front(1000U, 1941U), 0.15997f);
    CHECK(trig_front == 1U && capture_done == 0U);
    PASS("941 us pair yields 0.15997 m and consumes ready");
    setup(); NEAR(measure_front(65000U, 1229U), 0.30005f);
    PASS("16 bit counter wrap retains pulse width");
    setup(); CHECK(HCSR04_Rear_Trigger() == HAL_OK); complete(1U, 500U, 2441U);
    CHECK(HCSR04_Rear_IsReady()); NEAR(HCSR04_Rear_GetDistance(), 0.32997f);
    CHECK(HCSR04_Rear_Cancel() == HAL_OK && trig_rear == 1U && trig_front == 0U);
    PASS("rear acquisition uses its own channel stream and buffer");
    setup(); CHECK(HCSR04_Trigger() == HAL_OK);
    CHECK(HCSR04_Cancel() == HAL_OK && aborts == 1U);
    CHECK(hdma_tim3_ic.State == HAL_DMA_STATE_READY && !(test_stream4.CR & DMA_SxCR_EN));
    CHECK(!HCSR04_IsReady() && test_stream4.NDTR == 2U);
    PASS("incomplete DMA transfer is canceled synchronously");
    setup(); CHECK(HCSR04_Trigger() == HAL_OK); captures[0] = 1234U; hdma_tim3_ic.Instance->NDTR = 1U;
    CHECK(HCSR04_Cancel() == HAL_OK);
    CHECK(captures[0] == 1234U && test_stream4.NDTR == 1U && !(test_stream4.CR & DMA_SxCR_EN));
    PASS("partial acquisition cancels without clearing raw buffer");
    setup(); CHECK(HCSR04_Trigger() == HAL_OK); complete(0U, 100U, 1000U);
    hdma_tim3_ic.ErrorCode = HAL_DMA_ERROR_TE; CHECK(!HCSR04_IsReady());
    CHECK(HCSR04_Cancel() == HAL_OK);
    PASS("DMA errors suppress readiness");
    setup(); CHECK(HCSR04_Trigger() == HAL_OK); complete(0U, 100U, 1000U);
    hdma_tim3_ic.flags = __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_tim3_ic);
    NEAR(HCSR04_GetDistance(), 0.0f); CHECK(HCSR04_Cancel() == HAL_OK);
    CHECK(hdma_tim3_ic.flags == 0U);
    PASS("front DMA error flag rejects read and clears on cancel");
    setup(); start_result = HAL_ERROR; CHECK(HCSR04_Trigger() == HAL_ERROR && trig_front == 0U);
    CHECK(HCSR04_Cancel() == HAL_OK);
    PASS("failed arm emits no TRIG");
    setup(); test_gpioc.IDR = GPIO_PIN_6; CHECK(HCSR04_Trigger() == HAL_BUSY && trig_front == 0U);
    PASS("ECHO high preserves existing no-start behavior");
    setup(); CHECK(HCSR04_Trigger() == HAL_OK); abort_result = HAL_TIMEOUT; abort_keeps_enabled = 1;
    CHECK(HCSR04_Cancel() == HAL_TIMEOUT);
    CHECK(test_stream4.CR & DMA_SxCR_EN);
    PASS("failed cancellation reports timeout and keeps stream enabled");
    setup(); CHECK(HCSR04_Trigger() == HAL_OK); complete(0U, 100U, 1000U);
    (void)HCSR04_GetDistance(); irq_enabled[4] = 0U;
    CHECK(HCSR04_Cancel() == HAL_OK && irq_enabled[4] == 0U);
    PASS("previous IRQ disabled state preserved");
    setup(); NEAR(measure_front(100U, 150U), 0.0085f);
    PASS("short pulse conversion preserves task-level range responsibility");
    setup(); for (unsigned i = 0U; i < 40U; ++i) { NEAR(measure_front(100U, 1100U), 0.17f); }
    CHECK(trig_front == 40U && aborts == 0U && !HCSR04_IsReady());
    PASS("40 consecutive acquisitions rearm without stale readiness");
    setup(); CHECK(HCSR04_Trigger() == HAL_OK); complete(0U, 100U, 1100U);
    CHECK(HCSR04_Cancel() == HAL_OK);
    CHECK(!HCSR04_IsReady()); NEAR(HCSR04_GetDistance(), 0.0f);
    PASS("cancel discards an unread completed capture");
    setup(); CHECK(HCSR04_Trigger() == HAL_OK);
    CHECK(HCSR04_Cancel() == HAL_OK);
    NEAR(measure_front(2000U, 3765U), 0.30005f);
    PASS("cancel and rearm accept a fresh capture after missing ECHO");
    setup(); CHECK(HCSR04_Rear_Trigger() == HAL_OK); complete(1U, 100U, 1100U);
    hdma_tim3_ic_rear.flags = __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_tim3_ic_rear);
    NEAR(HCSR04_Rear_GetDistance(), 0.0f); CHECK(HCSR04_Rear_Cancel() == HAL_OK);
    PASS("rear stream error mask rejects read independently");
    setup(); CHECK(HCSR04_Trigger() == HAL_OK); CHECK(HCSR04_Trigger() == HAL_OK);
    complete(0U, 100U, 1100U); NEAR(HCSR04_GetDistance(), 0.17f);
    CHECK(HCSR04_Cancel() == HAL_OK && trig_front == 2U);
    CHECK(aborts == 1U);
    PASS("direct retrigger cancels the previous pending transfer");
    setup(); CHECK(HCSR04_Trigger() == HAL_OK); complete(0U, 100U, 1100U);
    NEAR(HCSR04_GetDistance(), 0.17f); NEAR(HCSR04_GetDistance(), 0.0f);
    CHECK(HCSR04_Cancel() == HAL_OK);
    PASS("a completed capture can only be consumed once");
    printf("RESULT %u driver tests passed. No physical validation.\n", tests);
    return 0;
}
