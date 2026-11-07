#ifndef TEST_ULTRASOUND_HAL_H
#define TEST_ULTRASOUND_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_DMA_STATE_RESET, HAL_DMA_STATE_READY, HAL_DMA_STATE_BUSY,
               HAL_DMA_STATE_TIMEOUT, HAL_DMA_STATE_ERROR, HAL_DMA_STATE_ABORT } HAL_DMA_StateTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef int IRQn_Type;
typedef struct { volatile uint32_t CR, NDTR, FCR; } DMA_Stream_TypeDef;
typedef struct {
    uint32_t Channel, Direction, PeriphInc, MemInc, PeriphDataAlignment;
    uint32_t MemDataAlignment, Mode, Priority, FIFOMode;
} DMA_InitTypeDef;
typedef struct {
    DMA_Stream_TypeDef *Instance;
    DMA_InitTypeDef Init;
    HAL_DMA_StateTypeDef State;
    uint32_t ErrorCode, flags;
    void *Parent;
} DMA_HandleTypeDef;
typedef struct { volatile uint32_t PSC, ARR, CCMR1, CCMR2, CCER, SMCR, SR, DIER, CR1; } TIM_TypeDef;
typedef struct { uint32_t Prescaler, CounterMode, Period, ClockDivision; } TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init;
                 DMA_HandleTypeDef *hdma[3]; uint32_t Channel; } TIM_HandleTypeDef;
typedef struct { uint32_t ICPolarity, ICSelection, ICPrescaler, ICFilter; } TIM_IC_InitTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { volatile uint32_t IDR, ODR; } GPIO_TypeDef;
typedef struct { volatile uint32_t CYCCNT, CTRL; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
extern uint32_t SystemCoreClock;
extern TIM_TypeDef test_tim3;
extern DMA_Stream_TypeDef test_stream4, test_stream7;
extern GPIO_TypeDef test_gpioc;
extern CoreDebug_Type test_coredebug;
DWT_Type *test_dwt(void);
#define TIM3 (&test_tim3)
#define DMA1_Stream4 (&test_stream4)
#define DMA1_Stream7 (&test_stream7)
#define GPIOC (&test_gpioc)
#define CoreDebug (&test_coredebug)
#define DWT test_dwt()
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_AF2_TIM3 2U
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_INPUTCHANNELPOLARITY_BOTHEDGE 10U
#define TIM_ICSELECTION_DIRECTTI 1U
#define TIM_ICPSC_DIV1 0U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_3 8U
#define TIM_DMA_ID_CC1 0U
#define TIM_DMA_ID_CC3 2U
#define TIM_DMA_CC1 (1U << 9)
#define TIM_DMA_CC3 (1U << 11)
#define TIM_FLAG_CC1 (1U << 1)
#define TIM_FLAG_CC3 (1U << 3)
#define TIM_FLAG_CC1OF (1U << 9)
#define TIM_FLAG_CC3OF (1U << 11)
#define HAL_TIM_ACTIVE_CHANNEL_1 1U
#define HAL_TIM_ACTIVE_CHANNEL_3 4U
#define DMA1_Stream4_IRQn 4
#define DMA1_Stream7_IRQn 7
#define DMA_SxCR_EN 1U
#define DMA_IT_TC (1U << 4)
#define DMA_IT_HT (1U << 3)
#define DMA_IT_TE (1U << 2)
#define DMA_IT_DME (1U << 1)
#define DMA_IT_FE (1U << 7)
#define HAL_DMA_ERROR_NONE 0U
#define HAL_DMA_ERROR_TE 1U
#define DMA_CHANNEL_5 5U
#define DMA_PERIPH_TO_MEMORY 0U
#define DMA_PINC_DISABLE 0U
#define DMA_MINC_ENABLE 1U
#define DMA_PDATAALIGN_HALFWORD 1U
#define DMA_MDATAALIGN_HALFWORD 1U
#define DMA_NORMAL 0U
#define DMA_PRIORITY_HIGH 2U
#define DMA_FIFOMODE_DISABLE 0U
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)
#define __HAL_LINKDMA(timer, field, dma) do { (timer)->field = &(dma); (dma).Parent = (timer); } while (0)
#define __HAL_TIM_DISABLE_DMA(timer, mask) ((timer)->Instance->DIER &= ~(mask))
#define __HAL_TIM_CLEAR_FLAG(timer, mask) ((timer)->Instance->SR &= ~(mask))
#define __HAL_DMA_DISABLE_IT(dma, mask) do { if ((mask) == DMA_IT_FE) (dma)->Instance->FCR &= ~(mask); \
    else (dma)->Instance->CR &= ~(mask); } while (0)
// Same flag positions as this HAL: streams 4/7 share HISR but have distinct masks.
#define TEST_DMA_FLAG(dma, front, rear) ((dma)->Instance == DMA1_Stream4 ? (front) : (rear))
#define __HAL_DMA_GET_TC_FLAG_INDEX(dma) TEST_DMA_FLAG(dma, 0x20U, 0x08000000U)
#define __HAL_DMA_GET_HT_FLAG_INDEX(dma) TEST_DMA_FLAG(dma, 0x10U, 0x04000000U)
#define __HAL_DMA_GET_TE_FLAG_INDEX(dma) TEST_DMA_FLAG(dma, 0x08U, 0x02000000U)
#define __HAL_DMA_GET_DME_FLAG_INDEX(dma) TEST_DMA_FLAG(dma, 0x04U, 0x01000000U)
#define __HAL_DMA_GET_FE_FLAG_INDEX(dma) TEST_DMA_FLAG(dma, 0x01U, 0x00400000U)
#define __HAL_DMA_GET_FLAG(dma, mask) ((dma)->flags & (mask))
#define __HAL_DMA_CLEAR_FLAG(dma, mask) ((dma)->flags &= ~(mask))
uint32_t NVIC_GetEnableIRQ(IRQn_Type);
void HAL_NVIC_DisableIRQ(IRQn_Type);
void HAL_NVIC_EnableIRQ(IRQn_Type);
void HAL_NVIC_SetPriority(IRQn_Type, uint32_t, uint32_t);
void NVIC_ClearPendingIRQ(IRQn_Type);
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *, TIM_IC_InitTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *, uint32_t, uint32_t *, uint16_t);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *);
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *);
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *);
#endif
