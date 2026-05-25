#ifndef HC_SR04_H
#define HC_SR04_H

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;

    volatile uint32_t ic_val1;
    volatile uint32_t ic_val2;
    volatile uint8_t is_first_captured;
    volatile uint32_t distance_cm;
} HCSR04_HandleTypeDef;

void HCSR04_Init(HCSR04_HandleTypeDef *dev);
void HCSR04_Trigger(HCSR04_HandleTypeDef *dev);
void HCSR04_IC_Callback(HCSR04_HandleTypeDef *dev);
uint32_t HCSR04_Read(HCSR04_HandleTypeDef *dev);

#endif
