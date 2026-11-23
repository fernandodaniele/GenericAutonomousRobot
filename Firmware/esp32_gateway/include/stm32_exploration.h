#ifndef STM32_EXPLORATION_H
#define STM32_EXPLORATION_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void Stm32Exploration_Feed(const uint8_t *p,size_t n);
void Stm32Exploration_Start(uint32_t key,const char *owner);
void Stm32Exploration_Revoke(void);
void Stm32Exploration_LinkRequest(uint32_t *key,uint32_t *ack);
bool Stm32Exploration_PollDue(void);
uint32_t Stm32Exploration_Query(unsigned *part);
bool Stm32Exploration_Fast(void);
int Stm32Exploration_Json(char *out,size_t cap);
#endif
