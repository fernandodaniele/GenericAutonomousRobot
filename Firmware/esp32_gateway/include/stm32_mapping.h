#ifndef STM32_MAPPING_H
#define STM32_MAPPING_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "mapping_grid.h"
#define MAPPING_POLL_MS 100U
#define MAPPING_REPLY_MS 200U
#define MAPPING_LINK_MS 1500U
bool Stm32Mapping_PollDue(void);
void Stm32Mapping_DeferPoll(void);
uint32_t Stm32Mapping_BeginQuery(unsigned *part);
void Stm32Mapping_QueryFailed(void);
void Stm32Mapping_Feed(const uint8_t *bytes,size_t length);
/* Unico consumidor HTTP; copia bajo lock, formatea fuera. No transmite UART. */
int Stm32Mapping_Json(char *out,size_t capacity,bool stm32_connected);
#endif
