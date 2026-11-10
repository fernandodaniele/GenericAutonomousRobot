#ifndef MID_MAPPING_H
#define MID_MAPPING_H
#include <stddef.h>
#include <stdbool.h>
#include "FreeRTOS.h"
bool Mapping_Query(const char *message, uint32_t *query, unsigned *part);
/* Solo Comms. Formato acotado, independiente de UART fisica. */
int Mapping_Format(char *out, size_t capacity, uint32_t query, unsigned part, TickType_t now);
#endif
