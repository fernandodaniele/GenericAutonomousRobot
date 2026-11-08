#ifndef STM32_STATUS_H
#define STM32_STATUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define STM32_STATUS_POLL_MS  1000U
#define STM32_STATUS_STALE_MS 3000U

typedef struct {
    bool valid;
    bool connected;
    uint32_t state;
    uint32_t faults;
    uint32_t stop;
    uint64_t age_ms;
} Stm32StatusSnapshot;

/* Unico productor: el lector UART existente. Solo acepta lineas completas. */
void Stm32Status_Feed(const uint8_t *bytes, size_t length);

/* Copia consistente; no mantiene el bloqueo durante HTTP ni E/S. */
void Stm32Status_GetSnapshot(Stm32StatusSnapshot *snapshot);

/* Solo app_main: decide una consulta, sin enviarla ni compensar retrasos. */
bool Stm32Status_PollDue(void);

#endif
