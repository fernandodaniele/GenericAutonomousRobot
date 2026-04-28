
/*
 * drv_TCRT5000.h
 *
 *  Created on: April, 2026
 *      Author: Matías H. Costamagna
 */

#ifndef CORE_INC_DRV_TCRT5000_H_
#define CORE_INC_DRV_TCRT5000_H_

#include <stdint.h>
#include <stdbool.h>

/* ================= TIPOS ================= */
//estructura para diferenciar mediciones
typedef enum
{
    TCRT_STATE_LINE = 0,
    TCRT_STATE_FLOOR,
    TCRT_STATE_CLIFF
} tcrt_state_t;

//Estructura de salida del sensor TCRT5000.
typedef struct
{
    tcrt_state_t state;
    bool unstable;
    uint16_t value;
} tcrt_reading_t;

/* ================= API ================= */

void drv_tcrt5000_init(void);
void drv_tcrt5000_update(void);

uint16_t drv_tcrt5000_read_left(void);
uint16_t drv_tcrt5000_read_right(void);

tcrt_reading_t drv_tcrt5000_get_left(void);
tcrt_reading_t drv_tcrt5000_get_right(void);

#endif /* CORE_INC_DRV_TCRT5000_H_ */
