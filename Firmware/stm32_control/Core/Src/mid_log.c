/**
 * @file    mid_log.c
 * @brief   Logger de desarrollo por USB CDC.
 */

#include "mid_log.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

#define LOG_BUF_SIZE 160

void Log_Write(const char *level, const char *fmt, ...) {
    char buf[LOG_BUF_SIZE];

    /* Prefijo con timestamp en ms */
    int n = snprintf(buf, sizeof(buf), "[%7u][%s] ", (unsigned)HAL_GetTick(), level);

    /* Mensaje formateado */
    va_list args;
    va_start(args, fmt);
    n += vsnprintf(buf + n, sizeof(buf) - (size_t)n - 2, fmt, args);
    va_end(args);

    /* CRLF al final */
    if (n < (int)sizeof(buf) - 2) {
        buf[n++] = '\r';
        buf[n++] = '\n';
    } else {
        /* Buffer lleno: trunca y fuerza CRLF */
        n = (int)sizeof(buf);
        buf[n - 2] = '\r';
        buf[n - 1] = '\n';
    }

    /* Reintenta si el CDC está ocupado transmitiendo el paquete anterior */
    for (int i = 0; i < 5; i++) {
        if (CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n) != USBD_BUSY) break;
        HAL_Delay(2);
    }
}
