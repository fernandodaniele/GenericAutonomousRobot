/**
 * @file    mid_log.c
 * @brief   Logger con timestamp del RTC DS1302.
 *
 * Cada línea va a dos destinos, serializados por un mutex:
 *   1) archivo "0:/GAR.LOG" en la SD (black box) — siempre, aunque no haya PC.
 *   2) USB CDC — solo si el host tiene el puerto abierto (DTR).
 */

#include "mid_log.h"
#include "usbd_cdc_if.h"
#include "ds1302.h"
#include "fatfs.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <stdarg.h>

#define LOG_BUF_SIZE     160
#define LOG_CDC_RETRIES  5
#define LOG_FILE_PATH    "0:/GAR.LOG"

static Ds1302_t         *s_rtc   = NULL;
static SemaphoreHandle_t s_mutex = NULL;

static FATFS   s_fs;
static FIL     s_fil;
static uint8_t s_sd_ok = 0U;   /* 1 = SD montada y GAR.LOG abierto en append */

static int scheduler_running(void)
{
    return (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
}

static void log_lock(void)
{
    if ((s_mutex != NULL) && scheduler_running())
    {
        (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
}

static void log_unlock(void)
{
    if ((s_mutex != NULL) && scheduler_running())
    {
        (void)xSemaphoreGive(s_mutex);
    }
}

static void log_delay(uint32_t ms)
{
    if (scheduler_running())
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
        HAL_Delay(ms);
    }
}

/* Prefijo con fecha/hora del DS1302, o "boot+Nms" si el RTC no responde. */
static int log_prefix(char *buf, size_t n)
{
    if ((s_rtc != NULL) && (DS1302_UpdateDateTime(s_rtc) == DS1302_OK))
    {
        uint8_t month = (uint8_t)((s_rtc->data.received_data.month.bits.month_tens * 10U)
                                +  s_rtc->data.received_data.month.bits.month_units);

        return snprintf(buf, n, "[%04u-%02u-%02u %02u:%02u:%02u]",
                        (unsigned)DS1302_GetYear(s_rtc),
                        (unsigned)month,
                        (unsigned)s_rtc->data.date_time.month_day,
                        (unsigned)s_rtc->data.date_time.hours,
                        (unsigned)s_rtc->data.date_time.minutes,
                        (unsigned)s_rtc->data.date_time.seconds);
    }

    return snprintf(buf, n, "[boot+%lums]", (unsigned long)HAL_GetTick());
}

void Log_Init(Ds1302_t *rtc)
{
    s_rtc = rtc;
    if (s_mutex == NULL)
    {
        s_mutex = xSemaphoreCreateMutex();
    }

    /* Montar la SD y abrir el log en append. Bloqueante (SPI bit por bit); se
     * llama antes del scheduler. Si no hay tarjeta, se sigue sin SD. */
    if (!s_sd_ok)
    {
        if ((f_mount(&s_fs, USERPath, 1) == FR_OK) &&
            (f_open(&s_fil, LOG_FILE_PATH, FA_OPEN_APPEND | FA_WRITE) == FR_OK))
        {
            s_sd_ok = 1U;
        }
    }
}

void Log_Write(const char *level, const char *fmt, ...)
{
    char buf[LOG_BUF_SIZE];
    /* Se reservan 2 bytes al final para el CRLF. */
    const int limit = LOG_BUF_SIZE - 2;
    int pos = 0;
    int r;

    /* Nada a lo que escribir: ni SD ni terminal. */
    if (!s_sd_ok && !CDC_IsConnected())
    {
        return;
    }

    log_lock();

    r = log_prefix(buf, (size_t)limit);
    if (r > 0)
    {
        pos = (r < limit) ? r : limit;
    }

    if (pos < limit)
    {
        r = snprintf(buf + pos, (size_t)(limit - pos), "[%s] ", level);
        if (r > 0)
        {
            pos += (r < (limit - pos)) ? r : (limit - pos);
        }
    }

    if (pos < limit)
    {
        va_list args;
        va_start(args, fmt);
        r = vsnprintf(buf + pos, (size_t)(limit - pos), fmt, args);
        va_end(args);
        if (r > 0)
        {
            pos += (r < (limit - pos)) ? r : (limit - pos);
        }
    }

    buf[pos++] = '\r';
    buf[pos++] = '\n';

    /* 1) SD (black box): siempre que esté montada, haya o no terminal. */
    if (s_sd_ok)
    {
        UINT bw = 0U;
        if ((f_write(&s_fil, buf, (UINT)pos, &bw) != FR_OK) ||
            (bw != (UINT)pos) ||
            (f_sync(&s_fil) != FR_OK))
        {
            /* Tarjeta retirada o error de escritura: dejar de intentar. */
            s_sd_ok = 0U;
        }
    }

    /* 2) USB CDC: solo si el host tiene el puerto abierto (DTR). */
    if (CDC_IsConnected())
    {
        for (int i = 0; i < LOG_CDC_RETRIES; i++)
        {
            if (CDC_Transmit_FS((uint8_t *)buf, (uint16_t)pos) != USBD_BUSY)
            {
                break;
            }
            log_delay(2U);
        }
    }

    log_unlock();
}
