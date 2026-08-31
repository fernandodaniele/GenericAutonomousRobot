/**
 * @file    mid_log.c
 * @brief   Logger de desarrollo por USB CDC con timestamp del RTC DS1302.
 */

#include "mid_log.h"
#include "usbd_cdc_if.h"
#include "ds1302.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <stdarg.h>

#define LOG_BUF_SIZE     160
#define LOG_CDC_RETRIES  5

static Ds1302_t         *s_rtc   = NULL;
static SemaphoreHandle_t s_mutex = NULL;

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
}

void Log_Write(const char *level, const char *fmt, ...)
{
    char buf[LOG_BUF_SIZE];
    /* Se reservan 2 bytes al final para el CRLF. */
    const int limit = LOG_BUF_SIZE - 2;
    int pos = 0;
    int r;

    /* Sin terminal abierta (DTR): se descarta la línea. */
    if (!CDC_IsConnected())
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

    for (int i = 0; i < LOG_CDC_RETRIES; i++)
    {
        if (CDC_Transmit_FS((uint8_t *)buf, (uint16_t)pos) != USBD_BUSY)
        {
            break;
        }
        log_delay(2U);
    }

    log_unlock();
}
