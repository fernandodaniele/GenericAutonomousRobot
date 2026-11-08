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
#include "queue.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <stdarg.h>

#define LOG_BUF_SIZE     160
#define LOG_CDC_RETRIES  5
#define LOG_FILE_PATH    "0:/GAR.LOG"
#define LOG_EVENT_QUEUE_LENGTH 8U
#define LOG_EVENT_TASK_STACK_WORDS 512U
#define LOG_EVENT_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)

/* Datos por valor: la cola conserva una copia independiente del productor. */
typedef struct
{
    TickType_t tick;
    uint16_t code;
    uint32_t data1;
    uint32_t data2;
    uint32_t data3;
} LogEvent_t;

/* Cola estatica: no reserva heap ni crea una tarea consumidora en D2.1. */
static StaticQueue_t s_event_queue_cb;
static uint8_t s_event_queue_storage[LOG_EVENT_QUEUE_LENGTH * sizeof(LogEvent_t)];
static QueueHandle_t s_event_queue = NULL;

static StackType_t s_event_task_stack[LOG_EVENT_TASK_STACK_WORDS];
static StaticTask_t s_event_task_cb;
static TaskHandle_t s_event_task = NULL;

static Ds1302_t         *s_rtc   = NULL;
static SemaphoreHandle_t s_mutex = NULL;

static FATFS   s_fs;
static FIL     s_fil;
static uint8_t s_sd_ok = 0U;   /* 1 = SD montada y GAR.LOG abierto en append */

static void Log_EventTask(void *parameters)
{
    LogEvent_t event;
    (void)parameters;

    for (;;)
    {
        (void)xQueueReceive(s_event_queue, &event, portMAX_DELAY);
        /* D2.2.2: descartar deliberadamente; aun no hay persistencia.
         * No conectar productores operativos durante esta etapa intermedia. */
    }
}

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

    /* Primero montar SD */
    if (!s_sd_ok)
    {
        if ((f_mount(&s_fs, USERPath, 1) == FR_OK) &&
            (f_open(&s_fil, LOG_FILE_PATH, FA_OPEN_APPEND | FA_WRITE) == FR_OK))
        {
            s_sd_ok = 1U;
        }
    }

    /* Crear mutex al final */
    if (s_mutex == NULL)
    {
        s_mutex = xSemaphoreCreateMutex();
    }

    /* Independiente del resultado de montaje SD. */
    if (s_event_queue == NULL)
    {
        s_event_queue = xQueueCreateStatic(LOG_EVENT_QUEUE_LENGTH,
                                           sizeof(LogEvent_t),
                                           s_event_queue_storage,
                                           &s_event_queue_cb);
    }

    if ((s_event_queue != NULL) && (s_mutex != NULL) && (s_event_task == NULL))
    {
        s_event_task = xTaskCreateStatic(Log_EventTask, "LogEvents",
                                          LOG_EVENT_TASK_STACK_WORDS, NULL,
                                          LOG_EVENT_TASK_PRIORITY,
                                          s_event_task_stack, &s_event_task_cb);
        if (s_event_task == NULL)
        {
            return; /* Sin bloquear el robot; otra Log_Init puede reintentar. */
        }
    }
}

BaseType_t Log_EventPublish(uint16_t code, uint32_t data1,
                            uint32_t data2, uint32_t data3)
{
    LogEvent_t event;

    if (s_event_queue == NULL)
    {
        return pdFAIL;
    }

    event.tick = xTaskGetTickCount();
    event.code = code;
    event.data1 = data1;
    event.data2 = data2;
    event.data3 = data3;

    return xQueueSend(s_event_queue, &event, 0U);
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
