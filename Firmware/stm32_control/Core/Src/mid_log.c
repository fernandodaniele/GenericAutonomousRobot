/**
 * @file    mid_log.c
 * @brief Mensajes síncronos por microSD y USB CDC, y eventos por cola.
 *
 * LogEvents consume la cola y mantiene EVENTOS.CSV separado de GAR.LOG.
 * El mutex compartido serializa los accesos al RTC y al volumen FatFs.
 */

#include "mid_log.h"
#include "usbd_cdc_if.h"
#include "ds1302.h"
#include "fatfs.h"
#include "task_control_robot.h"
#include "task_safety.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>
#include <string.h>

#define LOG_BUF_SIZE     160
#define LOG_CDC_RETRIES  5
#define LOG_FILE_PATH    "0:/GAR.LOG"
#define LOG_EVENT_QUEUE_LENGTH 8U
#define LOG_EVENT_TASK_STACK_WORDS 512U
#define LOG_EVENT_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define LOG_EVENT_FILE_PATH "0:/EVENTOS.CSV"
#define LOG_EVENT_HEADER "FECHA_HORA,TICK,CODIGO,DATO"

/* Datos por valor: la cola conserva una copia independiente del productor. */
typedef struct
{
    TickType_t tick;
    uint16_t code;
    uint32_t data1;
    uint32_t data2;
    uint32_t data3;
} LogEvent_t;

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
static uint8_t s_volume_mounted = 0U; /* No depende de abrir GAR.LOG. */

typedef enum {
    LOG_EVENT_FILE_PENDING = 0,
    LOG_EVENT_FILE_READY,
    LOG_EVENT_FILE_DISABLED
} LogEventFileState_t;

static FIL s_event_fil;
static LogEventFileState_t s_event_file_state = LOG_EVENT_FILE_PENDING;
/* Unico consumidor: buffers privados fuera del stack de 512 palabras. */
static char s_event_data[96];
static char s_event_line[LOG_BUF_SIZE];

static bool Log_EventArgumentsValid(const LogEvent_t *event)
{
    const uint32_t fault_mask = SAFETY_FAULT_CLIFF | SAFETY_FAULT_OBSTACLE |
                                SAFETY_FAULT_SENSOR | SAFETY_FAULT_EMERGENCY;
    switch (event->code)
    {
        case 100:
            return event->data1 <= ROBOT_STATE_SAFETY_STOP &&
                   (event->data2 & ~fault_mask) == 0U && event->data3 <= 1U;
        case 110:
            return event->data1 <= ROBOT_STATE_SAFETY_STOP &&
                   event->data2 <= ROBOT_STATE_SAFETY_STOP &&
                   event->data1 != event->data2 && event->data3 <= ROBOT_EVENT_SAFETY_RESET;
        case 201: case 202: case 203: case 204:
        {
            uint32_t bit = 1U << (event->code - 201U);
            return event->data1 == bit && (event->data2 & bit) != 0U &&
                   (event->data2 & ~fault_mask) == 0U && event->data3 == 0U;
        }
        case 210:
            return (event->data1 & ~fault_mask) == 0U &&
                   event->data2 == 0U && event->data3 == 0U;
        case 300:
            return event->data1 <= ROBOT_STATE_SAFETY_STOP &&
                   event->data2 == 0U && event->data3 == 0U;
        case 900:
            return event->data2 == 0U && event->data3 == 0U;
        default:
            return false;
    }
}

static int Log_EventFormatData(const LogEvent_t *event, char *data, size_t size)
{
    switch (event->code)
    {
        case 100:
            return snprintf(data, size, "estado=%" PRIu32 "|fallas=%" PRIu32 "|bloqueo=%" PRIu32,
                            event->data1, event->data2, event->data3);
        case 110:
            return snprintf(data, size, "anterior=%" PRIu32 "|nuevo=%" PRIu32 "|evento=%" PRIu32,
                            event->data1, event->data2, event->data3);
        case 201: case 202: case 203: case 204:
            return snprintf(data, size, "bit=%" PRIu32 "|mascara=%" PRIu32,
                            event->data1, event->data2);
        case 210:
            return snprintf(data, size, "fallas_anteriores=%" PRIu32 "|fallas=0|bloqueo=0", event->data1);
        case 300:
            return snprintf(data, size, "estado=%" PRIu32, event->data1);
        case 900:
            return snprintf(data, size, "descartados=%" PRIu32, event->data1);
        default:
            return -1;
    }
}

/* Cero significa calendario invalido; el llamador deja FECHA_HORA vacia.
 * Valida los registros crudos: DS1302_OK no prueba calendario ni presencia. */
static int Log_EventFormatDate(const RtcRegister_t *raw, char *date, size_t size)
{
    static const uint8_t days_per_month[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    unsigned second = raw->seconds.bits.seconds_tens * 10U + raw->seconds.bits.seconds_units;
    unsigned minute = raw->minutes.bits.minutes_tens * 10U + raw->minutes.bits.minutes_units;
    unsigned day = raw->month_day.bits.month_day_tens * 10U + raw->month_day.bits.month_day_units;
    unsigned month = raw->month.bits.month_tens * 10U + raw->month.bits.month_units;
    unsigned year = 2000U + raw->year.bits.year_tens * 10U + raw->year.bits.year_units;
    unsigned hour;

    if (raw->seconds.bits.clock_halt || raw->seconds.bits.seconds_units > 9U || second > 59U ||
        raw->minutes.bits.reserved || raw->minutes.bits.minutes_units > 9U || minute > 59U ||
        raw->month_day.bits.reserved || raw->month_day.bits.month_day_units > 9U ||
        raw->month.bits.reserved || raw->month.bits.month_units > 9U || month < 1U || month > 12U ||
        raw->year.bits.year_tens > 9U || raw->year.bits.year_units > 9U ||
        raw->weekday.bits.reserved || raw->weekday.bits.day < 1U || raw->weekday.bits.day > 7U)
    {
        return 0;
    }
    unsigned days = days_per_month[month - 1U];
    if (month == 2U && year % 4U == 0U && (year % 100U != 0U || year % 400U == 0U)) ++days;
    if (day < 1U || day > days) return 0;

    if (raw->hour.hour_24.bits.hour_format == 0U)
    {
        hour = raw->hour.hour_24.bits.hour_tens * 10U + raw->hour.hour_24.bits.hour_units;
        if (raw->hour.hour_24.bits.reserved || raw->hour.hour_24.bits.hour_units > 9U || hour > 23U) return 0;
    }
    else
    {
        hour = raw->hour.hour_12.bits.hour_tens * 10U + raw->hour.hour_12.bits.hour_units;
        if (raw->hour.hour_12.bits.reserved || raw->hour.hour_12.bits.hour_units > 9U ||
            hour < 1U || hour > 12U) return 0;
        hour = hour % 12U + (raw->hour.hour_12.bits.am_pm ? 12U : 0U);
    }
    return snprintf(date, size, "%04u-%02u-%02u %02u:%02u:%02u", year, month, day, hour, minute, second);
}

/* Requiere s_mutex. No remonta, trunca ni repara; lecturas de longitud fija. */
static bool Log_EventOpenFile(void)
{
    static const char header[] = LOG_EVENT_HEADER;
    static const char header_line[] = LOG_EVENT_HEADER "\r\n";
    char read_header[sizeof(header)];
    char last;
    UINT count = 0U;

    if (f_open(&s_event_fil, LOG_EVENT_FILE_PATH, FA_OPEN_APPEND | FA_READ | FA_WRITE) != FR_OK) return false;
    FSIZE_t size = f_size(&s_event_fil);
    if (size == 0U)
    {
        return f_write(&s_event_fil, header_line, sizeof(header_line) - 1U, &count) == FR_OK &&
               count == sizeof(header_line) - 1U && f_sync(&s_event_fil) == FR_OK;
    }
    if (size < sizeof(header) || f_lseek(&s_event_fil, 0U) != FR_OK ||
        f_read(&s_event_fil, read_header, sizeof(read_header), &count) != FR_OK ||
        count != sizeof(read_header) || memcmp(read_header, header, sizeof(header) - 1U) != 0) return false;
    last = read_header[sizeof(header) - 1U];
    if (last == '\r')
    {
        if (f_read(&s_event_fil, &last, 1U, &count) != FR_OK || count != 1U) return false;
    }
    if (last != '\n' || f_lseek(&s_event_fil, size - 1U) != FR_OK ||
        f_read(&s_event_fil, &last, 1U, &count) != FR_OK || count != 1U || last != '\n') return false;
    return true;
}

static void Log_EventProcess(const LogEvent_t *event)
{
    RtcRegister_t raw;
    bool rtc_read = false;
    char date[20] = "";
    int length;

    if (s_event_file_state == LOG_EVENT_FILE_DISABLED || !Log_EventArgumentsValid(event)) return;
    length = Log_EventFormatData(event, s_event_data, sizeof(s_event_data));
    if (length < 0 || (size_t)length >= sizeof(s_event_data))
    {
        s_event_file_state = LOG_EVENT_FILE_DISABLED;
        return;
    }
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdPASS)
    {
        s_event_file_state = LOG_EVENT_FILE_DISABLED;
        return;
    }
    if (!s_volume_mounted ||
        (s_event_file_state == LOG_EVENT_FILE_PENDING && !Log_EventOpenFile()))
    {
        s_event_file_state = LOG_EVENT_FILE_DISABLED;
    }
    else
    {
        s_event_file_state = LOG_EVENT_FILE_READY;
        if (s_rtc != NULL && DS1302_UpdateDateTime(s_rtc) == DS1302_OK)
        {
            raw = s_rtc->data.received_data;
            rtc_read = true;
        }
    }
    (void)xSemaphoreGive(s_mutex);
    if (s_event_file_state == LOG_EVENT_FILE_DISABLED) return;

    length = rtc_read ? Log_EventFormatDate(&raw, date, sizeof(date)) : 0;
    if (length < 0 || (size_t)length >= sizeof(date))
    {
        s_event_file_state = LOG_EVENT_FILE_DISABLED;
        return;
    }
    length = snprintf(s_event_line, sizeof(s_event_line), "%s,%" PRIu32 ",%u,%s\r\n",
                      date, (uint32_t)event->tick, (unsigned)event->code, s_event_data);
    if (length < 0 || (size_t)length >= sizeof(s_event_line))
    {
        s_event_file_state = LOG_EVENT_FILE_DISABLED;
        return;
    }
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdPASS)
    {
        s_event_file_state = LOG_EVENT_FILE_DISABLED;
        return;
    }
    UINT written = 0U;
    if (f_lseek(&s_event_fil, f_size(&s_event_fil)) != FR_OK ||
        f_write(&s_event_fil, s_event_line, (UINT)length, &written) != FR_OK ||
        written != (UINT)length || f_sync(&s_event_fil) != FR_OK)
    {
        s_event_file_state = LOG_EVENT_FILE_DISABLED;
    }
    (void)xSemaphoreGive(s_mutex);
}

static void Log_EventTask(void *parameters)
{
    LogEvent_t event;
    (void)parameters;

    for (;;)
    {
        if (xQueueReceive(s_event_queue, &event, portMAX_DELAY) == pdPASS)
        {
            Log_EventProcess(&event);
        }
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
        s_volume_mounted = (f_mount(&s_fs, USERPath, 1) == FR_OK);
        if (s_volume_mounted &&
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
