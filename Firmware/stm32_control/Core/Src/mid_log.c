/**
 * @file    mid_log.c
 * @brief   Logger del robot: archivo GAR.LOG en la SD y USB CDC.
 *
 * Basado en el mid_log de Grupo-1, reescrito con el estilo del proyecto y
 * separado en funciones chicas (formato, SD y USB).
 */

#include "mid_log.h"
#include "drv_sd_diskio.h"
#include "drv_sd_spi.h"
#include "mid_clock.h"
#include "ff.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

#define LOG_USB_RETRIES     5u      // Reintentos si el USB esta ocupado
#define LOG_USB_RETRY_MS    2u

static SemaphoreHandle_t log_mutex = NULL;
static FATFS log_fs;
static FIL log_file;
static volatile LogSdState_t log_sd_state = LOG_SD_ABSENT;
static volatile uint32_t log_lines_sd = 0u;

/* ===== Exclusion mutua ===== */

static bool log_scheduler_running(void) {
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

static void log_lock(void) {
    // Antes de arrancar el scheduler hay una sola "tarea": no hace falta mutex
    if ((log_mutex != NULL) && log_scheduler_running()) {
        (void) xSemaphoreTake(log_mutex, portMAX_DELAY);
    }
}

static void log_unlock(void) {
    if ((log_mutex != NULL) && log_scheduler_running()) {
        (void) xSemaphoreGive(log_mutex);
    }
}

static void log_delay(uint32_t ms) {
    if (log_scheduler_running()) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        HAL_Delay(ms);
    }
}

/* ===== Armado de la linea ===== */

/* Suma lo que agrego snprintf sin pasarse del limite */
static int log_advance(int position, int written, int limit) {
    if (written <= 0) {
        return position;
    }
    if (written > (limit - position)) {
        return limit;
    }
    return position + written;
}

/* Prefijo de tiempo: fecha del DS1302, o ms desde el arranque si no responde */
static int log_format_time(char *buffer, int size) {
    ClockDateTime_t now;

    if (Clock_Read(&now)) {
        return snprintf(buffer, (size_t)size, "[%04u-%02u-%02u %02u:%02u:%02u] ",
                        (unsigned)now.year, (unsigned)now.month, (unsigned)now.day,
                        (unsigned)now.hour, (unsigned)now.minute, (unsigned)now.second);
    }
    return snprintf(buffer, (size_t)size, "[boot+%lums] ", (unsigned long)HAL_GetTick());
}

/* Arma "[tiempo] [NIV] mensaje\r\n" y devuelve el largo */
static int log_format_line(char *buffer, const char *level, const char *fmt, va_list args) {
    const int limit = LOG_LINE_SIZE - 2;    // Lugar reservado para \r\n
    int position = 0;

    position = log_advance(position, log_format_time(buffer, limit), limit);
    if (position < limit) {
        int written = snprintf(&buffer[position], (size_t)(limit - position), "[%s] ", level);
        position = log_advance(position, written, limit);
    }
    if (position < limit) {
        int written = vsnprintf(&buffer[position], (size_t)(limit - position), fmt, args);
        position = log_advance(position, written, limit);
    }

    buffer[position++] = '\r';
    buffer[position++] = '\n';
    return position;
}

/* ===== Destinos ===== */

static void log_write_sd(const char *line, int length) {
    UINT written = 0u;

    if (log_sd_state != LOG_SD_ACTIVE) {
        return;
    }
    if ((f_write(&log_file, line, (UINT)length, &written) != FR_OK)
        || (written != (UINT)length)
        || (f_sync(&log_file) != FR_OK)) {
        // Tarjeta retirada o error de escritura: no insistir
        log_sd_state = LOG_SD_FAILED;
        return;
    }
    log_lines_sd++;
}

static void log_write_usb(const char *line, int length) {
    if (CDC_IsConnected() == 0u) {
        return;
    }
    for (uint32_t i = 0u; i < LOG_USB_RETRIES; i++) {
        if (CDC_Transmit_FS((uint8_t *)line, (uint16_t)length) != USBD_BUSY) {
            break;
        }
        log_delay(LOG_USB_RETRY_MS);
    }
}

/* ===== API publica ===== */

bool Log_Init(void) {
    if (log_sd_state != LOG_SD_ACTIVE) {
        if ((f_mount(&log_fs, SD_DISKIO_VOLUME, 1u) == FR_OK)
            && (f_open(&log_file, LOG_FILE_PATH, FA_OPEN_APPEND | FA_WRITE) == FR_OK)) {
            log_sd_state = LOG_SD_ACTIVE;
        } else {
            log_sd_state = LOG_SD_ABSENT;
        }
    }

    if (log_mutex == NULL) {
        log_mutex = xSemaphoreCreateMutex();
    }
    return log_sd_state == LOG_SD_ACTIVE;
}

bool Log_IsSdActive(void) {
    return log_sd_state == LOG_SD_ACTIVE;
}

void Log_GetStatus(LogStatus_t *out) {
    if (out == NULL) {
        return;
    }
    out->sd_state = log_sd_state;
    out->sd_type = (uint8_t)SD_GetType();
    out->lines_sd = log_lines_sd;
}

void Log_Write(const char *level, const char *fmt, ...) {
    char line[LOG_LINE_SIZE];
    va_list args;

    // Sin SD ni PC conectada no hay donde escribir
    if ((log_sd_state != LOG_SD_ACTIVE) && (CDC_IsConnected() == 0u)) {
        return;
    }

    log_lock();

    va_start(args, fmt);
    int length = log_format_line(line, level, fmt, args);
    va_end(args);

    log_write_sd(line, length);
    log_write_usb(line, length);

    log_unlock();
}
