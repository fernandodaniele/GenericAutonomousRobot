/**
 * @file    mid_log.h
 * @brief   Logger del robot: archivo GAR.LOG en la SD y USB CDC.
 *
 * Cada linea va a dos destinos:
 *   1) Archivo "0:/GAR.LOG" en la SD (caja negra), siempre que este montada.
 *   2) USB CDC (Virtual COM Port), solo si la PC tiene el puerto abierto (DTR).
 *
 * Formato:  [2026-11-14 17:30:05] [INF] mensaje\r\n
 * Si el reloj DS1302 no responde o no esta en hora: [boot+1234ms] [INF] ...
 *
 * - Log_Write se puede llamar desde varias tareas: un mutex serializa el
 *   acceso a la SD y al USB.
 * - Si la SD falta o falla, el logger deja de escribir en ella y el robot
 *   sigue funcionando. Para volver a intentarlo hay que reiniciar.
 * - Cada linea hace f_sync: ante un corte de energia se pierde como mucho
 *   la ultima. Escribir en la SD tarda decenas de ms: no llamar LOG_* desde
 *   lazos rapidos (task_nav, task_sensors).
 * - La tarea que llama necesita al menos 1 KB de stack (buffer de linea y FatFs).
 *
 * Uso:
 *   LOG_INFO("SD lista: %s", tipo);
 *   LOG_DATA("%lu,%u,%u", t, a, b);     // fila de datos del datalogger
 *   LOG_WARN("distancia fuera de rango: %u mm", d);
 *   LOG_ERROR("fallo la escritura, error %d", err);
 */

#ifndef MID_LOG_H
#define MID_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define LOG_LINE_SIZE   160     // Largo maximo de una linea (incluye prefijo y \r\n)
#define LOG_FILE_PATH   "0:/GAR.LOG"

/** @brief  Linea de nivel INFO. */
#define LOG_INFO(fmt, ...)   Log_Write("INF", fmt, ##__VA_ARGS__)
/** @brief  Linea de nivel WARNING. */
#define LOG_WARN(fmt, ...)   Log_Write("WRN", fmt, ##__VA_ARGS__)
/** @brief  Linea de nivel ERROR. */
#define LOG_ERROR(fmt, ...)  Log_Write("ERR", fmt, ##__VA_ARGS__)
/** @brief  Fila de datos del datalogger (nivel DAT). */
#define LOG_DATA(fmt, ...)   Log_Write("DAT", fmt, ##__VA_ARGS__)

/** @brief  Estado de la tarjeta SD para el log. */
typedef enum {
    LOG_SD_ABSENT = 0,      // No se pudo montar al arrancar (no hay tarjeta o no responde)
    LOG_SD_ACTIVE,          // Montada y escribiendo GAR.LOG
    LOG_SD_FAILED           // Estaba montada y fallo una escritura: se dejo de usar
} LogSdState_t;

/** @brief  Resumen del log para mostrar en la web. */
typedef struct {
    LogSdState_t sd_state;
    uint8_t sd_type;        // SD_Type_t de drv_sd_spi (0 = desconocido, 1 = SDSC, 2 = SDHC)
    uint32_t lines_sd;      // Lineas guardadas en GAR.LOG desde el arranque
} LogStatus_t;

/**
 * @brief  Monta la SD, abre GAR.LOG para agregar al final y crea el mutex.
 *
 * Llamar una vez desde main(), despues de MX_SPI2_Init y Clock_Init y antes
 * de iniciar el scheduler. Si la SD no esta o falla, el log sigue saliendo
 * por USB.
 *
 * @return true si la SD quedo montada y GAR.LOG abierto.
 */
bool Log_Init(void);

/**
 * @brief  Indica si el log se esta guardando en la SD.
 */
bool Log_IsSdActive(void);

/**
 * @brief  Copia el estado actual del log (no bloquea).
 */
void Log_GetStatus(LogStatus_t *out);

/**
 * @brief  Escribe una linea. Usar las macros LOG_* en lugar de llamarla directo.
 * @param  level  Etiqueta de 3 letras ("INF", "WRN", "ERR", "DAT").
 * @param  fmt    Formato estilo printf (sin %f: newlib-nano no imprime float).
 */
void Log_Write(const char *level, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* MID_LOG_H */
