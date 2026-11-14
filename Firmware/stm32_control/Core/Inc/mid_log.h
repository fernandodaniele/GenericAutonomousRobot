/**
 * @file    mid_log.h
 * @brief   Logger del robot: archivo GAR.LOG en la SD y USB CDC.
 *
 * Cada linea va a dos destinos:
 *   1) Archivo "0:/GAR.LOG" en la SD (caja negra), siempre que este montada.
 *   2) USB CDC (Virtual COM Port), solo si la PC tiene el puerto abierto (DTR).
 *
 * Formato:  [boot+1234ms] [INF] mensaje\r\n
 * (la fecha y hora del DS1302 se agrega en la parte 4 del datalogger)
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
 *   LOG_WARN("distancia fuera de rango: %u mm", d);
 *   LOG_ERROR("fallo la escritura, error %d", err);
 */

#ifndef MID_LOG_H
#define MID_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#define LOG_LINE_SIZE   160     // Largo maximo de una linea (incluye prefijo y \r\n)
#define LOG_FILE_PATH   "0:/GAR.LOG"

/** @brief  Linea de nivel INFO. */
#define LOG_INFO(fmt, ...)   Log_Write("INF", fmt, ##__VA_ARGS__)
/** @brief  Linea de nivel WARNING. */
#define LOG_WARN(fmt, ...)   Log_Write("WRN", fmt, ##__VA_ARGS__)
/** @brief  Linea de nivel ERROR. */
#define LOG_ERROR(fmt, ...)  Log_Write("ERR", fmt, ##__VA_ARGS__)

/**
 * @brief  Monta la SD, abre GAR.LOG para agregar al final y crea el mutex.
 *
 * Llamar una vez desde main(), despues de MX_SPI2_Init y antes de iniciar
 * el scheduler. Si la SD no esta o falla, el log sigue saliendo por USB.
 *
 * @return true si la SD quedo montada y GAR.LOG abierto.
 */
bool Log_Init(void);

/**
 * @brief  Indica si el log se esta guardando en la SD.
 */
bool Log_IsSdActive(void);

/**
 * @brief  Escribe una linea. Usar las macros LOG_* en lugar de llamarla directo.
 * @param  level  Etiqueta de 3 letras ("INF", "WRN", "ERR").
 * @param  fmt    Formato estilo printf (sin %f: newlib-nano no imprime float).
 */
void Log_Write(const char *level, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* MID_LOG_H */
