/**
 * @file    mid_log.h
 * @brief   Logger de desarrollo por USB CDC (Virtual COM Port).
 *
 * Formato de salida: [  1234ms][INF] mensaje\r\n
 *
 * Uso:
 *   LOG_INFO("SD init OK: %s", tipo);
 *   LOG_WARN("distancia fuera de rango: %u mm", d);
 *   LOG_ERROR("SD no responde, error %d", err);
 */

#ifndef MID_LOG_H
#define MID_LOG_H

/**
 * @brief  Escribe una línea de log con nivel INFO.
 */
#define LOG_INFO(fmt, ...)   Log_Write("INF", fmt, ##__VA_ARGS__)

/**
 * @brief  Escribe una línea de log con nivel WARNING.
 */
#define LOG_WARN(fmt, ...)   Log_Write("WRN", fmt, ##__VA_ARGS__)

/**
 * @brief  Escribe una línea de log con nivel ERROR.
 */
#define LOG_ERROR(fmt, ...)  Log_Write("ERR", fmt, ##__VA_ARGS__)

/**
 * @brief  Función interna — usar las macros LOG_* en lugar de llamar directamente.
 */
void Log_Write(const char *level, const char *fmt, ...);

#endif /* MID_LOG_H */
