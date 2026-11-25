/**
 * @file    mid_log.h
 * @brief Registro síncrono de mensajes y persistencia diferida de eventos.
 *
 * Log_Write escribe en GAR.LOG si está disponible y en USB CDC si hay DTR.
 * Sin hora válida del RTC, el mensaje se fecha con el tiempo desde el arranque.
 * Log_EventPublish encola eventos que LogEvents procesa para EVENTOS.CSV.
 * Ambos caminos comparten el mutex que protege RTC y FatFs.
 */

#ifndef MID_LOG_H
#define MID_LOG_H

#include "ds1302.h"
#include "FreeRTOS.h"

/** @brief  Línea de log nivel INFO. */
#define LOG_INFO(fmt, ...)   Log_Write("INF", fmt, ##__VA_ARGS__)
/** @brief  Línea de log nivel WARNING. */
#define LOG_WARN(fmt, ...)   Log_Write("WRN", fmt, ##__VA_ARGS__)
/** @brief  Línea de log nivel ERROR. */
#define LOG_ERROR(fmt, ...)  Log_Write("ERR", fmt, ##__VA_ARGS__)

/**
 * @brief  Inicializa el logger. Llamar una vez desde main() antes del scheduler.
 * @param rtc DS1302 inicializado. NULL implica tiempo desde el arranque
 *        para mensajes y fecha vacía para eventos.
 */
void Log_Init(Ds1302_t *rtc);

/**
 * @brief Escribe desde contexto de tarea. Puede bloquear por mutex o E/S.
 */
void Log_Write(const char *level, const char *fmt, ...);

/**
 * @brief Encola un evento para su procesamiento diferido (solo desde tareas).
 *
 * El tick FreeRTOS se captura al publicar. Los tres datos numericos tienen
 * significado segun el codigo definido en Docs/CODIGOS_EVENTOS.md; pasar 0
 * para los campos no utilizados. No accede al RTC, la SD ni el USB.
 *
 * @return pdPASS si el evento fue encolado; pdFAIL si la cola no esta
 *         disponible o esta llena. Nunca espera por espacio en la cola.
 * @note No llamar desde ISR.
 */
BaseType_t Log_EventPublish(uint16_t code, uint32_t data1,
                            uint32_t data2, uint32_t data3);

#endif /* MID_LOG_H */
