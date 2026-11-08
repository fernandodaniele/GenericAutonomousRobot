/**
 * @file    mid_log.h
 * @brief   Logger de desarrollo por USB CDC (Virtual COM Port) con timestamp del RTC.
 *
 * Formato:  [AAAA-MM-DD HH:MM:SS][INF] mensaje\r\n
 * Si el RTC no responde:  [boot+1234ms][INF] mensaje\r\n
 *
 * - Las líneas se descartan si el host no tiene el puerto abierto (DTR).
 * - `Log_Write` serializa el acceso al RTC y al CDC con un mutex de FreeRTOS,
 *   así que puede llamarse desde varias tareas.
 *
 * Uso:
 *   LOG_INFO("SD init OK: %s", tipo);
 *   LOG_WARN("distancia fuera de rango: %u mm", d);
 *   LOG_ERROR("SD no responde, error %d", err);
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
 * @param  rtc  DS1302 ya inicializado (para el timestamp). NULL = sin timestamp
 *              (se usa el tick de arranque en ms).
 */
void Log_Init(Ds1302_t *rtc);

/**
 * @brief  Función interna — usar las macros LOG_* en lugar de llamarla directo.
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
 * @note No llamar desde ISR; no se incluye variante FromISR en D2.1.
 */
BaseType_t Log_EventPublish(uint16_t code, uint32_t data1,
                            uint32_t data2, uint32_t data3);

#endif /* MID_LOG_H */
