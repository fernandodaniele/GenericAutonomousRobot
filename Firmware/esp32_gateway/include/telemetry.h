#ifndef TELEMETRY_H
#define TELEMETRY_H

/**
 * @file telemetry.h
 * @brief Ultimos datos del robot recibidos de la STM32 y su salida en JSON.
 */

#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Prepara el modulo. Llamar antes de usar las demas funciones.
 */
void Telemetry_Init(void);

/**
 * @brief Procesa una linea recibida de la STM32.
 * @param line Linea sin terminador.
 * @return true si la linea era de telemetria (TEL, EVT o MAP).
 */
bool Telemetry_HandleLine(const char *line);

/**
 * @brief Escribe el ultimo estado del robot en formato JSON.
 * @param out Buffer destino.
 * @param size Tamano del buffer (al menos 1024 bytes).
 * @return Largo del texto escrito, o 0 si no entro en el buffer.
 */
size_t Telemetry_ToJson(char *out, size_t size);

#endif /* TELEMETRY_H */
