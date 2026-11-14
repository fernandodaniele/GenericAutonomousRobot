/**
 * @file    mid_clock.h
 * @brief   Fecha y hora del robot a partir del RTC DS1302.
 *
 * Envuelve al driver ds1302 para el resto del firmware:
 *   - Valida lo que lee: si el DS1302 no esta conectado o nunca se puso en
 *     hora, Clock_Read devuelve false (el log usa entonces "boot+ms").
 *   - Un mutex permite usarlo desde varias tareas (log, datalogger y el
 *     comando TIME que llega del ESP32).
 *
 * Conexion: CLK -> PD8, DAT -> PD9, RST -> PD10, VCC 3,3 V, pila CR2032.
 */

#ifndef MID_CLOCK_H
#define MID_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define CLOCK_YEAR_MIN  2000u
#define CLOCK_YEAR_MAX  2099u

typedef struct {
    uint16_t year;      // 2000 a 2099
    uint8_t month;      // 1 a 12
    uint8_t day;        // 1 a 31
    uint8_t hour;       // 0 a 23
    uint8_t minute;     // 0 a 59
    uint8_t second;     // 0 a 59
} ClockDateTime_t;

/**
 * @brief  Inicializa el DS1302 y crea el mutex. Llamar una vez desde main().
 * @return true si el DS1302 respondio con una fecha valida.
 */
bool Clock_Init(void);

/**
 * @brief  Lee la fecha y hora actuales.
 * @param  out  Destino. Solo se completa si devuelve true.
 * @return true si el reloj respondio con una fecha valida y en marcha.
 */
bool Clock_Read(ClockDateTime_t *out);

/**
 * @brief  Pone en hora el DS1302 (formato 24 h) y lo arranca.
 * @param  value  Fecha y hora a cargar. El dia de la semana se calcula solo.
 * @return true si los valores eran validos y se escribieron.
 */
bool Clock_Set(const ClockDateTime_t *value);

/**
 * @brief  Resultado de la ultima lectura: true si fue valida.
 */
bool Clock_IsValid(void);

#ifdef __cplusplus
}
#endif

#endif /* MID_CLOCK_H */
