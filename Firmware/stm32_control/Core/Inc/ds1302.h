/**
 * @file ds1302.h
 * @brief Archivo de cabecera para el driver del RTC DS1302.
 *
 * Este archivo contiene las definiciones públicas necesarias para utilizar
 * el RTC DS1302. Incluye constantes, tipos de datos, estructuras de
 * configuración y prototipos de funciones.
 *
 * El DS1302 es un reloj de tiempo real que mantiene segundos, minutos,
 * horas, día, mes y año mediante una alimentación auxiliar. Utiliza un
 * protocolo propio de tres líneas: RST, CLK e IO.
 *
 * CLK = PA5
 * DAT = PA7
 * RST = PE3
 *
 * @author Lucas
 * @date 28/04/26
 */

#ifndef DS1302_H_
#define DS1302_H_

/******************************************************************************/
/*                               Inclusiones                                   */
/******************************************************************************/

#include "stm32f4xx_hal.h"

/******************************************************************************/
/*                          Constantes simbólicas                              */
/******************************************************************************/

/**
 * @brief Habilita el uso de nombres abreviados en español para días y meses.
 */
#define DS1302_USE_SPANISH_LANGUAGE

#ifdef DS1302_USE_SPANISH_LANGUAGE

/**
 * @brief Nombres abreviados de los días de la semana.
 */
#define DS1302_MONDAY              "Lun"
#define DS1302_TUESDAY             "Mar"
#define DS1302_WEDNESDAY           "Mie"
#define DS1302_THURSDAY            "Jue"
#define DS1302_FRIDAY              "Vie"
#define DS1302_SATURDAY            "Sab"
#define DS1302_SUNDAY              "Dom"

/**
 * @brief Nombres abreviados de los meses del año.
 */
#define DS1302_JANUARY             "Ene"
#define DS1302_FEBRUARY            "Feb"
#define DS1302_MARCH               "Mar"
#define DS1302_APRIL               "Abr"
#define DS1302_MAY                 "May"
#define DS1302_JUNE                "Jun"
#define DS1302_JULY                "Jul"
#define DS1302_AUGUST              "Ago"
#define DS1302_SEPTEMBER           "Sep"
#define DS1302_OCTOBER             "Oct"
#define DS1302_NOVEMBER            "Nov"
#define DS1302_DECEMBER            "Dic"

#endif

/**
 * @brief Direcciones internas de los registros del DS1302.
 */
#define DS1302_SECONDS_REGISTER            (0x80U)
#define DS1302_MINUTES_REGISTER            (0x82U)
#define DS1302_HOURS_REGISTER              (0x84U)
#define DS1302_DATE_REGISTER               (0x86U)
#define DS1302_MONTH_REGISTER              (0x88U)
#define DS1302_DAY_REGISTER                (0x8AU)
#define DS1302_YEAR_REGISTER               (0x8CU)

/**
 * @brief Comandos de lectura y escritura en modo burst del reloj.
 */
#define DS1302_CLOCK_BURST_WRITE_COMMAND   (0xBEU)
#define DS1302_CLOCK_BURST_READ_COMMAND    (0xBFU)

/**
 * @brief Direcciones y comandos asociados a la memoria RAM interna.
 */
#define DS1302_RAM_START_ADDRESS           (0xC0U)
#define DS1302_RAM_END_ADDRESS             (0xFCU)
#define DS1302_RAM_BURST_WRITE_COMMAND     (0xFEU)
#define DS1302_RAM_BURST_READ_COMMAND      (0xFFU)

/**
 * @brief Registros de control del DS1302.
 */
#define DS1302_ENABLE_REGISTER             (0x8EU)
#define DS1302_TRICKLE_REGISTER            (0x90U)

/**
 * @brief Valor máximo utilizado para controlar timeout de comunicación.
 */
#define DS1302_TIMEOUT_MAX                 (100U)

/******************************************************************************/
/*                              Tipos de datos                                 */
/******************************************************************************/

/**
 * @brief Estados posibles de retorno del driver DS1302.
 */
typedef enum {
    DS1302_ERROR = 0,      /**< Error general en la operación. */
    DS1302_OK,            /**< Operación realizada correctamente. */
    DS1302_TIMEOUT        /**< Timeout durante la comunicación. */
} Ds1302Error_t;

/**
 * @brief Representa los registros internos del RTC en formato BCD.
 *
 * Esta estructura replica la organización interna de los registros de tiempo
 * del DS1302. Se utilizan uniones y campos de bits para acceder tanto al byte
 * completo como a cada campo individual.
 */
typedef struct {
    union {
        uint8_t reg;
        struct {
            uint8_t seconds_units:4;      /**< Unidad de segundos. */
            uint8_t seconds_tens:3;       /**< Decena de segundos. */
            uint8_t clock_halt:1;         /**< Bit que detiene o habilita el reloj. */
        } bits;
    } seconds;

    union {
        uint8_t reg;
        struct {
            uint8_t minutes_units:4;      /**< Unidad de minutos. */
            uint8_t minutes_tens:3;       /**< Decena de minutos. */
            uint8_t reserved:1;           /**< Bit reservado. */
        } bits;
    } minutes;

    union {
        union {
            uint8_t reg;
            struct {
                uint8_t hour_units:4;     /**< Unidad de horas en formato 24 h. */
                uint8_t hour_tens:2;      /**< Decena de horas en formato 24 h. */
                uint8_t reserved:1;       /**< Bit reservado. */
                uint8_t hour_format:1;    /**< Selección de formato horario. */
            } bits;
        } hour_24;

        union {
            uint8_t reg;
            struct {
                uint8_t hour_units:4;     /**< Unidad de horas en formato 12 h. */
                uint8_t hour_tens:1;      /**< Decena de horas en formato 12 h. */
                uint8_t am_pm:1;          /**< Indicador AM/PM. */
                uint8_t reserved:1;       /**< Bit reservado. */
                uint8_t hour_format:1;    /**< Selección de formato horario. */
            } bits;
        } hour_12;
    } hour;

    union {
        uint8_t reg;
        struct {
            uint8_t month_day_units:4;    /**< Unidad del día del mes. */
            uint8_t month_day_tens:2;     /**< Decena del día del mes. */
            uint8_t reserved:2;           /**< Bits reservados. */
        } bits;
    } month_day;

    union {
        uint8_t reg;
        struct {
            uint8_t month_units:4;        /**< Unidad del mes. */
            uint8_t month_tens:1;         /**< Decena del mes. */
            uint8_t reserved:3;           /**< Bits reservados. */
        } bits;
    } month;

    union {
        uint8_t reg;
        struct {
            uint8_t day:3;                /**< Día de la semana en rango 1 a 7. */
            uint8_t reserved:5;           /**< Bits reservados. */
        } bits;
    } weekday;

    union {
        uint8_t reg;
        struct {
            uint8_t year_units:4;         /**< Unidad del año. */
            uint8_t year_tens:4;          /**< Decena del año. */
        } bits;
    } year;

    union {
        uint8_t reg;
        struct {
            uint8_t reserved:7;           /**< Bits reservados. */
            uint8_t write_protect:1;      /**< Bit de protección contra escritura. */
        } bits;
    } write_protect;
} RtcRegister_t;

/**
 * @brief Configuración de un pin GPIO utilizado por el driver.
 */
typedef struct {
    GPIO_TypeDef *mcu_port;               /**< Puerto GPIO asociado al pin. */
    uint16_t pin;                         /**< Pin GPIO asociado. */
} Ds1302GpioConfig_t;

/**
 * @brief Configuración general del driver DS1302.
 */
typedef struct {
    Ds1302GpioConfig_t reset_pin;         /**< Pin RST utilizado para habilitar comunicación. */
} Ds1302Config_t;

/**
 * @brief Estructura de fecha y hora en formato amigable para el usuario.
 */
typedef struct {
    uint8_t seconds;                      /**< Segundos actuales. */
    uint8_t hours;                        /**< Horas actuales. */
    uint8_t minutes;                      /**< Minutos actuales. */
    uint8_t month_day;                    /**< Día del mes actual. */
    const char *month;                    /**< Mes actual en formato texto. */
    const char *weekday;                  /**< Día de la semana en formato texto. */
    const char *am_pm;                    /**< Estado AM o PM. */
    uint16_t year;                        /**< Año actual. */
} Ds1302DateTime_t;

/**
 * @brief Datos internos utilizados por el driver.
 */
typedef struct {
    uint32_t delay_ticks;                 /**< Cantidad de ticks utilizados para retardos internos. */
    RtcRegister_t received_data;          /**< Datos recibidos desde el DS1302. */
    RtcRegister_t transmit_data;          /**< Datos a transmitir hacia el DS1302. */
    Ds1302DateTime_t date_time;           /**< Fecha y hora convertida a formato de usuario. */
} Ds1302Data_t;

/**
 * @brief Objeto principal del driver DS1302.
 */
typedef struct {
    Ds1302Config_t config;                /**< Configuración de hardware del driver. */
    Ds1302Data_t data;                    /**< Datos internos del driver. */
} Ds1302_t;

/******************************************************************************/
/*                         Prototipos de funciones                             */
/******************************************************************************/

/**
 * @brief Inicializa el driver DS1302.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 * @param config Puntero a la estructura de configuración del driver.
 *
 * @return Estado de la operación.
 */
Ds1302Error_t DS1302_Init(Ds1302_t *ds1302, const Ds1302Config_t *config);

/**
 * @brief Escribe una secuencia de bytes en el DS1302.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 * @param data Puntero al arreglo de datos a transmitir.
 * @param size Cantidad de bytes a transmitir.
 *
 * @return Estado de la operación.
 */
Ds1302Error_t DS1302_Write(const Ds1302_t *ds1302, const uint8_t *data, uint8_t size);

/**
 * @brief Configura la fecha y hora del DS1302.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 * @param hour_format Formato horario utilizado.
 * @param hours Valor de horas.
 * @param minutes Valor de minutos.
 * @param seconds Valor de segundos.
 * @param am_pm Indicador AM o PM.
 * @param weekday Día de la semana.
 * @param month_day Día del mes.
 * @param month Mes.
 * @param year Año.
 *
 * @return Estado de la operación.
 */
Ds1302Error_t DS1302_SetTime(Ds1302_t *ds1302,
                             uint8_t hour_format,
                             uint8_t hours,
                             uint8_t minutes,
                             uint8_t seconds,
                             uint8_t am_pm,
                             uint8_t weekday,
                             uint8_t month_day,
                             uint8_t month,
                             int year);

/**
 * @brief Lee una cantidad determinada de bytes desde un registro del DS1302.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 * @param register_address Dirección del registro a leer.
 * @param buffer Puntero al buffer donde se almacenarán los datos recibidos.
 * @param n_bytes Cantidad de bytes a leer.
 *
 * @return Estado de la operación.
 */
Ds1302Error_t DS1302_Read(Ds1302_t *ds1302,
                          uint8_t register_address,
                          uint8_t *buffer,
                          uint8_t n_bytes);

/**
 * @brief Actualiza la estructura de fecha y hora del driver.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Estado de la operación.
 */
Ds1302Error_t DS1302_UpdateDateTime(Ds1302_t *ds1302);

/**
 * @brief Obtiene la unidad de segundos.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Unidad de segundos.
 */
uint8_t DS1302_GetSecondsUnits(Ds1302_t *ds1302);

/**
 * @brief Obtiene la decena de segundos.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Decena de segundos.
 */
uint8_t DS1302_GetSecondsTens(Ds1302_t *ds1302);

/**
 * @brief Obtiene la unidad de minutos.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Unidad de minutos.
 */
uint8_t DS1302_GetMinutesUnits(Ds1302_t *ds1302);

/**
 * @brief Obtiene la decena de minutos.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Decena de minutos.
 */
uint8_t DS1302_GetMinutesTens(Ds1302_t *ds1302);

/**
 * @brief Obtiene la unidad de horas.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Unidad de horas.
 */
uint8_t DS1302_GetHourUnits(Ds1302_t *ds1302);

/**
 * @brief Obtiene la decena de horas.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Decena de horas.
 */
uint8_t DS1302_GetHourTens(Ds1302_t *ds1302);

/**
 * @brief Obtiene el día de la semana en formato texto.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Puntero al texto del día de la semana.
 */
const char *DS1302_GetWeekDay(Ds1302_t *ds1302);

/**
 * @brief Obtiene el estado AM o PM.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Puntero al texto AM o PM.
 */
const char *DS1302_GetAmPmStatus(Ds1302_t *ds1302);

/**
 * @brief Obtiene el día del mes.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Día del mes.
 */
uint8_t DS1302_GetMonthDay(Ds1302_t *ds1302);

/**
 * @brief Obtiene el mes en formato texto.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Puntero al texto del mes.
 */
const char *DS1302_GetMonth(Ds1302_t *ds1302);

/**
 * @brief Obtiene el año.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Año actual.
 */
uint16_t DS1302_GetYear(Ds1302_t *ds1302);

#endif /* DS1302_H_ */
