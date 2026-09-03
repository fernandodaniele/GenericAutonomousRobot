/**
 * @file ds1302.c
 * @brief Implementación del driver para el RTC DS1302.
 *
 * Este archivo implementa la comunicación con el RTC DS1302 mediante GPIO
 * usando la técnica de bit-banging. No utiliza periférico SPI por hardware.
 */

#include "ds1302.h"
#include <string.h>

/******************************************************************************/
/*                          Constantes privadas                                */
/******************************************************************************/

#define DS1302_24H_FORMAT              (0U)
#define DS1302_12H_FORMAT              (1U)

#define DS1302_SECONDS_MAX             (59U)
#define DS1302_MINUTES_MAX             (59U)
#define DS1302_WEEKDAY_MAX             (7U)
#define DS1302_YEAR_MAX                (2100U)
#define DS1302_HOUR_MAX                (23U)
#define DS1302_MONTH_DAY_MAX           (31U)
#define DS1302_MONTH_MAX               (12U)

#define DS1302_BURST_MAX_BYTES         (8U)
#define DS1302_MILLENNIUM              (2000U)
#define DS1302_12H_LIMIT               (12U)

#define DS1302_WEEK_DAYS_MAX           (8U)
#define DS1302_MONTHS_MAX              (13U)
#define DS1302_AM_PM_MAX               (2U)

#define DS1302_UNKNOWN                 ("Unknown")
#define DS1302_AM                      ("AM")
#define DS1302_PM                      ("PM")
#define DS1302_EMPTY                   ("  ")

#define DS1302_CLK_PORT                GPIOD
#define DS1302_CLK_PIN                 GPIO_PIN_8
#define DS1302_IO_PORT                 GPIOD
#define DS1302_IO_PIN                  GPIO_PIN_9

#define DS1302_BCD_TO_BIN(tens, units) (((tens) * 10U) + (units))
#define DS1302_BIN_TO_BCD_TENS(value)  ((value) / 10U)
#define DS1302_BIN_TO_BCD_UNITS(value) ((value) % 10U)

/******************************************************************************/
/*                           Variables privadas                                */
/******************************************************************************/

/**
 * @brief Tabla de días de la semana en formato texto.
 */
static const char *ds1302_days[DS1302_WEEK_DAYS_MAX] = {
    DS1302_UNKNOWN,
    DS1302_SUNDAY,
    DS1302_MONDAY,
    DS1302_TUESDAY,
    DS1302_WEDNESDAY,
    DS1302_THURSDAY,
    DS1302_FRIDAY,
    DS1302_SATURDAY
};

/**
 * @brief Tabla de meses del año en formato texto.
 */
static const char *ds1302_months[DS1302_MONTHS_MAX] = {
    DS1302_UNKNOWN,
    DS1302_JANUARY,
    DS1302_FEBRUARY,
    DS1302_MARCH,
    DS1302_APRIL,
    DS1302_MAY,
    DS1302_JUNE,
    DS1302_JULY,
    DS1302_AUGUST,
    DS1302_SEPTEMBER,
    DS1302_OCTOBER,
    DS1302_NOVEMBER,
    DS1302_DECEMBER
};

/**
 * @brief Tabla de estados AM y PM en formato texto.
 */
static const char *ds1302_am_pm[DS1302_AM_PM_MAX] = {
    DS1302_AM,
    DS1302_PM
};

/******************************************************************************/
/*                         Funciones privadas                                  */
/******************************************************************************/

/**
 * @brief Genera un pequeño retardo para temporizar la comunicación GPIO.
 *
 * Esta función inserta instrucciones NOP para cumplir con los tiempos mínimos
 * requeridos durante el intercambio de datos con el DS1302.
 */
static void DS1302_Delay(void) {
    for (volatile uint32_t index = 0U; index < 80U; index++) {
        __NOP();
    }
}

/**
 * @brief Configura el pin IO del DS1302 como salida.
 *
 * El pin IO es bidireccional. Esta función lo configura como salida push-pull
 * para transmitir bits desde el microcontrolador hacia el DS1302.
 */
static void DS1302_SetIoOutput(void) {
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = DS1302_IO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(DS1302_IO_PORT, &gpio_init);
}

/**
 * @brief Configura el pin IO del DS1302 como entrada.
 *
 * El pin IO es bidireccional. Esta función lo configura como entrada para
 * recibir bits enviados por el DS1302 hacia el microcontrolador.
 */
static void DS1302_SetIoInput(void) {
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = DS1302_IO_PIN;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(DS1302_IO_PORT, &gpio_init);
}

/**
 * @brief Escribe un byte en el DS1302 mediante GPIO.
 *
 * @param data Byte que se transmitirá hacia el DS1302.
 *
 * La transmisión se realiza bit a bit, comenzando por el bit menos
 * significativo. En cada bit se actualiza IO y luego se genera un pulso
 * de reloj por CLK.
 */
static void DS1302_WriteByteGpio(uint8_t data) {
    DS1302_SetIoOutput();

    for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++) {
        HAL_GPIO_WritePin(DS1302_IO_PORT,
                          DS1302_IO_PIN,
                          (data & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        DS1302_Delay();

        HAL_GPIO_WritePin(DS1302_CLK_PORT, DS1302_CLK_PIN, GPIO_PIN_SET);
        DS1302_Delay();

        HAL_GPIO_WritePin(DS1302_CLK_PORT, DS1302_CLK_PIN, GPIO_PIN_RESET);
        DS1302_Delay();

        data >>= 1U;
    }
}

/**
 * @brief Lee un byte desde el DS1302 mediante GPIO.
 *
 * @return Byte recibido desde el DS1302.
 *
 * La recepción se realiza bit a bit, comenzando por el bit menos
 * significativo. En cada ciclo se lee IO y luego se genera un pulso
 * de reloj por CLK.
 */
static uint8_t DS1302_ReadByteGpio(void) {
    uint8_t data = 0U;

    DS1302_SetIoInput();
    DS1302_Delay();

    for (uint8_t bit_index = 0U; bit_index < 8U; bit_index++) {
        if (HAL_GPIO_ReadPin(DS1302_IO_PORT, DS1302_IO_PIN) == GPIO_PIN_SET) {
            data |= (uint8_t)(1U << bit_index);
        }

        HAL_GPIO_WritePin(DS1302_CLK_PORT, DS1302_CLK_PIN, GPIO_PIN_SET);
        DS1302_Delay();

        HAL_GPIO_WritePin(DS1302_CLK_PORT, DS1302_CLK_PIN, GPIO_PIN_RESET);
        DS1302_Delay();
    }

    return data;
}

/**
 * @brief Activa la línea RST del DS1302.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * Al activar RST se habilita la comunicación con el DS1302.
 */
static void DS1302_SetChipEnable(const Ds1302_t *ds1302) {
    HAL_GPIO_WritePin(ds1302->config.reset_pin.mcu_port,
                      ds1302->config.reset_pin.pin,
                      GPIO_PIN_SET);
}

/**
 * @brief Desactiva la línea RST del DS1302.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * Al desactivar RST se finaliza la comunicación con el DS1302.
 */
static void DS1302_ResetChipEnable(const Ds1302_t *ds1302) {
    HAL_GPIO_WritePin(ds1302->config.reset_pin.mcu_port,
                      ds1302->config.reset_pin.pin,
                      GPIO_PIN_RESET);
}

/******************************************************************************/
/*                         Funciones públicas                                  */
/******************************************************************************/

/**
 * @brief Inicializa el driver DS1302.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 * @param config Puntero a la estructura de configuración del driver.
 *
 * @return Estado de la operación.
 *
 * Esta función limpia la estructura principal, copia la configuración de
 * hardware, coloca las líneas de comunicación en estado inicial y deshabilita
 * la protección de escritura y el cargador trickle.
 */
Ds1302Error_t DS1302_Init(Ds1302_t *ds1302, const Ds1302Config_t *config) {
    uint8_t write_protect_data[2] = {
        DS1302_ENABLE_REGISTER,
        0x00U
    };

    uint8_t trickle_data[2] = {
        DS1302_TRICKLE_REGISTER,
        0x00U
    };

    if ((ds1302 == NULL) || (config == NULL)) {
        return DS1302_ERROR;
    }

    memset(ds1302, 0, sizeof(Ds1302_t));

    ds1302->config.reset_pin = config->reset_pin;
    ds1302->data.delay_ticks = 0U;

    HAL_GPIO_WritePin(DS1302_CLK_PORT, DS1302_CLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_IO_PORT, DS1302_IO_PIN, GPIO_PIN_RESET);

    DS1302_ResetChipEnable(ds1302);
    DS1302_SetIoOutput();

    (void)DS1302_Write(ds1302, write_protect_data, 2U);
    HAL_Delay(1U);

    (void)DS1302_Write(ds1302, trickle_data, 2U);
    HAL_Delay(1U);

    return DS1302_OK;
}

/**
 * @brief Escribe una secuencia de bytes en el DS1302.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 * @param data Puntero al arreglo de datos a transmitir.
 * @param size Cantidad de bytes a transmitir.
 *
 * @return Estado de la operación.
 *
 * Esta función activa RST, transmite los bytes indicados por GPIO y luego
 * finaliza la comunicación dejando las líneas en estado seguro.
 */
Ds1302Error_t DS1302_Write(const Ds1302_t *ds1302, const uint8_t *data, uint8_t size) {
    if ((ds1302 == NULL) || (data == NULL) || (size == 0U)) {
        return DS1302_ERROR;
    }

    HAL_GPIO_WritePin(DS1302_CLK_PORT, DS1302_CLK_PIN, GPIO_PIN_RESET);

    DS1302_SetChipEnable(ds1302);
    DS1302_Delay();

    for (uint8_t byte_index = 0U; byte_index < size; byte_index++) {
        DS1302_WriteByteGpio(data[byte_index]);
    }

    DS1302_ResetChipEnable(ds1302);
    DS1302_SetIoOutput();

    HAL_GPIO_WritePin(DS1302_IO_PORT, DS1302_IO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_CLK_PORT, DS1302_CLK_PIN, GPIO_PIN_RESET);

    return DS1302_OK;
}

/**
 * @brief Lee una cantidad determinada de bytes desde un registro del DS1302.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 * @param register_address Dirección del registro a leer.
 * @param buffer Puntero al buffer donde se almacenarán los datos recibidos.
 * @param n_bytes Cantidad de bytes a leer.
 *
 * @return Estado de la operación.
 *
 * Esta función fuerza el bit menos significativo de la dirección en uno para
 * indicar operación de lectura, envía la dirección y luego recibe los bytes
 * solicitados desde el DS1302.
 */
Ds1302Error_t DS1302_Read(Ds1302_t *ds1302,
                          uint8_t register_address,
                          uint8_t *buffer,
                          uint8_t n_bytes) {
    if ((ds1302 == NULL) || (buffer == NULL) || (n_bytes == 0U)) {
        return DS1302_ERROR;
    }

    register_address |= 0x01U;

    HAL_GPIO_WritePin(DS1302_CLK_PORT, DS1302_CLK_PIN, GPIO_PIN_RESET);

    DS1302_SetChipEnable(ds1302);
    DS1302_Delay();

    DS1302_WriteByteGpio(register_address);

    for (uint8_t byte_index = 0U; byte_index < n_bytes; byte_index++) {
        buffer[byte_index] = DS1302_ReadByteGpio();
    }

    DS1302_ResetChipEnable(ds1302);
    DS1302_SetIoOutput();

    HAL_GPIO_WritePin(DS1302_IO_PORT, DS1302_IO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_CLK_PORT, DS1302_CLK_PIN, GPIO_PIN_RESET);

    return DS1302_OK;
}

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
 *
 * Esta función valida los valores recibidos, los convierte a formato BCD y
 * los escribe en el DS1302 utilizando el modo clock burst.
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
                             int year) {
    uint8_t transmit_buffer[9];

    if (ds1302 == NULL) {
        return DS1302_ERROR;
    }

    if ((seconds > DS1302_SECONDS_MAX) ||
        (minutes > DS1302_MINUTES_MAX) ||
        (hours > DS1302_HOUR_MAX) ||
        (weekday == 0U) ||
        (weekday > DS1302_WEEKDAY_MAX) ||
        (month_day == 0U) ||
        (month_day > DS1302_MONTH_DAY_MAX) ||
        (month == 0U) ||
        (month > DS1302_MONTH_MAX) ||
        (year < DS1302_MILLENNIUM) ||
        (year > DS1302_YEAR_MAX)) {
        return DS1302_ERROR;
    }

    transmit_buffer[0] = DS1302_CLOCK_BURST_WRITE_COMMAND;

    memset(&ds1302->data.transmit_data, 0, sizeof(RtcRegister_t));

    ds1302->data.transmit_data.seconds.bits.seconds_tens = DS1302_BIN_TO_BCD_TENS(seconds);
    ds1302->data.transmit_data.seconds.bits.seconds_units = DS1302_BIN_TO_BCD_UNITS(seconds);
    ds1302->data.transmit_data.seconds.bits.clock_halt = 0U;

    ds1302->data.transmit_data.minutes.bits.minutes_tens = DS1302_BIN_TO_BCD_TENS(minutes);
    ds1302->data.transmit_data.minutes.bits.minutes_units = DS1302_BIN_TO_BCD_UNITS(minutes);

    if ((hour_format == DS1302_12H_FORMAT) && (hours <= DS1302_12H_LIMIT)) {
        ds1302->data.transmit_data.hour.hour_12.bits.hour_tens = DS1302_BIN_TO_BCD_TENS(hours);
        ds1302->data.transmit_data.hour.hour_12.bits.hour_units = DS1302_BIN_TO_BCD_UNITS(hours);
        ds1302->data.transmit_data.hour.hour_12.bits.am_pm = (am_pm != 0U) ? 1U : 0U;
        ds1302->data.transmit_data.hour.hour_12.bits.hour_format = DS1302_12H_FORMAT;
    } else {
        ds1302->data.transmit_data.hour.hour_24.bits.hour_tens = DS1302_BIN_TO_BCD_TENS(hours);
        ds1302->data.transmit_data.hour.hour_24.bits.hour_units = DS1302_BIN_TO_BCD_UNITS(hours);
        ds1302->data.transmit_data.hour.hour_24.bits.hour_format = DS1302_24H_FORMAT;
    }

    ds1302->data.transmit_data.month_day.bits.month_day_tens = DS1302_BIN_TO_BCD_TENS(month_day);
    ds1302->data.transmit_data.month_day.bits.month_day_units = DS1302_BIN_TO_BCD_UNITS(month_day);

    ds1302->data.transmit_data.month.bits.month_tens = DS1302_BIN_TO_BCD_TENS(month);
    ds1302->data.transmit_data.month.bits.month_units = DS1302_BIN_TO_BCD_UNITS(month);

    ds1302->data.transmit_data.weekday.bits.day = weekday;

    ds1302->data.transmit_data.year.bits.year_tens =
        DS1302_BIN_TO_BCD_TENS((uint16_t)year - DS1302_MILLENNIUM);

    ds1302->data.transmit_data.year.bits.year_units =
        DS1302_BIN_TO_BCD_UNITS((uint16_t)year - DS1302_MILLENNIUM);

    ds1302->data.transmit_data.write_protect.bits.write_protect = 0U;

    memcpy(&transmit_buffer[1], &ds1302->data.transmit_data, DS1302_BURST_MAX_BYTES);

    return DS1302_Write(ds1302, transmit_buffer, sizeof(transmit_buffer));
}

/**
 * @brief Actualiza la fecha y hora almacenada en el objeto del driver.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Estado de la operación.
 *
 * Esta función lee los registros del DS1302 en modo burst, convierte los
 * valores BCD a decimal y actualiza la estructura de fecha y hora amigable
 * para el usuario.
 */
Ds1302Error_t DS1302_UpdateDateTime(Ds1302_t *ds1302) {
    uint8_t month;
    uint8_t weekday;

    if (ds1302 == NULL) {
        return DS1302_ERROR;
    }

    if (DS1302_Read(ds1302,
                    DS1302_CLOCK_BURST_READ_COMMAND,
                    (uint8_t *)&ds1302->data.received_data,
                    DS1302_BURST_MAX_BYTES) != DS1302_OK) {
        return DS1302_ERROR;
    }

    month = DS1302_BCD_TO_BIN(ds1302->data.received_data.month.bits.month_tens,
                              ds1302->data.received_data.month.bits.month_units);

    weekday = ds1302->data.received_data.weekday.bits.day;

    if (month >= DS1302_MONTHS_MAX) {
        month = 0U;
    }

    if (weekday >= DS1302_WEEK_DAYS_MAX) {
        weekday = 0U;
    }

    if (ds1302->data.received_data.hour.hour_24.bits.hour_format == DS1302_24H_FORMAT) {
        ds1302->data.date_time.hours =
            DS1302_BCD_TO_BIN(ds1302->data.received_data.hour.hour_24.bits.hour_tens,
                              ds1302->data.received_data.hour.hour_24.bits.hour_units);

        ds1302->data.date_time.am_pm = DS1302_EMPTY;
    } else {
        ds1302->data.date_time.hours =
            DS1302_BCD_TO_BIN(ds1302->data.received_data.hour.hour_12.bits.hour_tens,
                              ds1302->data.received_data.hour.hour_12.bits.hour_units);

        ds1302->data.date_time.am_pm =
            ds1302_am_pm[ds1302->data.received_data.hour.hour_12.bits.am_pm ? 1U : 0U];
    }

    ds1302->data.date_time.minutes =
        DS1302_BCD_TO_BIN(ds1302->data.received_data.minutes.bits.minutes_tens,
                          ds1302->data.received_data.minutes.bits.minutes_units);

    ds1302->data.date_time.seconds =
        DS1302_BCD_TO_BIN(ds1302->data.received_data.seconds.bits.seconds_tens,
                          ds1302->data.received_data.seconds.bits.seconds_units);

    ds1302->data.date_time.weekday = ds1302_days[weekday];
    ds1302->data.date_time.month = ds1302_months[month];

    ds1302->data.date_time.month_day =
        DS1302_BCD_TO_BIN(ds1302->data.received_data.month_day.bits.month_day_tens,
                          ds1302->data.received_data.month_day.bits.month_day_units);

    ds1302->data.date_time.year =
        DS1302_BCD_TO_BIN(ds1302->data.received_data.year.bits.year_tens,
                          ds1302->data.received_data.year.bits.year_units) + DS1302_MILLENNIUM;

    return DS1302_OK;
}

/**
 * @brief Obtiene la unidad de segundos.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Unidad de segundos.
 */
uint8_t DS1302_GetSecondsUnits(Ds1302_t *ds1302) {
    return ds1302->data.received_data.seconds.bits.seconds_units;
}

/**
 * @brief Obtiene la decena de segundos.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Decena de segundos.
 */
uint8_t DS1302_GetSecondsTens(Ds1302_t *ds1302) {
    return ds1302->data.received_data.seconds.bits.seconds_tens;
}

/**
 * @brief Obtiene la unidad de minutos.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Unidad de minutos.
 */
uint8_t DS1302_GetMinutesUnits(Ds1302_t *ds1302) {
    return ds1302->data.received_data.minutes.bits.minutes_units;
}

/**
 * @brief Obtiene la decena de minutos.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Decena de minutos.
 */
uint8_t DS1302_GetMinutesTens(Ds1302_t *ds1302) {
    return ds1302->data.received_data.minutes.bits.minutes_tens;
}

/**
 * @brief Obtiene la unidad de horas.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Unidad de horas.
 */
uint8_t DS1302_GetHourUnits(Ds1302_t *ds1302) {
    if (ds1302->data.received_data.hour.hour_24.bits.hour_format == DS1302_24H_FORMAT) {
        return ds1302->data.received_data.hour.hour_24.bits.hour_units;
    }

    return ds1302->data.received_data.hour.hour_12.bits.hour_units;
}

/**
 * @brief Obtiene la decena de horas.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Decena de horas.
 */
uint8_t DS1302_GetHourTens(Ds1302_t *ds1302) {
    if (ds1302->data.received_data.hour.hour_24.bits.hour_format == DS1302_24H_FORMAT) {
        return ds1302->data.received_data.hour.hour_24.bits.hour_tens;
    }

    return ds1302->data.received_data.hour.hour_12.bits.hour_tens;
}

/**
 * @brief Obtiene el estado AM o PM.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Puntero al texto AM, PM o vacío si se usa formato 24 h.
 */
const char *DS1302_GetAmPmStatus(Ds1302_t *ds1302) {
    return ds1302->data.date_time.am_pm;
}

/**
 * @brief Obtiene el día del mes.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Día del mes.
 */
uint8_t DS1302_GetMonthDay(Ds1302_t *ds1302) {
    return ds1302->data.date_time.month_day;
}

/**
 * @brief Obtiene el mes en formato texto.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Puntero al texto del mes.
 */
const char *DS1302_GetMonth(Ds1302_t *ds1302) {
    return ds1302->data.date_time.month;
}

/**
 * @brief Obtiene el año.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Año actual.
 */
uint16_t DS1302_GetYear(Ds1302_t *ds1302) {
    return ds1302->data.date_time.year;
}

/**
 * @brief Obtiene el día de la semana en formato texto.
 *
 * @param ds1302 Puntero al objeto principal del driver.
 *
 * @return Puntero al texto del día de la semana.
 */
const char *DS1302_GetWeekDay(Ds1302_t *ds1302) {
    return ds1302->data.date_time.weekday;
}
