/*
 * drv_TCRT5000.c
 *
 * Driver para sensores infrarrojos TCRT5000.
 * En esta aplicación se utilizan cuatro sensores para detectar
 * si el robot tiene piso/superficie debajo o si existe una posible caída.
 *
 *  Created on: Apr 20, 2026
 *      Author: Lucas
 */

#include "drv_TCRT5000.h"

/* ==================== Configuración de hardware ==================== */

/*
 * Asociación entre cada sensor lógico y su pin físico.
 * Los pines deben estar configurados como GPIO_Input desde CubeMX.
 */

#define TCRT_DEL_IZQ_PORT     GPIOE
#define TCRT_DEL_IZQ_PIN      GPIO_PIN_7

#define TCRT_DEL_DER_PORT     GPIOE
#define TCRT_DEL_DER_PIN      GPIO_PIN_8

#define TCRT_TRAS_IZQ_PORT    GPIOE
#define TCRT_TRAS_IZQ_PIN     GPIO_PIN_9

#define TCRT_TRAS_DER_PORT    GPIOE
#define TCRT_TRAS_DER_PIN     GPIO_PIN_10

/*
 * Nivel lógico que indica detección de superficie.
 *
 * Si el módulo entrega un 1 lógico cuando detecta piso,
 * dejar GPIO_PIN_SET.
 *
 * Si el módulo entrega un 0 lógico cuando detecta piso,
 * cambiar a GPIO_PIN_RESET.
 */
#define TCRT_NIVEL_DETECCION  GPIO_PIN_RESET


/* ==================== Prototipos de funciones privadas ==================== */

/*
 * Convierte el estado eléctrico leído desde el GPIO
 * al estado lógico usado por el driver.
 */
static TCRT_Estado_t TCRT_ConvertirEstado(GPIO_PinState estado_pin);


/* ==================== Funciones públicas ==================== */

void TCRT_Init(void)
{
    /*
     * Los GPIO ya son inicializados por CubeMX en MX_GPIO_Init().
     * Esta función se conserva para mantener una interfaz uniforme
     * del driver, similar a otros módulos del proyecto.
     */
}

TCRT_Estado_t TCRT_LeerSensor(TCRT_Sensor_t sensor)
{
    GPIO_PinState estado_pin;

    /*
     * Según el sensor solicitado, se lee el pin correspondiente.
     */
    switch(sensor)
    {
        case TCRT_DEL_IZQ:
            estado_pin = HAL_GPIO_ReadPin(TCRT_DEL_IZQ_PORT, TCRT_DEL_IZQ_PIN);
            break;

        case TCRT_DEL_DER:
            estado_pin = HAL_GPIO_ReadPin(TCRT_DEL_DER_PORT, TCRT_DEL_DER_PIN);
            break;

        case TCRT_TRAS_IZQ:
            estado_pin = HAL_GPIO_ReadPin(TCRT_TRAS_IZQ_PORT, TCRT_TRAS_IZQ_PIN);
            break;

        case TCRT_TRAS_DER:
            estado_pin = HAL_GPIO_ReadPin(TCRT_TRAS_DER_PORT, TCRT_TRAS_DER_PIN);
            break;

        default:
            /*
             * Si se recibe un sensor inválido, se devuelve NO_DETECTA
             * como estado seguro, ya que en esta aplicación implica
             * posible caída.
             */
            return TCRT_NO_DETECTA;
    }

    /*
     * Se convierte la lectura eléctrica del pin al estado lógico
     * definido por el driver.
     */
    return TCRT_ConvertirEstado(estado_pin);
}

TCRT_Lecturas_t TCRT_LeerTodos(void)
{
    TCRT_Lecturas_t lecturas;

    /*
     * Se leen los cuatro sensores y se agrupan en una estructura.
     */
    lecturas.del_izq  = TCRT_LeerSensor(TCRT_DEL_IZQ);
    lecturas.del_der  = TCRT_LeerSensor(TCRT_DEL_DER);
    lecturas.tras_izq = TCRT_LeerSensor(TCRT_TRAS_IZQ);
    lecturas.tras_der = TCRT_LeerSensor(TCRT_TRAS_DER);

    return lecturas;
}

uint8_t TCRT_HayCaidaAdelante(void)
{
    TCRT_Lecturas_t lecturas = TCRT_LeerTodos();

    /*
     * Hay caída adelante si alguno de los sensores delanteros
     * no detecta superficie.
     */
    if ((lecturas.del_izq == TCRT_NO_DETECTA) ||
        (lecturas.del_der == TCRT_NO_DETECTA))
    {
        return 1U;
    }

    return 0U;
}

uint8_t TCRT_HayCaidaAtras(void)
{
    TCRT_Lecturas_t lecturas = TCRT_LeerTodos();

    /*
     * Hay caída atrás si alguno de los sensores traseros
     * no detecta superficie.
     */
    if ((lecturas.tras_izq == TCRT_NO_DETECTA) ||
        (lecturas.tras_der == TCRT_NO_DETECTA))
    {
        return 1U;
    }

    return 0U;
}

uint8_t TCRT_HayCaida(void)
{
    TCRT_Lecturas_t lecturas = TCRT_LeerTodos();

    /*
     * Hay caída general si cualquiera de los cuatro sensores
     * deja de detectar superficie.
     */
    if ((lecturas.del_izq  == TCRT_NO_DETECTA) ||
        (lecturas.del_der  == TCRT_NO_DETECTA) ||
        (lecturas.tras_izq == TCRT_NO_DETECTA) ||
        (lecturas.tras_der == TCRT_NO_DETECTA))
    {
        return 1U;
    }

    return 0U;
}


/* ==================== Funciones privadas ==================== */

static TCRT_Estado_t TCRT_ConvertirEstado(GPIO_PinState estado_pin)
{
    /*
     * Si el nivel leído coincide con el nivel definido como detección,
     * entonces el sensor está detectando piso/superficie.
     */
    if (estado_pin == TCRT_NIVEL_DETECCION)
    {
        return TCRT_DETECTA;
    }

    /*
     * Si no coincide, se interpreta que el sensor no detecta superficie.
     */
    return TCRT_NO_DETECTA;
}
