/*
 * drv_TCRT5000.h
 *
 *  Created on: Apr 20, 2026
 *      Author: Lucas
 */

#ifndef INC_DRV_TCRT5000_H_
#define INC_DRV_TCRT5000_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * Identificación lógica de cada sensor TCRT5000.
 */
typedef enum
{
    TCRT_DEL_IZQ = 0,
    TCRT_DEL_DER,
    TCRT_TRAS_IZQ,
    TCRT_TRAS_DER,
    TCRT_CANT_SENSORES
} TCRT_Sensor_t;

/*
 * Estado lógico del sensor.
 *
 * TCRT_DETECTA:
 *   El sensor detecta superficie/piso.
 *
 * TCRT_NO_DETECTA:
 *   El sensor no detecta superficie.
 *   En esta aplicación se interpreta como posible caída.
 */
typedef enum
{
    TCRT_NO_DETECTA = 0,
    TCRT_DETECTA
} TCRT_Estado_t;

/*
 * Estructura para agrupar la lectura de los cuatro sensores.
 */
typedef struct
{
    TCRT_Estado_t del_izq;
    TCRT_Estado_t del_der;
    TCRT_Estado_t tras_izq;
    TCRT_Estado_t tras_der;
} TCRT_Lecturas_t;

void TCRT_Init(void);

TCRT_Estado_t TCRT_LeerSensor(TCRT_Sensor_t sensor);
TCRT_Lecturas_t TCRT_LeerTodos(void);

uint8_t TCRT_HayCaidaAdelante(void);
uint8_t TCRT_HayCaidaAtras(void);
uint8_t TCRT_HayCaida(void);

#endif /* INC_DRV_TCRT5000_H_ */
