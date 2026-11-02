/*
 * drv_ultrasound.h
 *
 *  Created on: Feb 15, 2026
 *      Author: Fernando E. Daniele
 *  Updated on: Oct 25, 2026
 *      Soporte para 3 sensores HC-SR04 (Frontal, Izquierdo, Derecho)
 */

#ifndef CORE_INC_DRV_ULTRASOUND_H_
#define CORE_INC_DRV_ULTRASOUND_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/**
 * @brief Identificador de los sensores ultrasonicos del robot.
 */
typedef enum {
    ULTRASOUND_FRONT = 0,
    ULTRASOUND_LEFT,
    ULTRASOUND_RIGHT,
    ULTRASOUND_COUNT
} UltrasoundIndex_t;

/**
 * @brief Motivo de una lectura de ultrasonido.
 *        Permite distinguir "sin obstaculo" de "obstaculo pegado", que con
 *        Ultrasound_Read_mm() devuelven ambos 0.
 */
typedef enum {
    ULTRASOUND_STATUS_OK = 0,     // Distancia valida (20 mm a 4000 mm)
    ULTRASOUND_STATUS_NO_ECHO,    // El ECHO nunca subio: sensor desconectado o falla
    ULTRASOUND_STATUS_FAR,        // Eco largo o fuera de rango: camino libre
    ULTRASOUND_STATUS_TOO_CLOSE   // Menos de 20 mm: obstaculo pegado
} UltrasoundStatus_t;

/**
 * @brief Estructura de control para un sensor HC-SR04.
 */
typedef struct {
    GPIO_TypeDef* trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef* echo_port;
    uint16_t echo_pin;
    TIM_HandleTypeDef* timer; // Timer base para medir microsegundos (ej. TIM2 a 1 MHz)
    uint32_t last_distance_mm;
} Ultrasound_t;

/**
 * @brief Inicializa un sensor individual.
 */
void Ultrasound_Init(Ultrasound_t* dev);

/**
 * @brief Realiza una lectura de distancia de un sensor y devuelve la distancia en milimetros.
 * @param dev Puntero a la estructura del sensor.
 * @return Distancia en mm. Retorna 0 si hay timeout o lectura fuera de rango.
 */
uint32_t Ultrasound_Read_mm(Ultrasound_t* dev);

/**
 * @brief Inicializa los 3 sensores ultrasonicos con sus pines asignados por defecto.
 *        Frontal:   TRIG=PB4, ECHO=PB5
 *        Izquierdo: TRIG=PE7, ECHO=PB0
 *        Derecho:   TRIG=PE8, ECHO=PB1
 * @param timer Puntero al temporizador base (ej. &htim2 configurado a 1 MHz).
 */
void Ultrasound_InitAll(TIM_HandleTypeDef* timer);

/**
 * @brief Obtiene el descriptor de uno de los 3 sensores.
 * @param index Identificador del sensor (ULTRASOUND_FRONT, ULTRASOUND_LEFT, ULTRASOUND_RIGHT).
 * @return Puntero a la estructura Ultrasound_t del sensor solicitado.
 */
Ultrasound_t* Ultrasound_GetDev(UltrasoundIndex_t index);

/**
 * @brief Realiza la lectura del sensor especificado por indice.
 * @param index Identificador del sensor.
 * @return Distancia en mm. Retorna 0 en caso de error o fuera de rango.
 */
uint32_t Ultrasound_ReadIndex_mm(UltrasoundIndex_t index);

/**
 * @brief Realiza una lectura e informa el motivo del resultado.
 *        No modifica el comportamiento de Ultrasound_Read_mm().
 * @param dev Puntero a la estructura del sensor.
 * @param out_mm Distancia medida en mm (solo valida si el estado es OK;
 *               con TOO_CLOSE devuelve la medicion cruda). Puede ser NULL.
 * @return Motivo de la lectura.
 */
UltrasoundStatus_t Ultrasound_ReadStatus(Ultrasound_t* dev, uint32_t* out_mm);

/**
 * @brief Igual que Ultrasound_ReadStatus() pero por indice de sensor.
 */
UltrasoundStatus_t Ultrasound_ReadStatusIndex(UltrasoundIndex_t index, uint32_t* out_mm);

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_DRV_ULTRASOUND_H_ */
