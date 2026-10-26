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

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_DRV_ULTRASOUND_H_ */
