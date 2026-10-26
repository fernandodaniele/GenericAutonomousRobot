/*
 * drv_ultrasound.c
 *
 *  Created on: Feb 15, 2026
 *      Author: Fernando E. Daniele
 *  Updated on: Oct 25, 2026
 *      Soporte e implementacion para 3 sensores HC-SR04 (Frontal, Izquierdo, Derecho)
 */

#include "drv_ultrasound.h"
#include "main.h"

#define HCSR04_TRIGGER_PULSE_US   10
#define HCSR04_WAIT_ECHO_TIMEOUT  5000   // 5 ms max para esperar flanco de subida
#define HCSR04_MAX_ECHO_US        30000  // 30 ms (~5.1 metros), timeout de eco
#define HCSR04_MIN_DIST_MM        20     // 2 cm
#define HCSR04_MAX_DIST_MM        4000   // 4 metros

/* Instancias internas de los 3 sensores */
static Ultrasound_t g_ultrasound_sensors[ULTRASOUND_COUNT];

/**
 * @brief Retardo bloqueante en microsegundos usando el contador del temporizador (1 MHz).
 */
static void delay_us(TIM_HandleTypeDef* htim, uint32_t us) {
    if (!htim) return;
    uint32_t start = __HAL_TIM_GET_COUNTER(htim);
    while ((__HAL_TIM_GET_COUNTER(htim) - start) < us) {
        __NOP();
    }
}

void Ultrasound_Init(Ultrasound_t* dev) {
    if (!dev) return;

    if (dev->timer) {
        HAL_TIM_Base_Start(dev->timer);
    }

    if (dev->trig_port) {
        HAL_GPIO_WritePin(dev->trig_port, dev->trig_pin, GPIO_PIN_RESET);
    }

    dev->last_distance_mm = 0;
}

uint32_t Ultrasound_Read_mm(Ultrasound_t* dev) {
    if (!dev || !dev->timer || !dev->trig_port || !dev->echo_port) {
        return 0;
    }

    // 1. Asegurar linea TRIG en bajo
    HAL_GPIO_WritePin(dev->trig_port, dev->trig_pin, GPIO_PIN_RESET);
    delay_us(dev->timer, 2);

    // 2. Enviar pulso de disparo de 10 us
    HAL_GPIO_WritePin(dev->trig_port, dev->trig_pin, GPIO_PIN_SET);
    delay_us(dev->timer, HCSR04_TRIGGER_PULSE_US);
    HAL_GPIO_WritePin(dev->trig_port, dev->trig_pin, GPIO_PIN_RESET);

    // 3. Esperar que la linea ECHO se ponga en alto (flanco de subida)
    uint32_t start_wait = __HAL_TIM_GET_COUNTER(dev->timer);
    while (HAL_GPIO_ReadPin(dev->echo_port, dev->echo_pin) == GPIO_PIN_RESET) {
        if ((__HAL_TIM_GET_COUNTER(dev->timer) - start_wait) > HCSR04_WAIT_ECHO_TIMEOUT) {
            return 0; // Sensor no respondio
        }
    }

    // 4. Medir duracion del pulso en alto (flanco de bajada)
    uint32_t echo_start = __HAL_TIM_GET_COUNTER(dev->timer);
    while (HAL_GPIO_ReadPin(dev->echo_port, dev->echo_pin) == GPIO_PIN_SET) {
        if ((__HAL_TIM_GET_COUNTER(dev->timer) - echo_start) > HCSR04_MAX_ECHO_US) {
            return 0; // Fuera de rango
        }
    }
    uint32_t echo_end = __HAL_TIM_GET_COUNTER(dev->timer);
    uint32_t pulse_duration_us = echo_end - echo_start;

    // Distancia en mm = (tiempo_us * 343) / 2000
    uint32_t distance_mm = (pulse_duration_us * 343) / 2000;

    if (distance_mm < HCSR04_MIN_DIST_MM || distance_mm > HCSR04_MAX_DIST_MM) {
        dev->last_distance_mm = 0;
        return 0;
    }

    dev->last_distance_mm = distance_mm;
    return distance_mm;
}

void Ultrasound_InitAll(TIM_HandleTypeDef* timer) {
    // Frontal: TRIG PB4, ECHO PB5
    g_ultrasound_sensors[ULTRASOUND_FRONT].trig_port = TRIG_FRONT_GPIO_Port;
    g_ultrasound_sensors[ULTRASOUND_FRONT].trig_pin  = TRIG_FRONT_Pin;
    g_ultrasound_sensors[ULTRASOUND_FRONT].echo_port = ECHO_FRONT_GPIO_Port;
    g_ultrasound_sensors[ULTRASOUND_FRONT].echo_pin  = ECHO_FRONT_Pin;
    g_ultrasound_sensors[ULTRASOUND_FRONT].timer     = timer;
    Ultrasound_Init(&g_ultrasound_sensors[ULTRASOUND_FRONT]);

    // Izquierdo: TRIG PE7, ECHO PB0
    g_ultrasound_sensors[ULTRASOUND_LEFT].trig_port = TRIG_LEFT_GPIO_Port;
    g_ultrasound_sensors[ULTRASOUND_LEFT].trig_pin  = TRIG_LEFT_Pin;
    g_ultrasound_sensors[ULTRASOUND_LEFT].echo_port = ECHO_LEFT_GPIO_Port;
    g_ultrasound_sensors[ULTRASOUND_LEFT].echo_pin  = ECHO_LEFT_Pin;
    g_ultrasound_sensors[ULTRASOUND_LEFT].timer     = timer;
    Ultrasound_Init(&g_ultrasound_sensors[ULTRASOUND_LEFT]);

    // Derecho: TRIG PE8, ECHO PB1
    g_ultrasound_sensors[ULTRASOUND_RIGHT].trig_port = TRIG_RIGHT_GPIO_Port;
    g_ultrasound_sensors[ULTRASOUND_RIGHT].trig_pin  = TRIG_RIGHT_Pin;
    g_ultrasound_sensors[ULTRASOUND_RIGHT].echo_port = ECHO_RIGHT_GPIO_Port;
    g_ultrasound_sensors[ULTRASOUND_RIGHT].echo_pin  = ECHO_RIGHT_Pin;
    g_ultrasound_sensors[ULTRASOUND_RIGHT].timer     = timer;
    Ultrasound_Init(&g_ultrasound_sensors[ULTRASOUND_RIGHT]);
}

Ultrasound_t* Ultrasound_GetDev(UltrasoundIndex_t index) {
    if (index >= ULTRASOUND_COUNT) {
        return NULL;
    }
    return &g_ultrasound_sensors[index];
}

uint32_t Ultrasound_ReadIndex_mm(UltrasoundIndex_t index) {
    if (index >= ULTRASOUND_COUNT) {
        return 0;
    }
    return Ultrasound_Read_mm(&g_ultrasound_sensors[index]);
}
