/*
 * task_nav.h
 *
 *  Created on: Nov 2, 2026
 *      Author: Grupo 2
 *
 *  Tarea de navegacion: cada 20 ms lee la foto de sensores, ejecuta mid_nav y
 *  aplica las velocidades a los motores con rampa y freno activo.
 *  Se comunica con el resto solo por colas y nunca se bloquea.
 */

#ifndef CORE_INC_TASK_NAV_H_
#define CORE_INC_TASK_NAV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "mid_nav.h"

#define TASK_NAV_PERIOD_MS        20
#define TASK_NAV_STALE_MS         300   // Foto mas vieja que esto: motores detenidos
#define TASK_NAV_RAMP_STEP_PWM    600   // Bajada maxima de PWM por periodo (frenar o invertir es rapido)

/* Arranque suave: desde parado la rueda salta a TASK_NAV_SOFT_START_PWM (por
 * debajo del PWM con el que empieza a girar) y sube de a
 * TASK_NAV_RAMP_UP_STEP_PWM cada 20 ms. Con los valores actuales llega al
 * 70 % en casi 1 s en lugar de dar un golpe. */
#ifndef TASK_NAV_SOFT_START_PWM
#define TASK_NAV_SOFT_START_PWM   4000
#endif
#ifndef TASK_NAV_RAMP_UP_STEP_PWM
#define TASK_NAV_RAMP_UP_STEP_PWM 40    // antes 100 (0,4 s); 40 = rampa de ~1 s
#endif

/* Arranque automatico: 1 = el robot arranca solo al encenderse, sin esperar
 * START del ESP32 (STOP por UART sigue funcionando). 0 = espera START. */
#ifndef TASK_NAV_AUTOSTART
#define TASK_NAV_AUTOSTART        1
#endif
#define TASK_NAV_AUTOSTART_DELAY_MS 3000  // Espera para alejar las manos del robot

typedef struct {
    uint32_t timestamp_ms;
    uint32_t events;   // Mascara NAV_EVT_*
} NavEventMsg_t;

typedef struct {
    NavState_t state;
    int16_t pwm_left;
    int16_t pwm_right;
    bool brake;           // Freno activo en este ciclo
    int32_t x_mm;        // Pose estimada desde el arranque (sin encoders: aproximada)
    int32_t y_mm;
    int32_t theta_mrad;
} NavStatus_t;

/**
 * @brief Crea la tarea de navegacion y sus colas. Llamar antes de osKernelStart().
 */
void TaskNav_Create(void);

/**
 * @brief Encola un comando START/STOP (no bloquea; si la cola esta llena se descarta).
 * @return true si se encolo.
 */
bool TaskNav_SendCommand(NavCommand_t cmd);

/**
 * @brief Saca un evento pendiente (no bloquea).
 * @return true si habia un evento.
 */
bool TaskNav_GetEvent(NavEventMsg_t* out);

/**
 * @brief Ultimo estado y velocidades aplicadas (para telemetria).
 */
NavStatus_t TaskNav_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_TASK_NAV_H_ */
