#ifndef MID_NAVIGATION_H_
#define MID_NAVIGATION_H_

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"

/* Provisionales: no representan distancia de frenado ni watchdog UART. */
#define NAV_FRONT_STOP_MM 300U
#define NAV_REAR_REVERSE_MIN_MM 300U
#define NAV_REVERSE_MAX_MS 300U
#define NAV_SESSION_MAX_MS 2000U

typedef enum {
    NAV_ESPERAR_DATOS = 0,
    NAV_AVANZAR,
    NAV_DETENIDO,
    NAV_RETROCEDER_ACOTADO
} NavigationState_t;

/* Begin solo al entrar efectivamente en AUTO; nunca al renovar consignas. */
void Navigation_Begin(TickType_t entered_tick);
/* Revoca sesion/permisos, sin escribir salidas; Stop del controlador lo invoca. */
void Navigation_Cancel(void);
/* Cancela solo la sesion indicada; una consigna vieja no cancela otra nueva. */
void Navigation_CancelSession(uint32_t session);
void Navigation_Step(void);
NavigationState_t Navigation_GetState(void);

/* Barreras internas del backend: permiso de emision de un solo uso y sesion
 * vigente tambien durante el consumo/PI. No consultan colas ni esperan. */
bool Navigation_ConsumeEmission(uint32_t session, TickType_t issued_tick, TickType_t now,
                                bool reversing);
bool Navigation_IsCommandCurrent(uint32_t session, TickType_t now);

#endif
