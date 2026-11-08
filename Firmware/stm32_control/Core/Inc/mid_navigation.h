#ifndef MID_NAVIGATION_H_
#define MID_NAVIGATION_H_

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "mid_motion.h"

/* Solo compilacion. Requiere autorizacion fisica especifica; no hay mando UART. */
#ifndef NAV_EXPERIMENTAL_TURN_ENABLED
#define NAV_EXPERIMENTAL_TURN_ENABLED 0
#endif
#ifndef NAV_TURN_LEFT
#define NAV_TURN_LEFT 1
#endif
#if (NAV_EXPERIMENTAL_TURN_ENABLED != 0 && NAV_EXPERIMENTAL_TURN_ENABLED != 1) || \
    (NAV_TURN_LEFT != 0 && NAV_TURN_LEFT != 1)
#error "Navigation experimental options must be 0 or 1"
#endif

/* Provisionales: no representan distancia de frenado ni watchdog UART. */
#define NAV_FRONT_STOP_MM 300U
#define NAV_REAR_REVERSE_MIN_MM 300U
#define NAV_REVERSE_MAX_MS 300U
#define NAV_SESSION_MAX_MS 2000U
#define NAV_TURN_MAX_MS 250U
#if NAV_TURN_LEFT
#define NAV_TURN_LEFT_CPS (-MOTION_TURN_CPS)
#define NAV_TURN_RIGHT_CPS MOTION_TURN_CPS
#else
#define NAV_TURN_LEFT_CPS MOTION_TURN_CPS
#define NAV_TURN_RIGHT_CPS (-MOTION_TURN_CPS)
#endif

typedef enum {
    NAV_ESPERAR_DATOS = 0,
    NAV_AVANZAR,
    NAV_DETENIDO,
    NAV_RETROCEDER_ACOTADO,
    NAV_GIRAR_ACOTADO
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
                                NavigationState_t phase);
bool Navigation_IsCommandCurrent(uint32_t session, TickType_t now);

#if NAV_EXPERIMENTAL_TURN_ENABLED
/* Acuse del backend despues de aplicar ambos objetivos. No acredita recorrido. */
void Navigation_CommandApplied(uint32_t session, TickType_t now);
/* Fin normal del retroceso: deja PWM a cero, pendiente de evaluar el giro.
 * No se usa para timeout de consigna, STOP, Safety o cambio de modo. */
bool Navigation_CompleteCommand(uint32_t session, TickType_t now);
#endif

#endif
