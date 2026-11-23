#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdbool.h>
#include <stdint.h>
typedef enum { UART_CONTROL_STOP, UART_CONTROL_RESET, UART_CONTROL_MANUAL, UART_CONTROL_AUTO, UART_CONTROL_AUTO_TURN, UART_CONTROL_AUTO_EVADE, UART_CONTROL_AUTO_CONTINUOUS, UART_CONTROL_AUTO_EXPLORE } UartControlCommand;
static inline bool Uart_ControlIsAuto(UartControlCommand c) {
    return c==UART_CONTROL_AUTO || c==UART_CONTROL_AUTO_TURN || c==UART_CONTROL_AUTO_EVADE || c==UART_CONTROL_AUTO_CONTINUOUS || c==UART_CONTROL_AUTO_EXPLORE;
}
/* Resultado de entrega al driver, no confirmacion de la STM32. Solo tareas. */
bool Uart_SendControl(UartControlCommand command);
typedef enum { UART_MOTION_FORWARD, UART_MOTION_REVERSE, UART_MOTION_LEFT, UART_MOTION_RIGHT } UartMotionCommand;
/* Solo app_main: usa permiso de una renovacion, comprobado dentro del turno TX. */
bool Uart_SendLeasedMotion(UartMotionCommand command, uint64_t generation, uint32_t sequence);
bool Uart_SendPendingHalt(void);

/**
 * @file uart_comm.h
 * @brief Módulo de comunicación UART con STM32.
 */

/**
 * @brief Inicializa la UART utilizada para comunicarse con la STM32.
 */
void Uart_Init(void);

/**
 * @brief Envía el comando PING hacia la STM32.
 */
void Uart_SendPing(void);

void Uart_SendStatus(void);
/* Solo app_main; se omite durante controles/sesiones y sin acumular consultas. */
void Uart_PollSensors(void);
void Uart_PollMapping(void);
void Uart_PollImu(void);
void Uart_PollImuFusion(void);
void Uart_PollNavigation(void);
void Uart_PollExploration(void);
bool Uart_SendImuCalibration(void);

void Uart_SendReset(void);

void Uart_SendStop(void);

void Uart_SendManual(void);
void Uart_SendAuto(void);

void Uart_SendForward(void);
void Uart_SendReverse(void);
void Uart_SendLeft(void);
void Uart_SendRight(void);
void Uart_SendHalt(void);

/**
 * @brief Lee e imprime datos recibidos desde la STM32.
 */
void Uart_ReadResponse(void);

#endif /* UART_COMM_H */
