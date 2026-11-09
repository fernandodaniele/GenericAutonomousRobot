#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdbool.h>
typedef enum { UART_CONTROL_STOP, UART_CONTROL_RESET, UART_CONTROL_MANUAL } UartControlCommand;
/* Resultado de entrega al driver, no confirmacion de la STM32. Solo tareas. */
bool Uart_SendControl(UartControlCommand command);

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

void Uart_SendReset(void);

void Uart_SendStop(void);

void Uart_SendManual(void);

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
