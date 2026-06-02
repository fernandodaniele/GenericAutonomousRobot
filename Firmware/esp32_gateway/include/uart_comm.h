#ifndef UART_COMM_H
#define UART_COMM_H

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

/**
 * @brief Lee e imprime datos recibidos desde la STM32.
 */
void Uart_ReadResponse(void);

#endif /* UART_COMM_H */
