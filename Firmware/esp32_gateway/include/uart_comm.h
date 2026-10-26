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

/**
 * @brief Envía una línea a la STM32 (agrega '\n' al final).
 * @param text Texto sin terminador de línea.
 */
void Uart_SendLine(const char *text);

/**
 * @brief Lee una línea completa enviada por la STM32 (hasta '\n', ignora '\r').
 * @param line Buffer destino, queda terminado en '\0'.
 * @param size Tamaño del buffer destino.
 * @param timeout_ms Tiempo máximo de espera para la línea completa.
 * @return Largo de la línea leída, o -1 si venció el tiempo.
 */
int Uart_ReadLine(char *line, int size, int timeout_ms);

/**
 * @brief Descarta los bytes pendientes en el buffer de recepción.
 */
void Uart_Flush(void);

#endif /* UART_COMM_H */
