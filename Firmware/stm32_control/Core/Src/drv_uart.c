/*
 * drv_uart.c
 *
 *  Created on: Apr 28, 2026
 *      Author: Grupo 1 y 2
 */

#include "drv_uart.h"
#include <string.h>

/**
 * @brief Inicializa la estructura del driver UART.
 * @param uart Puntero a la estructura del driver.
 * @param huart Puntero al handle UART generado por HAL.
 */
void DrvUart_Init(DrvUart_t *uart, UART_HandleTypeDef *huart) {
    uart->huart = huart;
    uart->rx_byte = 0;
    uart->rx_index = 0;
    uart->message_ready = 0;
    memset(uart->rx_buffer, 0, UART_RX_BUFFER_SIZE);
}

/**
 * @brief Inicia la recepción UART por interrupción byte a byte.
 * @param uart Puntero a la estructura del driver.
 */
void DrvUart_StartReceive(DrvUart_t *uart) {
    HAL_UART_Receive_IT(uart->huart, &uart->rx_byte, 1);
}

/**
 * @brief Procesa internamente el byte recibido por interrupción.
 * @param uart Puntero a la estructura del driver.
 */
void DrvUart_RxCallback(DrvUart_t *uart) {
    if (uart->message_ready == 0) {
        if (uart->rx_byte == '\n') {
            uart->rx_buffer[uart->rx_index] = '\0';
            uart->rx_index = 0;
            uart->message_ready = 1;
        } else {
            if (uart->rx_index < (UART_RX_BUFFER_SIZE - 1)) {
                uart->rx_buffer[uart->rx_index] = (char)uart->rx_byte;
                uart->rx_index++;
            } else {
                uart->rx_index = 0;
                memset(uart->rx_buffer, 0, UART_RX_BUFFER_SIZE);
            }
        }
    }

    HAL_UART_Receive_IT(uart->huart, &uart->rx_byte, 1);
}

/**
 * @brief Indica si hay un mensaje completo disponible.
 * @param uart Puntero a la estructura del driver.
 * @return 1 si hay un mensaje listo, 0 si no hay mensaje.
 */
uint8_t DrvUart_IsMessageReady(DrvUart_t *uart) {
    return uart->message_ready;
}

/**
 * @brief Copia el mensaje recibido hacia un buffer externo.
 * @param uart Puntero a la estructura del driver.
 * @param destination Buffer destino donde se copiará el mensaje.
 * @param destination_size Tamaño máximo del buffer destino.
 */
void DrvUart_GetMessage(DrvUart_t *uart, char *destination, uint16_t destination_size) {
    if (destination_size == 0) {
        return;
    }

    strncpy(destination, uart->rx_buffer, destination_size - 1);
    destination[destination_size - 1] = '\0';

    memset(uart->rx_buffer, 0, UART_RX_BUFFER_SIZE);
    uart->message_ready = 0;
}

/**
 * @brief Envía un string por UART en modo bloqueante.
 * @param uart Puntero a la estructura del driver.
 * @param message Mensaje terminado en null a transmitir.
 */
void DrvUart_SendString(DrvUart_t *uart, const char *message) {
    HAL_UART_Transmit(uart->huart, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}


