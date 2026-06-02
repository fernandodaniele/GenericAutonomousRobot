/*
 * drv_uart.h
 *
 *  Created on: Apr 28, 2026
 *      Author: Grupo 1 y 2
 */

#ifndef DRV_UART_H_
#define DRV_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define UART_RX_BUFFER_SIZE    64

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_byte;
    char rx_buffer[UART_RX_BUFFER_SIZE];
    uint16_t rx_index;
    uint8_t message_ready;
} DrvUart_t;

/**
 * @brief Inicializa la estructura del driver UART.
 * @param uart Puntero a la estructura del driver.
 * @param huart Puntero al handle UART generado por HAL.
 */
void DrvUart_Init(DrvUart_t *uart, UART_HandleTypeDef *huart);

/**
 * @brief Inicia la recepción UART por interrupción byte a byte.
 * @param uart Puntero a la estructura del driver.
 */
void DrvUart_StartReceive(DrvUart_t *uart);

/**
 * @brief Procesa internamente el byte recibido por interrupción.
 * @param uart Puntero a la estructura del driver.
 */
void DrvUart_RxCallback(DrvUart_t *uart);

/**
 * @brief Indica si hay un mensaje completo disponible.
 * @param uart Puntero a la estructura del driver.
 * @return 1 si hay un mensaje listo, 0 si no hay mensaje.
 */
uint8_t DrvUart_IsMessageReady(DrvUart_t *uart);

/**
 * @brief Copia el mensaje recibido hacia un buffer externo.
 * @param uart Puntero a la estructura del driver.
 * @param destination Buffer destino donde se copiará el mensaje.
 * @param destination_size Tamaño máximo del buffer destino.
 */
void DrvUart_GetMessage(DrvUart_t *uart, char *destination, uint16_t destination_size);

/**
 * @brief Envía un string por UART en modo bloqueante.
 * @param uart Puntero a la estructura del driver.
 * @param message Mensaje terminado en null a transmitir.
 */
void DrvUart_SendString(DrvUart_t *uart, const char *message);

#ifdef __cplusplus
}
#endif

#endif /* DRV_UART_H_ */
