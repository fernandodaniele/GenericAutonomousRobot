/*
 * drv_uart.c
 *
 *  Created on: Apr 28, 2026
 *      Author: Grupo 1 y 2
 */

#include "drv_uart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/** @brief Inicializa ambos buffers y el estado de recepcion. */
void DrvUart_Init(DrvUart_t *uart, UART_HandleTypeDef *huart) {
    memset(uart, 0, sizeof(*uart));
    uart->huart = huart;
}

/** @brief Inicia la recepcion UART por interrupcion byte a byte. */
void DrvUart_StartReceive(DrvUart_t *uart) {
    (void)HAL_UART_Receive_IT(uart->huart, &uart->rx_byte, 1U);
}

/* Mailbox acotado: STOP > DETENER > movimiento/modo > STATUS/PING > SENSORS.
 * No acumula movimientos; una consulta nunca desplaza una orden operativa.
 * ISR solo ensambla/fecha mensajes: no actua sobre FSM ni motores.
 */
void DrvUart_RxCallback(DrvUart_t *uart) {
    uint8_t byte = uart->rx_byte;
    if (byte == '\n') {
        if (!uart->discard_frame && uart->rx_index != 0U) {
            uart->frame_buffer[uart->rx_index] = '\0';
            const char *frame = uart->frame_buffer;
            uint8_t priority = 1U;
            if (strcmp(frame, "STOP") == 0) priority = 4U;
            else if (strcmp(frame, "DETENER") == 0) priority = 3U;
            else if (strcmp(frame, "AVANZAR") == 0 || strcmp(frame, "RETROCEDER") == 0 ||
                     strcmp(frame, "GIRAR_IZQUIERDA") == 0 || strcmp(frame, "GIRAR_DERECHA") == 0 ||
                     strcmp(frame, "RESET") == 0 || strcmp(frame, "MANUAL") == 0) priority = 2U;
            else if (strcmp(frame, "SENSORS") == 0 || strncmp(frame, "SENSORS Q=", 10U) == 0) priority = 0U;
            if (!uart->message_ready || priority > uart->message_priority) {
                memcpy(uart->rx_buffer, uart->frame_buffer, uart->rx_index + 1U);
                uart->message_tick = uart->frame_tick;
                uart->message_priority = priority;
                uart->message_ready = 1U;
            }
        }
        uart->rx_index = 0U;
        uart->discard_frame = 0U;
        uart->cr_seen = 0U;
    } else if (!uart->discard_frame) {
        if (byte == 0U && uart->rx_index == 0U && !uart->cr_seen) {
            /* Conservar tolerancia al NUL de arranque, nunca al NUL embebido. */
        } else if (byte == '\r' && !uart->cr_seen) {
            uart->cr_seen = 1U;
        } else if (uart->cr_seen || byte < 32U || byte > 126U ||
                   uart->rx_index >= UART_RX_BUFFER_SIZE - 1U) {
            /* Descartar hasta LF: un sufijo no puede convertirse en otra orden. */
            uart->discard_frame = 1U;
        } else {
            if (uart->rx_index == 0U) uart->frame_tick = xTaskGetTickCountFromISR();
            uart->frame_buffer[uart->rx_index++] = (char)byte;
        }
    }
    (void)HAL_UART_Receive_IT(uart->huart, &uart->rx_byte, 1U);
}

uint8_t DrvUart_IsMessageReady(DrvUart_t *uart) {
    return uart->message_ready;
}

bool DrvUart_GetMessageTimed(DrvUart_t *uart, char *destination,
                            uint16_t destination_size, TickType_t *issued_tick) {
    if (uart == NULL || destination == NULL || destination_size == 0U || issued_tick == NULL)
        return false;
    destination[0] = '\0';
    taskENTER_CRITICAL();
    bool ready = uart->message_ready != 0U;
    bool complete = ready && strlen(uart->rx_buffer) < destination_size;
    if (complete) {
        strcpy(destination, uart->rx_buffer);
        *issued_tick = uart->message_tick;
    }
    if (ready) {
        uart->message_ready = 0U;
        uart->message_priority = 0U;
        uart->rx_buffer[0] = '\0';
    }
    taskEXIT_CRITICAL();
    return complete;
}

/* Compatibilidad con consumidores que no necesitan la fecha de origen. */
void DrvUart_GetMessage(DrvUart_t *uart, char *destination, uint16_t destination_size) {
    TickType_t unused;
    (void)DrvUart_GetMessageTimed(uart, destination, destination_size, &unused);
}

void DrvUart_SendString(DrvUart_t *uart, const char *message) {
    (void)HAL_UART_Transmit(uart->huart, (uint8_t *)message, strlen(message), 50U);
}
