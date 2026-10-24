#ifndef TASK_COMMS_H_
#define TASK_COMMS_H_

#include "drv_uart.h"

/**
 * @brief Crea la tarea FreeRTOS de comunicaciones.
 * @param uart Driver UART utilizado para comunicarse con ESP32.
 */
void TaskComms_Create(DrvUart_t *uart);

#endif /* TASK_COMMS_H_ */