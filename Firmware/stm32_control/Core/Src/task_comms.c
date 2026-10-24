#include "task_comms.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include <string.h>

#define TASK_COMMS_PERIOD_MS 10U
#define TASK_COMMS_STACK_SIZE 256U
#define TASK_COMMS_PRIORITY (tskIDLE_PRIORITY + 3U)

/**
 * @brief Procesa los mensajes recibidos desde el ESP32.
 */
static void TaskComms_Run(void *parameters) {
    DrvUart_t *uart = (DrvUart_t *)parameters;
    char message[UART_RX_BUFFER_SIZE];

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(TASK_COMMS_PERIOD_MS));

        // Verifica si hay un mensaje completo disponible.
        if (DrvUart_IsMessageReady(uart) == 0U) {
            continue;
        }

        // Recupera el mensaje recibido.
        DrvUart_GetMessage(uart, message,
                           (uint16_t)sizeof(message));

        // Responde a la solicitud de prueba del ESP32.
        if (strcmp(message, "PING") == 0) {
            DrvUart_SendString(uart, "PONG\n");
        }
    }
}

/**
 * @brief Crea la tarea FreeRTOS de comunicaciones.
 */
void TaskComms_Create(DrvUart_t *uart) {
    if (uart == NULL || uart->huart == NULL) {
        Error_Handler();
        return;
    }

    BaseType_t result = xTaskCreate(
        TaskComms_Run,
        "Comms",
        TASK_COMMS_STACK_SIZE,
        uart,
        TASK_COMMS_PRIORITY,
        NULL
    );

    if (result != pdPASS) {
        Error_Handler();
    }
}