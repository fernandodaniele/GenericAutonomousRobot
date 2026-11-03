#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_comm.h"

// Tarea encargada de leer comandos del monitor serial.
static void ConsoleInputTask(void *parameters)
{
    (void)parameters;

    for (;;) {
        int input = getchar();

        if (input == 'p') {
            Uart_SendPing();

        } else if (input == 's') {
            Uart_SendStatus();

        } else if (input == 'r') {
            Uart_SendReset();

        } else if (input == 't') {
            Uart_SendStop();

        } else if (input == 'm') {
            Uart_SendManual();

        } else if (input == EOF) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void app_main(void)
{
    Uart_Init();

    printf("\n=== ESP32 UART TEST ===\n");
    printf("Comandos: p=PING, s=STATUS, r=RESET, t=STOP, m=MANUAL\n");

    if (xTaskCreate(
            ConsoleInputTask,
            "ConsoleInput",
            4096,
            NULL,
            5,
            NULL
        ) != pdPASS) {
        printf("Error al crear ConsoleInputTask\n");
        return;
    }

    // Recepcion UART independiente del monitor serial.
    while (1) {
        Uart_ReadResponse();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
