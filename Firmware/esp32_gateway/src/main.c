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

        } else if (input == 'n') {
            Uart_SendAuto();

        } else if (input == 'w') {
            Uart_SendForward();
        } else if (input == 'x') {
            Uart_SendReverse();
        } else if (input == 'a') {
            Uart_SendLeft();
        } else if (input == 'd') {
            Uart_SendRight();
        } else if (input == 'e') {
            Uart_SendHalt();

        } else if (input == EOF) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void app_main(void)
{
    Uart_Init();

    printf("\n=== ESP32 UART TEST ===\n");
    printf("Comandos: p=PING, s=STATUS, r=RESET, t=STOP, m=MANUAL, n=AUTO (puede avanzar)\n");
    printf("Movimiento: w=AVANZAR, x=RETROCEDER, a=GIRAR_IZQUIERDA, d=GIRAR_DERECHA, e=DETENER\n");

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
