#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_comm.h"

/**
 * @brief Punto de entrada principal del programa.
 */
void app_main(void) {
    Uart_Init();

    printf("\n=== ESP32 UART TEST ===\n");
    printf("Presionar 'p' en el monitor serial para enviar PING a STM32\n");

    while (1) {
        int input = getchar();

        if (input == 'p') {
            Uart_SendPing();
        }

        Uart_ReadResponse();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
