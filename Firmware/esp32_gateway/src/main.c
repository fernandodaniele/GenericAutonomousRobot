#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_comm.h"

/*
 * Prueba de comunicación ESP32 <-> STM32.
 *
 * Conexión (cruzada, 3,3 V en ambos lados, GND común):
 *   ESP32 GPIO17 (TX2) -> STM32 PA3 (USART2_RX)
 *   ESP32 GPIO16 (RX2) <- STM32 PA2 (USART2_TX)
 *   ESP32 GND          -  STM32 GND
 *
 * Cada segundo envía "PING\n" y espera "OK" de la STM32.
 * Muestra la latencia y un resumen de enviados / respondidos / sin respuesta.
 * Cualquier otra línea que mande la STM32 (por ejemplo "STM32 Control Ready"
 * al resetearla) se imprime tal cual.
 */

#define PING_PERIOD_MS    1000
#define RESPONSE_TIMEOUT  500
#define LINE_SIZE         64

void app_main(void) {
    char line[LINE_SIZE];
    unsigned sent = 0, ok = 0, lost = 0;

    Uart_Init();
    Uart_Flush();

    printf("\n=== PRUEBA UART ESP32 <-> STM32 ===\n");
    printf("GPIO17 (TX) -> PA3 | GPIO16 (RX) <- PA2 | GND comun | 115200 8N1\n\n");

    while (1) {
        TickType_t cycle_start = xTaskGetTickCount();

        Uart_SendLine("PING");
        sent++;

        int answered = 0;
        while (1) {
            int remaining = RESPONSE_TIMEOUT -
                            (int)((xTaskGetTickCount() - cycle_start) * portTICK_PERIOD_MS);
            if (remaining <= 0 || Uart_ReadLine(line, sizeof(line), remaining) < 0) {
                break;
            }
            if (strcmp(line, "OK") == 0) {
                unsigned latency_ms = (xTaskGetTickCount() - cycle_start) * portTICK_PERIOD_MS;
                answered = 1;
                ok++;
                printf("PING #%u -> OK (%u ms)", sent, latency_ms);
                break;
            }
            // Otra línea de la STM32: se muestra y se sigue esperando el OK
            printf("[STM32] %s\n", line);
        }

        if (!answered) {
            lost++;
            printf("PING #%u -> SIN RESPUESTA", sent);
        }
        printf("   | enviados %u, OK %u, sin respuesta %u\n", sent, ok, lost);

        vTaskDelayUntil(&cycle_start, pdMS_TO_TICKS(PING_PERIOD_MS));
    }
}
