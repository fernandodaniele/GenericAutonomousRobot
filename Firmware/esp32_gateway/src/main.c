#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "telemetry.h"
#include "uart_comm.h"
#include "wifi_control.h"

/*
 * Prueba de comunicación ESP32 <-> STM32 y panel web del robot.
 *
 * Conexión (cruzada, 3,3 V en ambos lados, GND común):
 *   ESP32 GPIO17 (TX2) -> STM32 PA3 (USART2_RX)
 *   ESP32 GPIO16 (RX2) <- STM32 PA2 (USART2_TX)
 *   ESP32 GND          -  STM32 GND
 *
 * Una tarea lee sin pausa todo lo que manda la STM32: la telemetría alimenta
 * el panel web y cualquier otra línea (por ejemplo "STM32 Control Ready" al
 * resetearla) se imprime tal cual.
 *
 * Cada segundo envía "PING\n" y espera "OK" de la STM32. Muestra la latencia
 * y un resumen de enviados / respondidos / sin respuesta.
 *
 * Panel desde el celular: conectarse a la red WiFi "GAR-Robot"
 * (clave robot1234) y abrir http://192.168.4.1. Muestra sensores y motores en
 * vivo, y los botones START y STOP se envían a la STM32 por la misma UART.
 */

#define PING_PERIOD_MS    1000
#define RESPONSE_TIMEOUT  500
#define LINE_SIZE         128
#define READER_STACK      4096
#define READER_PRIORITY   5
#define READ_TIMEOUT_MS   1000

static SemaphoreHandle_t ping_ok;

static void uart_reader_task(void *argument) {
    (void) argument;
    char line[LINE_SIZE];

    while (1) {
        if (Uart_ReadLine(line, sizeof(line), READ_TIMEOUT_MS) < 0) {
            continue;
        }
        if (strcmp(line, "OK") == 0) {
            xSemaphoreGive(ping_ok);
        } else if (!Telemetry_HandleLine(line)) {
            printf("[STM32] %s\n", line);
        }
    }
}

void app_main(void) {
    unsigned sent = 0, ok = 0, lost = 0;

    ping_ok = xSemaphoreCreateBinary();
    configASSERT(ping_ok);

    Uart_Init();
    Uart_Flush();
    Telemetry_Init();
    WifiControl_Start();
    xTaskCreate(uart_reader_task, "uart_reader", READER_STACK, NULL, READER_PRIORITY, NULL);

    printf("\n=== PRUEBA UART ESP32 <-> STM32 ===\n");
    printf("GPIO17 (TX) -> PA3 | GPIO16 (RX) <- PA2 | GND comun | 115200 8N1\n\n");

    while (1) {
        TickType_t cycle_start = xTaskGetTickCount();

        xSemaphoreTake(ping_ok, 0);   // Descarta un OK que llegó tarde
        Uart_SendLine("PING");
        sent++;

        if (xSemaphoreTake(ping_ok, pdMS_TO_TICKS(RESPONSE_TIMEOUT)) == pdTRUE) {
            unsigned latency_ms = (xTaskGetTickCount() - cycle_start) * portTICK_PERIOD_MS;
            ok++;
            printf("PING #%u -> OK (%u ms)", sent, latency_ms);
        } else {
            lost++;
            printf("PING #%u -> SIN RESPUESTA", sent);
        }
        printf("   | enviados %u, OK %u, sin respuesta %u\n", sent, ok, lost);

        vTaskDelayUntil(&cycle_start, pdMS_TO_TICKS(PING_PERIOD_MS));
    }
}
