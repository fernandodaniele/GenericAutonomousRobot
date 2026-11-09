#include "task_comms.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "task_control_robot.h"
#include "task_safety.h"
#include "mid_motion.h"
#include "task_sensores.h"

#include <stdio.h>
#include <string.h>

#define TASK_COMMS_PERIOD_MS 10U
#define TASK_COMMS_STACK_SIZE 256U
#define TASK_COMMS_PRIORITY (tskIDLE_PRIORITY + 3U)

/* SENSORS v1: una linea de menos de 96 bytes, enteros o '-' (ausente).
 * La consulta solo copia la cola; no accede a ningun driver de sensores. */
static uint32_t TaskComms_SensorAge(TickType_t now, TickType_t tick) {
    uint64_t ms = (uint64_t)(TickType_t)(now - tick) * 1000U / configTICK_RATE_HZ;
    return ms > UINT32_MAX ? UINT32_MAX : (uint32_t)ms;
}

static void TaskComms_SendSensors(DrvUart_t *uart, uint32_t query) {
    SensoresDatos_t data;
    char response[96];
    int length;
    if (TaskSensores_GetData(&data) != pdTRUE) {
        length = snprintf(response, sizeof(response),
            "SENSORS,1,%lu,0,-,0,-,-,0,-,-,-,-,-\n", (unsigned long)query);
    } else {
        TickType_t now = xTaskGetTickCount();
        char front_mm[6] = "-", rear_mm[6] = "-";
        char front_age[11] = "-", rear_age[11] = "-";
        unsigned front_state = 0U, rear_state = 0U;
        if (data.resultado_frontal_disponible != 0U) {
            front_state = data.ultrasonido_valido && data.distancia_m >= 0.02f && data.distancia_m <= 4.0f ? 1U : 2U;
            if (front_state == 1U) (void)snprintf(front_mm, sizeof(front_mm), "%u", (unsigned)(data.distancia_m * 1000.0f + 0.5f));
            (void)snprintf(front_age, sizeof(front_age), "%lu", (unsigned long)TaskComms_SensorAge(now, data.timestamp_resultado_frontal));
        }
        if (data.resultado_trasero_disponible != 0U) {
            rear_state = data.ultrasonido_trasero_valido && data.distancia_trasera_m >= 0.02f && data.distancia_trasera_m <= 4.0f ? 1U : 2U;
            if (rear_state == 1U) (void)snprintf(rear_mm, sizeof(rear_mm), "%u", (unsigned)(data.distancia_trasera_m * 1000.0f + 0.5f));
            (void)snprintf(rear_age, sizeof(rear_age), "%lu", (unsigned long)TaskComms_SensorAge(now, data.timestamp_resultado_trasero));
        }
        unsigned ir = (data.ir.del_izq == TCRT_DETECTA ? 1U : 0U) |
                      (data.ir.del_der == TCRT_DETECTA ? 2U : 0U) |
                      (data.ir.tras_izq == TCRT_DETECTA ? 4U : 0U) |
                      (data.ir.tras_der == TCRT_DETECTA ? 8U : 0U);
        length = snprintf(response, sizeof(response),
            "SENSORS,1,%lu,1,%lu,%u,%s,%s,%u,%s,%s,%u,%lu,%u\n",
            (unsigned long)query, (unsigned long)TaskComms_SensorAge(now, data.timestamp),
            front_state, front_mm, front_age, rear_state, rear_mm, rear_age, ir,
            (unsigned long)TaskComms_SensorAge(now, data.timestamp_ir), data.hay_caida ? 1U : 0U);
    }
    if (length > 0 && (size_t)length < sizeof(response)) DrvUart_SendString(uart, response);
}

static bool TaskComms_SensorsQuery(const char *message, uint32_t *query) {
    *query = 0U;
    if (strcmp(message, "SENSORS") == 0) return true;
    if (strncmp(message, "SENSORS Q=", 10U) != 0) return false;
    const char *p = message + 10;
    if (*p < '0' || *p > '9') return false;
    do {
        uint32_t digit = (uint32_t)(*p - '0');
        if (*query > (UINT32_MAX - digit) / 10U) return false;
        *query = *query * 10U + digit;
        ++p;
    } while (*p >= '0' && *p <= '9');
    return *p == '\0' && *query != 0U;
}

/**
 * @brief Procesa los mensajes recibidos desde el ESP32.
 */
static void TaskComms_ProcessMessage(DrvUart_t *uart, const char *message, TickType_t issued_tick) {
    uint32_t query;
    if (TaskComms_SensorsQuery(message, &query)) {
        TaskComms_SendSensors(uart, query);
        return;
    }
    if (strcmp(message, "PING") == 0) {
        DrvUart_SendString(uart, "PONG\n");

    } else if (strcmp(message, "STATUS") == 0) {

        char response[64];

        snprintf(
            response,
            sizeof(response),
            "STATE=%u FAULTS=%lu STOP=%u\n",
            (unsigned int)TaskControlRobot_GetState(),
            (unsigned long)TaskSafety_GetFaults(),
            TaskSafety_IsStopActive() ? 1U : 0U
        );

        DrvUart_SendString(uart, response);

    } else if (strcmp(message, "RESET") == 0) {

        if (TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET)) {
            DrvUart_SendString(uart, "RESET_QUEUED\n");
        } else {
            DrvUart_SendString(uart, "RESET_QUEUE_ERROR\n");
        }

    } else if (strcmp(message, "MANUAL") == 0) {

        if (TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL)) {
            DrvUart_SendString(uart, "MANUAL_QUEUED\n");
        } else {
            DrvUart_SendString(uart, "MANUAL_QUEUE_ERROR\n");
        }

    } else if (strcmp(message, "STOP") == 0) {

        if (TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)) {
            DrvUart_SendString(uart, "STOP_QUEUED\n");
        } else {
            DrvUart_SendString(uart, "STOP_QUEUE_ERROR\n");
        }

    } else {
        MotionCommand_t command;
        if (strcmp(message, "AVANZAR") == 0) command = MOTION_FORWARD;
        else if (strcmp(message, "RETROCEDER") == 0) command = MOTION_REVERSE;
        else if (strcmp(message, "GIRAR_IZQUIERDA") == 0) command = MOTION_TURN_LEFT;
        else if (strcmp(message, "GIRAR_DERECHA") == 0) command = MOTION_TURN_RIGHT;
        else if (strcmp(message, "DETENER") == 0) command = MOTION_HALT;
        else {
            // Comando no reconocido.
            DrvUart_SendString(uart, "ERR_UNKNOWN\n");
            return;
        }
        DrvUart_SendString(uart, Motion_Execute(command, ROBOT_STATE_MANUAL, issued_tick) ?
            "MOVE_ACCEPTED\n" : "MOVE_REJECTED\n");
    }
}

static void TaskComms_Run(void *parameters) {
    DrvUart_t *uart = (DrvUart_t *)parameters;
    char message[UART_RX_BUFFER_SIZE];
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_COMMS_PERIOD_MS));
        TickType_t issued_tick;
        if (DrvUart_GetMessageTimed(uart, message, sizeof(message), &issued_tick)) {
            TaskComms_ProcessMessage(uart, message, issued_tick);
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
