#include "task_imu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"

#define TASK_IMU_PERIOD_MS 20U
#define TASK_IMU_RETRY_MS 1000U
#define TASK_IMU_STACK_SIZE 512U
#define TASK_IMU_PRIORITY (tskIDLE_PRIORITY + 4U)

// Conserva la última medición válida de la IMU.
static QueueHandle_t imu_data_queue = NULL;

static void TaskIMU_Run(void *parameters) {
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)parameters;
    MPU6050_t imu_data = {0};

    for (;;) {
        // Inicializa el sensor o intenta recuperarlo tras un fallo.
        while (MPU6050_Init(hi2c) != HAL_OK) {
            (void)xQueueReset(imu_data_queue);
            vTaskDelay(pdMS_TO_TICKS(TASK_IMU_RETRY_MS));
        }

        // Descarta las mediciones anteriores después de inicializar.
        imu_data = (MPU6050_t){0};

        // Reinicia la referencia temporal de la tarea.
        TickType_t last_wake_time = xTaskGetTickCount();

        for (;;) {
            vTaskDelayUntil(&last_wake_time,
                            pdMS_TO_TICKS(TASK_IMU_PERIOD_MS));

            HAL_StatusTypeDef status = MPU6050_Read_All(hi2c, &imu_data);

            if (status != HAL_OK) {
                // Invalida las mediciones y reinicia el sensor.
                (void)xQueueReset(imu_data_queue);
                break;
            }

            // Publica la última medición válida.
            (void)xQueueOverwrite(imu_data_queue, &imu_data);
        }
    }
}

void TaskIMU_Create(I2C_HandleTypeDef *hi2c) {
    if (hi2c == NULL || imu_data_queue != NULL) {
        Error_Handler();
        return;
    }

    // Crea una cola de un elemento para las mediciones.
    imu_data_queue = xQueueCreate(1U, sizeof(MPU6050_t));

    if (imu_data_queue == NULL) {
        Error_Handler();
        return;
    }

    BaseType_t result = xTaskCreate(
        TaskIMU_Run,
        "IMU",
        TASK_IMU_STACK_SIZE,
        hi2c,
        TASK_IMU_PRIORITY,
        NULL
    );

    if (result != pdPASS) {
        vQueueDelete(imu_data_queue);
        imu_data_queue = NULL;
        Error_Handler();
    }
}

bool TaskIMU_GetLatest(MPU6050_t *data) {
    if (data == NULL || imu_data_queue == NULL) {
        return false;
    }

    // Consulta sin consumir la última medición.
    return (xQueuePeek(imu_data_queue, data, 0U) == pdTRUE);
}
