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
    TaskIMUSnapshot_t snapshot = { .state = IMU_INITIALIZING };

    for (;;) {
        // Inicializa el sensor o intenta recuperarlo tras un fallo.
        HAL_StatusTypeDef status;
        while ((status = MPU6050_Init(hi2c)) != HAL_OK) {
            snapshot.state = IMU_INIT_ERROR;
            snapshot.hal_status = status;
            snapshot.i2c_error = HAL_I2C_GetError(hi2c);
            (void)xQueueOverwrite(imu_data_queue, &snapshot);
            vTaskDelay(pdMS_TO_TICKS(TASK_IMU_RETRY_MS));
        }

        // Descarta las mediciones anteriores después de inicializar.
        snapshot.data = (MPU6050_t){0};
        snapshot.state = IMU_READY;
        snapshot.hal_status = HAL_OK;
        snapshot.i2c_error = 0U;
        (void)xQueueOverwrite(imu_data_queue, &snapshot);

        // Reinicia la referencia temporal de la tarea.
        TickType_t last_wake_time = xTaskGetTickCount();

        for (;;) {
            vTaskDelayUntil(&last_wake_time,
                            pdMS_TO_TICKS(TASK_IMU_PERIOD_MS));

            TickType_t started = xTaskGetTickCount();
            status = MPU6050_Read_All(hi2c, &snapshot.data);

            if (status != HAL_OK) {
                // Invalida las mediciones y reinicia el sensor.
                snapshot.state = IMU_READ_ERROR;
                snapshot.hal_status = status;
                snapshot.i2c_error = HAL_I2C_GetError(hi2c);
                (void)xQueueOverwrite(imu_data_queue, &snapshot);
                /* Keep the failure observable, without a tight recovery loop. */
                vTaskDelay(pdMS_TO_TICKS(TASK_IMU_RETRY_MS));
                break;
            }

            // Publica la última medición válida.
            snapshot.state = IMU_VALID;
            snapshot.hal_status = HAL_OK;
            snapshot.i2c_error = 0U;
            snapshot.has_sample = true;
            snapshot.sample_tick = started;
            (void)xQueueOverwrite(imu_data_queue, &snapshot);
        }
    }
}

void TaskIMU_Create(I2C_HandleTypeDef *hi2c) {
    if (hi2c == NULL || imu_data_queue != NULL) {
        Error_Handler();
        return;
    }

    // Crea una cola de un elemento para las mediciones.
    imu_data_queue = xQueueCreate(1U, sizeof(TaskIMUSnapshot_t));

    if (imu_data_queue == NULL) {
        Error_Handler();
        return;
    }

    TaskIMUSnapshot_t initial = { .state = IMU_INITIALIZING };
    (void)xQueueOverwrite(imu_data_queue, &initial);
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
    TaskIMUSnapshot_t snapshot;
    if (data == NULL || !TaskIMU_GetSnapshot(&snapshot) || snapshot.state != IMU_VALID)
        return false;
    *data = snapshot.data;
    return true;
}

bool TaskIMU_GetSnapshot(TaskIMUSnapshot_t *snapshot) {
    return snapshot != NULL && imu_data_queue != NULL &&
        xQueuePeek(imu_data_queue, snapshot, 0U) == pdTRUE;
}
