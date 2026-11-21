#include "task_imu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "task_control_robot.h"
#include <limits.h>
#include <string.h>

#define TASK_IMU_PERIOD_MS 20U
#define TASK_IMU_RETRY_MS 1000U
#define TASK_IMU_STACK_SIZE 512U
#define TASK_IMU_PRIORITY (tskIDLE_PRIORITY + 4U)

// Conserva la última medición válida de la IMU.
static QueueHandle_t imu_data_queue = NULL;
static uint32_t calibration_request, last_calibration_request;
static TickType_t calibration_issued;
/* Owned exclusively by TaskIMU; fixed storage, no sample history or new task. */
static struct {
    int64_t squares[3];
    int16_t gyro_min[3], gyro_max[3], accel_min[3], accel_max[3];
    TickType_t requested, first, last;
} calibration_work;

static bool IMU_CalibrationMode(void) {
    RobotControlState_t mode=TaskControlRobot_GetState();
    return mode==ROBOT_STATE_IDLE || mode==ROBOT_STATE_SAFETY_STOP;
}
static uint32_t IMU_ElapsedMs(TickType_t now, TickType_t then) {
    uint64_t ms=(uint64_t)(TickType_t)(now-then)*1000U/configTICK_RATE_HZ;
    return ms>UINT32_MAX ? UINT32_MAX : (uint32_t)ms;
}
static void IMU_CalibrationFail(TaskIMUSnapshot_t *s, uint8_t reason) {
    s->calibration.state=IMU_CAL_FAILED; s->calibration.reason=reason;
    memset(s->calibration.sum,0,sizeof(s->calibration.sum));
    s->calibration.completed_tick=0U;
}
static void IMU_CalibrationI2CError(TaskIMUSnapshot_t *s) {
    taskENTER_CRITICAL();
    uint32_t pending=calibration_request; calibration_request=0U;
    taskEXIT_CRITICAL();
    if (pending) { s->calibration=(TaskIMUCalibration_t){ .request=pending }; }
    if (pending || s->calibration.state==IMU_CAL_RUNNING || s->calibration.state==IMU_CAL_VALID)
        IMU_CalibrationFail(s,IMU_CAL_I2C);
}
static void IMU_CalibrationBegin(TaskIMUSnapshot_t *s, uint32_t request, TickType_t issued, TickType_t now) {
    s->calibration=(TaskIMUCalibration_t){ .request=request, .state=IMU_CAL_RUNNING };
    memset(&calibration_work,0,sizeof(calibration_work));
    calibration_work.requested=now;
    if (IMU_ElapsedMs(now,issued)>=500U) IMU_CalibrationFail(s,IMU_CAL_TIMING);
    else if (!IMU_CalibrationMode()) IMU_CalibrationFail(s,IMU_CAL_MODE);
}
static void IMU_CalibrationObserve(TaskIMUSnapshot_t *s, TickType_t now) {
    TaskIMUCalibration_t *c=&s->calibration;
    if (c->state!=IMU_CAL_RUNNING) return;
    if (!IMU_CalibrationMode()) { IMU_CalibrationFail(s,IMU_CAL_MODE); return; }
    if (IMU_ElapsedMs(now,calibration_work.requested)>IMU_CAL_MAX_SPAN_MS) {
        IMU_CalibrationFail(s,IMU_CAL_INSUFFICIENT); return;
    }
    if (c->samples && (now==calibration_work.last ||
        IMU_ElapsedMs(now,calibration_work.last)>IMU_CAL_MAX_GAP_MS)) {
        IMU_CalibrationFail(s,IMU_CAL_TIMING); return;
    }
    const int16_t a[]={s->data.Accel_X_RAW,s->data.Accel_Y_RAW,s->data.Accel_Z_RAW};
    const int16_t g[]={s->data.Gyro_X_RAW,s->data.Gyro_Y_RAW,s->data.Gyro_Z_RAW};
    int64_t norm=0;
    for(unsigned i=0;i<3U;++i) norm+=(int64_t)a[i]*a[i];
    /* 0.9..1.1 g, squared. Reject free fall, gross acceleration or clipping. */
    if (norm<IMU_CAL_ACCEL_NORM_MIN2 || norm>IMU_CAL_ACCEL_NORM_MAX2) { IMU_CalibrationFail(s,IMU_CAL_MOTION); return; }
    for(unsigned i=0;i<3U;++i) {
        if (g[i]<-IMU_CAL_GYRO_ABS_RAW || g[i]>IMU_CAL_GYRO_ABS_RAW) {
            IMU_CalibrationFail(s,IMU_CAL_MOTION); return;
        }
        if (!c->samples) {
            calibration_work.gyro_min[i]=calibration_work.gyro_max[i]=g[i];
            calibration_work.accel_min[i]=calibration_work.accel_max[i]=a[i];
        } else {
            if(g[i]<calibration_work.gyro_min[i])calibration_work.gyro_min[i]=g[i];
            if(g[i]>calibration_work.gyro_max[i])calibration_work.gyro_max[i]=g[i];
            if(a[i]<calibration_work.accel_min[i])calibration_work.accel_min[i]=a[i];
            if(a[i]>calibration_work.accel_max[i])calibration_work.accel_max[i]=a[i];
        }
        if ((int32_t)calibration_work.gyro_max[i]-calibration_work.gyro_min[i]>IMU_CAL_GYRO_RANGE_RAW ||
            (int32_t)calibration_work.accel_max[i]-calibration_work.accel_min[i]>IMU_CAL_ACCEL_RANGE_RAW) {
            IMU_CalibrationFail(s,IMU_CAL_MOTION); return;
        }
        c->sum[i]+=g[i]; calibration_work.squares[i]+=(int64_t)g[i]*g[i];
    }
    if (!c->samples) calibration_work.first=now;
    calibration_work.last=now;
    ++c->samples;
    if(c->samples<IMU_CAL_SAMPLES)return;
    if(IMU_ElapsedMs(now,calibration_work.first)<IMU_CAL_MIN_SPAN_MS) {
        IMU_CalibrationFail(s,IMU_CAL_TIMING); return;
    }
    for(unsigned i=0;i<3U;++i) {
        int64_t variance=(int64_t)c->samples*calibration_work.squares[i]-(int64_t)c->sum[i]*c->sum[i];
        int64_t limit=(int64_t)c->samples*c->samples*IMU_CAL_GYRO_STD_RAW*IMU_CAL_GYRO_STD_RAW;
        if(variance>limit) { IMU_CalibrationFail(s,IMU_CAL_MOTION); return; }
    }
    c->state=IMU_CAL_VALID; c->reason=IMU_CAL_NONE; c->completed_tick=now;
}

static void TaskIMU_Run(void *parameters) {
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)parameters;
    TaskIMUSnapshot_t snapshot = { .state = IMU_INITIALIZING };

    for (;;) {
        // Inicializa el sensor o intenta recuperarlo tras un fallo.
        HAL_StatusTypeDef status;
        while ((status = MPU6050_Init(hi2c)) != HAL_OK) {
            IMU_CalibrationI2CError(&snapshot);
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

            taskENTER_CRITICAL();
            uint32_t request=calibration_request; TickType_t issued=calibration_issued;
            calibration_request=0U;
            taskEXIT_CRITICAL();
            if (request) {
                IMU_CalibrationBegin(&snapshot,request,issued,xTaskGetTickCount());
                (void)xQueueOverwrite(imu_data_queue,&snapshot);
            }

            TickType_t started = xTaskGetTickCount();
            status = MPU6050_Read_All(hi2c, &snapshot.data);

            if (status != HAL_OK) {
                IMU_CalibrationI2CError(&snapshot);
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
            IMU_CalibrationObserve(&snapshot,started);
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

bool TaskIMU_RequestCalibration(uint32_t request, TickType_t issued_tick) {
    TaskIMUSnapshot_t s;
    if (!request || !TaskIMU_GetSnapshot(&s) || s.state!=IMU_VALID ||
        s.calibration.state==IMU_CAL_RUNNING || !s.has_sample) return false;
    TickType_t now=xTaskGetTickCount();
    if(IMU_ElapsedMs(now,issued_tick)>=500U || IMU_ElapsedMs(now,s.sample_tick)>100U) return false;
    taskENTER_CRITICAL();
    bool accepted=IMU_CalibrationMode() && calibration_request==0U && request!=last_calibration_request;
    if(accepted) { calibration_request=last_calibration_request=request; calibration_issued=issued_tick; }
    taskEXIT_CRITICAL();
    return accepted;
}
