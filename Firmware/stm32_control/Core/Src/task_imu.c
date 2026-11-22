#include "task_imu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "task_control_robot.h"
#include "mid_odometry.h"
#include <limits.h>
#include <math.h>
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

/* TaskIMU owns these values. One-sample delay allows the odometry history to
 * bracket the IMU acquisition tick without extrapolating a current pose. */
static struct {
    double previous_rate, pending_degrees, comparison_degrees;
    TickType_t pending_tick;
    bool pending, encoder_reference, reference_expired;
    float encoder_zero;
} orientation_work;

static double fusion_previous_encoder;

static double IMU_WrapDegrees(double degrees) {
    double wrapped=fmod(degrees+180.0,360.0);
    return (wrapped<0.0 ? wrapped+360.0 : wrapped)-180.0;
}
static void IMU_FusionInvalidate(TaskIMUSnapshot_t *s, uint8_t state, uint8_t reason) {
    s->fusion.state=state;s->fusion.reason=reason;
}
static void IMU_FusionObserve(TaskIMUSnapshot_t *s) {
    TaskIMUFusion_t *f=&s->fusion;
    const TaskIMUOrientation_t *o=&s->orientation;
    if(s->calibration.state==IMU_CAL_RUNNING || s->calibration.state==IMU_CAL_PENDING) {
        if(o->state==IMU_YAW_INVALID || f->reference_request)IMU_FusionInvalidate(s,IMU_FUSION_INVALID,IMU_FUSION_IMU);
        return;
    }
    if(s->state!=IMU_VALID || !s->has_sample || s->calibration.state!=IMU_CAL_VALID || o->state!=IMU_YAW_VALID) {
        IMU_FusionInvalidate(s,IMU_FUSION_INVALID,IMU_FUSION_IMU);return;
    }
    /* Never resume after losing an integration/reference interval. */
    if(f->state==IMU_FUSION_INVALID || f->state==IMU_FUSION_DEGRADED)return;
    if(o->comparison_state!=IMU_COMPARE_VALID) {
        if(f->state==IMU_FUSION_VALID)IMU_FusionInvalidate(s,IMU_FUSION_DEGRADED,IMU_FUSION_ENCODER);
        else if(orientation_work.reference_expired)IMU_FusionInvalidate(s,IMU_FUSION_INVALID,IMU_FUSION_REFERENCE);
        return;
    }
    if((uint64_t)(TickType_t)(s->sample_tick-o->comparison_tick)*1000U>
       (uint64_t)IMU_YAW_MAX_GAP_MS*configTICK_RATE_HZ) {
        IMU_FusionInvalidate(s,IMU_FUSION_INVALID,IMU_FUSION_TIMING);return;
    }
    if(f->state==IMU_FUSION_PENDING) {
        /* Start at IMU-3's zero, never rebase to an unrelated current pose. */
        *f=(TaskIMUFusion_t){.reference_request=o->reference_request,.reference_tick=o->reference_tick,
            .sample_tick=o->reference_tick,.alpha=(float)IMU_FUSION_TAU_MS/(IMU_FUSION_TAU_MS+TASK_IMU_PERIOD_MS),
            .encoder_calibration=o->encoder_calibration};
        fusion_previous_encoder=0.0;
        if(o->comparison_tick==o->reference_tick && orientation_work.comparison_degrees==0.0) {
            f->state=IMU_FUSION_VALID;return;
        }
    } else if(f->reference_request!=o->reference_request || f->reference_tick!=o->reference_tick) {
        IMU_FusionInvalidate(s,IMU_FUSION_INVALID,IMU_FUSION_REFERENCE);return;
    }
    TickType_t dt=(TickType_t)(o->comparison_tick-f->sample_tick);
    if(!dt || (uint64_t)dt*1000U>(uint64_t)IMU_YAW_MAX_GAP_MS*configTICK_RATE_HZ) {
        IMU_FusionInvalidate(s,IMU_FUSION_INVALID,IMU_FUSION_TIMING);return;
    }
    double encoder_step=IMU_WrapDegrees(o->encoder_delta_deg-fusion_previous_encoder);
    /* Exactly half a turn has no observable sign in a wrapped history. */
    if(fabs(encoder_step)>=180.0) {
        IMU_FusionInvalidate(s,IMU_FUSION_INVALID,IMU_FUSION_REFERENCE);return;
    }
    double encoder=f->encoder_degrees+encoder_step;
    double gyro=orientation_work.comparison_degrees;
    double prediction=f->degrees+(gyro-f->gyro_degrees);
    double alpha=(double)IMU_FUSION_TAU_MS/((double)IMU_FUSION_TAU_MS+(double)dt*1000.0/configTICK_RATE_HZ);
    double next=prediction+(1.0-alpha)*IMU_WrapDegrees(encoder-prediction);
    if(!isfinite(next) || !isfinite(encoder) || fabs(next)>(INT32_MAX-1)/1000.0 || fabs(encoder)>(INT32_MAX-1)/1000.0) {
        IMU_FusionInvalidate(s,IMU_FUSION_INVALID,IMU_FUSION_NUMERIC);return;
    }
    f->degrees=next;f->gyro_degrees=gyro;f->encoder_degrees=encoder;f->alpha=(float)alpha;
    f->sample_tick=o->comparison_tick;f->encoder_calibration=o->encoder_calibration;
    f->difference_gyro_deg=IMU_WrapDegrees(next-gyro);
    f->difference_encoder_deg=IMU_WrapDegrees(next-encoder);
    f->state=IMU_FUSION_VALID;f->reason=IMU_FUSION_NONE;
    fusion_previous_encoder=o->encoder_delta_deg;
}
static void IMU_OrientationInvalidate(TaskIMUSnapshot_t *s, uint8_t reason) {
    IMU_FusionInvalidate(s,IMU_FUSION_INVALID,IMU_FUSION_IMU);
    s->orientation.state=IMU_YAW_INVALID;
    s->orientation.reason=reason;
    s->orientation.comparison_state=IMU_COMPARE_YAW;
    orientation_work.pending=false;
}
static void IMU_OrientationObserve(TaskIMUSnapshot_t *s, bool completed) {
    TaskIMUOrientation_t *o=&s->orientation;
    if(s->calibration.state!=IMU_CAL_VALID) {
        if(o->state==IMU_YAW_VALID) IMU_OrientationInvalidate(s,IMU_YAW_CALIBRATION);
        return;
    }
    double rate=((double)s->data.Gyro_Z_RAW-s->calibration.sum[2]/(double)IMU_CAL_SAMPLES)/131.0;
    if(completed) {
        s->fusion=(TaskIMUFusion_t){0};fusion_previous_encoder=0.0;
        *o=(TaskIMUOrientation_t){.state=IMU_YAW_VALID,
            .reference_tick=s->sample_tick,.sample_tick=s->sample_tick,
            .reference_request=s->calibration.request};
        memset(&orientation_work,0,sizeof(orientation_work));
        orientation_work.previous_rate=rate;
    } else {
        if(o->state!=IMU_YAW_VALID)return; /* Never bridge a lost interval. */
        if(o->reference_request!=s->calibration.request || o->reference_tick!=s->calibration.completed_tick) {
            IMU_OrientationInvalidate(s,IMU_YAW_CALIBRATION);return;
        }
        TickType_t dt=(TickType_t)(s->sample_tick-o->sample_tick);
        if(!dt || (uint64_t)dt*1000U>(uint64_t)IMU_YAW_MAX_GAP_MS*configTICK_RATE_HZ) {
            IMU_OrientationInvalidate(s,IMU_YAW_TIMING);return;
        }
        double next=o->degrees+0.5*(orientation_work.previous_rate+rate)*(double)dt/configTICK_RATE_HZ;
        /* Wire unit is signed millidegrees; never wrap/saturate an overflow. */
        if(!isfinite(next) || fabs(next)>(INT32_MAX-1)/1000.0) {
            IMU_OrientationInvalidate(s,IMU_YAW_NUMERIC);return;
        }
        o->degrees=next;o->sample_tick=s->sample_tick;
        orientation_work.previous_rate=rate;
    }
    float x,y,theta;
    if(!orientation_work.encoder_reference && !orientation_work.reference_expired &&
       Odometry_PoseAt(o->reference_tick,&x,&y,&theta)) {
        orientation_work.encoder_reference=true;
        orientation_work.encoder_zero=theta;
    }
    /* An unavailable old reference must never alias a future tick wrap. */
    if(!orientation_work.encoder_reference &&
       (uint64_t)(TickType_t)(s->sample_tick-o->reference_tick)*1000U>
       (uint64_t)ODOM_HISTORY_LENGTH*ODOM_MAX_INTERVAL_MS*configTICK_RATE_HZ)
        orientation_work.reference_expired=true;
    o->comparison_state=IMU_COMPARE_ODOMETRY;
    if(orientation_work.encoder_reference && orientation_work.pending &&
       Odometry_PoseAt(orientation_work.pending_tick,&x,&y,&theta)) {
        OdometrySnapshot encoder;Odometry_GetSnapshot(&encoder);
        if(encoder.state==2U) {
            o->comparison_tick=orientation_work.pending_tick;
            orientation_work.comparison_degrees=orientation_work.pending_degrees;
            o->encoder_reference_deg=orientation_work.encoder_zero*(180.0/3.14159265358979323846);
            o->encoder_delta_deg=IMU_WrapDegrees((theta-orientation_work.encoder_zero)*(180.0/3.14159265358979323846));
            o->gyro_delta_deg=IMU_WrapDegrees(orientation_work.pending_degrees);
            o->difference_deg=IMU_WrapDegrees(o->gyro_delta_deg-o->encoder_delta_deg);
            o->encoder_calibration=(uint8_t)encoder.calibration;
            o->comparison_state=IMU_COMPARE_VALID;
        }
    }
    orientation_work.pending=true;
    orientation_work.pending_tick=o->sample_tick;
    orientation_work.pending_degrees=o->degrees;
}

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
    IMU_OrientationInvalidate(s,IMU_YAW_I2C);
    taskENTER_CRITICAL();
    uint32_t pending=calibration_request; calibration_request=0U;
    taskEXIT_CRITICAL();
    if (pending) { s->calibration=(TaskIMUCalibration_t){ .request=pending }; }
    if (pending || s->calibration.state==IMU_CAL_RUNNING || s->calibration.state==IMU_CAL_VALID)
        IMU_CalibrationFail(s,IMU_CAL_I2C);
}
static void IMU_CalibrationBegin(TaskIMUSnapshot_t *s, uint32_t request, TickType_t issued, TickType_t now) {
    s->fusion=(TaskIMUFusion_t){0};fusion_previous_encoder=0.0;
    s->orientation=(TaskIMUOrientation_t){0};
    memset(&orientation_work,0,sizeof(orientation_work));
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
        if(snapshot.orientation.state==IMU_YAW_VALID)
            IMU_OrientationInvalidate(&snapshot,IMU_YAW_REINITIALIZED);
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
            bool was_running=snapshot.calibration.state==IMU_CAL_RUNNING;
            IMU_CalibrationObserve(&snapshot,started);
            IMU_OrientationObserve(&snapshot,was_running && snapshot.calibration.state==IMU_CAL_VALID);
            IMU_FusionObserve(&snapshot);
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
