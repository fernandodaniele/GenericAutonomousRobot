#include "task_comms.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "task_control_robot.h"
#include "task_safety.h"
#include "mid_motion.h"
#include "task_sensores.h"
#include "mid_mapping.h"
#include "task_imu.h"
#include "mid_navigation.h"

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

/* IMU v1: raw signed axes, no float formatting, no new I2C acquisition.
 * State, HAL result, I2C error bits, age of last successful acquisition.
 * Missing age/axes are '-', never invented zero measurements. */
static void TaskComms_SendImu(DrvUart_t *uart, uint32_t query, unsigned version) {
    static TaskIMUSnapshot_t s; /* Single Comms task; bounded stack as snapshot grows. */
    s=(TaskIMUSnapshot_t){ .state = IMU_INITIALIZING };
    (void)TaskIMU_GetSnapshot(&s);
    const TickType_t now=xTaskGetTickCount(); /* One age origin for the coherent reply. */
    char age[11] = "-";
    static char response[256]; /* v3 worst-case 240 bytes; one Comms task. */
    if (s.has_sample) (void)snprintf(age, sizeof(age), "%lu",
        (unsigned long)TaskComms_SensorAge(now, s.sample_tick));
    int n;
    if (s.state == IMU_VALID && s.has_sample) {
        n = snprintf(response, sizeof(response),
            "IMU,%u,%lu,2,0,0,%s,%d,%d,%d,%d,%d,%d\n", version, (unsigned long)query, age,
            (int)s.data.Accel_X_RAW, (int)s.data.Accel_Y_RAW, (int)s.data.Accel_Z_RAW,
            (int)s.data.Gyro_X_RAW, (int)s.data.Gyro_Y_RAW, (int)s.data.Gyro_Z_RAW);
    } else {
        n = snprintf(response, sizeof(response), "IMU,%u,%lu,%u,%u,%lu,%s,-,-,-,-,-,-\n",
            version, (unsigned long)query, (unsigned)s.state, (unsigned)s.hal_status,
            (unsigned long)s.i2c_error, age);
    }
    if (version>=2U && n>0 && (size_t)n<sizeof(response)) {
        char cal_age[11]="-", sums[3][12]={{"-"},{"-"},{"-"}};
        if(s.calibration.state==IMU_CAL_VALID) {
            (void)snprintf(cal_age,sizeof(cal_age),"%lu",(unsigned long)TaskComms_SensorAge(now,s.calibration.completed_tick));
            for(unsigned i=0;i<3U;++i)(void)snprintf(sums[i],sizeof(sums[i]),"%ld",(long)s.calibration.sum[i]);
        }
        int extra=snprintf(response+n-1,sizeof(response)-(size_t)n+1U,",%u,%u,%u,%lu,%s,%s,%s,%s\n",
            (unsigned)s.calibration.state,(unsigned)s.calibration.reason,(unsigned)s.calibration.samples,
            (unsigned long)s.calibration.request,cal_age,sums[0],sums[1],sums[2]);
        if(extra<0 || (size_t)extra>=sizeof(response)-(size_t)n+1U)return;
        n=n-1+extra;
    }
    if(version==3U && n>0 && (size_t)n<sizeof(response)) {
        TaskIMUOrientation_t *o=&s.orientation;
        static char values[11][12]; /* No extra stack for optional integers. */
        for(unsigned i=0;i<11U;++i)strcpy(values[i],"-");
        if(o->reference_request) {
            (void)snprintf(values[0],12,"%lu",(unsigned long)o->reference_tick);
            (void)snprintf(values[1],12,"%lu",(unsigned long)TaskComms_SensorAge(now,o->sample_tick));
            (void)snprintf(values[10],12,"%lu",(unsigned long)o->reference_request);
        }
        if(o->state==IMU_YAW_VALID)
            (void)snprintf(values[2],12,"%ld",(long)(o->degrees*1000.0));
        if(o->comparison_state==IMU_COMPARE_VALID && o->state==IMU_YAW_VALID) {
            (void)snprintf(values[3],12,"%lu",(unsigned long)TaskComms_SensorAge(now,o->comparison_tick));
            (void)snprintf(values[4],12,"%lu",(unsigned long)o->comparison_tick);
            const float angles[]={o->gyro_delta_deg,o->encoder_delta_deg,o->difference_deg,o->encoder_reference_deg};
            for(unsigned i=0;i<4U;++i)(void)snprintf(values[5+i],12,"%ld",(long)(angles[i]*1000.0f));
            (void)snprintf(values[9],12,"%u",(unsigned)o->encoder_calibration);
        }
        int extra=snprintf(response+n-1,sizeof(response)-(size_t)n+1U,
            ",%u,%u,%s,%s,%s,%u,%s,%s,%s,%s,%s,%s,%s,%s\n",
            (unsigned)o->state,(unsigned)o->reason,values[0],values[1],values[2],
            (unsigned)o->comparison_state,values[3],values[4],values[5],values[6],values[7],values[8],values[9],values[10]);
        if(extra<0 || (size_t)extra>=sizeof(response)-(size_t)n+1U)return;
        n=n-1+extra;
    }
    if (n > 0 && (size_t)n < sizeof(response)) DrvUart_SendString(uart, response);
}

static bool TaskComms_ImuQuery(const char *message, uint32_t *query, unsigned version) {
    const char *name=version==3U ? "IMU3" : version==2U ? "IMU2" : "IMU";
    if (strcmp(message, name) == 0) { *query = 0U; return true; }
    unsigned prefix=version>=2U ? 7U : 6U;
    if (strncmp(message, version==3U ? "IMU3 Q=" : version==2U ? "IMU2 Q=" : "IMU Q=", prefix) != 0) return false;
    /* Reuse the strict decimal query grammar without accepting other frames. */
    char sensors[32];
    int n = snprintf(sensors, sizeof(sensors), "SENSORS Q=%s", message + prefix);
    return n > 0 && (size_t)n < sizeof(sensors) && TaskComms_SensorsQuery(sensors, query);
}

/* Separate short read-only packet keeps IMU v1/v2/v3 byte-compatible and
 * each reply below one ESP32 255-byte read, without increasing timeouts. */
static void TaskComms_SendFusion(DrvUart_t *uart, uint32_t query) {
    static TaskIMUSnapshot_t s;
    s=(TaskIMUSnapshot_t){.state=IMU_INITIALIZING};(void)TaskIMU_GetSnapshot(&s);
    const TaskIMUFusion_t *f=&s.fusion;
    static char values[11][12], response[192];
    for(unsigned i=0;i<11U;++i)strcpy(values[i],"-");
    TickType_t now=xTaskGetTickCount();
    if(f->reference_request) {
        (void)snprintf(values[0],12,"%lu",(unsigned long)f->reference_request);
        (void)snprintf(values[1],12,"%lu",(unsigned long)f->reference_tick);
        (void)snprintf(values[2],12,"%lu",(unsigned long)f->sample_tick);
        (void)snprintf(values[3],12,"%lu",(unsigned long)TaskComms_SensorAge(now,f->sample_tick));
        (void)snprintf(values[9],12,"%lu",(unsigned long)(f->alpha*1000000.0f));
        (void)snprintf(values[10],12,"%u",(unsigned)f->encoder_calibration);
    }
    if(f->state==IMU_FUSION_VALID) {
        const double angles[]={f->gyro_degrees,f->encoder_degrees,f->degrees,f->difference_gyro_deg,f->difference_encoder_deg};
        for(unsigned i=0;i<5U;++i)(void)snprintf(values[4+i],12,"%ld",(long)(angles[i]*1000.0));
    }
    int n=snprintf(response,sizeof response,"FUS,1,%lu,%u,%u,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u,%s\n",
        (unsigned long)query,(unsigned)f->state,(unsigned)f->reason,values[0],values[1],values[2],values[3],values[4],values[5],
        values[6],values[7],values[8],values[9],(unsigned)IMU_FUSION_TAU_MS,values[10]);
    if(n>0 && (size_t)n<sizeof response)DrvUart_SendString(uart,response);
}

/**
 * @brief Procesa los mensajes recibidos desde el ESP32.
 */
static void TaskComms_ProcessMessage(DrvUart_t *uart, const char *message, TickType_t issued_tick) {
    uint32_t query;
    unsigned mapping_part;
    if (strncmp(message, "NAV Q=", 6U) == 0) {
        char numeric[32];
        const char *suffix=message+6;
        size_t length=strlen(suffix);
        bool stabilization=length>4U && strcmp(suffix+length-4U," S=1")==0;
        int n=stabilization ? snprintf(numeric,sizeof numeric,"SENSORS Q=%.*s",(int)(length-4U),suffix) :
                             snprintf(numeric,sizeof numeric,"SENSORS Q=%s",suffix);
        if (n>0 && (size_t)n<sizeof numeric && TaskComms_SensorsQuery(numeric,&query)) {
            static char response[256];
            n=stabilization ? Navigation_FormatStabilization(response,sizeof response,query) :
                            Navigation_FormatDiagnostics(response,sizeof response,query);
            if (n>0) { DrvUart_SendString(uart,response); }
            else { DrvUart_SendString(uart,"ERR_UNKNOWN\n"); }
            return;
        }
    }
    if(strncmp(message,"IMUF Q=",7U)==0) {
        char numeric[32];int n=snprintf(numeric,sizeof numeric,"SENSORS Q=%s",message+7);
        if(n>0 && (size_t)n<sizeof numeric && TaskComms_SensorsQuery(numeric,&query)) {
            TaskComms_SendFusion(uart,query);return;
        }
    }
    if (strncmp(message,"IMUCAL N=",9U)==0) {
        char numeric[32];
        int n=snprintf(numeric,sizeof(numeric),"SENSORS Q=%s",message+9);
        if(n>0 && (size_t)n<sizeof(numeric) && TaskComms_SensorsQuery(numeric,&query)) {
            bool queued=TaskIMU_RequestCalibration(query,issued_tick);
            char ack[40];
            (void)snprintf(ack,sizeof(ack),"IMUCAL,1,%lu,%u\n",(unsigned long)query,queued?1U:0U);
            DrvUart_SendString(uart,ack); return;
        }
    }
    unsigned version=strncmp(message,"IMU3",4U)==0 ? 3U : strncmp(message,"IMU2",4U)==0 ? 2U : 1U;
    if (TaskComms_ImuQuery(message, &query,version)) {
        TaskComms_SendImu(uart, query,version);
        return;
    }
    if (Mapping_Query(message, &query, &mapping_part)) {
        static char mapping_response[160]; /* Una sola tarea; no crece el stack. */
        int n = Mapping_Format(mapping_response, sizeof(mapping_response), query,
                               mapping_part, xTaskGetTickCount());
        if (n > 0) DrvUart_SendString(uart, mapping_response);
        return;
    }
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

    } else if (strcmp(message, "AUTO") == 0) {
        DrvUart_SendString(uart, TaskControlRobot_RequestAutoAt(issued_tick) ?
            "AUTO_QUEUED\n" : "AUTO_REJECTED\n");

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
