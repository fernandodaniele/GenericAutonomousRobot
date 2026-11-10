/*
 * task_nav.c
 *
 *  Created on: Nov 2, 2026
 *      Author: Grupo 2
 */

#include "task_nav.h"
#include "task_sensors.h"
#include "mid_kinematics.h"
#include "drv_motor.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <math.h>

#define NAV_CMD_QUEUE_LEN   4
#define NAV_EVT_QUEUE_LEN   8

static osThreadId_t nav_task_handle;
static const osThreadAttr_t nav_task_attributes = {
    .name = "nav",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityAboveNormal,
};

static QueueHandle_t cmd_queue;
static QueueHandle_t evt_queue;
static volatile NavStatus_t nav_status = { NAV_STATE_IDLE, 0, 0 };

static void NavTask(void* argument);

void TaskNav_Create(void)
{
    cmd_queue = xQueueCreate(NAV_CMD_QUEUE_LEN, sizeof(NavCommand_t));
    evt_queue = xQueueCreate(NAV_EVT_QUEUE_LEN, sizeof(NavEventMsg_t));
    if (cmd_queue == NULL || evt_queue == NULL) {
        Error_Handler();
    }

    nav_task_handle = osThreadNew(NavTask, NULL, &nav_task_attributes);
    if (nav_task_handle == NULL) {
        Error_Handler();
    }
}

bool TaskNav_SendCommand(NavCommand_t cmd)
{
    if (!cmd_queue) return false;
    return xQueueSend(cmd_queue, &cmd, 0) == pdTRUE;
}

bool TaskNav_GetEvent(NavEventMsg_t* out)
{
    if (!evt_queue || !out) return false;
    return xQueueReceive(evt_queue, out, 0) == pdTRUE;
}

NavStatus_t TaskNav_GetStatus(void)
{
    NavStatus_t s;
    taskENTER_CRITICAL();
    s.state = nav_status.state;
    s.pwm_left = nav_status.pwm_left;
    s.pwm_right = nav_status.pwm_right;
    s.brake = nav_status.brake;
    s.x_mm = nav_status.x_mm;
    s.y_mm = nav_status.y_mm;
    s.theta_mrad = nav_status.theta_mrad;
    taskEXIT_CRITICAL();
    return s;
}

static NavRange_t to_range(UltrasoundStatus_t st)
{
    switch (st) {
        case ULTRASOUND_STATUS_OK:        return NAV_RANGE_OK;
        case ULTRASOUND_STATUS_FAR:       return NAV_RANGE_FAR;
        case ULTRASOUND_STATUS_TOO_CLOSE: return NAV_RANGE_TOO_CLOSE;
        default:                          return NAV_RANGE_NO_ECHO;
    }
}

static int16_t ramp(int16_t current, int16_t target)
{
    int32_t diff = (int32_t)target - (int32_t)current;
    if (diff > TASK_NAV_RAMP_STEP_PWM)  diff = TASK_NAV_RAMP_STEP_PWM;
    if (diff < -TASK_NAV_RAMP_STEP_PWM) diff = -TASK_NAV_RAMP_STEP_PWM;
    return (int16_t)(current + diff);
}

/* Indicador en LD3 (naranja):
 *   apagado            -> IDLE (esperando START)
 *   encendido fijo     -> FORWARD
 *   parpadeo lento     -> SCAN (midiendo o maniobrando entre tramos)
 *   parpadeo rapido    -> esquivando obstaculo o precipicio
 *   N destellos + pausa -> FAULT: 1 = MPU6050 no responde,
 *                         2 = ultrasonido frontal sin eco,
 *                         3 = sensores sin actualizar,
 *                         4 = atrapado: bucle de retroceso o de escaneo (salir con STOP y START) */
static void status_led(NavState_t state, bool fault_latched, const NavInput_t* in, uint32_t now)
{
    GPIO_PinState on = GPIO_PIN_RESET;
    switch (state) {
        case NAV_STATE_FORWARD:
            on = GPIO_PIN_SET;
            break;
        case NAV_STATE_SCAN:
            on = ((now / 500u) % 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            break;
        case NAV_STATE_AVOID_OBSTACLE:
        case NAV_STATE_CLIFF_RECOVER:
            on = ((now / 100u) % 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            break;
        case NAV_STATE_FAULT: {
            uint32_t code = fault_latched ? 4u
                          : (NAV_REQUIRE_IMU && !in->imu_ok) ? 1u
                          : (in->front_no_echo_count >= NAV_FRONT_NO_ECHO_MAX) ? 2u : 3u;
            uint32_t slot = (now % 2400u) / 200u;          /* 12 ranuras de 200 ms */
            on = (slot < code * 2u && (slot % 2u) == 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            break;
        }
        default:
            break;
    }
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, on);
}

static void NavTask(void* argument)
{
    (void) argument;

    NavContext_t ctx;
    Nav_Init(&ctx);

    int16_t pwm_l = 0;
    int16_t pwm_r = 0;
    float x_mm = 0.0f, y_mm = 0.0f, theta = 0.0f;
    NavState_t prev_state = NAV_STATE_IDLE;
    SensorSnapshot_t snap;
    TickType_t last_wake = xTaskGetTickCount();

    Motors_StopAll();

    for (;;)
    {
        NavInput_t in = {0};
        const uint32_t now = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
        in.now_ms = now;

        /* Un comando por periodo; los demas quedan para el proximo */
        NavCommand_t cmd;
        in.command = (xQueueReceive(cmd_queue, &cmd, 0) == pdTRUE) ? cmd : NAV_CMD_NONE;

#if TASK_NAV_AUTOSTART
        /* START automatico, una sola vez, a los 3 s de encender */
        static bool autostart_done = false;
        if (!autostart_done && now >= TASK_NAV_AUTOSTART_DELAY_MS) {
            autostart_done = true;
            if (in.command == NAV_CMD_NONE) {
                in.command = NAV_CMD_START;
            }
        }
#endif

        if (TaskSensors_GetSnapshot(&snap)) {
            in.snapshot_stale = (now - snap.timestamp_ms) > TASK_NAV_STALE_MS;
            in.front = to_range(snap.us_status[ULTRASOUND_FRONT]);
            in.front_mm = snap.dist_mm[ULTRASOUND_FRONT];
            in.left = to_range(snap.us_status[ULTRASOUND_LEFT]);
            in.left_mm = snap.dist_mm[ULTRASOUND_LEFT];
            in.right = to_range(snap.us_status[ULTRASOUND_RIGHT]);
            in.right_mm = snap.dist_mm[ULTRASOUND_RIGHT];
            in.front_no_echo_count = snap.front_no_echo_count;
            in.sensor_ms = snap.timestamp_ms;
            in.cliff_left = snap.cliff_left;
            in.cliff_right = snap.cliff_right;
            in.gyro_z_dps = snap.gyro_z_dps;
            in.imu_ok = snap.imu_ok;
        } else {
            in.snapshot_stale = true;
            in.front = NAV_RANGE_NO_ECHO;
        }

        NavOutput_t out = Nav_Step(&ctx, &in);

        /* Seguridad: con la foto vieja los motores se frenan siempre */
        if (in.snapshot_stale && out.state != NAV_STATE_IDLE) {
            out.brake = true;
        }

        if (out.brake) {
            pwm_l = 0;
            pwm_r = 0;
            Motor_Brake(Motor_GetLeft());
            Motor_Brake(Motor_GetRight());
        } else if (out.state == NAV_STATE_IDLE) {
            pwm_l = 0;
            pwm_r = 0;
            Motors_StopAll();
        } else {
            RobotCommand_t rc = { out.linear_mms, out.angular_rads };
            int16_t target_l = 0;
            int16_t target_r = 0;
            Kinematics_Compute(rc, &target_l, &target_r);
            pwm_l = ramp(pwm_l, target_l);
            pwm_r = ramp(pwm_r, target_r);
            Motors_SetSpeeds(pwm_l, pwm_r);
        }

        /* Odometria estimada a partir del PWM aplicado (no hay encoders) */
        if (prev_state == NAV_STATE_IDLE && out.state != NAV_STATE_IDLE) {
            x_mm = y_mm = theta = 0.0f;
        }
        prev_state = out.state;
        {
            const float k = ROBOT_MAX_WHEEL_SPEED_MMS / (float)MOTOR_MAX_SPEED;
            const float dt = (float)TASK_NAV_PERIOD_MS / 1000.0f;
            float v_l = (float)pwm_l * k;
            float v_r = (float)pwm_r * k;
            float v = (v_l + v_r) / 2.0f;
            x_mm += v * cosf(theta) * dt;
            y_mm += v * sinf(theta) * dt;
            theta += ((v_r - v_l) / ROBOT_WHEEL_BASE) * dt;
        }

        status_led(out.state, out.fault_latched, &in, now);

        taskENTER_CRITICAL();
        nav_status.state = out.state;
        nav_status.pwm_left = pwm_l;
        nav_status.pwm_right = pwm_r;
        nav_status.brake = out.brake;
        nav_status.x_mm = (int32_t)x_mm;
        nav_status.y_mm = (int32_t)y_mm;
        nav_status.theta_mrad = (int32_t)(theta * 1000.0f);
        taskEXIT_CRITICAL();

        if (out.events != 0u) {
            NavEventMsg_t ev = { now, out.events };
            (void) xQueueSend(evt_queue, &ev, 0);   // Cola llena: se descarta
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_NAV_PERIOD_MS));
    }
}
