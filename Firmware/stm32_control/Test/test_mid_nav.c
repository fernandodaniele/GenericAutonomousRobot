/*
 * test_mid_nav.c - Prueba de mid_nav en la PC con secuencias simuladas.
 * Compilar y correr desde Firmware/stm32_control:
 *   gcc -Wall -Wextra -ICore/Inc Test/test_mid_nav.c Core/Src/mid_nav.c -o test_mid_nav && ./test_mid_nav
 */
#include <stdio.h>
#include <string.h>
#include "mid_nav.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FALLA linea %d: %s\n", __LINE__, #c); fails++; } } while (0)

static NavInput_t base(uint32_t t)
{
    NavInput_t in;
    memset(&in, 0, sizeof(in));
    in.now_ms = t;
    in.front = NAV_RANGE_FAR;
    in.left = NAV_RANGE_FAR;
    in.right = NAV_RANGE_FAR;
    in.imu_ok = true;
    return in;
}

/* Avanza el tiempo en pasos de 20 ms con la misma entrada */
static NavOutput_t run(NavContext_t *c, NavInput_t in, uint32_t *t, uint32_t ms)
{
    NavOutput_t o = {0};
    for (uint32_t e = 0; e < ms; e += 20) {
        *t += 20; in.now_ms = *t; in.command = NAV_CMD_NONE;
        o = Nav_Step(c, &in);
    }
    return o;
}

int main(void)
{
    NavContext_t c; uint32_t t = 0; NavInput_t in; NavOutput_t o;

    /* Arranque seguro: sin START queda en IDLE */
    Nav_Init(&c);
    o = run(&c, base(0), &t, 200);
    CHECK(o.state == NAV_STATE_IDLE && o.linear_mms == 0.0f);

    /* START -> FORWARD */
    in = base(t); in.command = NAV_CMD_START; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms > 0.0f);

    /* Espacio abierto (eco largo) sigue avanzando */
    o = run(&c, base(t), &t, 500);
    CHECK(o.state == NAV_STATE_FORWARD);

    /* Obstaculo a 4 cm, derecha mas libre -> frena y gira a la derecha */
    in = base(t); in.front = NAV_RANGE_OK; in.front_mm = 40;
    in.left = NAV_RANGE_OK; in.left_mm = 100; in.right = NAV_RANGE_OK; in.right_mm = 900;
    t += 20; in.now_ms = t; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE && o.brake);
    o = run(&c, in, &t, NAV_BRAKE_MS + 40);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE && !o.brake && o.angular_rads < 0.0f);

    /* 6 cm: todavia no alcanza el margen de 8 cm */
    in.front_mm = 60; o = run(&c, in, &t, 40);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE);
    /* 9 cm: libre -> FORWARD */
    in.front_mm = 90; o = run(&c, in, &t, 20);
    CHECK(o.state == NAV_STATE_FORWARD);

    /* "Demasiado cerca" (<2 cm) tambien es obstaculo */
    in = base(t); in.front = NAV_RANGE_TOO_CLOSE; o = run(&c, in, &t, 20);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE);
    in.front = NAV_RANGE_FAR; o = run(&c, in, &t, NAV_BRAKE_MS + 60);
    CHECK(o.state == NAV_STATE_FORWARD);

    /* Precipicio izquierdo: evento, freno, retrocede, gira a la derecha */
    in = base(t); in.cliff_left = true; t += 20; in.now_ms = t; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && o.brake && (o.events & NAV_EVT_CLIFF));
    o = run(&c, in, &t, NAV_BRAKE_MS + 40);
    CHECK(o.linear_mms < 0.0f);
    in.cliff_left = false;
    o = run(&c, in, &t, NAV_BACKUP_MS);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && o.angular_rads < 0.0f);
    o = run(&c, in, &t, NAV_CLIFF_TURN_MS + 20);
    CHECK(o.state == NAV_STATE_FORWARD);

    /* Precipicio tiene prioridad sobre obstaculo */
    in = base(t); in.cliff_right = true; in.front = NAV_RANGE_TOO_CLOSE;
    o = run(&c, in, &t, 20);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER);
    in = base(t); o = run(&c, in, &t, NAV_BRAKE_MS + NAV_BACKUP_MS + NAV_CLIFF_TURN_MS + 100);
    CHECK(o.state == NAV_STATE_FORWARD);

    /* Giro brusco: alerta sin cambiar el estado, con retencion */
    in = base(t); in.gyro_z_dps = -350; t += 20; in.now_ms = t; o = Nav_Step(&c, &in);
    CHECK((o.events & NAV_EVT_ALERT) && o.state == NAV_STATE_FORWARD);
    t += 20; in.now_ms = t; o = Nav_Step(&c, &in);
    CHECK(!(o.events & NAV_EVT_ALERT));

    /* Frontal sin eco 3 veces -> FAULT; START no sale, STOP+START si */
    in = base(t); in.front = NAV_RANGE_NO_ECHO; in.front_no_echo_count = 2;
    o = run(&c, in, &t, 20); CHECK(o.state == NAV_STATE_FORWARD);
    in.front_no_echo_count = 3; t += 20; in.now_ms = t; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_FAULT && o.brake && (o.events & NAV_EVT_FAULT));
    in = base(t); in.command = NAV_CMD_START; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_FAULT);
    in.command = NAV_CMD_STOP; o = Nav_Step(&c, &in); CHECK(o.state == NAV_STATE_IDLE);
    in.command = NAV_CMD_START; o = Nav_Step(&c, &in); CHECK(o.state == NAV_STATE_FORWARD);

    /* Foto vieja -> FAULT */
    in = base(t); in.snapshot_stale = true; o = run(&c, in, &t, 20);
    CHECK(o.state == NAV_STATE_FAULT);

    /* Sin MPU6050 -> FAULT al arrancar (solo si NAV_REQUIRE_IMU) */
#if NAV_REQUIRE_IMU
    Nav_Init(&c); in = base(t); in.imu_ok = false; in.command = NAV_CMD_START;
    o = Nav_Step(&c, &in); CHECK(o.state == NAV_STATE_FAULT);
#endif

    /* STOP desde cualquier estado -> IDLE */
    Nav_Init(&c); in = base(t); in.command = NAV_CMD_START; Nav_Step(&c, &in);
    in.command = NAV_CMD_STOP; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_IDLE && o.linear_mms == 0.0f);

    printf(fails ? "%d pruebas fallaron\n" : "Todas las pruebas de mid_nav pasaron\n", fails);
    return fails ? 1 : 0;
}
