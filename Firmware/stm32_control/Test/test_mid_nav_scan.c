/*
 * test_mid_nav_scan.c - Prueba en la PC del avance por tramos con escaneo.
 * Compilar y correr desde Firmware/stm32_control:
 *   gcc -Wall -Wextra -ICore/Inc Test/test_mid_nav_scan.c Core/Src/mid_nav.c -o test_mid_nav_scan && ./test_mid_nav_scan
 */
#include <stdio.h>
#include <string.h>
#include "mid_nav.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FALLA linea %d: %s\n", __LINE__, #c); fails++; } } while (0)

#define STEP_MS 20
/* Tiempo quieto antes de medir, mas tres lecturas y la decision */
#define SCAN_TOTAL_MS (NAV_BRAKE_MS + NAV_SETTLE_MS + 6 * STEP_MS)

static uint32_t t = 0;

static NavInput_t base(void)
{
    NavInput_t in;
    memset(&in, 0, sizeof(in));
    in.front = NAV_RANGE_FAR;
    in.left = NAV_RANGE_FAR;
    in.right = NAV_RANGE_FAR;
    in.imu_ok = true;
    return in;
}

static NavOutput_t step(NavContext_t *c, NavInput_t in)
{
    t += STEP_MS; in.now_ms = t; in.sensor_ms = t;
    return Nav_Step(c, &in);
}

static NavOutput_t run(NavContext_t *c, NavInput_t in, uint32_t ms)
{
    NavOutput_t o = {0};
    for (uint32_t e = 0; e < ms; e += STEP_MS) o = step(c, in);
    return o;
}

static NavOutput_t start(NavContext_t *c)
{
    Nav_Init(c);
    NavInput_t in = base();
    in.command = NAV_CMD_START;
    return step(c, in);
}

static NavInput_t walls(uint32_t front, uint32_t left, uint32_t right)
{
    NavInput_t in = base();
    in.front = NAV_RANGE_OK; in.front_mm = front;
    in.left = NAV_RANGE_OK; in.left_mm = left;
    in.right = NAV_RANGE_OK; in.right_mm = right;
    return in;
}

int main(void)
{
    NavContext_t c;
    NavOutput_t o;
    NavInput_t in;

    /* START: frena y mide antes de moverse */
    o = start(&c);
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* Frente libre: arranca un tramo recto */
    o = run(&c, base(), SCAN_TOTAL_MS + STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && !o.brake);
    CHECK(o.linear_mms == NAV_FWD_SPEED_MMS && o.angular_rads == 0.0f);

    /* Rumbo: si se desvia a la derecha corrige a la izquierda y viceversa */
    in = base(); in.heading_mdeg = -5000;
    o = step(&c, in);
    CHECK(o.state == NAV_STATE_FORWARD && o.angular_rads > 0.0f && o.linear_mms == NAV_FWD_SPEED_MMS);
    in.heading_mdeg = 5000; o = step(&c, in);
    CHECK(o.angular_rads < 0.0f);
    in.heading_mdeg = 90000; o = step(&c, in);
    CHECK(o.angular_rads == -NAV_HEADING_MAX_RAD_S);              /* correccion limitada */
    in.heading_mdeg = NAV_HEADING_DEADBAND_MDEG - 100; o = step(&c, in);
    CHECK(o.angular_rads == 0.0f);                                /* desvio minimo: recto */
    in.heading_mdeg = -5000; in.imu_ok = false; o = step(&c, in);
    CHECK(o.angular_rads == 0.0f);                                /* sin IMU: recto */

    /* Tramo: sigue recto hasta NAV_STEP_MS y frena para escanear */
    o = run(&c, base(), NAV_STEP_MS - 15 * STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms == NAV_FWD_SPEED_MMS);
    o = run(&c, base(), 12 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* El rumbo de referencia es el del momento en que decide avanzar */
    start(&c);
    in = base(); in.heading_mdeg = 30000;
    o = run(&c, in, SCAN_TOTAL_MS + STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && o.angular_rads == 0.0f);
    in.heading_mdeg = 25000; o = step(&c, in);
    CHECK(o.angular_rads > 0.0f);

    /* No decide antes de terminar la pausa y las lecturas */
    in = walls(100, 900, 100);
    start(&c);
    o = run(&c, in, NAV_BRAKE_MS + NAV_SETTLE_MS - 2 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* Frente bloqueado, izquierda libre: dobla a la izquierda */
    o = run(&c, in, SCAN_TOTAL_MS - NAV_BRAKE_MS - NAV_SETTLE_MS + 4 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && !o.brake && o.angular_rads > 0.0f && o.linear_mms == 0.0f);

    /* El giro dura NAV_SCAN_TURN_MS y despues vuelve a medir quieto */
    o = run(&c, in, NAV_SCAN_TURN_MS - 10 * STEP_MS);
    CHECK(o.angular_rads > 0.0f);
    o = run(&c, in, 11 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* Frente bloqueado, derecha libre: dobla a la derecha */
    start(&c);
    o = run(&c, walls(100, 100, 900), SCAN_TOTAL_MS + 4 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.angular_rads < 0.0f);

    /* Ambos lados libres: el mas amplio */
    start(&c);
    o = run(&c, walls(100, 400, 900), SCAN_TOTAL_MS + 4 * STEP_MS);
    CHECK(o.angular_rads < 0.0f);

    /* Todo bloqueado: retrocede */
    start(&c);
    o = run(&c, walls(100, 100, 100), SCAN_TOTAL_MS + 4 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.linear_mms == NAV_BACK_SPEED_MMS && o.angular_rads == 0.0f);

    /* Obstaculo durante el tramo: no espera el fin, frena y escanea */
    start(&c);
    run(&c, base(), SCAN_TOTAL_MS + STEP_MS);
    o = step(&c, walls(NAV_OBSTACLE_MM - 10, 900, 900));
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* Un solo sensor valido por foto, como publica task_sensors */
    start(&c);
    {
        const uint32_t pause = NAV_BRAKE_MS + NAV_SETTLE_MS;
        NavRange_t none = NAV_RANGE_NO_ECHO;
        run(&c, base(), pause);
        for (int i = 0; i < 3; i++) {
            in = base(); in.left = none; in.right = none;
            in.front = NAV_RANGE_OK; in.front_mm = 100;
            o = step(&c, in);
            in.front = none; in.left = NAV_RANGE_OK; in.left_mm = 100;
            o = step(&c, in);
            in.left = none; in.front = NAV_RANGE_OK; in.front_mm = 100;
            o = step(&c, in);
            in.front = none; in.right = NAV_RANGE_OK; in.right_mm = 900;
            o = step(&c, in);
        }
        o = step(&c, in);
        CHECK(o.state == NAV_STATE_SCAN && o.angular_rads < 0.0f);
    }

    /* La misma foto repetida no suma lecturas */
    start(&c);
    in = walls(100, 900, 100);
    in.sensor_ms = 777;
    for (uint32_t e = 0; e < NAV_BRAKE_MS + NAV_SETTLE_MS + 5 * STEP_MS; e += STEP_MS) {
        t += STEP_MS; in.now_ms = t;
        o = Nav_Step(&c, &in);
    }
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* Una lectura espuria no cambia la decision: gana la mediana */
    start(&c);
    run(&c, base(), NAV_BRAKE_MS + NAV_SETTLE_MS);
    step(&c, walls(900, 900, 100));
    step(&c, walls(100, 900, 100));
    step(&c, walls(900, 900, 100));
    o = step(&c, walls(900, 900, 100));
    CHECK(o.state == NAV_STATE_FORWARD);

    /* Frontal sin eco en toda la ventana: no avanza a ciegas */
    start(&c);
    in = base(); in.front = NAV_RANGE_NO_ECHO;
    o = run(&c, in, NAV_BRAKE_MS + NAV_SETTLE_MS + NAV_SCAN_MAX_MS + 4 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.linear_mms == 0.0f);

    /* Precipicio durante el escaneo: tiene prioridad */
    start(&c);
    in = base(); in.cliff_left = true;
    o = run(&c, in, (NAV_CLIFF_CONFIRM + 1) * STEP_MS);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER);

    /* Tras el borde mide antes de volver a avanzar */
    in = base();
    o = run(&c, in, NAV_BRAKE_MS + 2 * NAV_SETTLE_MS + NAV_BACKUP_MS + NAV_CLIFF_TURN_MS + 10 * STEP_MS);
    for (int i = 0; i < 100 && o.state == NAV_STATE_CLIFF_RECOVER; i++) o = step(&c, in);
    CHECK(o.state == NAV_STATE_SCAN);

    /* Atrapado: tras NAV_SCAN_STUCK_MAX maniobras queda en FAULT */
    start(&c);
    in = walls(100, 100, 100);
    {
        uint32_t evt = 0;
        o = step(&c, in);
        for (uint32_t e = 0; e < 60000u && o.state != NAV_STATE_FAULT; e += STEP_MS) {
            o = step(&c, in);
            evt |= o.events;
        }
        CHECK(o.state == NAV_STATE_FAULT && o.brake && o.fault_latched && (evt & NAV_EVT_FAULT));
    }

    /* No se recupera solo; STOP y START si */
    o = run(&c, base(), 1000);
    CHECK(o.state == NAV_STATE_FAULT);
    in = base(); in.command = NAV_CMD_STOP; o = step(&c, in);
    CHECK(o.state == NAV_STATE_IDLE && !o.fault_latched);
    in.command = NAV_CMD_START; o = step(&c, in);
    CHECK(o.state == NAV_STATE_SCAN);

    /* Un tramo completo reinicia el contador de maniobras */
    start(&c);
    for (int k = 0; k < NAV_SCAN_STUCK_MAX + 2; k++) {
        o = step(&c, walls(100, 900, 100));
        for (int i = 0; i < 200 && o.angular_rads == 0.0f; i++) o = step(&c, walls(100, 900, 100));
        CHECK(o.state == NAV_STATE_SCAN && o.angular_rads > 0.0f);
        o = run(&c, walls(100, 900, 100), NAV_SCAN_TURN_MS + STEP_MS);
        CHECK(o.state == NAV_STATE_SCAN && o.brake);
        for (int i = 0; i < 200 && o.state != NAV_STATE_FORWARD; i++) o = step(&c, base());
        CHECK(o.state == NAV_STATE_FORWARD);
        o = run(&c, base(), NAV_STEP_MS + 4 * STEP_MS);
        CHECK(o.state == NAV_STATE_SCAN);
    }

    /* STOP desde el escaneo */
    start(&c);
    in = base(); in.command = NAV_CMD_STOP; o = step(&c, in);
    CHECK(o.state == NAV_STATE_IDLE);

    CHECK(strcmp(Nav_StateName(NAV_STATE_SCAN), "SCAN") == 0);

    printf(fails ? "%d pruebas fallaron\n" : "Todas las pruebas del escaneo pasaron\n", fails);
    return fails ? 1 : 0;
}
