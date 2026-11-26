/*
 * test_mid_nav.c - Prueba de mid_nav en la PC: precipicio, fallas, alerta y STOP.
 * Compilar y correr desde Firmware/stm32_control:
 *   gcc -Wall -Wextra -ICore/Inc Test/test_mid_nav.c Core/Src/mid_nav.c -o test_mid_nav && ./test_mid_nav
 */
#include <stdio.h>
#include <string.h>
#include "mid_nav.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FALLA linea %d: %s\n", __LINE__, #c); fails++; } } while (0)

#define STEP_MS 20

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

/* Un paso de 20 ms con la entrada dada (sin comando) */
static NavOutput_t step(NavContext_t *c, NavInput_t *in, uint32_t *t)
{
    *t += STEP_MS; in->now_ms = *t; in->sensor_ms = *t; in->command = NAV_CMD_NONE;
    return Nav_Step(c, in);
}

/* Avanza el tiempo en pasos de 20 ms con la misma entrada */
static NavOutput_t run(NavContext_t *c, NavInput_t in, uint32_t *t, uint32_t ms)
{
    NavOutput_t o = {0};
    for (uint32_t e = 0; e < ms; e += STEP_MS) o = step(c, &in, t);
    return o;
}

/* Avanza hasta que el estado sea 'st' (o hasta max_ms). Devuelve el ultimo paso. */
static NavOutput_t run_until(NavContext_t *c, NavInput_t in, uint32_t *t, NavState_t st,
                             uint32_t max_ms, uint32_t *took)
{
    NavOutput_t o = {0};
    uint32_t e = 0;
    for (; e < max_ms; e += STEP_MS) {
        o = step(c, &in, t);
        if (o.state == st) break;
    }
    if (took) *took = e + STEP_MS;
    return o;
}

static void start(NavContext_t *c, uint32_t t)
{
    Nav_Init(c);
    NavInput_t in = base(t); in.command = NAV_CMD_START;
    Nav_Step(c, &in);
}

/* Escaneo con todo libre: pausa quieto, tres lecturas y la decision */
#define SCAN_TOTAL_MS (NAV_BRAKE_MS + NAV_SETTLE_MS + 6 * STEP_MS)
/* Recuperacion de borde completa; el rumbo no cambia en esta prueba, asi que
 * el giro termina por NAV_TURN_TIMEOUT_MS */
#define CLIFF_TOTAL_MS (NAV_BRAKE_MS + NAV_SETTLE_MS + NAV_CLIFF_BACKUP_MS + NAV_SETTLE_MS \
                        + NAV_TURN_TIMEOUT_MS + NAV_SETTLE_MS)

/* START y espera a estar avanzando */
static NavOutput_t start_forward(NavContext_t *c, uint32_t *t)
{
    start(c, *t);
    return run_until(c, base(*t), t, NAV_STATE_FORWARD, SCAN_TOTAL_MS + 200, NULL);
}

int main(void)
{
    NavContext_t c; uint32_t t = 0; NavInput_t in; NavOutput_t o;

    /* Arranque seguro: sin START queda en IDLE */
    Nav_Init(&c);
    o = run(&c, base(0), &t, 200);
    CHECK(o.state == NAV_STATE_IDLE && o.linear_mms == 0.0f);

    /* START -> mide quieto antes de moverse */
    in = base(t); in.command = NAV_CMD_START; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* Espacio abierto: arranca y sigue avanzando */
    o = run_until(&c, base(t), &t, NAV_STATE_FORWARD, SCAN_TOTAL_MS + 200, NULL);
    CHECK(o.state == NAV_STATE_FORWARD);
    o = run(&c, base(t), &t, 500);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms == NAV_FWD_SPEED_MMS);

    /* ---- Precipicio izquierdo: freno, pausa, atras, pausa, giro a la derecha en el lugar ---- */
    in = base(t); in.cliff_left = true;
    o = run_until(&c, in, &t, NAV_STATE_CLIFF_RECOVER, 500, NULL);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && o.brake && (o.events & NAV_EVT_CLIFF));
    o = run(&c, in, &t, NAV_BRAKE_MS + NAV_SETTLE_MS + 40);
    CHECK(o.linear_mms == NAV_BACK_SPEED_MMS && o.angular_rads == 0.0f);
    in.cliff_left = false;
    o = run(&c, in, &t, NAV_CLIFF_BACKUP_MS);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && o.brake);           /* pausa antes de girar */
    o = run(&c, in, &t, NAV_SETTLE_MS);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && o.angular_rads == -NAV_TURN_SPOT_RAD_S);
    CHECK(o.linear_mms == 0.0f);                                    /* en el lugar, sin pivote */
    /* Al terminar el giro vuelve a medir (no sale avanzando) */
    o = run_until(&c, in, &t, NAV_STATE_SCAN, NAV_TURN_TIMEOUT_MS + NAV_SETTLE_MS + 100, NULL);
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* Precipicio derecho: gira a la izquierda */
    start_forward(&c, &t);
    in = base(t); in.cliff_right = true;
    o = run_until(&c, in, &t, NAV_STATE_CLIFF_RECOVER, 500, NULL);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER);
    in = base(t);
    o = run(&c, in, &t, NAV_BRAKE_MS + 2 * NAV_SETTLE_MS + NAV_CLIFF_BACKUP_MS + 2 * STEP_MS);
    CHECK(o.angular_rads == NAV_TURN_SPOT_RAD_S && o.linear_mms == 0.0f);

    /* Los dos sensores ven borde: gira a la derecha */
    start_forward(&c, &t);
    in = base(t); in.cliff_left = true; in.cliff_right = true;
    o = run_until(&c, in, &t, NAV_STATE_CLIFF_RECOVER, 500, NULL);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER);
    in = base(t);
    o = run(&c, in, &t, NAV_BRAKE_MS + 2 * NAV_SETTLE_MS + NAV_CLIFF_BACKUP_MS + 2 * STEP_MS);
    CHECK(o.angular_rads == -NAV_TURN_SPOT_RAD_S);

    /* Precipicio tiene prioridad sobre obstaculo, tambien durante el escaneo */
    start(&c, t);
    in = base(t); in.cliff_right = true; in.front = NAV_RANGE_TOO_CLOSE;
    o = run_until(&c, in, &t, NAV_STATE_CLIFF_RECOVER, 500, NULL);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER);
    in = base(t); o = run_until(&c, in, &t, NAV_STATE_SCAN, CLIFF_TOTAL_MS + 200, NULL);
    CHECK(o.state == NAV_STATE_SCAN);

    /* Giro brusco: alerta sin cambiar el estado, con retencion */
    start_forward(&c, &t);
    in = base(t); in.gyro_z_dps = -350; o = step(&c, &in, &t);
    CHECK((o.events & NAV_EVT_ALERT) && o.state == NAV_STATE_FORWARD);
    o = step(&c, &in, &t);
    CHECK(!(o.events & NAV_EVT_ALERT));

    /* Frontal sin eco: sigue andando, no se queda quieto */
    in = base(t); in.front = NAV_RANGE_NO_ECHO; in.front_no_echo_count = 200;
    o = run(&c, in, &t, 100); CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms > 0.0f);

    /* Foto vieja -> FAULT y freno; al volver los datos, retoma solo midiendo */
    in = base(t); in.snapshot_stale = true; o = run(&c, in, &t, STEP_MS);
    CHECK(o.state == NAV_STATE_FAULT && o.brake && (o.events & NAV_EVT_FAULT));
    in = base(t); o = run(&c, in, &t, STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && !o.fault_latched);

    /* Sin MPU6050 -> FAULT al arrancar; al responder, retoma solo */
#if NAV_REQUIRE_IMU
    Nav_Init(&c); in = base(t); in.imu_ok = false; in.command = NAV_CMD_START;
    o = Nav_Step(&c, &in); CHECK(o.state == NAV_STATE_FAULT && o.brake && !o.fault_latched);
    in = base(t); o = run(&c, in, &t, STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN);
#endif

    /* STOP desde cualquier estado -> IDLE */
    start(&c, t);
    in = base(t); in.command = NAV_CMD_STOP; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_IDLE && o.linear_mms == 0.0f);
    start_forward(&c, &t);
    in = base(t); in.command = NAV_CMD_STOP; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_IDLE && o.linear_mms == 0.0f);

    /* ===== El borde se confirma con NAV_CLIFF_CONFIRM lecturas ===== */
    start_forward(&c, &t);
    in = base(t); in.cliff_left = true;
    for (int i = 0; i < NAV_CLIFF_CONFIRM - 1; i++) {
        o = step(&c, &in, &t);
        CHECK(o.state == NAV_STATE_FORWARD && !(o.events & NAV_EVT_CLIFF));
    }
    o = step(&c, &in, &t);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && (o.events & NAV_EVT_CLIFF));

    /* Falso positivo aislado (una o dos lecturas) no dispara la maniobra */
    start(&c, t);
    for (int k = 0; k < 5; k++) {
        in = base(t); in.cliff_left = true; o = step(&c, &in, &t); o = step(&c, &in, &t);
        in.cliff_left = false; o = step(&c, &in, &t);
        CHECK(o.state != NAV_STATE_CLIFF_RECOVER);
    }

    /* La misma foto repetida (task_nav corre mas rapido que task_sensors)
     * no cuenta como lecturas nuevas */
    start(&c, t);
    in = base(t); in.cliff_left = true; in.sensor_ms = 12345;
    for (int i = 0; i < 10; i++) { t += STEP_MS; in.now_ms = t; o = Nav_Step(&c, &in); }
    CHECK(o.state != NAV_STATE_CLIFF_RECOVER);

    /* ===== Bucle de retroceso -> FAULT frenado ===== */
    start_forward(&c, &t);
    in = base(t); in.cliff_left = true;
    o = run_until(&c, in, &t, NAV_STATE_CLIFF_RECOVER, 500, NULL);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER);
    /* Sigue viendo borde al terminar cada giro: repite el retroceso */
    o = run(&c, in, &t, CLIFF_TOTAL_MS - NAV_SETTLE_MS + 40);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER);                         /* 2.a vez */
    uint32_t evt = 0;
    for (uint32_t e = 0; e < 2 * CLIFF_TOTAL_MS && o.state != NAV_STATE_FAULT; e += STEP_MS) {
        o = step(&c, &in, &t); evt |= o.events;
    }
    CHECK(o.state == NAV_STATE_FAULT && o.brake && o.fault_latched && (evt & NAV_EVT_FAULT));
    /* No se recupera solo aunque desaparezca el borde */
    in = base(t); o = run(&c, in, &t, 1000);
    CHECK(o.state == NAV_STATE_FAULT && o.brake);
    /* START solo no alcanza; STOP + START si */
    in.command = NAV_CMD_START; t += STEP_MS; in.now_ms = t; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_FAULT);
    in.command = NAV_CMD_STOP; o = Nav_Step(&c, &in); CHECK(o.state == NAV_STATE_IDLE && !o.fault_latched);
    in.command = NAV_CMD_START; o = Nav_Step(&c, &in); CHECK(o.state == NAV_STATE_SCAN);

    /* Bordes separados por NAV_CLIFF_LOOP_RESET_MS de avance no cuentan como bucle */
    start_forward(&c, &t);
    for (int k = 0; k < NAV_CLIFF_LOOP_MAX + 1; k++) {
        in = base(t); in.cliff_left = true;
        o = run_until(&c, in, &t, NAV_STATE_CLIFF_RECOVER, 500, NULL);
        CHECK(o.state == NAV_STATE_CLIFF_RECOVER);
        in = base(t);
        o = run_until(&c, in, &t, NAV_STATE_SCAN, CLIFF_TOTAL_MS + 200, NULL);
        o = run_until(&c, in, &t, NAV_STATE_FORWARD, SCAN_TOTAL_MS + 200, NULL);
        o = run(&c, in, &t, NAV_CLIFF_LOOP_RESET_MS + 100);
        CHECK(o.state == NAV_STATE_FORWARD);
    }

    /* Bordes seguidos con poco avance entre medio si son un bucle */
    start_forward(&c, &t);
    for (int k = 0; k < NAV_CLIFF_LOOP_MAX; k++) {
        in = base(t); in.cliff_left = true;
        o = run_until(&c, in, &t, (k < NAV_CLIFF_LOOP_MAX - 1) ? NAV_STATE_CLIFF_RECOVER : NAV_STATE_FAULT, 500, NULL);
        if (k < NAV_CLIFF_LOOP_MAX - 1) {
            in = base(t);
            o = run_until(&c, in, &t, NAV_STATE_SCAN, CLIFF_TOTAL_MS + 200, NULL);
            o = run_until(&c, in, &t, NAV_STATE_FORWARD, SCAN_TOTAL_MS + 200, NULL);
            o = run(&c, in, &t, 500);                 /* avanza menos de NAV_CLIFF_LOOP_RESET_MS */
        }
    }
    CHECK(o.state == NAV_STATE_FAULT && o.fault_latched);

    /* ===== Derecho sin eco: no dobla hacia el lado sin lectura ===== */
    start(&c, t);
    in = base(t); in.front = NAV_RANGE_OK; in.front_mm = 100;
    in.left = NAV_RANGE_OK; in.left_mm = 900; in.right = NAV_RANGE_NO_ECHO;
    o = run(&c, in, &t, NAV_BRAKE_MS + NAV_SETTLE_MS + NAV_SCAN_MAX_MS + 4 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.angular_rads > 0.0f);   /* dobla a la izquierda */

    CHECK(strcmp(Nav_StateName(NAV_STATE_CLIFF_RECOVER), "CLIFF") == 0);

    printf(fails ? "%d pruebas fallaron\n" : "Todas las pruebas de mid_nav pasaron\n", fails);
    return fails ? 1 : 0;
}
