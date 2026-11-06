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
    *t += STEP_MS; in->now_ms = *t; in->command = NAV_CMD_NONE;
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

/* Duracion minima de una evasiva completa y de una recuperacion de borde */
#define EVADE_TOTAL_MS (NAV_BRAKE_MS + NAV_SETTLE_MS + NAV_EVADE_BACK_MS + NAV_SETTLE_MS \
                        + NAV_EVADE_TURN_MIN_MS + NAV_SETTLE_MS)
#define CLIFF_TOTAL_MS (NAV_BRAKE_MS + NAV_SETTLE_MS + NAV_BACKUP_MS + NAV_SETTLE_MS \
                        + NAV_CLIFF_TURN_MS + NAV_SETTLE_MS)

int main(void)
{
    NavContext_t c; uint32_t t = 0, took = 0; NavInput_t in; NavOutput_t o;

    /* Arranque seguro: sin START queda en IDLE */
    Nav_Init(&c);
    o = run(&c, base(0), &t, 200);
    CHECK(o.state == NAV_STATE_IDLE && o.linear_mms == 0.0f);

    /* START -> FORWARD */
    in = base(t); in.command = NAV_CMD_START; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms == NAV_FWD_SPEED_MMS);

    /* Espacio abierto (eco largo) sigue avanzando */
    o = run(&c, base(t), &t, 500);
    CHECK(o.state == NAV_STATE_FORWARD);

    /* ---- Evasiva con pausas: freno, pausa, atras, pausa, giro, pausa ---- */
    in = base(t); in.front = NAV_RANGE_OK; in.front_mm = NAV_OBSTACLE_MM - 10;
    in.left = NAV_RANGE_OK; in.left_mm = 150; in.right = NAV_RANGE_OK; in.right_mm = 900;
    o = step(&c, &in, &t);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE && o.brake);
    o = run(&c, in, &t, NAV_BRAKE_MS + NAV_SETTLE_MS - STEP_MS);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE && o.brake);          /* sigue quieto */
    o = run(&c, in, &t, 2 * STEP_MS);
    CHECK(!o.brake && o.linear_mms == NAV_BACK_SPEED_MMS);          /* retrocede */
    o = run(&c, in, &t, NAV_EVADE_BACK_MS);
    CHECK(o.brake && o.linear_mms == 0.0f);                         /* pausa antes de girar */
    o = run(&c, in, &t, NAV_SETTLE_MS);
    CHECK(!o.brake && o.angular_rads < 0.0f);                       /* gira a la derecha (mas libre) */

    /* Por debajo del margen sigue girando */
    in.front_mm = NAV_CLEAR_MM - 10; o = run(&c, in, &t, NAV_EVADE_TURN_MIN_MS);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE && o.angular_rads < 0.0f);
    /* Libre -> pausa con freno -> FORWARD */
    in.front_mm = NAV_CLEAR_MM + 10; o = step(&c, &in, &t);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE && o.brake);
    o = run_until(&c, in, &t, NAV_STATE_FORWARD, 2000, &took);
    CHECK(o.state == NAV_STATE_FORWARD && took >= NAV_SETTLE_MS - STEP_MS);

    /* Sigue bloqueado tras el giro maximo -> pausa y retrocede otra vez */
    in = base(t); in.front = NAV_RANGE_OK; in.front_mm = NAV_OBSTACLE_MM - 10;
    o = run(&c, in, &t, NAV_BRAKE_MS + 2 * NAV_SETTLE_MS + NAV_EVADE_BACK_MS + NAV_EVADE_TURN_MAX_MS + 60);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE && o.brake);
    o = run(&c, in, &t, NAV_SETTLE_MS);
    CHECK(o.linear_mms == NAV_BACK_SPEED_MMS);
    in = base(t); o = run_until(&c, in, &t, NAV_STATE_FORWARD, EVADE_TOTAL_MS + 200, NULL);
    CHECK(o.state == NAV_STATE_FORWARD);

    /* ---- Seguimiento de pared izquierda a 10 cm ---- */
    start(&c, t);
    in = base(t); in.left = NAV_RANGE_OK; in.left_mm = NAV_WALL_MM;
    o = run(&c, in, &t, STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms == NAV_FWD_SPEED_MMS && o.angular_rads == 0.0f);
    in.left_mm = NAV_WALL_MM + 50; o = run(&c, in, &t, STEP_MS); CHECK(o.linear_mms > 0.0f && o.angular_rads > 0.0f);
    in.left_mm = NAV_WALL_MM - 40; o = run(&c, in, &t, STEP_MS); CHECK(o.linear_mms > 0.0f && o.angular_rads < 0.0f);
    /* Pared perdida (esquina exterior): arco hacia la pared */
    in.left = NAV_RANGE_FAR; o = run(&c, in, &t, STEP_MS);
    CHECK(o.linear_mms > 0.0f && o.angular_rads == NAV_WALL_MAX_RAD_S);

    /* Esquina a 10 cm de frente: gira en el lugar, sin evasiva, con histeresis */
    in.left = NAV_RANGE_OK; in.left_mm = NAV_WALL_MM;
    in.front = NAV_RANGE_OK; in.front_mm = NAV_WALL_MM - 10; o = run(&c, in, &t, STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms == 0.0f && o.angular_rads < 0.0f);
    in.front_mm = NAV_CLEAR_MM - 10; o = run(&c, in, &t, STEP_MS); CHECK(o.linear_mms == 0.0f);
    in.front_mm = NAV_CLEAR_MM + 10; o = run(&c, in, &t, STEP_MS); CHECK(o.linear_mms > 0.0f);

    /* Menos de 5 cm de frente: evasiva */
    in.front_mm = NAV_OBSTACLE_MM - 10; o = step(&c, &in, &t);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE && o.brake);
    in = base(t); o = run_until(&c, in, &t, NAV_STATE_FORWARD, EVADE_TOTAL_MS + 200, &took);
    CHECK(o.state == NAV_STATE_FORWARD && took >= EVADE_TOTAL_MS - NAV_EVADE_TURN_MIN_MS);

    /* "Demasiado cerca" (<2 cm) tambien es obstaculo */
    in = base(t); in.front = NAV_RANGE_TOO_CLOSE; o = run(&c, in, &t, STEP_MS);
    CHECK(o.state == NAV_STATE_AVOID_OBSTACLE);
    in.front = NAV_RANGE_FAR; o = run_until(&c, in, &t, NAV_STATE_FORWARD, EVADE_TOTAL_MS + 200, NULL);
    CHECK(o.state == NAV_STATE_FORWARD);

    /* ---- Precipicio izquierdo: freno, pausa, atras, pausa, giro a la derecha, pausa ---- */
    start(&c, t);
    in = base(t); in.cliff_left = true;
    o = run_until(&c, in, &t, NAV_STATE_CLIFF_RECOVER, 500, NULL);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && o.brake && (o.events & NAV_EVT_CLIFF));
    o = run(&c, in, &t, NAV_BRAKE_MS + NAV_SETTLE_MS + 40);
    CHECK(o.linear_mms == NAV_BACK_SPEED_MMS);
    in.cliff_left = false;
    o = run(&c, in, &t, NAV_BACKUP_MS);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && o.brake);           /* pausa antes de girar */
    o = run(&c, in, &t, NAV_SETTLE_MS);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && o.angular_rads < 0.0f);
    o = run_until(&c, in, &t, NAV_STATE_FORWARD, NAV_CLIFF_TURN_MS + NAV_SETTLE_MS + 100, NULL);
    CHECK(o.state == NAV_STATE_FORWARD);

    /* Precipicio tiene prioridad sobre obstaculo */
    in = base(t); in.cliff_right = true; in.front = NAV_RANGE_TOO_CLOSE;
    o = run_until(&c, in, &t, NAV_STATE_CLIFF_RECOVER, 500, NULL);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER);
    in = base(t); o = run_until(&c, in, &t, NAV_STATE_FORWARD, CLIFF_TOTAL_MS + 200, NULL);
    CHECK(o.state == NAV_STATE_FORWARD);

    /* Precipicio siguiendo una pared: al terminar no vuelve hacia ese lado */
    start(&c, t);
    in = base(t); in.left = NAV_RANGE_OK; in.left_mm = NAV_WALL_MM; o = run(&c, in, &t, STEP_MS);
    in.cliff_right = true; o = run_until(&c, in, &t, NAV_STATE_CLIFF_RECOVER, 500, NULL);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER);
    in = base(t); o = run_until(&c, in, &t, NAV_STATE_FORWARD, CLIFF_TOTAL_MS + 200, NULL);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms > 0.0f && o.angular_rads == 0.0f);

    /* Giro brusco: alerta sin cambiar el estado, con retencion */
    in = base(t); in.gyro_z_dps = -350; o = step(&c, &in, &t);
    CHECK((o.events & NAV_EVT_ALERT) && o.state == NAV_STATE_FORWARD);
    o = step(&c, &in, &t);
    CHECK(!(o.events & NAV_EVT_ALERT));

    /* Frontal sin eco: sigue andando, no se queda quieto */
    in = base(t); in.front = NAV_RANGE_NO_ECHO; in.front_no_echo_count = 200;
    o = run(&c, in, &t, 100); CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms > 0.0f);

    /* Foto vieja -> FAULT y freno; al volver los datos, arranca solo */
    in = base(t); in.snapshot_stale = true; o = run(&c, in, &t, STEP_MS);
    CHECK(o.state == NAV_STATE_FAULT && o.brake);
    in = base(t); o = run(&c, in, &t, STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms > 0.0f);

    /* Sin MPU6050 -> FAULT al arrancar (solo si NAV_REQUIRE_IMU) */
#if NAV_REQUIRE_IMU
    Nav_Init(&c); in = base(t); in.imu_ok = false; in.command = NAV_CMD_START;
    o = Nav_Step(&c, &in); CHECK(o.state == NAV_STATE_FAULT);
#endif

    /* STOP desde cualquier estado -> IDLE */
    start(&c, t);
    in = base(t); in.command = NAV_CMD_STOP; o = Nav_Step(&c, &in);
    CHECK(o.state == NAV_STATE_IDLE && o.linear_mms == 0.0f);

    printf(fails ? "%d pruebas fallaron\n" : "Todas las pruebas de mid_nav pasaron\n", fails);
    return fails ? 1 : 0;
}
