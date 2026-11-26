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
/* Giroscopio simulado: mientras se pide girar, el rumbo cambia esto por paso
 * (100 grados/s) en el sentido pedido */
#define SIM_MDEG_PER_STEP 2000
/* Angulo en el que se corta el giro */
#define TURN_CUT_MDEG (NAV_TURN_ANGLE_MDEG - NAV_TURN_COAST_MDEG)

static uint32_t t = 0;
static bool sim_gyro = false;
static int32_t sim_heading = 0;

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

/* Con sim_gyro, el rumbo de la entrada lo pone el simulador */
static NavOutput_t step(NavContext_t *c, NavInput_t in)
{
    t += STEP_MS; in.now_ms = t; in.sensor_ms = t;
    if (sim_gyro) {
        in.heading_mdeg = sim_heading;
    }
    NavOutput_t o = Nav_Step(c, &in);
    if (sim_gyro && !o.brake) {
        if (o.angular_rads > 0.0f) {
            sim_heading += SIM_MDEG_PER_STEP;
        } else if (o.angular_rads < 0.0f) {
            sim_heading -= SIM_MDEG_PER_STEP;
        }
    }
    return o;
}

static NavOutput_t run(NavContext_t *c, NavInput_t in, uint32_t ms)
{
    NavOutput_t o = {0};
    for (uint32_t e = 0; e < ms; e += STEP_MS) o = step(c, in);
    return o;
}

/* Corre hasta que empiece a girar (o max_ms) */
static NavOutput_t run_until_turning(NavContext_t *c, NavInput_t in, uint32_t max_ms)
{
    NavOutput_t o = {0};
    for (uint32_t e = 0; e < max_ms; e += STEP_MS) {
        o = step(c, in);
        if (o.angular_rads != 0.0f && !o.brake) break;
    }
    return o;
}

/* Corre mientras siga girando (o max_ms); devuelve el primer paso frenado */
static NavOutput_t run_while_turning(NavContext_t *c, NavInput_t in, uint32_t max_ms, uint32_t *took)
{
    NavOutput_t o = {0};
    uint32_t e = 0;
    for (; e < max_ms; e += STEP_MS) {
        o = step(c, in);
        if (o.brake || o.angular_rads == 0.0f) break;
    }
    if (took) *took = e;
    return o;
}

static NavOutput_t start(NavContext_t *c)
{
    Nav_Init(c);
    NavInput_t in = base();
    in.command = NAV_CMD_START;
    return step(c, in);
}

/* Reinicia el giroscopio simulado (sim_gyro = on) */
static void sim(bool on)
{
    sim_gyro = on;
    sim_heading = 0;
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
    uint32_t took = 0;

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
    in.heading_mdeg = NAV_HEADING_DEADBAND_MDEG + 100; o = step(&c, in);
    CHECK(o.angular_rads < 0.0f);                                 /* pasando la zona muerta corrige */

    /* Sin MPU6050 -> FAULT frenado; cuando vuelve a responder, retoma solo */
    in = base(); in.imu_ok = false; o = step(&c, in);
    CHECK(o.state == NAV_STATE_FAULT && o.brake && !o.fault_latched && (o.events & NAV_EVT_FAULT));
    o = run(&c, in, 500);
    CHECK(o.state == NAV_STATE_FAULT && o.brake);
    o = step(&c, base());
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

#if NAV_STEP_MS > 0
    /* Tramo: sigue recto hasta NAV_STEP_MS y frena para escanear */
    start(&c);
    o = run(&c, base(), SCAN_TOTAL_MS + STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD);
    o = run(&c, base(), NAV_STEP_MS - 15 * STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms == NAV_FWD_SPEED_MMS);
    o = run(&c, base(), 12 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.brake);
#endif

    /* El rumbo de referencia se toma al recibir START... */
    Nav_Init(&c);
    in = base(); in.heading_mdeg = 30000; in.command = NAV_CMD_START;
    o = step(&c, in);
    in.command = NAV_CMD_NONE;
    o = run(&c, in, SCAN_TOTAL_MS + STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && o.angular_rads == 0.0f);
    in.heading_mdeg = 25000; o = step(&c, in);
    CHECK(o.angular_rads > 0.0f);

    /* ...y NO se vuelve a tomar al pasar de SCAN a FORWARD con el frente libre:
     * el desvio de un corte no se vuelve el nuevo "recto" */
    start(&c);                                     /* referencia: 0 */
    in = base(); in.heading_mdeg = 10000;          /* se desvio 10 grados al frenar */
    o = run(&c, in, SCAN_TOTAL_MS + STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && o.angular_rads < 0.0f);   /* corrige hacia 0 */

    /* Umbral para arrancar: con NAV_CLEAR_MM - 10 de frente no avanza */
    start(&c);
    o = run(&c, walls(NAV_CLEAR_MM - 10, 100, 100), SCAN_TOTAL_MS + 4 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && !(o.linear_mms > 0.0f));
    /* Con NAV_CLEAR_MM avanza, y no vuelve a frenar hasta bajar de NAV_OBSTACLE_MM */
    start(&c);
    o = run(&c, walls(NAV_CLEAR_MM, 100, 100), SCAN_TOTAL_MS + STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms == NAV_FWD_SPEED_MMS);
    o = run(&c, walls(NAV_OBSTACLE_MM, 100, 100), 10 * STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && !o.brake);
    o = step(&c, walls(NAV_OBSTACLE_MM - 10, 100, 100));
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* No decide antes de terminar la pausa y las lecturas */
    in = walls(100, 900, 100);
    start(&c);
    o = run(&c, in, NAV_BRAKE_MS + NAV_SETTLE_MS - 2 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* Frente bloqueado, izquierda libre: pivota a la izquierda.
     * La rueda interna (izquierda) queda en 0 y la externa va a NAV_FWD_SPEED_MMS */
    o = run(&c, in, SCAN_TOTAL_MS - NAV_BRAKE_MS - NAV_SETTLE_MS + 4 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && !o.brake && o.angular_rads == NAV_TURN_PIVOT_RAD_S);
    CHECK(o.linear_mms - o.angular_rads * NAV_WHEEL_BASE_MM * 0.5f == 0.0f);
    CHECK(o.linear_mms + o.angular_rads * NAV_WHEEL_BASE_MM * 0.5f == NAV_FWD_SPEED_MMS);

    /* ===== Giro de 90 grados por angulo ===== */
    /* Izquierda: termina en ANGLE - COAST leido del giroscopio */
    sim(true);
    start(&c);
    o = run_until_turning(&c, walls(100, 900, 100), SCAN_TOTAL_MS + 200);
    CHECK(o.state == NAV_STATE_SCAN && o.angular_rads > 0.0f);
    o = run_while_turning(&c, walls(100, 900, 100), NAV_TURN_TIMEOUT_MS + 200, &took);
    CHECK(o.state == NAV_STATE_SCAN && o.brake);
    CHECK(sim_heading == TURN_CUT_MDEG);
    CHECK(took < NAV_TURN_TIMEOUT_MS);                           /* corto por angulo, no por tiempo */

    /* Derecha: idem hacia el otro lado */
    sim(true);
    start(&c);
    o = run_until_turning(&c, walls(100, 100, 900), SCAN_TOTAL_MS + 200);
    CHECK(o.state == NAV_STATE_SCAN && o.angular_rads == -NAV_TURN_PIVOT_RAD_S);
    CHECK(o.linear_mms + o.angular_rads * NAV_WHEEL_BASE_MM * 0.5f == 0.0f);   /* rueda derecha en 0 */
    o = run_while_turning(&c, walls(100, 100, 900), NAV_TURN_TIMEOUT_MS + 200, &took);
    CHECK(o.brake && sim_heading == -TURN_CUT_MDEG);
    CHECK(took < NAV_TURN_TIMEOUT_MS);
    /* Tras el giro sigue de largo por el lateral libre, sin otra ronda de
     * medicion, y el rumbo nuevo pasa a ser el "recto" */
    for (took = 0; took < NAV_SETTLE_MS + 4 * STEP_MS && o.state != NAV_STATE_FORWARD; took += STEP_MS) {
        o = step(&c, walls(900, 100, 900));
    }
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms == NAV_FWD_SPEED_MMS);
    CHECK(took < NAV_SETTLE_MS + 4 * STEP_MS);
    o = run(&c, walls(900, 100, 900), 500);
    CHECK(o.state == NAV_STATE_FORWARD && o.linear_mms > 0.0f && o.angular_rads == 0.0f);
    /* La pared siguiente a menos de 30 cm: frena y decide */
    o = step(&c, walls(NAV_OBSTACLE_MM - 10, 900, 900));
    CHECK(o.state == NAV_STATE_SCAN && o.brake);
    sim(false);

    /* Sin cambio de rumbo, el giro se corta por NAV_TURN_TIMEOUT_MS */
    start(&c);
    o = run_until_turning(&c, walls(100, 900, 100), SCAN_TOTAL_MS + 200);
    CHECK(o.angular_rads > 0.0f);
    o = run(&c, walls(100, 900, 100), NAV_TURN_TIMEOUT_MS - 10 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.angular_rads > 0.0f);
    o = run(&c, walls(100, 900, 100), 11 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* Ambos lados libres: prefiere la derecha aunque la izquierda sea mas amplia */
    start(&c);
    o = run(&c, walls(100, 900, NAV_SIDE_FREE_MM), SCAN_TOTAL_MS + 4 * STEP_MS);
    CHECK(o.angular_rads < 0.0f);

    /* Derecha con menos de 40 cm: dobla a la izquierda */
    start(&c);
    o = run(&c, walls(100, NAV_SIDE_FREE_MM, NAV_SIDE_FREE_MM - 10), SCAN_TOTAL_MS + 4 * STEP_MS);
    CHECK(o.angular_rads > 0.0f);

    /* Ningun lado llega a 40 cm: retrocede */
    start(&c);
    o = run(&c, walls(100, NAV_SIDE_FREE_MM - 10, NAV_SIDE_FREE_MM - 10), SCAN_TOTAL_MS + 4 * STEP_MS);
    CHECK(o.linear_mms == NAV_BACK_SPEED_MMS && o.angular_rads == 0.0f);

    /* Frena a 30 cm de frente */
    start(&c);
    run(&c, base(), SCAN_TOTAL_MS + STEP_MS);
    o = step(&c, walls(NAV_OBSTACLE_MM + 10, 900, 900));
    CHECK(o.state == NAV_STATE_FORWARD && !o.brake);
    o = step(&c, walls(NAV_OBSTACLE_MM - 10, 900, 900));
    CHECK(o.state == NAV_STATE_SCAN && o.brake);

    /* Sin paso: NAV_SCAN_BACKUP_MAX retrocesos y despues FAULT */
    start(&c);
    {
        int backups = 0; bool backing = false; uint32_t evt = 0;
        in = walls(100, 100, 100);
        for (uint32_t e = 0; e < 120000u && o.state != NAV_STATE_FAULT; e += STEP_MS) {
            o = step(&c, in); evt |= o.events;
            bool now_back = (o.linear_mms == NAV_BACK_SPEED_MMS);
            if (now_back && !backing) backups++;
            backing = now_back;
            CHECK(o.angular_rads == 0.0f);                         /* nunca gira buscando */
        }
        CHECK(backups == NAV_SCAN_BACKUP_MAX);
        CHECK(o.state == NAV_STATE_FAULT && o.fault_latched && (evt & NAV_EVT_FAULT));
    }

    /* Un lado libre despues de retroceder corta la cuenta de retrocesos */
    start(&c);
    {
        int backups = 0; bool backing = false;
        for (uint32_t e = 0; e < 120000u && backups < NAV_SCAN_BACKUP_MAX - 1; e += STEP_MS) {
            o = step(&c, walls(100, 100, 100));
            bool now_back = (o.linear_mms == NAV_BACK_SPEED_MMS);
            if (now_back && !backing) backups++;
            backing = now_back;
        }
        /* aparece paso a la derecha: dobla, no FAULT */
        for (uint32_t e = 0; e < 20000u && o.angular_rads == 0.0f && o.state != NAV_STATE_FAULT; e += STEP_MS) {
            o = step(&c, walls(100, 100, 900));
        }
        CHECK(o.state == NAV_STATE_SCAN && o.angular_rads < 0.0f);
    }

    /* Todo bloqueado: retrocede */
    start(&c);
    o = run(&c, walls(100, 100, 100), SCAN_TOTAL_MS + 4 * STEP_MS);
    CHECK(o.state == NAV_STATE_SCAN && o.linear_mms == NAV_BACK_SPEED_MMS && o.angular_rads == 0.0f);

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
    CHECK(o.state == NAV_STATE_SCAN && !(o.linear_mms > 0.0f && o.angular_rads == 0.0f));

    /* ===== Precipicio ===== */
    /* Durante el escaneo tiene prioridad */
    start(&c);
    in = base(); in.cliff_left = true;
    o = run(&c, in, (NAV_CLIFF_CONFIRM + 1) * STEP_MS);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER);

    /* Retrocede NAV_CLIFF_BACKUP_MS, gira 90 grados EN EL LUGAR (por angulo)
     * alejandose del borde y vuelve a medir con el rumbo nuevo como "recto" */
    sim(true);
    start(&c);
    run(&c, base(), SCAN_TOTAL_MS + STEP_MS);                     /* avanzando */
    in = base(); in.cliff_left = true;
    for (int i = 0; i < 10 && o.state != NAV_STATE_CLIFF_RECOVER; i++) o = step(&c, in);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && (o.events & NAV_EVT_CLIFF));
    in = base();
    o = run(&c, in, NAV_BRAKE_MS + NAV_SETTLE_MS + 2 * STEP_MS);
    CHECK(o.linear_mms == NAV_BACK_SPEED_MMS && o.angular_rads == 0.0f);
    {
        /* Ya van dos pasos hacia atras; cuenta el resto */
        uint32_t back_ms = 2 * STEP_MS;
        while (back_ms < 2000u && o.linear_mms == NAV_BACK_SPEED_MMS) {
            o = step(&c, in);
            if (o.linear_mms == NAV_BACK_SPEED_MMS) back_ms += STEP_MS;
        }
        CHECK(back_ms >= NAV_CLIFF_BACKUP_MS && back_ms <= NAV_CLIFF_BACKUP_MS + 2 * STEP_MS);
    }
    o = run_until_turning(&c, in, NAV_SETTLE_MS + 200);
    CHECK(o.state == NAV_STATE_CLIFF_RECOVER && o.angular_rads == -NAV_TURN_SPOT_RAD_S);
    CHECK(o.linear_mms == 0.0f);                                   /* en el lugar, sin pivote */
    o = run_while_turning(&c, in, NAV_TURN_TIMEOUT_MS + 200, &took);
    CHECK(o.brake && sim_heading == -TURN_CUT_MDEG && took < NAV_TURN_TIMEOUT_MS);
    for (int i = 0; i < 100 && o.state == NAV_STATE_CLIFF_RECOVER; i++) o = step(&c, in);
    CHECK(o.state == NAV_STATE_SCAN);                              /* mide antes de avanzar */
    o = run(&c, in, SCAN_TOTAL_MS + 2 * STEP_MS);
    CHECK(o.state == NAV_STATE_FORWARD && o.angular_rads == 0.0f); /* recto = rumbo tras el giro */
    sim(false);

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

#if NAV_STEP_MS > 0
    /* Un tramo completo reinicia el contador de maniobras */
    sim(true);
    start(&c);
    for (int k = 0; k < NAV_SCAN_STUCK_MAX + 2; k++) {
        o = run_until_turning(&c, walls(100, 900, 100), 4000);
        CHECK(o.state == NAV_STATE_SCAN && o.angular_rads > 0.0f);
        o = run_while_turning(&c, walls(100, 900, 100), NAV_TURN_TIMEOUT_MS + 200, NULL);
        CHECK(o.state == NAV_STATE_SCAN && o.brake);
        for (int i = 0; i < 200 && o.state != NAV_STATE_FORWARD; i++) o = step(&c, base());
        CHECK(o.state == NAV_STATE_FORWARD);
        o = run(&c, base(), NAV_STEP_MS + 4 * STEP_MS);
        CHECK(o.state == NAV_STATE_SCAN);
    }
    sim(false);
#endif

    /* STOP desde el escaneo */
    start(&c);
    in = base(); in.command = NAV_CMD_STOP; o = step(&c, in);
    CHECK(o.state == NAV_STATE_IDLE);

    CHECK(strcmp(Nav_StateName(NAV_STATE_SCAN), "SCAN") == 0);

    printf(fails ? "%d pruebas fallaron\n" : "Todas las pruebas del escaneo pasaron\n", fails);
    return fails ? 1 : 0;
}
