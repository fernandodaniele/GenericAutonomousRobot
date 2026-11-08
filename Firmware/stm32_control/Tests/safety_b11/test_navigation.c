/* Included by test_safety.c: real navigation/motion/FSM/backend sources and
 * the existing deterministic RTOS/HAL/sensor substitutes, no copied algorithm. */
static void b73_fresh(void) {
    b65_sensor_snapshot(0U);
    input.distancia_m = input.distancia_trasera_m = 0.50f;
    input.ultrasonido_valido = input.ultrasonido_trasero_valido = 1U;
    input.ir = (TCRT_Lecturas_t){ TCRT_DETECTA, TCRT_DETECTA, TCRT_DETECTA, TCRT_DETECTA };
    queue_available = 1;
}
static void b73_step(void) {
    TaskHandle_t previous = current_task; current_task = CONTROL_TASK_ID;
    Navigation_Step(); current_task = previous;
}
static void b73_begin(void) {
    setup(); robot_state = ROBOT_STATE_IDLE; b73_fresh();
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_AUTO)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS);
    CHECK(Navigation_GetState() == NAV_ESPERAR_DATOS);
    CHECK(navigation.started_tick == clock_tick); expect_zero();
    ++clock_tick;
}
static void b73_advance(void) {
    b73_begin(); b73_fresh(); b73_step();
    CHECK(Navigation_GetState() == NAV_AVANZAR && target_pending);
    CHECK(queued_target.left_speed_cps == 1600 && queued_target.right_speed_cps == 1600);
    CHECK(queued_target.mode == ROBOT_STATE_AUTONOMOUS);
    CHECK(queued_target.navigation_session == navigation.session);
    run_motor(1);
    CHECK(pwm.compare[0] > 0U && pwm.compare[0] <= 840U);
    CHECK(pwm.compare[1] > 0U && pwm.compare[1] <= 840U);
}
static void b73_terminal(void) {
    CHECK(Navigation_GetState() == NAV_DETENIDO); expect_zero();
    uint32_t generation = command_generation;
    clock_tick += 20U; b73_fresh(); b73_step(); run_motor(1);
    CHECK(Navigation_GetState() == NAV_DETENIDO && command_generation == generation);
    if (robot_state == ROBOT_STATE_AUTONOMOUS) CHECK(!TaskControlRobot_RequestAutoAt(clock_tick));
    expect_zero();
}
static void test_navigation_b73(void) {
    b73_begin(); b73_fresh(); run_fsm(1); run_motor(1);
    CHECK(Navigation_GetState() == NAV_AVANZAR && pwm.compare[0] > 0U);
    PASS("B7.3 existing controller task loop emits real navigation targets after processing AUTO events");
    b73_begin(); queue_available = 0; b73_step();
    CHECK(Navigation_GetState() == NAV_ESPERAR_DATOS && !target_pending); expect_zero();
    PASS("B7.3 missing initial snapshot keeps AUTO waiting with zero targets");
    b73_fresh(); b73_step(); run_motor(1);
    CHECK(Navigation_GetState() == NAV_AVANZAR && pwm.compare[0] > 0U);
    PASS("B7.3 waiting can advance after fresh safe data using real PI and driver");

    const float distances[] = {0.301f, 0.300f, 0.299f};
    for (unsigned i = 0; i < 3U; ++i) {
        b73_begin(); input.distancia_m = distances[i]; b73_step();
        if (i == 0U) {
            CHECK(Navigation_GetState() == NAV_AVANZAR); run_motor(1);
            CHECK(task_context.left_target_cps == 1600 && task_context.right_target_cps == 1600);
        } else b73_terminal();
        PASS("B7.3 301 mm permits advance while 300 mm and below stop the session");
    }
    b73_advance(); b73_fresh(); input.distancia_m = 0.300f; b73_step();
    expect_zero(); CHECK(TaskSafety_GetFaults() == SAFETY_FAULT_NONE);
    b73_terminal(); PASS("B7.3 preventive obstacle immediately clears live targets without inventing a Safety fault");
    b73_begin(); input.distancia_m = 0.300f; input.ultrasonido_trasero_valido = 0U;
    b73_step(); b73_terminal();
    PASS("B7.3 current front obstacle is terminal even while still waiting for rear data");
    for (unsigned stale = 0U; stale < 2U; ++stale) {
        b73_begin(); input.distancia_m = 0.300f;
        if (stale == 0U) input.timestamp_ultrasonido_frontal = clock_tick - 201U;
        else input.timestamp = clock_tick - 101U;
        b73_step(); CHECK(Navigation_GetState() == NAV_ESPERAR_DATOS); expect_zero();
        PASS("B7.3 old front distance or old snapshot cannot masquerade as a current front reading");
    }

    /* Sensor-only failures allow waiting at startup, but never a restart after advance. */
    for (unsigned phase = 0; phase < 2U; ++phase) {
        for (unsigned failure = 0; failure < 9U; ++failure) {
            if (phase) b73_advance(); else b73_begin();
            b73_fresh();
            if (failure == 0U) queue_available = 0;
            if (failure == 1U) input.ultrasonido_valido = 0U;
            if (failure == 2U) input.ultrasonido_trasero_valido = 0U;
            if (failure == 3U) input.timestamp = clock_tick - 101U;
            if (failure == 4U) input.timestamp_ultrasonido_frontal = clock_tick - 201U;
            if (failure == 5U) input.timestamp_ultrasonido_trasero = clock_tick - 201U;
            if (failure == 6U) input.distancia_m = INFINITY;
            if (failure == 7U) input.distancia_trasera_m = INFINITY;
            if (failure == 8U) input.ultrasonido_valido = input.ultrasonido_trasero_valido = 0U;
            b73_step(); expect_zero();
            if (phase) b73_terminal();
            else CHECK(Navigation_GetState() == NAV_ESPERAR_DATOS && !target_pending);
            PASS("B7.3 missing invalid stale or nonfinite sensors wait initially and cancel active movement");
        }
    }
    for (unsigned failure = 0; failure < 6U; ++failure) {
        b73_advance(); b73_fresh();
        if (failure == 0U) input.distancia_trasera_m = 0.200f;
        if (failure == 1U) input.distancia_trasera_m = 0.199f;
        if (failure == 2U) input.distancia_m = NAN;
        if (failure == 3U) input.distancia_trasera_m = NAN;
        if (failure == 4U) { input.hay_caida = 1U; input.ir.del_izq = TCRT_NO_DETECTA; }
        if (failure == 5U) safety_stop_active = true;
        b73_step(); expect_zero();
        CHECK(Navigation_GetState() == NAV_DETENIDO);
        if (failure == 4U) {
            CHECK(TaskSafety_GetFaults() == SAFETY_FAULT_NONE);
            run_safety(1); CHECK(TaskSafety_GetFaults() == SAFETY_FAULT_CLIFF);
        }
        b73_terminal();
        PASS("B7.3 rear danger NaN IR or Safety cancels navigation and preserves Safety authority");
    }
    b73_begin(); input.timestamp = clock_tick - 100U;
    input.timestamp_ultrasonido_frontal = input.timestamp_ultrasonido_trasero = clock_tick - 200U;
    b73_step(); CHECK(Navigation_GetState() == NAV_AVANZAR && target_pending);
    PASS("B7.3 reuses exact Safety boundaries of 100 ms snapshot and 200 ms individual ages");
    b73_begin(); clock_tick = 10U;
    navigation.started_tick = mode_entry_tick = 0U; last_stop_tick = UINT32_MAX;
    b73_fresh(); input.timestamp_ultrasonido_frontal = 0U;
    input.timestamp_ultrasonido_trasero = UINT32_MAX - 5U;
    b73_step(); run_motor(1); CHECK(pwm.compare[0] > 0U);
    PASS("B7.3 valid measurement tick zero and wrapped sensor ages permit advance");

    b73_advance(); TickType_t entered = navigation.started_tick;
    clock_tick = entered + 1980U; b73_fresh(); b73_step(); run_motor(1);
    CHECK(clock_tick == entered + 1990U && pwm.compare[0] > 0U);
    run_motor(1);
    CHECK(clock_tick == entered + 2000U); expect_zero();
    b73_step(); b73_terminal();
    PASS("B7.3 motor backend rejects the current session at 2000 ms even without another navigation cycle");
    b73_begin(); entered = navigation.started_tick;
    clock_tick = entered + 1999U; b73_fresh(); b73_step();
    CHECK(Navigation_GetState() == NAV_AVANZAR && target_pending);
    ++clock_tick; b73_step(); b73_terminal();
    PASS("B7.3 decision at 1999 ms is allowed but deadline at 2000 ms is terminal");
    b73_begin(); entered = navigation.started_tick;
    clock_tick = entered + 1999U; b73_fresh(); advance_tick_before_navigation_emit = 1U;
    b73_step(); CHECK(clock_tick == entered + 2000U && !target_pending); b73_terminal();
    PASS("B7.3 deadline crossed after decision but before publication rejects the emission");
    b73_begin(); entered = navigation.started_tick; queue_available = 0; b73_step();
    clock_tick = entered + 1900U; b73_fresh(); b73_step(); run_motor(1);
    CHECK(pwm.compare[0] > 0U && navigation.started_tick == entered);
    clock_tick = entered + 1990U; run_motor(1); expect_zero(); b73_step(); b73_terminal();
    PASS("B7.3 initial wait is part of the two-second session instead of restarting the timer on advance");
    b73_begin(); entered = navigation.started_tick; queue_available = 0;
    clock_tick = entered + 2000U; b73_step(); b73_terminal();
    PASS("B7.3 a session with no usable data also expires terminally at 2000 ms");
    b73_begin(); entered = navigation.started_tick;
    for (unsigned i = 1U; i < 100U; ++i) {
        clock_tick = entered + i * 20U; b73_fresh(); b73_step(); run_motor(1);
        CHECK(pwm.compare[0] > 0U && navigation.started_tick == entered);
    }
    clock_tick = entered + 2000U; b73_fresh(); b73_step(); expect_zero(); b73_terminal();
    PASS("B7.3 periodic fresh emissions cannot extend the original session beyond two seconds");
    b73_advance(); entered = navigation.started_tick;
    clock_tick = task_context.command_tick + 490U; run_motor(1); expect_zero();
    CHECK((TickType_t)(clock_tick - entered) < 2000U);
    PASS("B7.3 permanent 500 ms target timeout still stops a stalled navigation producer");
    b73_fresh(); b73_step();
    CHECK(Navigation_GetState() == NAV_DETENIDO); expect_zero();
    PASS("B7.3 a target timeout is terminal and resumed execution cannot restart navigation");

    b73_advance(); DrvUart_t uart; UART_HandleTypeDef peripheral = {0}; DrvUart_Init(&uart, &peripheral);
    b65_uart_command(&uart, "STOP\n", "STOP_QUEUED\n"); expect_zero();
    run_motor(1); run_fsm(1); CHECK(robot_state == ROBOT_STATE_IDLE); b73_terminal();
    PASS("B7.3 actual UART STOP cancels active navigation immediately without waiting for target expiry");
    b73_advance();
    for (unsigned i = 0; i < 8U; ++i) CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); expect_zero();
    run_fsm(1); CHECK(robot_state == ROBOT_STATE_IDLE); b73_terminal();
    PASS("B7.3 full FSM queue cannot delay cancellation of active navigation");
    b73_advance(); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick)); expect_zero(); run_fsm(1);
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick)); ++clock_tick; b73_fresh();
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1);
    CHECK(Navigation_GetState() == NAV_ESPERAR_DATOS); expect_zero();
    PASS("B7.3 STOP AUTO within the same tick cannot restart navigation but a fresh session can");
    for (unsigned action = 1U; action <= 4U; ++action) {
        if (action == 2U) continue; /* Existing action 2 tests the manual API separately. */
        b73_begin(); b73_fresh(); overwrite_action = (int)action; b73_step();
        expect_zero(); run_motor(1); expect_zero();
        CHECK(Navigation_GetState() == NAV_DETENIDO);
        PASS("B7.3 STOP mode change or Safety between real admission and enqueue invalidates the pending emission");
    }
    b73_begin(); b73_fresh(); peek_action = 7; b73_step();
    CHECK(!target_pending); b73_terminal();
    PASS("B7.3 cancellation during snapshot copy cannot publish a new target");
    b73_advance(); b73_fresh(); overwrite_action = 6; b73_step();
    CHECK(Navigation_GetState() == NAV_AVANZAR && target_pending); expect_zero();
    run_motor(1); CHECK(pwm.compare[0] > 0U && Navigation_GetState() == NAV_AVANZAR);
    PASS("B7.3 motor preemption before a normal renewal enqueue does not cancel the current session");
    b73_advance(); robot_state = ROBOT_STATE_IDLE; b73_step(); b73_terminal();
    PASS("B7.3 leaving AUTO cancels the session instead of retaining its objectives");
    b73_advance(); b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); expect_zero();
    run_fsm(1); CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); b73_terminal();
    PASS("B7.3 healthy RESET cancels the session without changing FSM semantics or restarting it");
    b73_advance(); TaskSafety_ReportFault(SAFETY_FAULT_CLIFF); expect_zero();
    run_safety(1); run_fsm(1); CHECK(robot_state == ROBOT_STATE_SAFETY_STOP);
    clock_tick += 100U; b73_fresh();
    b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); run_fsm(1); run_motor(1);
    CHECK(robot_state == ROBOT_STATE_IDLE && !TaskSafety_IsStopActive()); b73_terminal();
    PASS("B7.3 explicit safe rearm clears Safety but never restores an old AUTO session");

    b73_advance(); TaskMotorControlTarget_t old_target = queued_target;
    uint32_t old_session = navigation.session;
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1); ++clock_tick;
    b73_fresh(); CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    b73_fresh(); b73_step(); CHECK(navigation.session != old_session);
    old_target.generation = command_generation; /* Isolate session barrier from generation barrier. */
    CHECK(xQueueOverwrite(target_speed_queue, &old_target) == pdPASS); run_motor(1); expect_zero();
    current_task = CONTROL_TASK_ID;
    CHECK(!Motion_NavigationAdvance(clock_tick, old_session)); current_task = (TaskHandle_t)12;
    CHECK(Navigation_GetState() == NAV_AVANZAR);
    b73_fresh(); b73_step(); run_motor(1); CHECK(pwm.compare[0] > 0U);
    PASS("B7.3 old session is rejected at backend consumption even with a matching generation");
    b73_advance(); uint32_t generation = command_generation;
    const char *manual[] = {"AVANZAR\n", "RETROCEDER\n", "GIRAR_IZQUIERDA\n", "GIRAR_DERECHA\n", "DETENER\n"};
    for (unsigned i = 0; i < 5U; ++i) {
        b65_uart_command(&uart, manual[i], "MOVE_REJECTED\n");
        CHECK(command_generation == generation && Navigation_GetState() == NAV_AVANZAR);
        PASS("B7.3 active navigation does not authorize any manual UART movement command");
    }
    /* Isolate backend defenses with the same pending one-use permission that
     * the real Navigation_Step creates. No replacement of decision logic. */
    navigation.emission_ready = true; navigation.decision_tick = clock_tick;
    uint32_t session = navigation.session;
    CHECK(!Motion_NavigationAdvance(clock_tick, session));
    CHECK(navigation.emission_ready && command_generation == generation);
    PASS("B7.3 communications task cannot consume a navigation permission");
    current_task = CONTROL_TASK_ID;
    CHECK(!TaskMotorControl_TrySetNavigationSpeedsAt(-1600, 1600, clock_tick, session));
    CHECK(!TaskMotorControl_TrySetNavigationSpeedsAt(1601, 1601, clock_tick, session));
    CHECK(!TaskMotorControl_TrySetTargetSpeedsAt(1600, 1600, clock_tick, ROBOT_STATE_AUTONOMOUS));
    TaskMotorControl_SetTargetSpeeds(1600, 1600);
    CHECK(navigation.emission_ready && command_generation == generation);
    CHECK(Motion_NavigationAdvance(clock_tick, session));
    CHECK(!navigation.emission_ready);
    generation = command_generation;
    CHECK(!Motion_NavigationAdvance(clock_tick, session));
    CHECK(command_generation == generation); current_task = (TaskHandle_t)12;
    PASS("B7.3 fixed advance requires navigation path and a single-use permission even in the controller task");

    b73_advance(); old_target = queued_target;
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1);
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL)); run_fsm(1);
    old_target.generation = command_generation;
    CHECK(xQueueOverwrite(target_speed_queue, &old_target) == pdPASS); run_motor(1); expect_zero();
    TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1);
    CHECK(robot_state == ROBOT_STATE_MANUAL && pwm.compare[0] > 0U);
    PASS("B7.3 AUTO target cannot cross into MANUAL and a fresh B6 manual target still works");
    setup(); clock_tick = UINT32_MAX - 1200U; robot_state = ROBOT_STATE_IDLE;
    mode_entry_tick = auto_barrier_tick = clock_tick - 1U; b73_fresh();
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    entered = navigation.started_tick; b73_fresh(); b73_step(); run_motor(1);
    CHECK(pwm.compare[0] > 0U);
    clock_tick = entered + 1990U; b73_fresh(); b73_step(); run_motor(1);
    CHECK((TickType_t)(clock_tick - entered) == 2000U); expect_zero(); b73_step(); b73_terminal();
    PASS("B7.3 session deadline and backend checks survive tick wrap");
    setup(); robot_state = ROBOT_STATE_IDLE; b73_fresh(); navigation.session = UINT32_MAX - 1U;
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    CHECK(navigation.session == 0U); b73_fresh(); b73_step(); run_motor(1);
    CHECK(pwm.compare[0] > 0U);
    PASS("B7.3 wrapped session zero is a legitimate active navigation session");
}
