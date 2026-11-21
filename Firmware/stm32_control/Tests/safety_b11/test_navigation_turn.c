/* One set of production sources. Experimental variants are HOST tests only. */
#if !NAV_EXPERIMENTAL_TURN_ENABLED
static void test_turn_disabled_b77(void) {
    CHECK(NAV_EXPERIMENTAL_TURN_ENABLED == 0 && NAV_TURN_LEFT == 1);
    PASS("B7.7 production defaults disable turn and select left only for future activation");
    b75_reverse(); TickType_t reverse_start = navigation.reverse_started_tick;
    clock_tick = reverse_start + 290U; run_motor(1); b73_terminal();
    PASS("B7.7 default build completes reverse in DETENIDO exactly as B7.5");
    b75_reverse(); uint32_t session = navigation.session;
    navigation.state = NAV_GIRAR_ACOTADO; navigation.emission_ready = true;
    navigation.decision_tick = clock_tick; current_task = CONTROL_TASK_ID;
    CHECK(!Motion_NavigationTurn(clock_tick, session));
    CHECK(!TaskMotorControl_TrySetNavigationSpeedsAt(-1200, 1200, clock_tick, session));
    CHECK(!TaskMotorControl_TrySetNavigationSpeedsAt(1200, -1200, clock_tick, session));
    CHECK(!Navigation_IsCommandCurrent(session, clock_tick));
    current_task = (TaskHandle_t)12; run_motor(1); expect_zero();
    PASS("B7.7 disabled turn cannot acquire backend authority even with injected turn state");
    b75_reverse(); CHECK(!Motion_NavigationTurn(clock_tick, navigation.session));
    CHECK(Navigation_GetState() == NAV_RETROCEDER_ACOTADO && left_motor.current_speed < 0);
    PASS("B7.7 rejected experimental API leaves legitimate default reverse unchanged");
}
#else
static void b77_complete_reverse(void) {
    TickType_t started = navigation.started_tick;
    TickType_t end = navigation.reverse_started_tick + 300U;
    CHECK(navigation.reverse_applied);
    clock_tick = end - 10U; run_motor(1); expect_zero();
    CHECK(clock_tick == end && Navigation_GetState() == NAV_RETROCEDER_ACOTADO);
    CHECK(navigation.active && !navigation.emission_ready);
    b75_data(0.250f, 0.50f); b73_step(); expect_zero();
    CHECK(Navigation_GetState() == NAV_GIRAR_ACOTADO && navigation.turn_used);
    CHECK(navigation.started_tick == started && navigation.turn_started_tick == clock_tick);
}
static void b77_finish_reverse(void) {
    b75_reverse(); b77_complete_reverse();
}
static void b77_turn(void) {
    b77_finish_reverse(); clock_tick += 20U; b75_data(0.250f, 0.250f); b73_step();
    CHECK(queued_target.left_speed_cps == NAV_TURN_LEFT_CPS);
    CHECK(queued_target.right_speed_cps == NAV_TURN_RIGHT_CPS);
#if NAV_TURN_LEFT
    CHECK(queued_target.left_speed_cps == -1200 && queued_target.right_speed_cps == 1200);
#else
    CHECK(queued_target.left_speed_cps == 1200 && queued_target.right_speed_cps == -1200);
#endif
    run_motor(1);
    CHECK((left_motor.current_speed < 0) == (NAV_TURN_LEFT != 0));
    CHECK((right_motor.current_speed > 0) == (NAV_TURN_LEFT != 0));
    CHECK(pwm.compare[0] > 0U && pwm.compare[0] <= TEST_PWM_LIMIT);
    CHECK(pwm.compare[1] > 0U && pwm.compare[1] <= TEST_PWM_LIMIT);
}
static void b77_terminal(void) {
    CHECK(Navigation_GetState() == NAV_DETENIDO); expect_zero();
    uint32_t generation = command_generation;
    clock_tick += 20U; b73_fresh(); b73_step(); run_motor(1); expect_zero();
    CHECK(Navigation_GetState() == NAV_DETENIDO && generation == command_generation);
    if (robot_state == ROBOT_STATE_AUTONOMOUS) CHECK(!TaskControlRobot_RequestAutoAt(clock_tick));
}
static void b77_late_reverse(unsigned finish_age) {
    b73_begin(); TickType_t entered = navigation.started_tick;
    queue_available = 0; b73_step();
    clock_tick = entered + finish_age - 350U; b73_fresh(); b73_step(); run_motor(1);
    clock_tick = entered + finish_age - 300U; b75_data(0.300f, 0.50f); b73_step();
    CHECK(Navigation_GetState() == NAV_RETROCEDER_ACOTADO); expect_zero();
    clock_tick += 20U; b75_data(0.250f, 0.50f); b73_step(); run_motor(1);
    CHECK(navigation.reverse_applied);
    clock_tick = entered + finish_age - 10U; run_motor(1); expect_zero();
}
static void test_turn_enabled_b77(void) {
    b77_finish_reverse();
    PASS("B7.7 completed applied reverse stops before controller enters experimental turn");
    uint32_t generation = command_generation; b73_step();
    CHECK(Navigation_GetState() == NAV_GIRAR_ACOTADO && !target_pending);
    CHECK(command_generation == generation); expect_zero();
    PASS("B7.7 same-tick turn entry cannot emit a motor target");
    b77_turn();
    PASS("B7.7 configured fixed turn signs use real PI and limited driver");
    b73_advance(); b75_data(0.250f, 0.50f); b73_step();
    CHECK(Navigation_GetState() == NAV_RETROCEDER_ACOTADO); expect_zero();
    PASS("B7.7 front obstacle cannot jump directly from advance to turn");
    b75_prepare_reverse(); clock_tick += 300U; b75_data(0.250f, 0.50f); b73_step(); b77_terminal();
    PASS("B7.7 reverse never applied by motor backend cannot qualify for turn");
    b75_reverse(); clock_tick = navigation.reverse_started_tick + 299U;
    b75_data(0.250f, 0.50f); b73_step();
    CHECK(Navigation_GetState() == NAV_RETROCEDER_ACOTADO && queued_target.left_speed_cps == -1600);
    PASS("B7.7 reverse cannot complete early before 300 ms");

    b77_turn(); TickType_t turn_start = navigation.turn_started_tick;
    clock_tick = turn_start + 230U; b75_data(0.250f, 0.250f); b73_step(); run_motor(1);
    CHECK(clock_tick == turn_start + 240U && Navigation_GetState() == NAV_GIRAR_ACOTADO);
    run_motor(1); CHECK(clock_tick == turn_start + 250U); b77_terminal();
    PASS("B7.7 motor backend stops at 250 ms without another navigation cycle");
    b77_finish_reverse(); turn_start = navigation.turn_started_tick;
    clock_tick = turn_start + 249U; b75_data(0.250f, 0.250f); b73_step();
    CHECK(queued_target.left_speed_cps == NAV_TURN_LEFT_CPS); ++clock_tick; b73_step(); b77_terminal();
    PASS("B7.7 turn decision at 249 ms is permitted and 250 ms is terminal");
    b77_finish_reverse(); turn_start = navigation.turn_started_tick;
    clock_tick = turn_start + 249U; b75_data(0.250f, 0.250f);
    advance_tick_before_navigation_emit = 1U; b73_step();
    CHECK(clock_tick == turn_start + 250U && !target_pending); b77_terminal();
    PASS("B7.7 deadline crossed between turn decision and publication rejects the target");
    b77_finish_reverse(); turn_start = navigation.turn_started_tick;
    clock_tick = turn_start + 240U; b75_data(0.250f, 0.250f); b73_step();
    CHECK(target_pending); run_motor(1); b77_terminal();
    PASS("B7.7 queued turn first consumed at its deadline cannot reactivate PWM");

    b77_late_reverse(1750U); TickType_t entered = navigation.started_tick;
    b75_data(0.250f, 0.50f); b73_step();
    CHECK(Navigation_GetState() == NAV_GIRAR_ACOTADO);
    CHECK(navigation.started_tick == entered && navigation.turn_started_tick == entered + 1750U);
    clock_tick += 20U; b75_data(0.250f, 0.250f); b73_step(); run_motor(1);
    CHECK(pwm.compare[0] > 0U);
    clock_tick = entered + 1990U; b75_data(0.250f, 0.250f); b73_step(); run_motor(1);
    CHECK(clock_tick == entered + 2000U); b77_terminal();
    PASS("B7.7 exactly 250 ms remaining permits turn and the global deadline remains 2000 ms");
    b77_late_reverse(1751U); b77_terminal();
    PASS("B7.7 only 249 ms remaining rejects the whole turn");
    b77_late_reverse(1750U); clock_tick += 20U; b75_data(0.250f, 0.50f); b73_step(); b77_terminal();
    PASS("B7.7 budget is rechecked when controller runs after motor completion");
    b77_turn(); CHECK(navigation.turn_used);
    for (unsigned i = 0; i < 10U; ++i) { clock_tick += 20U; b75_data(0.50f, 0.50f); b73_step(); run_motor(1); }
    clock_tick = navigation.turn_started_tick + 250U; b73_step(); b77_terminal();
    PASS("B7.7 fresh data cannot extend or repeat a turn or restart advance");

    for (unsigned failure = 0; failure < 12U; ++failure) {
        b77_turn(); b75_data(0.250f, 0.250f);
        if (failure == 0U) queue_available = 0;
        if (failure == 1U) input.ultrasonido_valido = 0U;
        if (failure == 2U) input.ultrasonido_trasero_valido = 0U;
        if (failure == 3U) input.timestamp = clock_tick - 101U;
        if (failure == 4U) input.timestamp_ultrasonido_frontal = clock_tick - 201U;
        if (failure == 5U) input.timestamp_ultrasonido_trasero = clock_tick - 201U;
        if (failure == 6U) input.distancia_m = 0.200f;
        if (failure == 7U) input.distancia_trasera_m = 0.200f;
        if (failure == 8U) input.distancia_m = NAN;
        if (failure == 9U) input.hay_caida = 1U;
        if (failure == 10U) TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
        if (failure == 11U) input.distancia_trasera_m = INFINITY;
        b73_step(); expect_zero();
        if (failure == 9U) { CHECK(TaskSafety_GetFaults() == 0U); run_safety(1); CHECK(TaskSafety_GetFaults() == SAFETY_FAULT_CLIFF); }
        b77_terminal();
        PASS("B7.7 missing stale invalid dangerous IR or Safety input cancels active turn");
    }
    b75_reverse(); clock_tick = navigation.reverse_started_tick + 300U;
    b75_data(0.250f, 0.300f); b73_step(); b77_terminal();
    PASS("B7.7 successful completion also requires rear clearance at the last reverse check");
    b75_reverse(); clock_tick = navigation.reverse_started_tick + 299U;
    b75_data(0.250f, 0.300f); b73_step(); b77_terminal();
    PASS("B7.7 interrupted reverse cannot later qualify as successful completion");

    DrvUart_t uart; UART_HandleTypeDef peripheral = {0};
    b77_turn(); DrvUart_Init(&uart, &peripheral);
    b65_uart_command(&uart, "STOP\n", "STOP_QUEUED\n"); expect_zero();
    run_motor(1); run_fsm(1); CHECK(robot_state == ROBOT_STATE_IDLE); b77_terminal();
    PASS("B7.7 actual UART STOP cancels turn immediately");
    b77_turn(); for (unsigned i = 0; i < 8U; ++i) CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); expect_zero(); run_fsm(1); b77_terminal();
    PASS("B7.7 full event queue cannot delay STOP cancellation of turn");
    for (int action = 1; action <= 4; ++action) {
        if (action == 2) continue;
        b77_finish_reverse(); clock_tick += 20U; b75_data(0.250f, 0.250f);
        overwrite_action = action; b73_step(); expect_zero(); run_motor(1); b77_terminal();
        PASS("B7.7 STOP mode change or Safety between turn admission and enqueue invalidates it");
    }
    b77_turn(); b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); expect_zero(); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); b77_terminal();
    PASS("B7.7 healthy RESET does not restart turn");
    b77_turn(); TaskSafety_ReportFault(SAFETY_FAULT_CLIFF); run_safety(1); run_fsm(1);
    clock_tick += 100U; b73_fresh(); b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE); b77_terminal();
    PASS("B7.7 safe explicit rearm never restores an old turn");

    const char *manual[] = {"AVANZAR\n", "RETROCEDER\n", "GIRAR_IZQUIERDA\n", "GIRAR_DERECHA\n", "DETENER\n"};
    b77_turn(); generation = command_generation;
    for (unsigned i = 0; i < 5U; ++i) {
        b65_uart_command(&uart, manual[i], "MOVE_REJECTED\n");
        CHECK(command_generation == generation && Navigation_GetState() == NAV_GIRAR_ACOTADO);
        PASS("B7.7 manual UART movement remains rejected during experimental turn");
    }

    for (unsigned old_phase = 0U; old_phase < 2U; ++old_phase) {
        b73_advance(); TaskMotorControlTarget_t stale = queued_target;
        b75_data(0.300f, 0.50f); b73_step(); clock_tick += 20U;
        b75_data(0.250f, 0.50f); b73_step();
        if (old_phase) stale = queued_target;
        run_motor(1); b77_complete_reverse(); clock_tick += 20U;
        b75_data(0.250f, 0.250f); b73_step();
        stale.generation = command_generation; /* Isolate the phase/session barrier. */
        CHECK((TickType_t)(clock_tick - stale.issued_tick) < 500U);
        CHECK(xQueueOverwrite(target_speed_queue, &stale) == pdPASS); run_motor(1); expect_zero();
        CHECK(Navigation_GetState() == NAV_GIRAR_ACOTADO);
        b75_data(0.250f, 0.250f); b73_step(); run_motor(1); CHECK(pwm.compare[0] > 0U);
        PASS("B7.7 prior advance or reverse target cannot enter turn even with current generation");
    }
    b77_turn(); TaskMotorControlTarget_t stale_turn = queued_target;
    clock_tick = navigation.turn_started_tick + 240U; run_motor(1); expect_zero();
    CHECK(xQueueOverwrite(target_speed_queue, &stale_turn) == pdPASS); run_motor(1); b77_terminal();
    PASS("B7.7 old turn target cannot reactivate after phase completion");
    b77_turn(); stale_turn = queued_target;
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1); ++clock_tick; b73_fresh();
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    b73_fresh(); b73_step(); stale_turn.generation = command_generation;
    CHECK(xQueueOverwrite(target_speed_queue, &stale_turn) == pdPASS); run_motor(1); expect_zero();
    CHECK(Navigation_GetState() == NAV_AVANZAR); b73_fresh(); b73_step(); run_motor(1);
    CHECK(left_motor.current_speed > 0 && right_motor.current_speed > 0);
    PASS("B7.7 a turn from a previous session cannot act in a new forward session");

    for (unsigned phase = 0U; phase < 3U; ++phase) {
        if (phase == 0U) b73_advance();
        else if (phase == 1U) b75_reverse();
        else b77_turn();
        uint32_t permission = navigation.session;
        navigation.emission_ready = true; navigation.decision_tick = clock_tick;
        CHECK(!Motion_NavigationTurn(clock_tick, permission)); /* Wrong task. */
        CHECK(navigation.emission_ready);
        current_task = CONTROL_TASK_ID;
        CHECK(!TaskMotorControl_TrySetNavigationSpeedsAt(-1199, 1199, clock_tick, permission));
        CHECK(!Motion_Execute(MOTION_TURN_LEFT, ROBOT_STATE_AUTONOMOUS, clock_tick));
        CHECK(!TaskMotorControl_TrySetNavigationSpeedsAt(-NAV_TURN_LEFT_CPS, -NAV_TURN_RIGHT_CPS,
                                                       clock_tick, permission));
        if (phase == 2U) {
            CHECK(!Motion_NavigationAdvance(clock_tick, permission));
            CHECK(!Motion_NavigationReverse(clock_tick, permission));
            CHECK(navigation.emission_ready && Motion_NavigationTurn(clock_tick, permission));
            CHECK(!Motion_NavigationTurn(clock_tick, permission));
        } else {
            CHECK(!Motion_NavigationTurn(clock_tick, permission) && navigation.emission_ready);
            bool accepted = phase == 0U ? Motion_NavigationAdvance(clock_tick, permission) :
                                         Motion_NavigationReverse(clock_tick, permission);
            CHECK(accepted);
        }
        current_task = (TaskHandle_t)12;
        PASS("B7.7 permission enforces task exact vector active phase configured direction and one use");
    }
    for (unsigned failure = 0U; failure < 5U; ++failure) {
        b75_reverse(); clock_tick = navigation.reverse_started_tick + 300U;
        b75_data(0.250f, 0.50f);
        if (failure == 0U) queue_available = 0;
        if (failure == 1U) input.ultrasonido_valido = 0U;
        if (failure == 2U) input.timestamp_ultrasonido_trasero = clock_tick - 201U;
        if (failure == 3U) input.hay_caida = 1U;
        if (failure == 4U) TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
        b73_step(); b77_terminal(); CHECK(!navigation.turn_used);
        PASS("B7.7 unsafe or unavailable final snapshot cannot authorize turn entry");
    }
    b75_reverse(); clock_tick = navigation.reverse_started_tick + 300U;
    b75_data(0.250f, 0.50f); fault_at_inner_exit = 1; b73_step();
    CHECK(TaskSafety_IsStopActive()); b77_terminal();
    PASS("B7.7 Safety made ready during reverse-to-turn transition wins over phase activation");

    setup(); clock_tick = UINT32_MAX - 400U; robot_state = ROBOT_STATE_IDLE;
    mode_entry_tick = auto_barrier_tick = clock_tick - 1U; b73_fresh();
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    b73_fresh(); b73_step(); run_motor(1);
    clock_tick = UINT32_MAX - 299U; b75_data(0.300f, 0.50f); b73_step();
    clock_tick += 20U; b75_data(0.250f, 0.50f); b73_step(); run_motor(1);
    b77_complete_reverse(); CHECK(navigation.turn_started_tick == 0U);
    clock_tick = 20U; b75_data(0.250f, 0.250f); b73_step(); run_motor(1);
    CHECK(pwm.compare[0] > 0U);
    clock_tick = 240U; b75_data(0.250f, 0.250f); b73_step(); run_motor(1); b77_terminal();
    PASS("B7.7 turn start tick zero and tick wrap preserve the 250 ms deadline");
    setup(); robot_state = ROBOT_STATE_IDLE; b73_fresh(); navigation.session = UINT32_MAX - 3U;
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    b73_fresh(); b73_step(); run_motor(1); b75_data(0.300f, 0.50f); b73_step();
    clock_tick += 20U; b75_data(0.250f, 0.50f); b73_step(); run_motor(1);
    b77_complete_reverse(); CHECK(navigation.session == 0U); clock_tick += 20U;
    b75_data(0.250f, 0.250f); b73_step(); run_motor(1); CHECK(pwm.compare[0] > 0U);
    PASS("B7.7 wrapped phase permission zero can authorize turn without reviving old tokens");
}
#endif
