/* B7.5 uses the same real sources and RTOS/HAL harness as B7.3. */
static void b75_data(float front, float rear) {
    b73_fresh(); input.distancia_m = front; input.distancia_trasera_m = rear;
}
static void b75_prepare_reverse(void) {
    b73_advance();
    TickType_t started = navigation.started_tick;
    uint32_t forward_permission = navigation.session;
    CHECK(task_context.left_target_cps == 1600 && task_context.right_target_cps == 1600);
    b75_data(0.300f, 0.50f); b73_step();
    CHECK(Navigation_GetState() == NAV_RETROCEDER_ACOTADO);
    CHECK(navigation.started_tick == started && navigation.reverse_started_tick == clock_tick);
    CHECK(navigation.session != forward_permission && !navigation.emission_ready && !target_pending);
    expect_zero();
}
static void b75_reverse(void) {
    b75_prepare_reverse(); clock_tick += 20U; b75_data(0.250f, 0.50f); b73_step();
    CHECK(queued_target.left_speed_cps == -1600 && queued_target.right_speed_cps == -1600);
    CHECK(queued_target.navigation_session == navigation.session);
    run_motor(1);
    CHECK(task_context.left_target_cps == -1600 && task_context.right_target_cps == -1600);
    CHECK(left_motor.current_speed < 0 && right_motor.current_speed < 0);
    CHECK(pwm.compare[0] > 0U && pwm.compare[0] <= 840U && pwm.compare[1] <= 840U);
}
static void test_navigation_b75(void) {
    b73_advance(); CHECK(Navigation_GetState() == NAV_AVANZAR);
    PASS("B7.5 clear path keeps the existing fixed forward movement");
    const float blocked[] = {0.300f, 0.299f, 0.200f};
    for (unsigned i = 0; i < 3U; ++i) {
        setup(); robot_state = ROBOT_STATE_IDLE; b75_data(blocked[i], 0.50f);
        CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1);
        b73_terminal();
        PASS("B7.5 AUTO starting in front of an obstacle never starts reverse");
    }
    b75_prepare_reverse();
    PASS("B7.5 300 mm obstacle during advance clears positive targets integral PWM and permission first");
    uint32_t generation = command_generation; b73_step();
    CHECK(Navigation_GetState() == NAV_RETROCEDER_ACOTADO && !target_pending);
    CHECK(command_generation == generation); expect_zero();
    PASS("B7.5 another evaluation in the same stop tick cannot emit reverse");
    b75_reverse();
    PASS("B7.5 safe rear permits only paired minus 1600 cps through the real PI and limited driver");

    const float rear[] = {0.301f, 0.300f, 0.299f};
    for (unsigned i = 0; i < 3U; ++i) {
        b73_advance(); b75_data(0.300f, rear[i]); b73_step(); expect_zero();
        if (i == 0U) CHECK(Navigation_GetState() == NAV_RETROCEDER_ACOTADO);
        else b73_terminal();
        PASS("B7.5 rear must be strictly above 300 mm before accepting reverse");
    }
    const float front[] = {0.201f, 0.200f, 0.199f};
    for (unsigned i = 0; i < 3U; ++i) {
        b73_advance(); b75_data(front[i], 0.50f); b73_step(); expect_zero();
        if (i == 0U) CHECK(Navigation_GetState() == NAV_RETROCEDER_ACOTADO);
        else b73_terminal();
        PASS("B7.5 front must remain strictly above the unchanged 200 mm Safety limit");
    }
    b73_begin(); b75_data(0.50f, 0.250f); b73_step(); run_motor(1);
    CHECK(pwm.compare[0] > 0U); b75_data(0.300f, 0.250f); b73_step(); b73_terminal();
    PASS("B7.5 rear 300 mm rule applies to reverse and does not change forward Safety policy");

    for (unsigned phase = 0; phase < 2U; ++phase) {
        for (unsigned failure = 0; failure < 12U; ++failure) {
            if (phase) b75_reverse(); else b73_advance();
            b75_data(0.300f, 0.50f);
            if (failure == 0U) queue_available = 0;
            if (failure == 1U) input.ultrasonido_valido = 0U;
            if (failure == 2U) input.ultrasonido_trasero_valido = 0U;
            if (failure == 3U) input.timestamp = clock_tick - 101U;
            if (failure == 4U) input.timestamp_ultrasonido_frontal = clock_tick - 201U;
            if (failure == 5U) input.timestamp_ultrasonido_trasero = clock_tick - 201U;
            if (failure == 6U) input.distancia_m = NAN;
            if (failure == 7U) input.distancia_trasera_m = INFINITY;
            if (failure == 8U) input.hay_caida = 1U;
            if (failure == 9U) input.distancia_m = 0.200f;
            if (failure == 10U) input.distancia_trasera_m = 0.300f;
            if (failure == 11U) TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
            b73_step(); expect_zero();
            if (failure == 8U) {
                CHECK(TaskSafety_GetFaults() == SAFETY_FAULT_NONE);
                run_safety(1); CHECK(TaskSafety_GetFaults() == SAFETY_FAULT_CLIFF);
            }
            b73_terminal();
            PASS("B7.5 unavailable invalid stale dangerous IR or blocked conditions reject or cancel reverse");
        }
    }
    b75_reverse(); TickType_t reverse_start = navigation.reverse_started_tick;
    TickType_t session_start = navigation.started_tick;
    clock_tick = reverse_start + 280U; b75_data(0.50f, 0.50f); b73_step(); run_motor(1);
    CHECK(clock_tick == reverse_start + 290U && left_motor.current_speed < 0);
    CHECK(navigation.started_tick == session_start);
    run_motor(1); CHECK(clock_tick == reverse_start + 300U);
    b73_terminal();
    PASS("B7.5 motor backend stops at 300 ms even without another navigation cycle");
    b75_prepare_reverse(); reverse_start = navigation.reverse_started_tick;
    clock_tick = reverse_start + 299U; b75_data(0.50f, 0.50f); b73_step();
    CHECK(queued_target.left_speed_cps == -1600); ++clock_tick; b73_step(); b73_terminal();
    PASS("B7.5 reverse emission at 299 ms is allowed but 300 ms is terminal");
    b75_prepare_reverse(); reverse_start = navigation.reverse_started_tick;
    clock_tick = reverse_start + 299U; b75_data(0.250f, 0.50f);
    advance_tick_before_navigation_emit = 1U; b73_step();
    CHECK(clock_tick == reverse_start + 300U && !target_pending); b73_terminal();
    PASS("B7.5 deadline crossed between reverse decision and publication rejects the target");
    b73_begin(); session_start = navigation.started_tick; queue_available = 0; b73_step();
    clock_tick = session_start + 1800U; b73_fresh(); b73_step(); run_motor(1);
    CHECK(left_motor.current_speed > 0);
    clock_tick = session_start + 1900U; b75_data(0.300f, 0.50f); b73_step();
    reverse_start = navigation.reverse_started_tick;
    clock_tick += 20U; b75_data(0.250f, 0.50f); b73_step(); run_motor(1);
    CHECK(left_motor.current_speed < 0);
    clock_tick = session_start + 1990U; b75_data(0.250f, 0.50f); b73_step(); run_motor(1);
    CHECK((TickType_t)(clock_tick - session_start) == 2000U);
    CHECK((TickType_t)(clock_tick - reverse_start) == 100U); b73_terminal();
    PASS("B7.5 global two-second lease truncates reverse without extending the session");
    b75_prepare_reverse(); reverse_start = navigation.reverse_started_tick;
    for (unsigned i = 1U; i < 15U; ++i) {
        clock_tick = reverse_start + i * 20U; b75_data(0.50f, 0.50f); b73_step(); run_motor(1);
        CHECK(left_motor.current_speed < 0 && navigation.reverse_started_tick == reverse_start);
    }
    clock_tick = reverse_start + 300U; b73_step(); b73_terminal();
    PASS("B7.5 rear-clear renewals and disappearance of the front obstacle cannot extend reverse or restart advance");

    DrvUart_t uart; UART_HandleTypeDef peripheral = {0};
    b75_reverse(); DrvUart_Init(&uart, &peripheral);
    b65_uart_command(&uart, "STOP\n", "STOP_QUEUED\n"); expect_zero();
    run_motor(1); run_fsm(1); CHECK(robot_state == ROBOT_STATE_IDLE); b73_terminal();
    PASS("B7.5 actual UART STOP cancels reverse before its deadline");
    b75_reverse();
    for (unsigned i = 0; i < 8U; ++i) CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); expect_zero(); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE); b73_terminal();
    PASS("B7.5 full FSM queue cannot delay reverse cancellation by STOP");
    for (unsigned action = 1U; action <= 4U; ++action) {
        if (action == 2U) continue;
        b75_prepare_reverse(); clock_tick += 20U; b75_data(0.250f, 0.50f);
        overwrite_action = (int)action; b73_step(); expect_zero(); run_motor(1); b73_terminal();
        PASS("B7.5 STOP mode change or concurrent Safety invalidates a late negative enqueue");
    }
    b73_advance(); b75_data(0.300f, 0.50f); fault_at_inner_exit = 1; b73_step();
    CHECK(TaskSafety_IsStopActive()); b73_terminal();
    PASS("B7.5 Safety made ready during stop-to-reverse transition cannot be overwritten by phase activation");
    b75_reverse(); b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); expect_zero();
    run_fsm(1); CHECK(robot_state == ROBOT_STATE_AUTONOMOUS); b73_terminal();
    PASS("B7.5 healthy RESET leaves the reverse session terminal without auto reactivation");
    b75_reverse(); TaskSafety_ReportFault(SAFETY_FAULT_CLIFF); expect_zero(); run_safety(1); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_SAFETY_STOP); clock_tick += 100U; b73_fresh();
    b65_uart_command(&uart, "RESET\n", "RESET_QUEUED\n"); run_fsm(1); run_motor(1);
    CHECK(robot_state == ROBOT_STATE_IDLE && !TaskSafety_IsStopActive()); b73_terminal();
    PASS("B7.5 safety recovery and RESET never restore a previous reverse session");

    b75_reverse(); b75_data(0.201f, 0.301f); b73_step(); run_motor(1);
    CHECK(left_motor.current_speed < 0 && Navigation_GetState() == NAV_RETROCEDER_ACOTADO);
    PASS("B7.5 reverse remains permitted at front 201 mm and rear 301 mm");
    b75_prepare_reverse(); reverse_start = navigation.reverse_started_tick;
    clock_tick = reverse_start + 290U; b75_data(0.250f, 0.50f); b73_step();
    CHECK(target_pending && queued_target.left_speed_cps == -1600); run_motor(1);
    CHECK(clock_tick == reverse_start + 300U); b73_terminal();
    PASS("B7.5 freshly queued negative target is rejected if first consumed at the reverse deadline");

    b73_advance(); TaskMotorControlTarget_t old_forward = queued_target;
    b75_data(0.300f, 0.50f); b73_step();
    old_forward.generation = command_generation; /* Isolate phase permission from generation. */
    CHECK(xQueueOverwrite(target_speed_queue, &old_forward) == pdPASS); run_motor(1); expect_zero();
    CHECK(Navigation_GetState() == NAV_RETROCEDER_ACOTADO);
    b75_data(0.250f, 0.50f); b73_step(); run_motor(1); CHECK(left_motor.current_speed < 0);
    PASS("B7.5 old positive target cannot cross into reverse even with matching generation");
    b75_reverse(); TaskMotorControlTarget_t old_reverse = queued_target;
    reverse_start = navigation.reverse_started_tick;
    clock_tick = reverse_start + 290U; run_motor(1); expect_zero();
    CHECK(xQueueOverwrite(target_speed_queue, &old_reverse) == pdPASS); run_motor(1); b73_terminal();
    PASS("B7.5 late negative target cannot reactivate motors after reverse completion");
    b75_reverse(); old_reverse = queued_target;
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1); ++clock_tick;
    b73_fresh(); CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    b73_fresh(); b73_step();
    CHECK(navigation.session != old_reverse.navigation_session);
    CHECK((TickType_t)(clock_tick - old_reverse.issued_tick) < 500U);
    old_reverse.generation = command_generation;
    CHECK(xQueueOverwrite(target_speed_queue, &old_reverse) == pdPASS); run_motor(1); expect_zero();
    CHECK(Navigation_GetState() == NAV_AVANZAR);
    b73_fresh(); b73_step(); run_motor(1); CHECK(left_motor.current_speed > 0);
    PASS("B7.5 reverse from an old session cannot enter a new AUTO session before the 500 ms deadline");
    b75_reverse(); old_reverse = queued_target;
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1);
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL)); run_fsm(1);
    old_reverse.generation = command_generation;
    CHECK(xQueueOverwrite(target_speed_queue, &old_reverse) == pdPASS); run_motor(1); expect_zero();
    TaskMotorControl_SetTargetSpeeds(100, 100); run_motor(1); CHECK(left_motor.current_speed > 0);
    PASS("B7.5 an old AUTO reverse cannot cross to MANUAL and a fresh manual order still works");

    /* Exercise direction binding independently of origin and one-use binding. */
    for (unsigned reversing = 0; reversing < 2U; ++reversing) {
        if (reversing) b75_reverse(); else b73_advance();
        uint32_t permission = navigation.session;
        uint32_t generation_before = command_generation;
        navigation.emission_ready = true; navigation.decision_tick = clock_tick;
        CHECK(!Motion_NavigationReverse(clock_tick, permission)); /* Wrong task. */
        CHECK(navigation.emission_ready && command_generation == generation_before);
        current_task = CONTROL_TASK_ID;
        bool wrong = reversing ? Motion_NavigationAdvance(clock_tick, permission) :
                                 Motion_NavigationReverse(clock_tick, permission);
        CHECK(!wrong && navigation.emission_ready);
        CHECK(!TaskMotorControl_TrySetNavigationSpeedsAt(-1600, 1600, clock_tick, permission));
        CHECK(!TaskMotorControl_TrySetNavigationSpeedsAt(-1601, -1601, clock_tick, permission));
        CHECK(!TaskMotorControl_TrySetTargetSpeedsAt(-1600, -1600, clock_tick, ROBOT_STATE_AUTONOMOUS));
        bool accepted = reversing ? Motion_NavigationReverse(clock_tick, permission) :
                                    Motion_NavigationAdvance(clock_tick, permission);
        CHECK(accepted && !navigation.emission_ready);
        bool repeated = reversing ? Motion_NavigationReverse(clock_tick, permission) :
                                    Motion_NavigationAdvance(clock_tick, permission);
        CHECK(!repeated); current_task = (TaskHandle_t)12; run_motor(1);
        CHECK((left_motor.current_speed < 0) == (reversing != 0U));
        PASS("B7.5 permission is bound to task direction magnitude phase and a single emission");
    }
    b75_reverse(); uint32_t generation_before = command_generation;
    const char *manual[] = {"AVANZAR\n", "RETROCEDER\n", "GIRAR_IZQUIERDA\n", "GIRAR_DERECHA\n", "DETENER\n"};
    for (unsigned i = 0; i < 5U; ++i) {
        b65_uart_command(&uart, manual[i], "MOVE_REJECTED\n");
        CHECK(command_generation == generation_before && Navigation_GetState() == NAV_RETROCEDER_ACOTADO);
        PASS("B7.5 manual UART commands cannot use the active AUTO reverse authorization");
    }
    b75_reverse(); robot_state = ROBOT_STATE_IDLE; b73_step(); b73_terminal();
    PASS("B7.5 exiting AUTO cancels an active reverse");
    b75_reverse(); b75_data(0.250f, 0.50f); overwrite_action = 6; b73_step(); expect_zero();
    CHECK(Navigation_GetState() == NAV_RETROCEDER_ACOTADO); run_motor(1);
    CHECK(left_motor.current_speed < 0);
    PASS("B7.5 motor preemption before a normal negative renewal does not terminate the active phase");

    setup(); clock_tick = UINT32_MAX - 100U; robot_state = ROBOT_STATE_IDLE;
    mode_entry_tick = auto_barrier_tick = clock_tick - 1U; b73_fresh();
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    b73_fresh(); b73_step(); run_motor(1); CHECK(left_motor.current_speed > 0);
    clock_tick = UINT32_MAX - 10U; b75_data(0.300f, 0.50f); b73_step();
    reverse_start = navigation.reverse_started_tick;
    clock_tick += 20U; b75_data(0.250f, 0.50f); b73_step(); run_motor(1);
    CHECK(left_motor.current_speed < 0);
    clock_tick = reverse_start + 290U; b75_data(0.250f, 0.50f); b73_step(); run_motor(1);
    CHECK((TickType_t)(clock_tick - reverse_start) == 300U); b73_terminal();
    PASS("B7.5 reverse deadline survives TickType wrap and remains inside the global lease");
    setup(); clock_tick = UINT32_MAX - 50U; robot_state = ROBOT_STATE_IDLE;
    mode_entry_tick = auto_barrier_tick = clock_tick - 1U; b73_fresh();
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    b73_fresh(); b73_step(); run_motor(1);
    clock_tick = 0U; b75_data(0.300f, 0.50f); b73_step();
    CHECK(navigation.reverse_started_tick == 0U && Navigation_GetState() == NAV_RETROCEDER_ACOTADO);
    clock_tick = 20U; b75_data(0.250f, 0.50f); b73_step(); run_motor(1);
    CHECK(left_motor.current_speed < 0);
    clock_tick = 290U; b75_data(0.250f, 0.50f); b73_step(); run_motor(1); b73_terminal();
    PASS("B7.5 reverse start tick zero is legitimate and not an inactive sentinel");
    setup(); robot_state = ROBOT_STATE_IDLE; b73_fresh(); navigation.session = UINT32_MAX - 2U;
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1); ++clock_tick;
    b73_fresh(); b73_step(); run_motor(1);
    b75_data(0.300f, 0.50f); b73_step(); CHECK(navigation.session == 0U);
    clock_tick += 20U; b75_data(0.250f, 0.50f); b73_step(); run_motor(1);
    CHECK(left_motor.current_speed < 0);
    PASS("B7.5 phase permission zero after epoch wrap can authorize the correct negative target");
}
