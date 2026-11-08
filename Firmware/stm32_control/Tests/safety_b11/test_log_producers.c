/* Included by test_safety.c: exercises real producers through existing mocks.
 * Capture is enabled only for these cases; legacy regressions keep an inert logger. */
typedef struct {
    uint16_t code;
    uint32_t a, b, c;
    TickType_t tick;
} ObservedEvent;
static ObservedEvent logged[8], attempted[512];
static unsigned logged_count, attempted_count, logger_capacity = 8U;
static int in_isr, publication_hook;
static bool producer_logging_enabled = false;

BaseType_t Log_EventPublish(uint16_t code, uint32_t a, uint32_t b, uint32_t c)
{
    CHECK(!in_isr && critical_depth == 0);
    if (!producer_logging_enabled) return pdPASS;
    CHECK(scheduler_depth == 0 || (scheduler_depth == 1 && code == 300U));
    CHECK(code == 100U || code == 110U || code == 201U || code == 202U ||
          code == 203U || code == 210U || code == 300U);
    CHECK(attempted_count < 512U);
    ObservedEvent event = {code, a, b, c, xTaskGetTickCount()};
    attempted[attempted_count++] = event;
    if (code >= 201U && code <= 203U) {
        CHECK(TaskSafety_IsStopActive()); expect_zero();
    }
    if (code == 300U) { CHECK(stop_pending); expect_zero(); }
    if (publication_hook) {
        publication_hook = 0;
        TaskSafety_ReportFault(SAFETY_FAULT_SENSOR);
    }
    if (logged_count == logger_capacity) return pdFALSE; /* pdFAIL */
    CHECK(logged_count < 8U); logged[logged_count++] = event;
    return pdPASS;
}
static void clear_log(void) { logged_count = attempted_count = 0U; }
static void producer_setup(void)
{
    setup(); clear_log(); producer_logging_enabled = true; logger_capacity = 8U; publication_hook = in_isr = 0;
}
static unsigned count_code(uint16_t code)
{
    unsigned n = 0U;
    for (unsigned i = 0; i < logged_count; ++i) if (logged[i].code == code) ++n;
    return n;
}
static ObservedEvent get_code(uint16_t code)
{
    for (unsigned i = logged_count; i > 0U; --i) if (logged[i-1U].code == code) return logged[i-1U];
    CHECK(0 && "missing event"); return (ObservedEvent){0};
}
static void check_event(uint16_t code, uint32_t a, uint32_t b, uint32_t c)
{
    ObservedEvent e = get_code(code);
    CHECK(e.a == a && e.b == b && e.c == c);
}
static void new_safe_captures(void)
{
    clock_tick += 100U; input.timestamp = clock_tick;
    input.timestamp_ultrasonido_frontal = input.timestamp_ultrasonido_trasero = clock_tick;
    input.ultrasonido_valido = input.ultrasonido_trasero_valido = 1U;
    input.distancia_m = input.distancia_trasera_m = 0.5f; input.hay_caida = 0U;
}

static void test_log_producers(void)
{
    producer_setup(); robot_state = ROBOT_STATE_IDLE; TaskSafety_Create();
    CHECK(attempted_count == 0U); run_safety(5);
    CHECK(count_code(100U) == 1U && attempted_count == 1U);
    CHECK(logged[0].a == 0U && logged[0].b == 0U && logged[0].c == 1U && logged[0].tick == 1000U);
    CHECK(robot_state == ROBOT_STATE_IDLE && TaskSafety_IsStopActive());
    PASS("real Safety task emits one initial snapshot before its periodic loop, never from Create");
    producer_setup(); logger_capacity = 0U; robot_state = ROBOT_STATE_IDLE; TaskSafety_Create(); run_safety(5);
    CHECK(attempted_count == 1U && !logged_count && safety_stop_active && safety_faults == 0U);
    PASS("startup publication failure is not retried and does not change startup block");
    producer_setup(); robot_state = ROBOT_STATE_SAFETY_STOP; safety_stop_active = true;
    safety_faults = SAFETY_FAULT_OBSTACLE; clock_tick = 1234U; run_safety(0);
    check_event(100U, 3U, 2U, 1U);
    PASS("startup arguments are actual FSM/mask/block values, not synthetic constants");

    const uint32_t bits[] = {SAFETY_FAULT_CLIFF,SAFETY_FAULT_OBSTACLE,SAFETY_FAULT_SENSOR};
    for (unsigned i = 0; i < 3U; ++i) {
        producer_setup();
        for (unsigned j = 0; j < 20U; ++j) TaskSafety_ReportFault((SafetyFault_t)bits[i]);
        CHECK(attempted_count == 1U && logged_count == 1U);
        check_event((uint16_t)(201U + i),bits[i],bits[i],0U);
        CHECK(safety_fault_revision == 20U && command_generation == 20U && safety_fault_tick == clock_tick);
        PASS("new Safety bit logs once; repeated reports still perform original fault/stop/revision behavior");
    }
    producer_setup(); TaskSafety_ReportFault((SafetyFault_t)7U);
    CHECK(logged_count == 3U); check_event(201U,1U,7U,0U); check_event(202U,2U,7U,0U); check_event(203U,4U,7U,0U);
    TaskSafety_ReportFault((SafetyFault_t)7U); CHECK(logged_count == 3U);
    PASS("simultaneous three bits produce individual contract records with the same accumulated mask");
    producer_setup(); TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);
    TaskSafety_ReportFault((SafetyFault_t)3U); TaskSafety_ReportFault((SafetyFault_t)7U);
    CHECK(logged_count == 3U); check_event(201U,1U,1U,0U); check_event(202U,2U,3U,0U); check_event(203U,4U,7U,0U);
    PASS("incremental mask changes log only newly latched bits");
    producer_setup(); TaskSafety_ReportFault(SAFETY_FAULT_EMERGENCY);
    CHECK(!attempted_count && safety_faults == 8U && safety_stop_active); expect_zero();
    PASS("EMERGENCY behavior preserved without connecting reserved code 204");
    producer_setup(); TaskSafety_ReportFault((SafetyFault_t)15U);
    CHECK(logged_count == 3U); check_event(201U,1U,15U,0U); check_event(202U,2U,15U,0U); check_event(203U,4U,15U,0U);
    PASS("allowed event masks retain an emergency bit without emitting 204");
    producer_setup(); logger_capacity = 1U; TaskSafety_ReportFault((SafetyFault_t)7U);
    CHECK(attempted_count == 3U && logged_count == 1U && safety_faults == 7U);
    TaskSafety_ReportFault((SafetyFault_t)7U); CHECK(attempted_count == 3U && safety_fault_revision == 2U);
    PASS("partial logger saturation cannot alter Safety or cause retries of already latched bits");
    producer_setup(); publication_hook = 1; TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);
    CHECK(safety_faults == 5U && safety_fault_revision == 2U && logged_count == 2U);
    check_event(201U,1U,1U,0U); check_event(203U,4U,5U,0U);
    PASS("concurrent fault during publication preserves both actual latched-mask snapshots");
    producer_setup(); input.hay_caida = 1U; run_safety(5);
    CHECK(count_code(100U) == 1U && count_code(201U) == 1U && safety_fault_revision == 5U);
    PASS("actual periodic supervision does not log an IR fault every 20 ms");

    producer_setup(); TaskSafety_ReportFault((SafetyFault_t)7U); new_safe_captures(); clear_log();
    CHECK(TaskSafety_RequestReset()); check_event(210U,7U,0U,0U);
    CHECK(logged_count == 1U && !safety_stop_active && !safety_faults); expect_zero();
    clear_log(); TaskSafety_ReportFault(SAFETY_FAULT_SENSOR); check_event(203U,4U,4U,0U);
    PASS("successful rearm logs old mask; the same fault can log again in a new latch episode");
    producer_setup(); safety_stop_active = true; robot_state = ROBOT_STATE_SAFETY_STOP;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_RESET);
    CHECK(robot_state == ROBOT_STATE_IDLE); check_event(210U,0U,0U,0U); check_event(110U,3U,0U,4U);
    unsigned n = attempted_count; TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_RESET);
    CHECK(attempted_count == n); expect_zero();
    PASS("initial zero-mask rearm logs 210 and actual transition; repeated RESET in IDLE logs nothing");
    for (unsigned bad = 0; bad < 7U; ++bad) {
        producer_setup(); TaskSafety_ReportFault(SAFETY_FAULT_SENSOR); new_safe_captures(); clear_log();
        if (bad == 0U) queue_available = 0;
        if (bad == 1U) input.hay_caida = 1U;
        if (bad == 2U) input.ultrasonido_trasero_valido = 0U;
        if (bad == 3U) input.timestamp_ultrasonido_frontal = safety_fault_tick;
        if (bad == 4U) peek_action = 4;
        if (bad == 5U) pending_fault = 2;
        if (bad == 6U) TaskSafety_ReportFault(SAFETY_FAULT_EMERGENCY);
        CHECK(!TaskSafety_RequestReset() && count_code(210U) == 0U && safety_stop_active);
        PASS("unsafe/stale/old/concurrent/emergency reset rejected without successful-rearm event");
    }
    producer_setup(); TaskSafety_ReportFault(SAFETY_FAULT_SENSOR); new_safe_captures(); clear_log(); logger_capacity=0U;
    CHECK(TaskSafety_RequestReset() && attempted_count == 1U && !logged_count && !safety_stop_active);
    PASS("full logger cannot reject an otherwise valid rearm");

    producer_setup(); robot_state=ROBOT_STATE_IDLE;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL); check_event(110U,0U,2U,1U);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_AUTO); CHECK(logged_count == 1U);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP); check_event(110U,2U,0U,2U);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP); CHECK(logged_count == 2U && count_code(300U) == 0U);
    PASS("MANUAL and STOP effective transitions only; rejected/repeated events and internal STOP do not duplicate logs");
    producer_setup(); robot_state=ROBOT_STATE_IDLE;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_AUTO); check_event(110U,0U,1U,0U);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL); CHECK(logged_count == 1U);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP); check_event(110U,1U,0U,2U); expect_zero();
    PASS("AUTO enum transitions preserve existing FSM behavior without adding navigation");
    producer_setup(); robot_state=ROBOT_STATE_IDLE; safety_stop_active=true;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL); TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_AUTO);
    CHECK(!logged_count && robot_state == ROBOT_STATE_IDLE);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_STOP); check_event(110U,0U,3U,3U);
    n=attempted_count;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_AUTO); TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_STOP); TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_STOP);
    CHECK(attempted_count == n && robot_state == ROBOT_STATE_SAFETY_STOP);
    PASS("blocked mode requests and repeated Safety/STOP attempts do not claim transitions");
    producer_setup(); TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_STOP);
    CHECK(!logged_count && robot_state == ROBOT_STATE_MANUAL);
    TaskControlRobot_ProcessEvent((RobotControlEvent_t)99); CHECK(!logged_count);
    PASS("obsolete Safety and unknown FSM events preserve state without logging a transition");
    producer_setup(); robot_state=ROBOT_STATE_IDLE; logger_capacity=0U;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL);
    CHECK(robot_state == ROBOT_STATE_MANUAL && attempted_count == 1U && !logged_count);
    PASS("failed transition publication does not undo MANUAL entry");

    producer_setup(); DrvUart_t uart; UART_HandleTypeDef peripheral={0}; DrvUart_Init(&uart,&peripheral);
    in_isr=1; uart_feed(&uart,"STOP\n"); in_isr=0; CHECK(!attempted_count);
    CHECK(uart_consume(&uart) && !strcmp(uart_response,"STOP_QUEUED\n"));
    check_event(300U,2U,0U,0U); CHECK(stop_pending && robot_state == ROBOT_STATE_MANUAL);
    run_fsm(1); CHECK(robot_state == ROBOT_STATE_IDLE && !stop_pending);
    check_event(110U,2U,0U,2U); CHECK(count_code(300U) == 1U); expect_zero();
    CHECK(get_code(300U).tick == 1000U && get_code(110U).tick == 1020U);
    PASS("actual UART STOP logs once from task context; cancellation precedes logging and later IDLE transition");
    producer_setup(); for (unsigned i=0;i<8U;++i) CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); check_event(300U,2U,0U,0U);
    CHECK(stop_pending); expect_zero(); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_IDLE && count_code(300U) == 1U && count_code(110U) == 1U);
    PASS("full FSM queue still cancels/logs request once; internal pending-STOP fallback does not emit another 300");
    producer_setup(); logger_capacity=0U; for (unsigned i=0;i<8U;++i) CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1); expect_zero();
    CHECK(robot_state == ROBOT_STATE_IDLE && !stop_pending && attempted_count == 2U && !logged_count);
    PASS("both queues saturated: STOP and IDLE continue, no logger retries or altered UART/FSM decisions");
    producer_setup(); TaskSafety_ReportFault(SAFETY_FAULT_CLIFF); robot_state=ROBOT_STATE_SAFETY_STOP; clear_log();
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1);
    check_event(300U,3U,0U,0U); CHECK(count_code(110U) == 0U && safety_faults == 1U && safety_stop_active);
    PASS("STOP received during latched Safety is logged without claiming IDLE or clearing the fault");
    producer_setup(); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1);
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP)); run_fsm(1);
    CHECK(count_code(300U) == 2U && count_code(110U) == 1U);
    PASS("two real STOP requests produce two request events but only one effective transition");
    producer_setup(); CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL)); run_fsm(1);
    CHECK(robot_state == ROBOT_STATE_MANUAL && count_code(110U) == 2U && count_code(300U) == 1U);
    CHECK(logged[1].a == 2U && logged[1].b == 0U && logged[2].a == 0U && logged[2].b == 2U);
    CHECK(logged[0].tick == 1000U && logged[1].tick == 1020U && logged[1].tick == logged[2].tick); expect_zero();
    PASS("STOP/MANUAL in the same processing cycle logs both actual transitions with no stale motion");
    producer_setup(); robot_state=ROBOT_STATE_IDLE; stop_pending=true;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_MANUAL); TaskControlRobot_ProcessEvent(ROBOT_EVENT_START_AUTO);
    CHECK(robot_state == ROBOT_STATE_IDLE && !logged_count);
    PASS("pending STOP still bars both mode entries without false transition records");
    producer_setup(); TaskSafety_ReportFault(SAFETY_FAULT_SENSOR); new_safe_captures();
    robot_state=ROBOT_STATE_SAFETY_STOP; clear_log(); publication_hook=1;
    TaskControlRobot_ProcessEvent(ROBOT_EVENT_SAFETY_RESET);
    CHECK(safety_stop_active && robot_state == ROBOT_STATE_SAFETY_STOP && count_code(110U) == 0U); expect_zero();
    PASS("fault during post-rearm publication still wins the existing final FSM recheck; no false IDLE transition");

    producer_logging_enabled = false;
}
