/* Real TaskComms and UART driver, scheduler/interrupt delivery substituted. */
static DrvUart_t *startup_uart;
static TickType_t startup_first_tick;

static void startup_comms_schedule(void) {
    CHECK(mock_scheduler_started && uart_rearms != 0U);
    if (delay_calls == 1) {
        CHECK(uart_rearms == 1U && uart_isr_ticks == 0U);
        startup_first_tick = clock_tick;
        uart_feed(startup_uart, "AVAN");
        CHECK(uart_isr_ticks == 1U && !DrvUart_IsMessageReady(startup_uart));
    } else if (delay_calls == 2) {
        uart_feed(startup_uart, "ZAR\n");
        CHECK(startup_uart->message_tick == startup_first_tick);
    } else if (delay_calls == 3) {
        CHECK(!strcmp(uart_response, "MOVE_ACCEPTED\n") && target_pending);
        CHECK(queued_target.issued_tick == startup_first_tick);
        CHECK(queued_target.left_speed_cps == 1600 && queued_target.right_speed_cps == 1600);
        uart_feed(startup_uart, "STATUS\n");
    } else if (delay_calls == 4) {
        CHECK(!strcmp(uart_response, "STATE=2 FAULTS=0 STOP=0\n"));
        uart_feed(startup_uart, "STOP\n");
    }
}

static void test_uart_startup(void) {
    const TickType_t starts[] = {0U, UINT32_MAX - 14U, UINT32_MAX - 9U};
    for (unsigned i = 0; i < sizeof(starts) / sizeof(starts[0]); ++i) {
        setup(); mock_scheduler_started = false;
        DrvUart_t uart; UART_HandleTypeDef peripheral = {0};
        DrvUart_Init(&uart, &peripheral);
        TaskComms_Create(&uart);
        CHECK(uart_rearms == 0U && uart_isr_ticks == 0U);
        CHECK(comms_entry == TaskComms_Run && comms_argument == &uart);
        /* Incoming bytes can only reach RxCallback once Receive_IT is armed.
         * No ISR FreeRTOS call is permitted by the mock before scheduler start. */
        mock_scheduler_started = true;
        clock_tick = starts[i]; mode_entry_tick = clock_tick - 1U;
        startup_uart = &uart; comms_cycle_hook = startup_comms_schedule;
        current_task = COMMS_TASK_ID; cycles = 4; delay_calls = 0;
        if (setjmp(finish) == 0) comms_entry(comms_argument);
        comms_cycle_hook = NULL;
        CHECK(!strcmp(uart_response, "STOP_QUEUED\n") && stop_pending);
        CHECK(uart_isr_ticks == 3U && uart_rearms == 21U);
        run_motor(1); /* STOP revokes any earlier queued target at the real backend. */
        expect_zero();
        PASS("UART starts reception only from scheduled TaskComms; first-byte ticks, fragmented movement, STATUS and priority STOP survive tick zero/wrap");
    }
}
