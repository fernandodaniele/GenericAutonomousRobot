/* AUTO-2E: datos sinteticos sobre FSM/MotorControl/odometria reales. */
#if NAV_AUTO2_ENABLED
static void a2e_wait(void) {
    a1_advance();run_motor(1);a2_data(0.270f,0.487f);b73_step();
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && !navigation.emission_ready);
    CHECK(!navigation_odometry.ready && navigation_diagnostics.settle_state==1U);expect_zero();
}
static void a2e_sample(int32_t l,int32_t r,TickType_t dt) {
    a1_sample(l,r,dt);a2_data(0.270f,0.487f);b73_step();
}
static unsigned a2e_residual_cycles;
static void a2e_real_residual(void) {
    a2_motor_schedule();
    if(navigation.state==NAV_DETENER_Y_ESPERAR) {
        CHECK(task_context.left_target_cps==0 && task_context.right_target_cps==0);expect_zero();
        if(a2e_residual_cycles++<3U) { left_encoder_delta=3;right_encoder_delta=2; }
    }
}
#endif
static void test_navigation_settle(void) {
#if NAV_AUTO2_ENABLED
    CHECK(NAV_SETTLE_WINDOW_MS==60U && NAV_SETTLE_MIN_OBSERVATIONS==4U);
    CHECK(NAV_SETTLE_MAX_SPAN_COUNTS==1U && NAV_SETTLE_TIMEOUT_MS==200U);
    a2e_wait();TickType_t entry=navigation.phase_started_tick;
    int64_t l=s_odom.left_count,r=s_odom.right_count;
    uint32_t generation=command_generation;
    for(unsigned i=0;i<4U;++i) {
        a2e_sample(i==0U?3:0,i==0U?2:0,20U);expect_zero();
        CHECK(command_generation==generation+(i==3U?1U:0U));
        CHECK(!(nav_diag().flags & NAV_DIAG_REVERSE_REQUESTED));
        if(i<3U) {
            CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);
            CHECK(!Motion_NavigationReverse(clock_tick,navigation.session));
        }
    }
    NavigationDiagnostics_t d=nav_diag();
    CHECK(navigation.state==NAV_RETROCEDER_ACOTADO && d.settle_state==2U);
    CHECK(d.settle_observations==4U && d.settle_window_ms==60U && d.settle_elapsed_ms==80U);
    CHECK(!d.reverse_reference_available && !navigation_odometry.ready);
    CHECK(navigation.reverse_started_tick==entry+80U);
    a2e_sample(0,0,20U);d=nav_diag();
    CHECK(d.reverse_reference_available && d.reverse_reference_left==l+3 && d.reverse_reference_right==r+2);
    CHECK(d.reverse_reference_tick==clock_tick && d.reverse_reference_tick!=entry);
    CHECK(queued_target.left_speed_cps==-1600 && queued_target.right_speed_cps==-1600);
    CHECK(nav_diag().flags & NAV_DIAG_REVERSE_PUBLISHED);
    a2e_sample(-100,-99,10U);CHECK(navigation.active && nav_diag().reverse_um>0);
    a2e_sample(-260,-258,20U);CHECK(navigation.reverse_completed);expect_zero();
    CHECK(nav_diag().odometry_error==NAV_ODOM_OK);
    PASS("AUTO-2E physical-case +3/+2 residuals settle at zero before a new reference, valid negative progress and 20 mm completion");

    a2e_wait();
    for(unsigned i=0;i<6U;++i) {
        a2e_sample(0,i<3U?2:0,20U);expect_zero();
        if(i<5U)CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);
    }
    CHECK(navigation.state==NAV_RETROCEDER_ACOTADO && nav_diag().settle_observations==6U);
    CHECK(nav_diag().settle_window_observations==4U && nav_diag().settle_elapsed_ms==120U);
    PASS("AUTO-2E one stopped wheel cannot authorize reverse while the other still changes its count range");
    a2e_wait();const TickType_t irregular[]={20U,10U,15U,35U};
    for(unsigned i=0;i<4U;++i) { a2e_sample(0,0,irregular[i]);if(i<3U)CHECK(navigation.state==NAV_DETENER_Y_ESPERAR); }
    CHECK(nav_diag().settle_state==2U && nav_diag().settle_window_ms==60U);
    a2e_wait();
    for(unsigned i=0;i<6U;++i) { a2e_sample(0,0,10U);CHECK(navigation.state==NAV_DETENER_Y_ESPERAR); }
    a2e_sample(0,0,10U);CHECK(nav_diag().settle_state==2U && nav_diag().settle_window_observations==7U);
    PASS("AUTO-2E irregular sampling enforces both four new observations and 60 ms, not a count-only or fixed-delay shortcut");

    a2e_wait();a2e_sample(0,0,20U);uint16_t observations=nav_diag().settle_observations;
    b73_step();clock_tick+=10U;a2_data(0.270f,0.487f);b73_step();
    CHECK(nav_diag().settle_observations==observations && navigation.state==NAV_DETENER_Y_ESPERAR);
    ++s_odom.left_count;b73_step();a1_terminal(NAV_FINISH_ODOMETRY);
    CHECK(nav_diag().odometry_error==NAV_ODOM_TIMESTAMP);
    PASS("AUTO-2E repeated timestamps are not new observations and changed counts at the same timestamp are rejected");
    a2e_wait();a2e_sample(0,0,20U);--s_odom.tick;b73_step();
    a1_terminal(NAV_FINISH_ODOMETRY);CHECK(nav_diag().odometry_error==NAV_ODOM_TIMESTAMP);
    PASS("AUTO-2E out-of-order odometry cancels the wait instead of confirming stability");
    a2e_wait();
    for(unsigned i=0;i<8U;++i) { a2e_sample(i%2U==0U?2:-2,0,20U);CHECK(navigation.state==NAV_DETENER_Y_ESPERAR); }
    a2e_sample(2,0,20U);CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);
    a2e_sample(-2,0,20U);a1_terminal(NAV_FINISH_TIMEOUT);
    CHECK(nav_diag().settle_state==3U && nav_diag().last_phase==NAV_DETENER_Y_ESPERAR);
    PASS("AUTO-2E oscillation with zero net displacement fails the per-wheel range guard and times out safely at 200 ms");
    a2e_wait();a2e_sample(0,0,20U);
    a2e_sample(1,-1,20U);a2e_sample(-1,1,20U);a2e_sample(1,-1,20U);
    CHECK(nav_diag().settle_state==2U && nav_diag().settle_left_span==1U && nav_diag().settle_right_span==1U);
    PASS("AUTO-2E bounded one-count quantization over four new samples is accepted without a zero-speed assumption");
    a2e_wait();a2e_sample(0,0,20U);s_odom.left_count=INT64_MIN;s_odom.tick=++clock_tick;b73_step();
    a1_terminal(NAV_FINISH_ODOMETRY);CHECK(nav_diag().odometry_error==NAV_ODOM_OVERFLOW);
    PASS("AUTO-2E extreme count span cannot overflow into a false stability confirmation");

    for(unsigned fault=0;fault<9U;++fault) {
        a2e_wait();uint32_t session=navigation.session;
        if(fault==0U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
        if(fault==1U)TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);
        if(fault==2U)input.ultrasonido_trasero_valido=0U;
        if(fault==3U)input.timestamp_ultrasonido_trasero=clock_tick-201U;
        if(fault==4U)input.distancia_trasera_m=0.250f;
        if(fault==5U)s_odom.tick=clock_tick-51U;
        if(fault==6U)input.hay_caida=1U;
        if(fault==7U)robot_state=ROBOT_STATE_IDLE;
        if(fault==8U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET));
        if(fault==8U)run_fsm(1);else b73_step();
        CHECK(navigation.state==NAV_DETENIDO && nav_diag().settle_state==4U);
        CHECK(!(nav_diag().flags & NAV_DIAG_REVERSE_AUTHORIZED));
        CHECK(!Navigation_IsCommandCurrent(session,clock_tick));expect_zero();
        NavigationFinishReason_t reason=nav_diag().reason;uint32_t id=nav_diag().session_id;
        b73_fresh();b73_step();run_motor(1);expect_zero();
        CHECK(nav_diag().reason==reason && nav_diag().session_id==id);
        PASS("AUTO-2E STOP/Safety/rear sensor/odometry/mode/RESET cancel the wait with retained diagnostics and no reverse permission");
    }
    a2e_wait();entry=navigation.phase_started_tick;
    clock_tick=entry+199U;s_odom.tick=clock_tick;a2_data(0.270f,0.487f);b73_step();
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);++clock_tick;b73_step();
    a1_terminal(NAV_FINISH_TIMEOUT);CHECK(nav_diag().settle_elapsed_ms==200U && nav_diag().settle_state==3U);
    PASS("AUTO-2E 199 ms wait remains stopped and the exact 200 ms deadline is terminal before any new emission");
    a2e_wait();navigation.started_tick=clock_tick-2990U;
    a2e_sample(0,0,10U);a1_terminal(NAV_FINISH_TIMEOUT);
    CHECK(nav_diag().settle_state==4U && nav_diag().settle_elapsed_ms==10U);
    PASS("AUTO-2E global 3000 ms deadline cancels even a fresh wait and is distinct from stability timeout");

    a2e_wait();entry=navigation.phase_started_tick;
    for(unsigned i=0;i<9U;++i)a2e_sample(i<6U?2:0,i<6U?2:0,20U);
    CHECK(navigation.state==NAV_RETROCEDER_ACOTADO && navigation.reverse_started_tick==entry+180U);
    a2e_sample(0,0,20U);CHECK(navigation_odometry.progress_tick==clock_tick);
    CHECK(nav_diag().settle_elapsed_ms==180U && nav_diag().phase_elapsed_ms==20U);
    a2e_sample(-360,-357,30U);CHECK(navigation.reverse_completed);expect_zero();
    PASS("AUTO-2E 180 ms settling consumes only global time, not the unchanged reverse or no-progress budgets");
    a2_reverse();a2e_sample(1,0,10U);a1_terminal(NAV_FINISH_ODOMETRY);
    CHECK(nav_diag().odometry_error==NAV_ODOM_DIRECTION);
    PASS("AUTO-2E does not relax strict wrong-direction detection after the verified reverse reference");

    setup();clock_tick=UINT32_MAX-131U;robot_state=ROBOT_STATE_IDLE;Odometry_Init(clock_tick);b73_fresh();
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick));run_fsm(1);++clock_tick;Odometry_Update(0,0,clock_tick);b73_fresh();b73_step();
    run_motor(1);a2_data(0.270f,0.487f);b73_step();a2_stabilize();
    a2e_sample(0,0,21U);CHECK(clock_tick==0U && nav_diag().reverse_reference_available);
    CHECK(nav_diag().reverse_reference_tick==0U && queued_target.left_speed_cps==-1600);
    PASS("AUTO-2E settling across tick wrap accepts a legitimate zero reverse-reference timestamp");
    setup();clock_tick=UINT32_MAX-110U;robot_state=ROBOT_STATE_IDLE;Odometry_Init(clock_tick);b73_fresh();
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick));run_fsm(1);++clock_tick;Odometry_Update(0,0,clock_tick);b73_fresh();b73_step();
    run_motor(1);a2_data(0.270f,0.487f);b73_step();a2_stabilize();
    CHECK(clock_tick==0U && nav_diag().reverse_entered && nav_diag().reverse_entry_tick==0U);
    a2e_sample(0,0,20U);CHECK(navigation_odometry.ready && queued_target.left_speed_cps==-1600);
    PASS("AUTO-2E a legitimate zero reverse-entry timestamp remains valid after the new waiting phase");

    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a2e_real_residual;
    a2e_residual_cycles=0U;a2_with_obstacle=true;a2_positive_pwm=a2_negative_pwm=a2_turn_pwm=false;
    run_motor(150);motor_cycle_hook=NULL;
    CHECK(a2e_residual_cycles>=4U && a2_positive_pwm && a2_negative_pwm && navigation.reverse_completed);
    CHECK(nav_diag().settle_state==2U && nav_diag().odometry_error==NAV_ODOM_OK);expect_zero();
    PASS("AUTO-2E continuous real MotorControl loop tolerates residual +3/+2 in wait and then completes bounded reverse");

    a2_reverse();char base[256],detail[256];uint32_t id=nav_diag().session_id;
    CHECK(Navigation_FormatDiagnostics(base,sizeof base,91U)>0);
    CHECK(Navigation_FormatStabilization(detail,sizeof detail,91U)>0 && strlen(detail)<=255U);
    CHECK(strstr(detail,"NAVW,1,91,")==detail);
    printf("NAV_SETTLE_BASE %sNAV_SETTLE_DETAIL %s",base,detail);
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);
    CHECK(nav_diag().session_id==id && nav_diag().settle_state==2U && nav_diag().reverse_reference_available);
    /* La parte complementaria pertenece a la copia inicial, aun tras STOP. */
    char after[256];CHECK(Navigation_FormatStabilization(after,sizeof after,91U)>0);
    CHECK(Navigation_FormatStabilization(after,sizeof after,92U)<0);
    DrvUart_t uart;UART_HandleTypeDef peripheral={0};DrvUart_Init(&uart,&peripheral);
    CHECK(Navigation_FormatStabilization(after,sizeof after,91U)>0);
    b65_uart_command(&uart,"NAV Q=91 S=1\n",after);
    b65_uart_command(&uart,"NAV Q=91 S=4\n","ERR_UNKNOWN\n");
    b65_uart_command(&uart,"NAV Q=0 S=1\n","ERR_UNKNOWN\n");
    clock_tick=diagnostic_reply_tick+1000U;
    b65_uart_command(&uart,"NAV Q=91 S=1\n","ERR_UNKNOWN\n");
    PASS("AUTO-2E bounded complementary UART response shares the original snapshot and Q, rejects malformed/expired parts, and retains settle reference after STOP");
    navigation_diagnostics.reverse_reference_available=true;navigation_diagnostics.reverse_entered=true;
    navigation_diagnostics.reverse_reference_left=INT64_MIN;navigation_diagnostics.reverse_reference_right=INT64_MAX;
    navigation_diagnostics.reverse_reference_tick=navigation_diagnostics.reverse_entry_tick=UINT32_MAX;
    CHECK(Navigation_FormatDiagnostics(base,sizeof base,UINT32_MAX)>0);
    CHECK(Navigation_FormatStabilization(detail,sizeof detail,UINT32_MAX)>0 && strlen(detail)<=255U);
    CHECK(strstr(detail,"-9223372036854775808,9223372036854775807"));
    PASS("AUTO-2E full int64 stabilization reference remains bounded without changing UART driver buffers");
#endif
}
