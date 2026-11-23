/* AUTO-4 production modules; all counts, failures and clocks are synthetic. */
#if NAV_AUTO2_ENABLED
static void a4_advance(void) {a3_begin(NAV_PROFILE_CONTINUOUS);CHECK(navigation.state==NAV_AVANZAR);}
static void a4_wait_reverse(void) {
    a4_advance();a2_data(0.270f,0.500f);b73_step();CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);expect_zero();
}
static void a4_reverse(void) {
    a4_wait_reverse();a3_stabilize();a1_sample(0,0,20U);a2_data(0.270f,0.500f);b73_step();run_motor(1);
    CHECK(navigation.reverse_applied && navigation.state==NAV_RETROCEDER_ACOTADO);
}
static void a4_wait_turn(void) {
    a4_reverse();a1_sample(-360,-357,20U);b73_step();CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && navigation.settle_target==NAV_GIRAR_ACOTADO);
}
static void a4_turn(void) {a4_wait_turn();a3_stabilize();a1_sample(0,0,20U);b73_step();CHECK(navigation.state==NAV_GIRAR_ACOTADO);}
static void a4_wait_resume(void) {
    a4_turn();a1_sample(NAV_TURN_LEFT?-240:240,NAV_TURN_LEFT?238:-238,20U);b73_step();
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && navigation.settle_target==NAV_AVANZAR);expect_zero();
}
static void a4_phase(unsigned phase) {
    if(phase==0U)a4_advance();else if(phase==1U)a4_wait_reverse();else if(phase==2U)a4_reverse();
    else if(phase==3U)a4_wait_turn();else if(phase==4U)a4_turn();else a4_wait_resume();
}
static unsigned a4_obstacles,a4_stop_phase,a4_failure,a4_segments_seen;
static bool a4_injected,a4_had_pwm,a4_cadenced;
static TickType_t a4_front_tick,a4_rear_tick;
static float a4_front_value;
static void a4_schedule(void) {
    if(delay_calls==1U){
        CHECK(TaskControlRobot_RequestAutoProfileAt(clock_tick,NAV_PROFILE_CONTINUOUS));
        TaskControlRobotEventMessage_t m;CHECK(xQueueReceive(event_queue,&m,0U)==pdTRUE);TaskControlRobot_ProcessEventAt(m.event,&m);
    }
    bool obstacle=navigation.state==NAV_AVANZAR && navigation_diagnostics.continuous.attempts<a4_obstacles && navigation_odometry.progress_mm>60;
    a2_data(obstacle?0.270f:0.500f,0.500f);
    if(a4_cadenced) {
        TickType_t f=((clock_tick-20U)/140U)*140U;
        TickType_t r=((clock_tick-90U)/140U)*140U+70U;
        if(f!=a4_front_tick) {
            a4_front_tick=f;
            a4_front_value=obstacle?0.270f:
                (navigation.state==NAV_RETROCEDER_ACOTADO || (navigation.state==NAV_DETENER_Y_ESPERAR && navigation.settle_target==NAV_RETROCEDER_ACOTADO)) ?
                    (navigation_odometry.progress_mm>6.0 ? 0.290f : 0.270f) : 0.500f;
        }
        a4_rear_tick=r;input.distancia_m=a4_front_value;
        input.timestamp_ultrasonido_frontal=a4_front_tick;input.timestamp_ultrasonido_trasero=a4_rear_tick;
    }
    if(delay_calls%2==1U)b73_step();
    if(pwm.compare[0] || pwm.compare[1])a4_had_pwm=true;
    if(navigation_diagnostics.continuous.segments>a4_segments_seen)a4_segments_seen=navigation_diagnostics.continuous.segments;
    left_encoder_delta=right_encoder_delta=0;
    if(task_context.left_target_cps==1600&&task_context.right_target_cps==1600)left_encoder_delta=right_encoder_delta=12;
    else if(task_context.left_target_cps==-1600&&task_context.right_target_cps==-1600)left_encoder_delta=right_encoder_delta=-7;
    else if(task_context.left_target_cps==NAV_TURN_LEFT_CPS&&task_context.right_target_cps==NAV_TURN_RIGHT_CPS){left_encoder_delta=NAV_TURN_LEFT?-8:8;right_encoder_delta=-left_encoder_delta;}
    if(a4_stop_phase==((unsigned)navigation.state+1U) && !a4_injected){
        a4_injected=true;
        if(a4_failure==0U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
        else if(a4_failure==1U)TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);
        else if(a4_failure==2U)input.ultrasonido_valido=0;
        else if(a4_failure==3U)input.distancia_m=0.199f;
    }
}
static unsigned a4_odom_error;
static void a4_bad_odometry(void) {
    if(a4_odom_error==0U)s_odom.state=3U;
    if(a4_odom_error==1U)s_odom.tick=clock_tick-51U;
    if(a4_odom_error==2U)s_odom.x_m=NAN;
}
static void a4_loop(unsigned obstacles,unsigned phase,unsigned failure) {
    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a4_schedule;
    a4_obstacles=obstacles;a4_stop_phase=phase;a4_failure=failure;a4_injected=a4_had_pwm=false;a4_segments_seen=0;
    a4_front_tick=a4_rear_tick=0;a4_front_value=0.500f;
    run_motor(1400);motor_cycle_hook=NULL;expect_zero();
}
#endif
static void test_navigation_auto4(void) {
#if NAV_AUTO2_ENABLED
    CHECK(NAV_SESSION_MAX_MS==3000U && NAV_AUTO3_SESSION_MAX_MS==4200U && NAV_AUTO4_SESSION_MAX_MS==12000U);
    CHECK(NAV_AUTO2_ADVANCE_MAX_MS==2000U && NAV_AUTO4_ADVANCE_MAX_MS==3000U);
    CHECK(NAV_AUTO4_MAX_SEGMENTS==4U && NAV_AUTO4_MAX_EVASIONS==2U && NAV_AUTO4_MAX_PATH_MM==500U);
    CHECK(NAV_FRONT_STOP_MM==275U&&NAV_REAR_REVERSE_MIN_MM==250U&&TASK_SAFETY_OBSTACLE_DISTANCE_M==0.20f);
    /* Physical trial: valid forward progress was stopped before 100 mm at
     * the old 2-second deadline. Synthetic counts below use real geometry. */
    a4_advance();TickType_t slow_start=navigation.phase_started_tick;
    while(navigation.state==NAV_AVANZAR && (TickType_t)(clock_tick-slow_start)<NAV_AUTO4_ADVANCE_MAX_MS) {
        a1_sample(16,16,20U);b73_step();
        if((TickType_t)(clock_tick-slow_start)==2000U) {
            CHECK(navigation.active && navigation.state==NAV_AVANZAR);
            CHECK(navigation_odometry.progress_mm<100 && navigation_diagnostics.continuous.completed_segments==0U);
            CHECK(Navigation_IsCommandCurrent(navigation.session,clock_tick));
        }
    }
    CHECK(navigation.active && navigation.state==NAV_DETENER_Y_ESPERAR);
    CHECK((TickType_t)(clock_tick-slow_start)>2000U && (TickType_t)(clock_tick-slow_start)<NAV_AUTO4_ADVANCE_MAX_MS);
    CHECK(navigation_diagnostics.continuous.completed_segments==1U && navigation_odometry.progress_mm>=100);
    expect_zero();
    PASS("AUTO-4 slow real-geometry segment reaches 100 mm after 2000 ms and stops without a premature timeout");
    for(unsigned segment=1U;segment<NAV_AUTO4_MAX_SEGMENTS;++segment) {
        a3_stabilize();a1_sample(0,0,20U);b73_step();
        CHECK(navigation.state==NAV_AVANZAR && navigation_diagnostics.continuous.segments==segment+1U);
        TickType_t phase_start=navigation.phase_started_tick;
        while(navigation.state==NAV_AVANZAR && (TickType_t)(clock_tick-phase_start)<NAV_AUTO4_ADVANCE_MAX_MS) {
            a1_sample(16,16,20U);b73_step();
        }
        CHECK(navigation_diagnostics.continuous.completed_segments==segment+1U);
        CHECK((TickType_t)(clock_tick-phase_start)>2000U);
        expect_zero();
    }
    CHECK(navigation.finish_reason==NAV_FINISH_LIMIT_REACHED && !navigation.active);
    CHECK((TickType_t)(clock_tick-navigation.started_tick)>8000U &&
          (TickType_t)(clock_tick-navigation.started_tick)<NAV_AUTO4_SESSION_MAX_MS);
    CHECK(navigation_diagnostics.continuous.segments==4U);
    a1_terminal(NAV_FINISH_LIMIT_REACHED);
    PASS("AUTO-4 four slow segments and their stabilizations fit the unchanged 12-second budget without a fifth segment");
    for(unsigned backend=0;backend<2U;++backend) {
        a4_advance();
        for(unsigned elapsed=20U;elapsed<=2980U;elapsed+=20U){a1_sample(8,8,20U);b73_step();}
        a1_sample(8,8,19U);b73_step();
        CHECK(navigation.active && navigation.state==NAV_AVANZAR && navigation_odometry.progress_mm<100);
        CHECK((TickType_t)(clock_tick-navigation.phase_started_tick)==2999U);
        CHECK(Navigation_IsCommandCurrent(navigation.session,clock_tick));
        a1_sample(0,0,backend?21U:1U);
        if(backend)CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));
        b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
        CHECK(navigation_diagnostics.continuous.completed_segments==0U);
        CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));
        PASS("AUTO-4 safe timeout at exactly 3000 ms or the first later cycle, including rejection at motor authorization");
    }
    for(unsigned profile=0;profile<2U;++profile) {
        if(profile)a3_advance();else a1_advance();
        while((TickType_t)(clock_tick-navigation.started_tick)<1980U){a1_sample(8,8,20U);b73_step();}
        clock_tick=navigation.started_tick+2000U;s_odom.tick=clock_tick;b73_fresh();
        CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
        PASS("AUTO-2F and integrated AUTO-3 retain their original 2000-ms forward deadline");
    }
    a4_advance();clock_tick=navigation.started_tick+NAV_AUTO4_SESSION_MAX_MS;
    navigation.phase_started_tick=clock_tick-2000U;s_odom.tick=clock_tick;b73_fresh();
    CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-4 global 12000-ms deadline preempts a later segment that still has local time remaining");
    DrvUart_t uart;UART_HandleTypeDef peripheral={0};setup();robot_state=ROBOT_STATE_IDLE;b73_fresh();DrvUart_Init(&uart,&peripheral);
    b65_uart_command(&uart,"AUTO_CONTINUOUS\n","AUTO_QUEUED\n");CHECK(robot_state==ROBOT_STATE_IDLE);run_fsm(1);
    CHECK(navigation.profile==5U);b65_uart_command(&uart,"AUTO_CONTINUOUS\n","AUTO_REJECTED\n");
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);CHECK(robot_state==ROBOT_STATE_IDLE);
    ++clock_tick;b65_uart_command(&uart,"AUTO\n","AUTO_QUEUED\n");run_fsm(1);CHECK(navigation.profile==2U);
    PASS("AUTO-4 explicit UART request uses the existing FSM and never replaces ordinary AUTO or reselects itself");
    for(unsigned n=0;n<3U;++n){
        a4_loop(n,0U,0U);NavigationDiagnostics_t d=nav_diag();
        CHECK(a4_had_pwm && d.profile==5U && d.reason==NAV_FINISH_LIMIT_REACHED && d.continuous.segments==4U);
        CHECK(d.continuous.attempts==n && d.continuous.evasions==n && d.continuous.path_um<=501000U);
        CHECK(d.continuous.completed_segments==4U-n && d.continuous.forward_um>200000U);
        CHECK(robot_state==ROBOT_STATE_AUTONOMOUS && !TaskSafety_IsStopActive());
        CHECK(d.continuous.critical_faults==0U && !navigation.active);
        char wire[256];CHECK(Navigation_FormatDiagnostics(wire,sizeof wire,71)>0);printf("AUTO4_BASE_WIRE %s",wire);
        CHECK(Navigation_FormatContinuous(wire,sizeof wire,71)>0);printf("AUTO4_CONTINUOUS_WIRE %s",wire);
        uint32_t id=d.session_id;CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);d=nav_diag();
        CHECK(robot_state==ROBOT_STATE_IDLE && d.session_id==id && d.reason==NAV_FINISH_LIMIT_REACHED && d.continuous.evasions==n);
        PASS("AUTO-4 real uninterrupted motor loop supports multiple segments and zero/one/two evasions, then retains a finite terminal result after STOP");
    }
    a4_cadenced=true;a4_loop(2U,0U,0U);a4_cadenced=false;
    CHECK(navigation_diagnostics.continuous.evasions==2U && navigation_diagnostics.reason==NAV_FINISH_LIMIT_REACHED);
    PASS("AUTO-4 full sequence also works with alternating 140-ms ultrasound timestamps, sample-and-hold values and 20-ms capture latency");
    a4_loop(3U,0U,0U);CHECK(navigation_diagnostics.reason==NAV_FINISH_BLOCKED_NO_SAFE_PATH);
    CHECK(navigation_diagnostics.continuous.attempts==2U && navigation_diagnostics.continuous.evasions==2U && navigation_diagnostics.continuous.block==6U);
    PASS("AUTO-4 complete execution refuses a third recovery without latching a fictitious hardware failure");
    a4_wait_resume();navigation_diagnostics.continuous.segments=NAV_AUTO4_MAX_SEGMENTS;
    for(unsigned i=0;i<4U && navigation.active;++i){a1_sample(0,0,20U);b73_step();}
    CHECK(!navigation.active && navigation.finish_reason==NAV_FINISH_LIMIT_REACHED);expect_zero();
    PASS("AUTO-4 completing an avoidance in the last allowed segment cannot open a fifth forward segment");
    a4_advance();run_motor(1);TaskMotorControlTarget_t old=queued_target;
    a2_data(0.275f,0.500f);run_motor(1);CHECK(navigation.active && navigation.motion_hold);expect_zero();
    b73_fresh();b73_step();CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && navigation.settle_target==NAV_RETROCEDER_ACOTADO);
    CHECK(!TaskSafety_IsStopActive() && TaskSafety_GetFaults()==0U);
    old.generation=command_generation;CHECK(xQueueOverwrite(target_speed_queue,&old)==pdPASS);run_motor(1);expect_zero();
    PASS("AUTO-4 backend stops a queued positive target at 275 mm before Navigation runs; a clear reading never revives that target");
    a4_reverse();a2_data(0.270f,0.250f);run_motor(1);CHECK(navigation.motion_hold);expect_zero();
    b73_fresh();b73_step();a1_terminal(NAV_FINISH_BLOCKED_NO_SAFE_PATH);CHECK(!TaskSafety_IsStopActive());
    PASS("AUTO-4 rear 250 mm inhibits reverse at the motor backend and terminal blocked state requires no invented Safety fault");
    a4_turn();a2_data(0.275f,0.500f);run_motor(1);expect_zero();CHECK(navigation.motion_hold);
    b73_fresh();b73_step();a1_terminal(NAV_FINISH_BLOCKED_NO_SAFE_PATH);
    PASS("AUTO-4 turn cannot continue on an unsafe clearance observed between navigation and consumption even if it clears again");
    for(unsigned phase=0;phase<3U;++phase) {
        if(phase==0U)a4_wait_reverse();else if(phase==1U)a4_wait_turn();else a4_wait_resume();
        TickType_t barrier=navigation_settle.started_tick;
        for(unsigned i=0;i<4U;++i){a1_sample(0,0,20U);input.timestamp_ultrasonido_frontal=barrier;input.timestamp_ultrasonido_trasero=barrier;b73_step();expect_zero();}
        CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && navigation_diagnostics.continuous.refresh_pending);
        a1_sample(0,0,20U);b73_step();CHECK(navigation.state==navigation.settle_target && !navigation_odometry.ready);
        CHECK(!navigation_diagnostics.continuous.refresh_pending);
        PASS("AUTO-4 cannot leave stabilization on old or same-tick clearance echoes; both new captures are required before reversing, turning or resuming");
    }
    for(unsigned phase=0;phase<6U;++phase){
        for(unsigned fault=0;fault<9U;++fault){
            a4_phase(phase);b73_fresh();
            if(fault==0U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
            if(fault==1U)input.hay_caida=1U;
            if(fault==2U)input.distancia_m=0.200f;
            if(fault==3U)input.distancia_trasera_m=0.199f;
            if(fault==4U)input.ultrasonido_valido=0U;
            if(fault==5U)input.ultrasonido_trasero_valido=0U;
            if(fault==6U)input.timestamp_ultrasonido_frontal=clock_tick-201U;
            if(fault==7U)queue_available=0;
            if(fault==8U)TaskSafety_ReportFault(SAFETY_FAULT_EMERGENCY);
            run_motor(1);expect_zero();CHECK(!navigation.active);
            CHECK(navigation.finish_reason==(fault==0U?NAV_FINISH_OPERATOR_STOP:NAV_FINISH_SAFETY));
            if(fault!=0U)CHECK(TaskSafety_IsStopActive() && TaskSafety_GetFaults()!=0U);
            uint32_t mask=TaskSafety_GetFaults();b73_fresh();b73_step();CHECK(!navigation.active && TaskSafety_GetFaults()==mask);
            PASS("AUTO-4 every phase preserves immediate STOP and latched cliff/critical proximity/no-echo/stale/missing/emergency protections");
        }
    }
    a4_advance();a2_data(0.250f,0.240f);b73_step();a1_terminal(NAV_FINISH_BLOCKED_NO_SAFE_PATH);CHECK(!TaskSafety_IsStopActive());
    a4_advance();a2_data(0.199f,0.199f);run_motor(1);CHECK(TaskSafety_GetFaults()&SAFETY_FAULT_OBSTACLE);expect_zero();
    PASS("AUTO-4 blocked both directions above critical is terminal without a fault; either side at or below 200 mm still latches Safety");
    a4_wait_resume();CHECK(navigation_diagnostics.continuous.evasions==1U);a2_data(0.275f,0.500f);b73_step();a1_terminal(NAV_FINISH_BLOCKED_NO_SAFE_PATH);
    PASS("AUTO-4 reevaluates before resuming and does not advance into a persistent obstacle after turning");
    a4_wait_resume();a3_stabilize();a1_sample(0,0,20U);b73_step();CHECK(navigation.state==NAV_AVANZAR);
    a2_data(0.270f,0.500f);b73_step();a1_terminal(NAV_FINISH_BLOCKED_NO_SAFE_PATH);CHECK(navigation_diagnostics.continuous.block==4U);
    PASS("AUTO-4 repeated maneuvers without at least 25 mm useful net improvement stop instead of cycling");
    a4_advance();navigation_diagnostics.continuous.attempts=2U;a2_data(0.270f,0.500f);b73_step();a1_terminal(NAV_FINISH_BLOCKED_NO_SAFE_PATH);
    CHECK(navigation_diagnostics.continuous.block==6U);
    a4_advance();navigation_continuous.recovery_count=1U;navigation_continuous.recovery[0].x=s_odom.x_m;navigation_continuous.recovery[0].y=s_odom.y_m;
    navigation_continuous.recovery[0].theta=s_odom.theta_rad+6.28318530718f;a2_data(0.270f,0.500f);b73_step();a1_terminal(NAV_FINISH_BLOCKED_NO_SAFE_PATH);
    CHECK(navigation_diagnostics.continuous.block==5U);
    PASS("AUTO-4 bounded recovery history rejects repeated poses modulo angle and more than two recovery attempts");
    a4_advance();navigation_continuous.path_mm=499.5;a1_sample(20,20,20U);
    CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));b73_step();a1_terminal(NAV_FINISH_LIMIT_REACHED);
    PASS("AUTO-4 accumulated absolute wheel travel is independently bounded at 500 mm in the motor authorization path");
    a4_advance();clock_tick=navigation.phase_started_tick+NAV_AUTO4_ADVANCE_MAX_MS;s_odom.tick=clock_tick;b73_fresh();b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    a4_turn();clock_tick=navigation.started_tick+12000U;s_odom.tick=clock_tick;b73_fresh();CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-4 retains finite per-segment and turn/global deadlines; no phase transition renews the 12-second session");
    a4_advance();a2_data(0.199f,0.500f);run_motor(1);run_safety(1);run_fsm(1);
    CHECK(robot_state==ROBOT_STATE_SAFETY_STOP && TaskSafety_IsStopActive());
    clock_tick+=100U;b73_fresh();DrvUart_Init(&uart,&peripheral);
    b65_uart_command(&uart,"RESET\n","RESET_QUEUED\n");run_fsm(1);
    CHECK(robot_state==ROBOT_STATE_IDLE && !TaskSafety_IsStopActive() && !navigation.active);
    CHECK(navigation_diagnostics.continuous.critical_faults & SAFETY_FAULT_OBSTACLE);
    PASS("AUTO-4 critical proximity needs original explicit safe rearm; retained critical history and old navigation never reactivate");
    a4_wait_reverse();TickType_t shift=(TickType_t)(UINT32_MAX-30U-clock_tick);
    navigation.started_tick+=shift;navigation.phase_started_tick+=shift;navigation.reverse_started_tick+=shift;
    navigation_odometry.reference_tick+=shift;navigation_odometry.last_tick+=shift;navigation_odometry.progress_tick+=shift;
    navigation_continuous.tick+=shift;navigation_settle.started_tick+=shift;navigation_settle.last_tick+=shift;navigation_settle.window_tick+=shift;
    mode_entry_tick+=shift;last_stop_tick+=shift;diagnostic_phase_tick+=shift;s_odom.tick+=shift;clock_tick+=shift;
    a3_stabilize();CHECK(navigation.state==NAV_RETROCEDER_ACOTADO);
    a1_sample(0,0,20U);a2_data(0.270f,0.500f);b73_step();CHECK(navigation_odometry.ready);
    a1_sample(-20,-20,20U);a2_data(0.270f,0.500f);b73_step();CHECK(navigation.active && navigation_diagnostics.continuous.path_um>1000U);
    PASS("AUTO-4 path accumulation and strict post-stop sensor freshness work across tick wrap without treating tick zero as missing");
    a4_advance();a1_sample(0,0,200U); /* Invalid producer gap, not valid zero velocity. */
    b73_step();a1_terminal(NAV_FINISH_ODOMETRY);
    a4_advance();clock_tick=navigation_odometry.progress_tick+200U;s_odom.tick=clock_tick;b73_fresh();b73_step();a1_terminal(NAV_FINISH_NO_PROGRESS);
    PASS("AUTO-4 odometry interruption or no measured progress cannot produce indefinite renewals");
    for(a4_odom_error=0U;a4_odom_error<3U;++a4_odom_error) {
        a4_advance();motor_cycle_hook=a4_bad_odometry;run_motor(1);motor_cycle_hook=NULL;
        expect_zero();CHECK(!navigation.active && navigation.finish_reason==NAV_FINISH_ODOMETRY);
        CHECK(navigation_diagnostics.odometry_error==(a4_odom_error==0U?NAV_ODOM_UNAVAILABLE:a4_odom_error==1U?NAV_ODOM_STALE:NAV_ODOM_NONFINITE));
        PASS("AUTO-4 invalid/stale/nonfinite odometry racing real target consumption stops and retains the precise diagnostic error");
    }
    a4_advance();input.distancia_m=INFINITY;run_motor(1);expect_zero();
    CHECK(TaskSafety_GetFaults()&SAFETY_FAULT_SENSOR);
    PASS("AUTO-4 a nonfinite value marked valid cannot masquerade as clear space at motor consumption");
    for(unsigned phase=0;phase<3U;++phase)for(unsigned failure=0;failure<4U;++failure){
        a4_loop(1U,phase==0U ? NAV_AVANZAR+1U : phase==1U ? NAV_RETROCEDER_ACOTADO+1U : NAV_GIRAR_ACOTADO+1U,failure);
        CHECK(a4_injected && !navigation.active);expect_zero();
        CHECK(navigation.finish_reason==(failure?NAV_FINISH_SAFETY:NAV_FINISH_OPERATOR_STOP));
        PASS("AUTO-4 real motor-loop concurrent STOP/cliff/no-echo/subcritical obstacle cancels motion without a stale command restart");
    }
#endif
}
