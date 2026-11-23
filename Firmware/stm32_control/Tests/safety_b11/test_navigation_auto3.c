/* AUTO-3: real navigation/FSM/motor backend, synthetic encoder stimuli only. */
#if NAV_AUTO2_ENABLED
static void a3_begin(uint8_t profile) {
    setup();robot_state=ROBOT_STATE_IDLE;Odometry_Init(clock_tick);b73_fresh();
    CHECK(TaskControlRobot_RequestAutoProfileAt(clock_tick,profile));run_fsm(1);
    CHECK(navigation.profile==profile && navigation.state==(profile==NAV_PROFILE_TURN ? NAV_DETENER_Y_ESPERAR : NAV_ESPERAR_DATOS));expect_zero();
    a1_sample(0,0,1U);b73_step();
}
static void a3_stabilize(void) {
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);
    for(unsigned i=0;i<4U && navigation.state==NAV_DETENER_Y_ESPERAR;++i){a1_sample(0,0,20U);b73_step();expect_zero();}
    CHECK(navigation.state==navigation.settle_target && !navigation_odometry.ready);
}
static void a3_turn(void) {
    a3_begin(NAV_PROFILE_TURN);CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);
    a3_stabilize();CHECK(navigation.state==NAV_GIRAR_ACOTADO);
    a1_sample(0,0,20U);b73_step();
    CHECK(target_pending && queued_target.left_speed_cps==NAV_TURN_LEFT_CPS && queued_target.right_speed_cps==NAV_TURN_RIGHT_CPS);
}
static void a3_advance(void) {
    a3_begin(NAV_PROFILE_EVASION);CHECK(navigation.state==NAV_AVANZAR);run_motor(1);
}
static void a3_reverse(void) {
    a3_advance();a2_data(0.270f,0.500f);b73_step();a3_stabilize();
    CHECK(navigation.state==NAV_RETROCEDER_ACOTADO);
    a1_sample(0,0,20U);a2_data(0.270f,0.500f);b73_step();run_motor(1);
    CHECK(navigation.reverse_applied);
}
static void a3_second_wait(void) {
    a3_reverse();a1_sample(-360,-357,20U);a2_data(0.500f,0.500f);b73_step();
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && navigation.settle_target==NAV_GIRAR_ACOTADO);
    CHECK(navigation.reverse_completed && navigation_diagnostics.reverse_wait.settle_state==2U);expect_zero();
}
static void a3_phase(unsigned p) {
    if(p==0U)a3_advance();else if(p==1U){a3_advance();a2_data(0.270f,0.500f);b73_step();}
    else if(p==2U)a3_reverse();else if(p==3U)a3_second_wait();else a3_turn();
}
/* Continuous real MotorControl task: no restart between phases/references. */
static uint8_t a3_loop_profile;
static unsigned a3_seen, a3_stop_phase, a3_fault;
static bool a3_injected;
static void a3_schedule(void) {
    if(delay_calls==1U){
        CHECK(TaskControlRobot_RequestAutoProfileAt(clock_tick,a3_loop_profile));
        TaskControlRobotEventMessage_t m;CHECK(xQueueReceive(event_queue,&m,0U)==pdTRUE);
        TaskControlRobot_ProcessEventAt(m.event,&m);
    }
    if(delay_calls%2==1U){
        a2_data(navigation.state==NAV_AVANZAR && navigation_odometry.progress_mm>3.0 ? 0.270f : 0.500f,0.500f);
        b73_step();
    }
    a3_seen |= 1U<<navigation.state;
    left_encoder_delta=right_encoder_delta=0;
    if(task_context.left_target_cps==1600 && task_context.right_target_cps==1600)left_encoder_delta=right_encoder_delta=12;
    else if(task_context.left_target_cps==-1600 && task_context.right_target_cps==-1600)left_encoder_delta=right_encoder_delta=-7;
    else if(task_context.left_target_cps==NAV_TURN_LEFT_CPS && task_context.right_target_cps==NAV_TURN_RIGHT_CPS){
        left_encoder_delta=NAV_TURN_LEFT?-8:8;right_encoder_delta=-left_encoder_delta;
    }
    /* Residual negative counts after reverse, including the wheel that must invert. */
    if(navigation.state==NAV_DETENER_Y_ESPERAR && navigation.settle_target==NAV_GIRAR_ACOTADO &&
        (TickType_t)(clock_tick-navigation.phase_started_tick)<30U)left_encoder_delta=right_encoder_delta=-1;
    if(a3_stop_phase && navigation.state==a3_stop_phase && !a3_injected){
        a3_injected=true;
        if(a3_fault)TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);else CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
    }
}
static void a3_loop(uint8_t profile,unsigned stop_phase,unsigned fault) {
    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a3_schedule;
    a3_loop_profile=profile;a3_seen=0;a3_stop_phase=stop_phase;a3_fault=fault;a3_injected=false;
    run_motor(450);motor_cycle_hook=NULL;expect_zero();
}
#endif
static void test_navigation_auto3(void) {
#if NAV_AUTO2_ENABLED
    CHECK(NAV_EXPERIMENTAL_TURN_ENABLED==0 || NAV_EXPERIMENTAL_TURN_ENABLED==1);
    CHECK(NAV_SESSION_MAX_MS==3000U && NAV_AUTO3_SESSION_MAX_MS==4200U && NAV_AUTO3_ISOLATED_MAX_MS==1400U);
    CHECK(NAV_AUTO3_TURN_MAX_MS==800U && NAV_AUTO2_TURN_TARGET_DEG==10U);
    CHECK(NAV_FRONT_STOP_MM==275U && NAV_REAR_REVERSE_MIN_MM==250U && TASK_SAFETY_OBSTACLE_DISTANCE_M==0.20f);
    for(unsigned profile=3U;profile<=4U;++profile){
        for(unsigned mode=1U;mode<=3U;++mode){setup();robot_state=(RobotControlState_t)mode;
            CHECK(!TaskControlRobot_RequestAutoProfileAt(clock_tick,(uint8_t)profile));expect_zero();}
        setup();robot_state=ROBOT_STATE_IDLE;CHECK(!TaskControlRobot_RequestAutoProfileAt(clock_tick,6U));
        CHECK(TaskControlRobot_RequestAutoProfileAt(clock_tick,(uint8_t)profile));
        CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);CHECK(robot_state==ROBOT_STATE_IDLE);expect_zero();
        PASS("AUTO-3 explicit profile uses original FSM safety/epoch barriers; queued request cannot survive STOP");
    }
    for(unsigned profile=3U;profile<=4U;++profile) {
        DrvUart_t uart;UART_HandleTypeDef peripheral={0};setup();robot_state=ROBOT_STATE_IDLE;b73_fresh();DrvUart_Init(&uart,&peripheral);
        b65_uart_command(&uart,profile==3U?"AUTO_TURN\n":"AUTO_EVADE\n","AUTO_QUEUED\n");
        CHECK(robot_state==ROBOT_STATE_IDLE && !target_pending);run_fsm(1);CHECK(navigation.profile==profile);
        b65_uart_command(&uart,"AUTO_TURN\n","AUTO_REJECTED\n");
        b65_uart_command(&uart,"STOP\n","STOP_QUEUED\n");run_fsm(1);CHECK(robot_state==ROBOT_STATE_IDLE);
        ++clock_tick;b73_fresh();b65_uart_command(&uart,"AUTO\n","AUTO_QUEUED\n");run_fsm(1);
        CHECK(navigation.profile==2U && !navigation.turn_used);
        PASS("AUTO-3 real UART selects one immutable profile and STOP followed by ordinary AUTO restores AUTO-2F");
        setup();robot_state=ROBOT_STATE_IDLE;
        for(unsigned i=0;i<8U;++i)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
        CHECK(!TaskControlRobot_RequestAutoProfileAt(clock_tick,(uint8_t)profile));expect_zero();
        setup();robot_state=ROBOT_STATE_IDLE;CHECK(TaskControlRobot_RequestAutoProfileAt(clock_tick,(uint8_t)profile));
        clock_tick+=500U;run_fsm(1);CHECK(robot_state==ROBOT_STATE_IDLE);expect_zero();
        PASS("AUTO-3 full queue or expired queued profile cannot start a maneuver");
    }
    for(unsigned boundary=3U;boundary<=4U;++boundary){
        a3_turn();navigation_odometry.left_mm_per_count=navigation_odometry.right_mm_per_count=1.0;
        a1_sample(NAV_TURN_LEFT?-(int)boundary:(int)boundary,0,20U);b73_step();
        if(boundary==3U){CHECK(navigation.active);TaskMotorControl_Stop();}else a1_terminal(NAV_FINISH_ODOMETRY);
        PASS("AUTO-3 exact 3 mm turn discrepancy is allowed; larger disagreement is terminal");
    }
    a3_turn();TickType_t shift=(TickType_t)(UINT32_MAX-10U-clock_tick);
    navigation.started_tick+=shift;navigation.phase_started_tick+=shift;navigation.turn_started_tick+=shift;
    navigation_odometry.reference_tick+=shift;navigation_odometry.last_tick+=shift;navigation_odometry.progress_tick+=shift;
    diagnostic_phase_tick+=shift;mode_entry_tick+=shift;last_stop_tick+=shift;s_odom.tick+=shift;clock_tick+=shift;
    a1_sample(NAV_TURN_LEFT?-100:100,NAV_TURN_LEFT?99:-99,17U);b73_step();CHECK(navigation.active);
    a1_sample(NAV_TURN_LEFT?-140:140,NAV_TURN_LEFT?139:-139,23U);b73_step();a1_terminal(NAV_FINISH_TARGET);
    PASS("AUTO-3 angular progress and deadlines survive tick wrap and irregular sample intervals");
    a3_begin(NAV_PROFILE_TURN);uint32_t session=navigation.session;
    a1_sample(2,3,20U);b73_step();CHECK(!target_pending);expect_zero();
    uint16_t observed=navigation_diagnostics.turn_wait.settle_observations;
    for(unsigned i=0;i<6U;++i){b73_step();CHECK(!target_pending);}
    CHECK(navigation_diagnostics.turn_wait.settle_observations==observed);
    a1_sample(0,2,20U);b73_step();CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);
    a1_sample(0,0,20U);b73_step();a1_sample(0,0,20U);b73_step();
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);a1_sample(0,0,20U);b73_step();
    CHECK(navigation.state==NAV_GIRAR_ACOTADO && !navigation_odometry.ready && navigation.session!=session);expect_zero();
    a1_sample(0,0,20U);b73_step();CHECK(navigation_odometry.left==2 && navigation_odometry.right==5);
    CHECK(navigation_diagnostics.turn_wait.reverse_reference_available);
    PASS("AUTO-3 isolated turn tolerates residual counts, waits both wheels with four distinct timestamps, then takes a new reference");
    for(unsigned over=0;over<2U;++over){
        a3_turn();a1_sample(NAV_TURN_LEFT?-200:200,NAV_TURN_LEFT?198:-198,17U);b73_step();CHECK(navigation.active);
        a1_sample(NAV_TURN_LEFT?-(35+(int)over*40):35+(int)over*40,NAV_TURN_LEFT?34+(int)over*40:-(34+(int)over*40),23U);b73_step();
        a1_terminal(NAV_FINISH_TARGET);CHECK(navigation_diagnostics.turn_mdeg>=10000U && (navigation_diagnostics.turn_flags&8U));
        PASS("AUTO-3 measured angular target reached or exceeded stops without automatically resuming advance");
    }
    a3_turn();navigation_odometry.left_mm_per_count=navigation_odometry.right_mm_per_count=
        NAV_AUTO2_TURN_TARGET_DEG*(3.14159265358979323846/180.0)*ODOM_TRACK_M*500.0/100.0;
    a1_sample(NAV_TURN_LEFT?-100:100,NAV_TURN_LEFT?100:-100,20U);b73_step();a1_terminal(NAV_FINISH_TARGET);
    CHECK(navigation_diagnostics.turn_mdeg==10000U);
    PASS("AUTO-3 exact angular target is serialized without a false one-millidegree shortfall");
    a3_turn();clock_tick=navigation.phase_started_tick+799U;s_odom.tick=navigation_odometry.progress_tick=clock_tick;
    b73_fresh();b73_step();CHECK(navigation.active);++clock_tick;
    CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));b73_fresh();b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-3 turn 799/800 ms boundary is enforced by the motor permission as well as navigation");
    a3_turn();clock_tick=navigation_odometry.progress_tick+200U;s_odom.tick=clock_tick;b73_fresh();b73_step();a1_terminal(NAV_FINISH_NO_PROGRESS);
    PASS("AUTO-3 stationary wheels cannot keep renewing a turn");
    a3_turn();a1_sample(NAV_TURN_LEFT?-60:60,0,20U);b73_step();a1_terminal(NAV_FINISH_ODOMETRY);
    CHECK(navigation_diagnostics.odometry_error==NAV_ODOM_DISAGREEMENT);
    PASS("AUTO-3 3 mm turn-specific wheel discrepancy guard rejects a stalled wheel without relaxing AUTO-2F");
    for(unsigned phase=0;phase<5U;++phase){
        for(unsigned fault=0;fault<10U;++fault){
            a3_phase(phase);b73_fresh();
            if(fault==0U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
            if(fault==1U)TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);
            if(fault==2U)input.ultrasonido_valido=0;
            if(fault==3U)input.ultrasonido_trasero_valido=0;
            if(fault==4U)input.timestamp_ultrasonido_frontal=clock_tick-201U;
            if(fault==5U)input.hay_caida=1;
            if(fault==6U)s_odom.state=3U;
            if(fault==7U)s_odom.tick=clock_tick-51U;
            if(fault==8U)input.distancia_trasera_m=0.250f;
            if(fault==9U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET));
            b73_step();
            if(fault==8U && phase==0U){CHECK(navigation.active);TaskMotorControl_Stop();}else CHECK(!navigation.active);
            expect_zero();b73_fresh();b73_step();CHECK(!navigation.active);expect_zero();
            PASS("AUTO-3 STOP/Safety/invalid/old/IR/odometry/rear/RESET cancellation is terminal in every phase");
        }
    }
    a3_turn();a1_sample(NAV_TURN_LEFT?1:-1,0,20U);b73_step();a1_terminal(NAV_FINISH_ODOMETRY);
    CHECK(navigation_diagnostics.odometry_error==NAV_ODOM_DIRECTION);
    a3_turn();a1_sample(0,0,20U);s_odom.left_count=INT64_MIN;b73_step();a1_terminal(NAV_FINISH_ODOMETRY);
    PASS("AUTO-3 wrong wheel sign and extreme counts remain rejected");
    a3_second_wait();int64_t reverse_ref=navigation_diagnostics.reverse_reference_left;
    a1_sample(-2,-3,20U);b73_step();a3_stabilize();CHECK(navigation.state==NAV_GIRAR_ACOTADO);
    a1_sample(0,0,20U);b73_step();CHECK(navigation_odometry.left==-362 && navigation_odometry.right==-360);
    CHECK(navigation_diagnostics.reverse_reference_left==reverse_ref && navigation_diagnostics.settle_state==2U);
    CHECK(navigation_diagnostics.turn_wait.settle_state==2U);
    TaskMotorControlTarget_t stale=queued_target;CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);
    CHECK(robot_state==ROBOT_STATE_IDLE);CHECK(xQueueOverwrite(target_speed_queue,&stale)==pdPASS);run_motor(1);expect_zero();
    NavigationDiagnostics_t d;Navigation_GetDiagnostics(&d);CHECK(d.turn_wait.settle_state==2U && (d.flags & NAV_DIAG_REVERSE_COMPLETED)); /* flags checked below */
    PASS("AUTO-3 second stabilization and independent count reference survive STOP without accepting old turn targets");
    a3_begin(NAV_PROFILE_TURN);clock_tick=navigation.phase_started_tick+200U;s_odom.tick=clock_tick;b73_fresh();b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    CHECK(navigation_diagnostics.turn_wait.settle_state==3U);
    a3_turn();clock_tick=navigation.started_tick+1400U;s_odom.tick=clock_tick;b73_fresh();b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    a3_second_wait();clock_tick=navigation.started_tick+4200U;s_odom.tick=clock_tick;b73_fresh();b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-3 stabilization, isolated and integrated global deadlines stay finite and never restart");
    a3_second_wait();a2_data(0.275f,0.500f);b73_step();a1_terminal(NAV_FINISH_EVASION_NOT_AUTHORIZED);
    PASS("AUTO-3 front not cleared above 275 mm cannot authorize turn even after reverse completion");
    for(unsigned profile=3U;profile<=4U;++profile){
        a3_loop((uint8_t)profile,0,0);Navigation_GetDiagnostics(&d);
        CHECK(d.turn_flags==31U && d.turn_mdeg>=10000U && d.turn_wait.settle_state==2U);
        CHECK(d.reason==(profile==3U ? NAV_FINISH_TARGET : NAV_FINISH_EVASION_COMPLETED));
        if(profile==3U)CHECK(!(a3_seen&(1U<<NAV_AVANZAR)) && !(a3_seen&(1U<<NAV_RETROCEDER_ACOTADO)));
        else CHECK(d.settle_state==2U && d.reverse_reference_available && d.reverse_um>=20000);
        char wire[256];CHECK(Navigation_FormatDiagnostics(wire,sizeof wire,33)>0);printf("AUTO3_BASE_WIRE %s",wire);
        CHECK(Navigation_FormatTurn(wire,sizeof wire,33)>0 && strlen(wire)<=255U);printf("AUTO3_TURN_WIRE %s",wire);
        CHECK(Navigation_FormatStabilization(wire,sizeof wire,33)>0);
        PASS("AUTO-3 continuous real motor loop completes supervised profile, applied flags and measured angle, with zero terminal PWM");
    }
    for(unsigned fault=0;fault<2U;++fault){a3_loop(NAV_PROFILE_EVASION,NAV_GIRAR_ACOTADO,fault);CHECK(a3_injected);
        CHECK(navigation_diagnostics.reason==(fault?NAV_FINISH_SAFETY:NAV_FINISH_OPERATOR_STOP));expect_zero();
        PASS("AUTO-3 STOP or Safety racing the continuous turn backend wins and cannot restart");}
#endif
}
