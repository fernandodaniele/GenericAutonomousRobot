/* Real navigation, odometry, FSM, PI and driver in the existing host harness.
 * Synthetic counts/times are test inputs, never production measurements. */
#if NAV_AUTO1_ODOMETRY
static void a1_begin(void) {
    setup(); robot_state=ROBOT_STATE_IDLE; Odometry_Init(clock_tick); b73_fresh();
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick)); run_fsm(1);
    CHECK(robot_state==ROBOT_STATE_AUTONOMOUS && Navigation_GetState()==NAV_ESPERAR_DATOS);
    CHECK(!navigation_odometry.ready); expect_zero();
    ++clock_tick; Odometry_Update(0,0,clock_tick); b73_fresh();
}
static void a1_advance(void) {
    a1_begin(); b73_step();
    CHECK(navigation_odometry.ready && Navigation_GetState()==NAV_AVANZAR && target_pending);
    CHECK(queued_target.left_speed_cps==1600 && queued_target.right_speed_cps==1600);
}
static void a1_sample(int32_t l,int32_t r,TickType_t dt) {
    clock_tick+=dt; Odometry_Update(l,r,clock_tick); b73_fresh();
}
static void a1_terminal(NavigationFinishReason_t reason) {
    CHECK(Navigation_GetState()==NAV_DETENIDO && Navigation_GetFinishReason()==reason);
    CHECK(!navigation.emission_ready); expect_zero();
    uint32_t generation=command_generation;
    clock_tick+=20U; b73_fresh(); b73_step();
    CHECK(Navigation_GetState()==NAV_DETENIDO && Navigation_GetFinishReason()==reason);
    CHECK(command_generation==generation); expect_zero();
}
/* Deterministic scheduling of the real control task between motor cycles.
 * The real motor task initializes/updates odometry only once in this run. */
static bool a1_saw_pwm;
static bool a1_stationary;
static unsigned a1_schedule_nav_until;
static void a1_motor_schedule(void) {
    if(pwm.compare[0] && pwm.compare[1])a1_saw_pwm=true;
    if(delay_calls==1) {
        CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
        TaskControlRobotEventMessage_t message;
        CHECK(xQueueReceive(event_queue,&message,0U)==pdTRUE);
        TaskControlRobot_ProcessEventAt(message.event,&message);
    }
    if(delay_calls%2==1 && (!a1_schedule_nav_until || (unsigned)delay_calls<=a1_schedule_nav_until)) {
        b73_fresh(); b73_step();
    }
    left_encoder_delta=right_encoder_delta=
        !a1_stationary && Navigation_GetState()==NAV_AVANZAR ? 12 : 0;
}
static unsigned a1_backend_fault;
static void a1_backend_failure_schedule(void) {
    a1_motor_schedule();
    if(delay_calls==5) {
        /* Inyeccion entre decision y consumo reales, solo en el arnes host. */
        if(a1_backend_fault==0U)s_odom.state=3U;
        if(a1_backend_fault==1U)s_odom.tick=clock_tick-51U;
        if(a1_backend_fault==2U) {
            s_odom.left_count=1800;s_odom.right_count=1783;s_odom.tick=clock_tick;
        }
    }
}
static void test_navigation_auto1(void) {
    CHECK(NAV_AUTO1_ODOMETRY==1 && NAV_AUTO1_TARGET_MM==100U);
    CHECK(NAV_EXPERIMENTAL_TURN_ENABLED==0 && NAV_SESSION_MAX_MS==2000U);
    a1_begin(); CHECK(Navigation_GetFinishReason()==NAV_FINISH_NONE);
    CHECK(!target_pending); expect_zero();
    PASS("AUTO-1 safe IDLE entry waits for odometry newer than AUTO, without movement");

    const RobotControlState_t denied[]={ROBOT_STATE_MANUAL,ROBOT_STATE_SAFETY_STOP,ROBOT_STATE_AUTONOMOUS};
    for(unsigned i=0;i<3U;++i) {
        setup();robot_state=denied[i];CHECK(!TaskControlRobot_RequestAutoAt(clock_tick));
        CHECK(!navigation.active && !target_pending);expect_zero();
        PASS("AUTO-1 rejects entry from unauthorized FSM state");
    }
    setup();robot_state=ROBOT_STATE_IDLE;safety_stop_active=true;
    CHECK(!TaskControlRobot_RequestAutoAt(clock_tick));expect_zero();
    PASS("AUTO-1 cannot enter while Safety is blocking");
    for(unsigned failure=0;failure<3U;++failure) {
        a1_begin();
        if(failure==0U)s_odom.state=0U;
        if(failure==1U)s_odom.tick=clock_tick-51U;
        if(failure==2U)s_odom.calibration=ODOM_GEOMETRY_INCOMPLETE;
        b73_step();CHECK(!target_pending);a1_terminal(NAV_FINISH_ODOMETRY);
        PASS("AUTO-1 cannot publish its first target without existing fresh metric odometry");
    }

    a1_begin(); /* Counts can already be negative from previous manual travel. */
    a1_sample(-1000,-990,10U);OdometrySnapshot before;Odometry_GetSnapshot(&before);
    b73_step();CHECK(navigation_odometry.left==-1000 && navigation_odometry.right==-990);
    OdometrySnapshot after;Odometry_GetSnapshot(&after);
    CHECK(!memcmp(&before,&after,sizeof before) && navigation_odometry.progress_mm==0);
    CHECK(target_pending && navigation_odometry.reference_tick==before.tick);
    PASS("AUTO-1 captures both signed accumulated counts atomically without resetting global odometry");
    a1_sample(900,892,30U);b73_step();
    double expected=(900.0*navigation_odometry.left_mm_per_count+892.0*navigation_odometry.right_mm_per_count)*0.5;
    CHECK(fabs(navigation_odometry.progress_mm-expected)<1e-9 && expected>49 && expected<51);
    CHECK(Navigation_GetState()==NAV_AVANZAR);
    PASS("AUTO-1 uses current wheel-specific factors and accumulated count differences, including negative baselines");

    a1_advance();a1_sample(500,495,10U);a1_sample(500,495,17U);b73_step();
    CHECK(navigation_odometry.progress_mm>55 && navigation_odometry.progress_mm<57);
    CHECK(fabs(s_odom.increment_m*1000-navigation_odometry.progress_mm)>20);
    PASS("AUTO-1 retains encoder deltas between irregular navigation cycles instead of summing latest increment_m");

    /* Exact boundary with a synthetic 1 mm/count conversion isolates >= from
     * float geometry rounding. Production-config end-to-end coverage below. */
    for(int distance=99;distance<=101;++distance) {
        a1_advance();navigation_odometry.left_mm_per_count=navigation_odometry.right_mm_per_count=1.0;
        a1_sample(distance,distance,10U);b73_step();
        if(distance<100)CHECK(Navigation_GetState()==NAV_AVANZAR);
        else { a1_terminal(NAV_FINISH_TARGET);CHECK(robot_state==ROBOT_STATE_AUTONOMOUS); }
        PASS("AUTO-1 below / exact / exceeded 100 mm boundary and terminal FSM AUTO semantics");
    }
    a1_advance();run_motor(1);CHECK(pwm.compare[0]>0 && pwm.compare[1]>0);
    a1_sample(1800,1783,10U);b73_step();a1_terminal(NAV_FINISH_TARGET);
    CHECK(task_context.left_integral==0 && task_context.right_integral==0);
    run_motor(1);expect_zero();
    PASS("AUTO-1 real geometry exceeds 100 mm, clears live PI/PWM immediately and invalidates queued target");

    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a1_motor_schedule;
    a1_stationary=false;a1_saw_pwm=false;a1_schedule_nav_until=0U;run_motor(190);motor_cycle_hook=NULL;
    CHECK(a1_saw_pwm && navigation_odometry.progress_mm>=100);
    CHECK(Navigation_GetState()==NAV_DETENIDO && Navigation_GetFinishReason()==NAV_FINISH_TARGET);
    CHECK(left_encoder_reads==191U && right_encoder_reads==191U);
    CHECK(robot_state==ROBOT_STATE_AUTONOMOUS);expect_zero();
    PASS("AUTO-1 continuous real motor loop, encoder deltas and 20 ms navigation reach target once, then stay stopped");
    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a1_motor_schedule;
    a1_stationary=true;a1_saw_pwm=false;run_motor(202);motor_cycle_hook=NULL;
    CHECK(a1_saw_pwm && navigation_odometry.progress_mm==0);
    CHECK((TickType_t)(clock_tick-navigation.started_tick)>=2000U);
    a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-1 no encoder progress never claims success and terminates at original 2000 ms deadline");
    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a1_motor_schedule;
    a1_stationary=true;a1_saw_pwm=false;a1_schedule_nav_until=3U;
    run_motor(52);CHECK(a1_saw_pwm && Navigation_GetState()==NAV_AVANZAR && pwm.compare[0]>0);
    motor_cycle_hook=NULL;
    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a1_motor_schedule;
    a1_stationary=true;a1_saw_pwm=false;a1_schedule_nav_until=3U;run_motor(53);motor_cycle_hook=NULL;
    CHECK(a1_saw_pwm && (TickType_t)(clock_tick-task_context.command_tick)>=500U);
    a1_terminal(NAV_FINISH_CANCELLED);a1_schedule_nav_until=0U;
    PASS("AUTO-1 stalled navigation producer retains permanent 500 ms target timeout with fresh odometry");

    for(unsigned fault=0;fault<12U;++fault) {
        a1_advance();a1_sample(1,1,10U);
        if(fault==0U)s_odom.state=1U;
        if(fault==1U)s_odom.state=3U;
        if(fault==2U)s_odom.calibration=ODOM_GEOMETRY_INCOMPLETE;
        if(fault==3U)s_odom.tick=clock_tick-51U;
        if(fault==4U)s_odom.tick=clock_tick+1U;
        if(fault==5U)s_odom.x_m=NAN;
        if(fault==6U)s_odom.increment_m=INFINITY;
        if(fault==7U)s_odom.left_count=-1;
        if(fault==8U)s_odom.right_count=1000;
        if(fault==9U)s_odom.tick=navigation_odometry.last_tick-1U;
        if(fault==10U)s_odom.tick=navigation_odometry.last_tick;
        if(fault==11U)s_odom.left_count=s_odom.right_count=INT64_MAX;
        b73_step();a1_terminal(NAV_FINISH_ODOMETRY);run_motor(1);expect_zero();
        PASS("AUTO-1 invalid/stale/future/backward/nonfinite/opposed/inconsistent odometry is terminal before publication");
    }
    a1_advance();a1_sample(100,100,10U);b73_step();
    a1_sample(-1,0,10U);b73_step();a1_terminal(NAV_FINISH_ODOMETRY);
    PASS("AUTO-1 backwards wheel increment is rejected even while net travel is still positive");
    a1_advance();b73_step();CHECK(Navigation_GetState()==NAV_AVANZAR);
    PASS("AUTO-1 consistent duplicate snapshot is accepted without inventing progress");
    a1_advance();navigation_odometry.left_mm_per_count=navigation_odometry.right_mm_per_count=1;
    a1_sample(10,0,10U);b73_step();CHECK(Navigation_GetState()==NAV_AVANZAR);
    a1_sample(1,0,10U);b73_step();a1_terminal(NAV_FINISH_ODOMETRY);
    PASS("AUTO-1 wheel disagreement guard accepts exactly 10 mm and rejects more");

    int64_t delta;
    CHECK(Navigation_SubtractCount(INT64_MIN+20,INT64_MIN+10,&delta) && delta==10);
    CHECK(Navigation_SubtractCount(INT64_MAX,INT64_MAX-10,&delta) && delta==10);
    CHECK(!Navigation_SubtractCount(INT64_MAX,INT64_MIN,&delta));
    CHECK(!Navigation_SubtractCount(INT64_MIN,INT64_MAX,&delta));
    a1_advance();navigation_odometry.left=INT64_MIN;s_odom.left_count=INT64_MAX;
    b73_step();a1_terminal(NAV_FINISH_ODOMETRY);
    PASS("AUTO-1 count subtraction rejects signed overflow while accepting large finite baselines");

    for(unsigned fault=0;fault<9U;++fault) {
        a1_advance();run_motor(1);CHECK(pwm.compare[0]>0);
        b73_fresh();
        if(fault==0U)input.distancia_m=0.300f;
        if(fault==1U)input.distancia_m=0.299f;
        if(fault==2U)input.distancia_trasera_m=0.200f;
        if(fault==3U)input.ultrasonido_valido=0U;
        if(fault==4U)input.ultrasonido_trasero_valido=0U;
        if(fault==5U)input.timestamp=clock_tick-101U;
        if(fault==6U)input.timestamp_ultrasonido_frontal=clock_tick-201U;
        if(fault==7U)input.timestamp_ultrasonido_trasero=clock_tick-201U;
        if(fault==8U)input.hay_caida=1U;
        b73_step();CHECK(Navigation_GetState()==NAV_DETENIDO);expect_zero();
        CHECK(Navigation_GetFinishReason()==(fault<2U ? NAV_FINISH_OBSTACLE : NAV_FINISH_SENSORS));
        b73_fresh();b73_step();CHECK(Navigation_GetState()==NAV_DETENIDO);
        PASS("AUTO-1 front obstacle / rear danger / invalid or stale sensors / cliff stop without reverse or resumption");
    }
    a1_advance();a1_sample(0,0,10U);input.distancia_m=0.301f;b73_step();
    CHECK(Navigation_GetState()==NAV_AVANZAR);
    PASS("AUTO-1 preserves strict front >300 mm advance threshold");

    a1_advance();run_motor(1);TaskMotorControlTarget_t old=queued_target;
    uint32_t old_session=navigation.session;
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));expect_zero();
    a1_terminal(NAV_FINISH_CANCELLED);run_fsm(1);CHECK(robot_state==ROBOT_STATE_IDLE);
    CHECK(!Motion_NavigationAdvance(clock_tick,old_session));
    ++clock_tick;b73_fresh();Odometry_Init(clock_tick);
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick));run_fsm(1);
    ++clock_tick;Odometry_Update(0,0,clock_tick);b73_fresh();b73_step();
    CHECK(Navigation_GetState()==NAV_AVANZAR && navigation.session!=old_session);
    old.generation=command_generation;CHECK(xQueueOverwrite(target_speed_queue,&old)==pdPASS);
    run_motor(1);expect_zero();
    CHECK(Navigation_GetState()==NAV_AVANZAR);
    PASS("AUTO-1 STOP invalidates immediately and old session is rejected even after fresh AUTO with matching generation");
    a1_advance();run_motor(1);TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);
    a1_terminal(NAV_FINISH_SAFETY);b73_fresh();run_safety(1);run_fsm(1);CHECK(robot_state==ROBOT_STATE_SAFETY_STOP);
    b73_fresh();b73_step();expect_zero();
    PASS("AUTO-1 Safety preempts active PI immediately, latches and cannot restore navigation");
    for(int action=1;action<=4;++action) {
        if(action==2)continue;
        a1_begin();overwrite_action=action;b73_step();run_motor(1);expect_zero();
        CHECK(Navigation_GetState()==NAV_DETENIDO);
        PASS("AUTO-1 concurrent STOP/mode/Safety between admission and enqueue cannot activate motors");
    }
    a1_begin();peek_action=7;b73_step();CHECK(!target_pending);a1_terminal(NAV_FINISH_CANCELLED);
    PASS("AUTO-1 cancellation during sensor snapshot cannot emit after invalidation");
    a1_begin();target_speed_queue=NULL;b73_step();CHECK(!target_pending);a1_terminal(NAV_FINISH_EMISSION);
    PASS("AUTO-1 refused publication is terminal, not an automatic retry");
    a1_advance();run_motor(1);
    for(unsigned i=0;i<8U;++i)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_START_MANUAL));
    CHECK(!TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));expect_zero();run_fsm(1);
    CHECK(robot_state==ROBOT_STATE_IDLE);a1_terminal(NAV_FINISH_CANCELLED);
    PASS("AUTO-1 STOP still cancels live motors with a full FSM queue");
    for(unsigned phase=0;phase<2U;++phase) {
        a1_advance();navigation.state=phase==0U ? NAV_RETROCEDER_ACOTADO : NAV_GIRAR_ACOTADO;
        navigation.emission_ready=true;navigation.decision_tick=clock_tick;
        current_task=CONTROL_TASK_ID;
        CHECK(!Motion_NavigationReverse(clock_tick,navigation.session));
        CHECK(!Motion_NavigationTurn(clock_tick,navigation.session));
        CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));
        current_task=(TaskHandle_t)12;run_motor(1);expect_zero();
        CHECK(Navigation_GetState()==NAV_DETENIDO);
        PASS("AUTO-1 backend excludes reverse/turn phases even with an injected pending phase permission");
    }
    a1_advance();uint32_t permission_session=navigation.session;
    navigation.emission_ready=true;navigation.decision_tick=clock_tick;
    CHECK(!Motion_NavigationAdvance(clock_tick,permission_session));
    current_task=CONTROL_TASK_ID;
    CHECK(!TaskMotorControl_TrySetNavigationSpeedsAt(1601,1601,clock_tick,permission_session));
    CHECK(Motion_NavigationAdvance(clock_tick,permission_session));
    CHECK(!Motion_NavigationAdvance(clock_tick,permission_session));
    current_task=(TaskHandle_t)12;
    PASS("AUTO-1 navigation task identity, fixed speeds and one-use authorization remain mandatory");
    a1_advance();run_motor(1);DrvUart_t uart;UART_HandleTypeDef peripheral={0};DrvUart_Init(&uart,&peripheral);
    const char *manual[]={"AVANZAR\n","RETROCEDER\n","GIRAR_IZQUIERDA\n","GIRAR_DERECHA\n","DETENER\n"};
    for(unsigned i=0;i<5U;++i)b65_uart_command(&uart,manual[i],"MOVE_REJECTED\n");
    b65_uart_command(&uart,"RESET\n","RESET_QUEUED\n");run_fsm(1);
    CHECK(robot_state==ROBOT_STATE_AUTONOMOUS);a1_terminal(NAV_FINISH_CANCELLED);
    PASS("AUTO-1 manual UART remains rejected; RESET cancels but never reactivates AUTO or targets");

    a1_advance();clock_tick=navigation.started_tick+1999U;s_odom.tick=clock_tick;b73_fresh();
    b73_step();CHECK(Navigation_GetState()==NAV_AVANZAR);
    ++clock_tick;b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-1 deadline accepts 1999 ms but expires at exactly 2000, without renewal");
    a1_begin();clock_tick=navigation.started_tick+1999U;s_odom.tick=clock_tick;b73_fresh();
    advance_tick_before_navigation_emit=1U;b73_step();CHECK(!target_pending);a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-1 deadline crossed after decision rejects publication and never generates late PWM");
    a1_advance();a1_sample(1800,1783,10U);
    CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));
    run_motor(1);a1_terminal(NAV_FINISH_TARGET);
    PASS("AUTO-1 backend also rejects target-distance goal before pending motor consumption");
    for(a1_backend_fault=0U;a1_backend_fault<3U;++a1_backend_fault) {
        setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a1_backend_failure_schedule;
        a1_schedule_nav_until=0U;a1_stationary=false;a1_saw_pwm=false;
        run_motor(6);motor_cycle_hook=NULL;CHECK(a1_saw_pwm);
        a1_terminal(a1_backend_fault==2U ? NAV_FINISH_TARGET : NAV_FINISH_ODOMETRY);
        CHECK(!target_pending);
        PASS("AUTO-1 real pending motor consumption rejects invalid/stale/goal odometry arriving after navigation decision");
    }

    a1_begin();clock_tick=10U;navigation.started_tick=mode_entry_tick=0U;last_stop_tick=UINT32_MAX;
    Odometry_Init(0U);b73_fresh();b73_step();CHECK(!target_pending && Navigation_GetState()==NAV_ESPERAR_DATOS);
    Odometry_Update(0,0,clock_tick);b73_step();CHECK(Navigation_GetState()==NAV_AVANZAR);
    PASS("AUTO-1 tick zero is valid but a same-entry-tick odometry reference is conservatively postponed");
    a1_begin();clock_tick=UINT32_MAX-20U;navigation.started_tick=mode_entry_tick=clock_tick-1U;
    Odometry_Init(clock_tick);b73_fresh();b73_step();CHECK(Navigation_GetState()==NAV_AVANZAR);
    a1_sample(100,99,30U);b73_step();CHECK(Navigation_GetState()==NAV_AVANZAR);
    clock_tick=navigation.started_tick+2000U;b73_fresh();b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-1 wrapped ticks and irregular navigation periods preserve count progress and global deadline");
}
#endif
