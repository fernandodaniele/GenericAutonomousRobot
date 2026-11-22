/* Fuentes reales; conteos, fallas y tiempos sinteticos solo en este arnes. */
#if NAV_AUTO2_ENABLED
static void a2_data(float front, float rear) {
    b73_fresh(); input.distancia_m=front; input.distancia_trasera_m=rear;
}
static void a2_stabilize(void) {
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);
    for(unsigned i=0;i<4U;++i) {
        a1_sample(0,0,20U);a2_data(0.250f,0.500f);b73_step();expect_zero();
        CHECK(navigation.state==(i<3U ? NAV_DETENER_Y_ESPERAR : NAV_RETROCEDER_ACOTADO));
    }
    CHECK(navigation_diagnostics.settle_state==2U && !navigation_odometry.ready);
}
static void a2_reverse(void) {
    a1_advance();run_motor(1);CHECK(pwm.compare[0]>0 && pwm.compare[1]>0);
    uint32_t old_session=navigation.session;TickType_t entered=navigation.started_tick;
    a2_data(0.250f,0.500f);b73_step();
    CHECK(Navigation_GetState()==NAV_DETENER_Y_ESPERAR && navigation.session!=old_session);
    CHECK(navigation.started_tick==entered && !navigation_odometry.ready);expect_zero();
    b73_step();CHECK(!target_pending && !navigation_odometry.ready);expect_zero();
    a2_stabilize();
    a1_sample(0,0,20U);a2_data(0.250f,0.500f);b73_step();
    CHECK(queued_target.left_speed_cps==-1600 && queued_target.right_speed_cps==-1600);
    run_motor(1);CHECK(left_motor.current_speed<0 && right_motor.current_speed<0);
    CHECK(navigation.reverse_applied);
}
#if NAV_EXPERIMENTAL_TURN_ENABLED
static void a2_turn(void) {
    a2_reverse();a1_sample(-360,-357,10U);a2_data(0.500f,0.500f);b73_step();
    CHECK(Navigation_GetState()==NAV_GIRAR_ACOTADO && navigation.turn_used && navigation.reverse_completed);
    expect_zero();CHECK(!navigation_odometry.ready);
    a1_sample(0,0,20U);a2_data(0.500f,0.500f);b73_step();
    CHECK(queued_target.left_speed_cps==NAV_TURN_LEFT_CPS && queued_target.right_speed_cps==NAV_TURN_RIGHT_CPS);
}
#endif
static void a2_phase(unsigned phase) {
    if(phase==0U)a1_advance();
    if(phase==1U)a2_reverse();
#if NAV_EXPERIMENTAL_TURN_ENABLED
    if(phase==2U)a2_turn();
#endif
}
/* Planificador host: el lazo real MotorControl corre sin reinicios; navegacion
 * cada 20 ms, deltas una vez por ciclo. No se sustituyen algoritmos de control. */
static bool a2_with_obstacle, a2_positive_pwm, a2_negative_pwm, a2_turn_pwm;
static bool a2_hold_navigation;
static void a2_motor_schedule(void) {
    if(left_motor.current_speed>0 && right_motor.current_speed>0)a2_positive_pwm=true;
    if(left_motor.current_speed<0 && right_motor.current_speed<0)a2_negative_pwm=true;
    if(left_motor.current_speed*right_motor.current_speed<0)a2_turn_pwm=true;
    if(delay_calls==1) {
        CHECK(TaskControlRobot_RequestAutoAt(clock_tick));
        TaskControlRobotEventMessage_t message;
        CHECK(xQueueReceive(event_queue,&message,0U)==pdTRUE);
        TaskControlRobot_ProcessEventAt(message.event,&message);
    }
    if(delay_calls%2==1 && (!a2_hold_navigation || delay_calls<=3)) {
        float front=a2_with_obstacle && delay_calls>=11 && !navigation.reverse_completed ? 0.250f : 0.500f;
        /* En el ensayo experimental el frontal se despeja al separarse; se
         * modela una muestra nueva, no se cambia la regla de autorizacion. */
        if(navigation.state==NAV_RETROCEDER_ACOTADO && navigation_odometry.progress_mm>15)front=0.500f;
        a2_data(front,0.500f);b73_step();
    }
    left_encoder_delta=right_encoder_delta=0;
    if(task_context.left_target_cps==1600 && task_context.right_target_cps==1600) {
        left_encoder_delta=right_encoder_delta=12;
    } else if(task_context.left_target_cps==-1600 && task_context.right_target_cps==-1600) {
        left_encoder_delta=right_encoder_delta=-14;
    } else if(task_context.left_target_cps==NAV_TURN_LEFT_CPS && task_context.right_target_cps==NAV_TURN_RIGHT_CPS) {
        int32_t turn_delta=11+(delay_calls%2==0 ? 1 : 0);
        left_encoder_delta=NAV_TURN_LEFT ? -turn_delta : turn_delta;
        right_encoder_delta=-left_encoder_delta;
    }
}
static unsigned a2_race_phase, a2_race_kind;
static bool a2_race_injected;
static void a2_race_schedule(void) {
    a2_motor_schedule();
    NavigationState_t phase=a2_race_phase==0U ? NAV_AVANZAR :
                            a2_race_phase==1U ? NAV_RETROCEDER_ACOTADO : NAV_GIRAR_ACOTADO;
    if(!a2_race_injected && navigation.state==phase && target_pending) {
        a2_race_injected=true;
        if(a2_race_kind==0U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
        if(a2_race_kind==1U)TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);
        if(a2_race_kind==2U)s_odom.state=3U;
        if(a2_race_kind==3U)s_odom.tick=clock_tick-51U;
    }
}
static void test_navigation_auto2(void) {
    CHECK(NAV_AUTO2_ENABLED==1 && NAV_SESSION_MAX_MS==3000U);
    CHECK(NAV_AUTO1_TARGET_MM==100U && NAV_AUTO2_ADVANCE_MAX_MS==2000U);
    CHECK(NAV_FRONT_STOP_MM==275U && NAV_REAR_REVERSE_MIN_MM==250U);
    CHECK(NAV_REVERSE_MAX_MS==800U && NAV_TURN_MAX_MS==250U);
    a1_begin();expect_zero();CHECK(!target_pending);b73_step();
    CHECK(Navigation_GetState()==NAV_AVANZAR);
    PASS("AUTO-2 starts only after actual safe IDLE transition and post-entry odometry reference");

    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a2_motor_schedule;
    a2_with_obstacle=false;a2_positive_pwm=a2_negative_pwm=a2_turn_pwm=false;
    run_motor(210);motor_cycle_hook=NULL;
    CHECK(a2_positive_pwm && !a2_negative_pwm && !a2_turn_pwm);
    a1_terminal(NAV_FINISH_TARGET);CHECK(robot_state==ROBOT_STATE_AUTONOMOUS);
    PASS("AUTO-2 real encoder/PI/driver straight sequence reaches 100 mm once and remains stopped in AUTO");
    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a2_motor_schedule;
    a2_hold_navigation=true;a2_with_obstacle=false;
    run_motor(53);motor_cycle_hook=NULL;a2_hold_navigation=false;
    CHECK((TickType_t)(clock_tick-queued_target.issued_tick)==500U);
    a1_terminal(NAV_FINISH_CANCELLED);
    PASS("AUTO-2 fresh encoder progress cannot defeat the permanent 500 ms command timeout if navigation stalls");
    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a2_motor_schedule;
    a2_with_obstacle=true;a2_positive_pwm=a2_negative_pwm=a2_turn_pwm=false;
    run_motor(150);motor_cycle_hook=NULL;
    CHECK(a2_positive_pwm && a2_negative_pwm && navigation.reverse_completed);
#if NAV_EXPERIMENTAL_TURN_ENABLED
    CHECK(a2_turn_pwm && navigation.turn_used);a1_terminal(NAV_FINISH_EVASION_COMPLETED);
#else
    CHECK(!a2_turn_pwm && !navigation.turn_used);a1_terminal(NAV_FINISH_EVASION_NOT_AUTHORIZED);
#endif
    CHECK(left_encoder_reads==151U && right_encoder_reads==151U);
    PASS("AUTO-2 real finite obstacle sequence zeros each phase; default blocks lateral turn, experimental variant completes it");

    const float front[]={0.276f,0.275f,0.250f,0.200f,0.199f};
    for(unsigned i=0;i<5U;++i) {
        a1_advance();a2_data(front[i],0.500f);b73_step();
        if(i==0U)CHECK(Navigation_GetState()==NAV_AVANZAR);
        else if(i<3U){CHECK(Navigation_GetState()==NAV_DETENER_Y_ESPERAR);expect_zero();}
        else a1_terminal(NAV_FINISH_EVASION_NOT_AUTHORIZED);
        PASS("AUTO-2F exact preventive 275 mm and unchanged critical 200 mm boundaries never bypass Safety snapshot criteria");
    }
    a1_begin();a2_data(0.275f,0.500f);b73_step();a1_terminal(NAV_FINISH_OBSTACLE);
    CHECK(!navigation.reverse_completed);
    PASS("AUTO-2 obstacle present on entry stops without initiating reverse");
    const float rear[]={0.251f,0.250f,0.249f,0.200f};
    for(unsigned i=0;i<4U;++i) {
        a1_advance();a2_data(0.250f,rear[i]);b73_step();
        if(i==0U){CHECK(Navigation_GetState()==NAV_DETENER_Y_ESPERAR);expect_zero();}
        else a1_terminal(NAV_FINISH_EVASION_NOT_AUTHORIZED);
        PASS("AUTO-2F reverse authorization requires rear strictly above 250 mm");
    }
    for(unsigned invalid=0;invalid<4U;++invalid) {
        a1_advance();a2_data(0.250f,0.500f);
        if(invalid==0U)input.ultrasonido_trasero_valido=0U;
        if(invalid==1U)input.timestamp_ultrasonido_trasero=clock_tick-201U;
        if(invalid==2U)input.timestamp=clock_tick-101U;
        if(invalid==3U)input.distancia_trasera_m=NAN;
        b73_step();CHECK(Navigation_GetState()==NAV_DETENIDO);expect_zero();
        PASS("AUTO-2 no echo, old rear/snapshot or NaN cannot authorize avoidance");
    }

    const unsigned phases=NAV_EXPERIMENTAL_TURN_ENABLED ? 3U : 2U;
    for(unsigned phase=0;phase<phases;++phase) {
        for(unsigned fault=0;fault<11U;++fault) {
            a2_phase(phase);b73_fresh();
            if(phase==1U)input.distancia_m=0.250f;
            if(fault==0U)input.ultrasonido_valido=0U;
            if(fault==1U)input.ultrasonido_trasero_valido=0U;
            if(fault==2U)input.timestamp_ultrasonido_frontal=clock_tick-201U;
            if(fault==3U)input.timestamp_ultrasonido_trasero=clock_tick-201U;
            if(fault==4U)input.timestamp=clock_tick-101U;
            if(fault==5U)input.hay_caida=1U;
            if(fault==6U)s_odom.state=3U;
            if(fault==7U)s_odom.tick=clock_tick-51U;
            if(fault==8U)s_odom.left_count=INT64_MAX;
            if(fault==9U)s_odom.left_count+=phase==0U || (phase==2U && !NAV_TURN_LEFT) ? -1 : 1;
            if(fault==10U)s_odom.right_count+=phase==1U || (phase==2U && !NAV_TURN_LEFT) ? 1 : -1;
            b73_step();CHECK(Navigation_GetState()==NAV_DETENIDO);expect_zero();
            b73_fresh();b73_step();CHECK(Navigation_GetState()==NAV_DETENIDO);
            PASS("AUTO-2 sensor/IR/odometry faults cancel every active phase without automatic recovery");
        }
        a2_phase(phase);uint32_t session=navigation.session;
        CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));a1_terminal(NAV_FINISH_OPERATOR_STOP);
        run_fsm(1);CHECK(robot_state==ROBOT_STATE_IDLE);
        CHECK(!Navigation_IsCommandCurrent(session,clock_tick));
        PASS("AUTO-2 operator STOP preempts each phase and returns healthy FSM to IDLE");
        a2_phase(phase);TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);a1_terminal(NAV_FINISH_SAFETY);
        b73_fresh();run_safety(1);run_fsm(1);CHECK(robot_state==ROBOT_STATE_SAFETY_STOP);
        PASS("AUTO-2 Safety immediately cancels each phase and retains its original latch");
        clock_tick+=100U;b73_fresh();DrvUart_t reset_uart;UART_HandleTypeDef reset_peripheral={0};
        DrvUart_Init(&reset_uart,&reset_peripheral);
        b65_uart_command(&reset_uart,"RESET\n","RESET_QUEUED\n");run_fsm(1);run_motor(1);
        CHECK(robot_state==ROBOT_STATE_IDLE && !TaskSafety_IsStopActive());
        CHECK(Navigation_GetState()==NAV_DETENIDO);expect_zero();
        PASS("AUTO-2 explicit safe rearm after any phase restores IDLE but never restores its old movement");
        a2_phase(phase);DrvUart_t uart;UART_HandleTypeDef peripheral={0};DrvUart_Init(&uart,&peripheral);
        b65_uart_command(&uart,"RESET\n","RESET_QUEUED\n");run_fsm(1);
        a1_terminal(NAV_FINISH_CANCELLED);CHECK(robot_state==ROBOT_STATE_AUTONOMOUS);
        PASS("AUTO-2 healthy RESET cancels every phase without restoring a session or changing FSM semantics");
        a2_phase(phase);navigation_odometry.progress_tick=clock_tick-199U;
        b73_step();CHECK(navigation.active);
        ++clock_tick;b73_step();a1_terminal(NAV_FINISH_NO_PROGRESS);
        PASS("AUTO-2 no-progress window is admitted at 199 ms and terminal at 200 ms in every phase");
        for(a2_race_kind=0U;a2_race_kind<4U;++a2_race_kind) {
            setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a2_race_schedule;
            a2_race_phase=phase;a2_race_injected=false;a2_with_obstacle=phase!=0U;
            run_motor(100);motor_cycle_hook=NULL;CHECK(a2_race_injected);
            a1_terminal(a2_race_kind==0U ? NAV_FINISH_OPERATOR_STOP :
                        a2_race_kind==1U ? NAV_FINISH_SAFETY : NAV_FINISH_ODOMETRY);
            PASS("AUTO-2 real pending PWM consumption rejects STOP/Safety/invalid/stale odometry racing every phase decision");
        }
    }
    a2_reverse();a1_sample(-300,-297,17U);a2_data(0.250f,0.500f);b73_step();
    CHECK(Navigation_GetState()==NAV_RETROCEDER_ACOTADO && navigation_odometry.progress_mm<20);
    a1_sample(-60,-60,23U);a2_data(0.500f,0.500f);b73_step();
    CHECK(navigation.reverse_completed);expect_zero();
#if NAV_EXPERIMENTAL_TURN_ENABLED
    CHECK(Navigation_GetState()==NAV_GIRAR_ACOTADO);
#else
    a1_terminal(NAV_FINISH_EVASION_NOT_AUTHORIZED);
#endif
    PASS("AUTO-2 accumulated negative counts with separate CPR reach 20 mm independent of navigation period");
    a2_reverse();navigation_odometry.left_mm_per_count=navigation_odometry.right_mm_per_count=1.0;
    a1_sample(-19,-19,10U);a2_data(0.500f,0.500f);b73_step();CHECK(navigation.state==NAV_RETROCEDER_ACOTADO);
    a1_sample(-1,-1,10U);a2_data(0.500f,0.500f);b73_step();CHECK(navigation.reverse_completed);expect_zero();
    PASS("AUTO-2 exact 20 mm reverse target is terminal for the old phase; 19 mm is not completion");

    a2_reverse();TickType_t phase_start=navigation.phase_started_tick;
    clock_tick=phase_start+799U;s_odom.tick=clock_tick;navigation_odometry.progress_tick=clock_tick;
    a2_data(0.250f,0.500f);b73_step();CHECK(navigation.active);
    ++clock_tick;b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-2F reverse timeout at exactly 800 ms is not misreported as successful distance completion");
    a1_advance();clock_tick=navigation.started_tick+1999U;s_odom.tick=clock_tick;
    navigation_odometry.progress_tick=clock_tick;b73_fresh();b73_step();CHECK(navigation.active);
    ++clock_tick;b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-2 unobstructed advance keeps the validated 2000 ms cap despite longer global sequence budget");
    a2_reverse();clock_tick=navigation.started_tick+3000U;b73_fresh();b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-2 original global deadline wins during avoidance; transitions never extend it");
    a2_reverse();clock_tick=navigation.started_tick+2999U;
    navigation.phase_started_tick=navigation.reverse_started_tick=clock_tick-10U;
    s_odom.tick=navigation_odometry.progress_tick=clock_tick;a2_data(0.250f,0.500f);b73_step();
    CHECK(navigation.active);++clock_tick;
    CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-2 backend exact 3000 ms global boundary revokes even a numerically current local phase");

    a1_advance();TaskMotorControlTarget_t forward=queued_target;uint32_t prior=navigation.session;
    a2_data(0.250f,0.500f);b73_step();CHECK(navigation.session!=prior);expect_zero();
    a2_stabilize();
    a1_sample(0,0,20U);a2_data(0.250f,0.500f);b73_step();
    forward.generation=command_generation;CHECK(xQueueOverwrite(target_speed_queue,&forward)==pdPASS);
    run_motor(1);expect_zero();CHECK(Navigation_GetState()==NAV_RETROCEDER_ACOTADO);
    CHECK(!Navigation_IsCommandCurrent(prior,clock_tick));
    PASS("AUTO-2 old forward target cannot cross into reverse even with forged current generation");
    for(unsigned action=1U;action<=4U;++action) {
        if(action==2U)continue;
        a1_begin();overwrite_action=(int)action;b73_step();run_motor(1);expect_zero();
        CHECK(Navigation_GetState()==NAV_DETENIDO);
        PASS("AUTO-2 concurrent STOP/mode/Safety at enqueue preserves generation barriers");
    }
    a2_reverse();a2_data(0.250f,0.500f);peek_action=7;b73_step();expect_zero();
    CHECK(Navigation_GetState()==NAV_DETENIDO);
    PASS("AUTO-2 cancellation while copying sensors cannot reactivate reverse");
    a2_reverse();DrvUart_t uart;UART_HandleTypeDef peripheral={0};DrvUart_Init(&uart,&peripheral);
    const char *manual[]={"AVANZAR\n","RETROCEDER\n","GIRAR_IZQUIERDA\n","GIRAR_DERECHA\n","DETENER\n"};
    for(unsigned i=0;i<5U;++i)b65_uart_command(&uart,manual[i],"MOVE_REJECTED\n");
    PASS("AUTO-2 autonomous phase never admits manual UART movements or DETENER as global STOP");
    a1_begin();queue_available=0;b73_step();CHECK(Navigation_GetState()==NAV_ESPERAR_DATOS);
    clock_tick=navigation.started_tick+2000U;b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-2 waiting without data remains inside original advance budget");

#if NAV_EXPERIMENTAL_TURN_ENABLED
    a2_reverse();TaskMotorControlTarget_t old_reverse=queued_target;uint32_t reverse_session=navigation.session;
    a1_sample(-360,-357,10U);a2_data(0.500f,0.500f);b73_step();
    CHECK(navigation.state==NAV_GIRAR_ACOTADO && navigation.session!=reverse_session);expect_zero();
    old_reverse.generation=command_generation;CHECK(xQueueOverwrite(target_speed_queue,&old_reverse)==pdPASS);
    run_motor(1);expect_zero();CHECK(navigation.state==NAV_GIRAR_ACOTADO);
    CHECK(!Navigation_IsCommandCurrent(reverse_session,clock_tick));
    PASS("AUTO-2 stale reverse target cannot cross turn transition, even before new turn reference exists");
    a2_turn();TaskMotorControlTarget_t turn=queued_target;
    CHECK((turn.left_speed_cps<0)==(NAV_TURN_LEFT!=0));
    a1_sample(NAV_TURN_LEFT ? -200 : 200,NAV_TURN_LEFT ? 198 : -198,17U);
    a2_data(0.500f,0.500f);b73_step();CHECK(Navigation_GetState()==NAV_GIRAR_ACOTADO);
    a1_sample(NAV_TURN_LEFT ? -40 : 40,NAV_TURN_LEFT ? 40 : -40,13U);
    a2_data(0.500f,0.500f);b73_step();a1_terminal(NAV_FINISH_EVASION_COMPLETED);
    CHECK(navigation.turn_used);run_motor(1);expect_zero();
    PASS("AUTO-2 experimental signed opposite wheel counts limit turn to 10 degrees, one turn only, then terminal zero");
    a2_turn();clock_tick=navigation.phase_started_tick+250U;
    CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));b73_fresh();b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-2 experimental turn backend refuses exact 250 ms boundary without waiting for a navigation iteration");
    a2_turn();a2_data(0.500f,0.500f);overwrite_action=4;b73_step();run_motor(1);expect_zero();
    CHECK(navigation.finish_reason==NAV_FINISH_SAFETY);
    PASS("AUTO-2 Safety concurrent with turn renewal invalidates both the pending and current target");
    a2_reverse();a1_sample(-360,-357,10U);a2_data(0.275f,0.500f);b73_step();
    a1_terminal(NAV_FINISH_EVASION_NOT_AUTHORIZED);CHECK(!navigation.turn_used);
    PASS("AUTO-2F even explicitly enabled turn is refused when front has not cleared 275 mm");
    a2_turn();a2_data(0.500f,0.250f);b73_step();a1_terminal(NAV_FINISH_EVASION_NOT_AUTHORIZED);
    PASS("AUTO-2F turn cancels immediately if rear clearance falls to 250 mm");
#else
    a2_reverse();a1_sample(-360,-357,10U);a2_data(0.500f,0.500f);b73_step();
    a1_terminal(NAV_FINISH_EVASION_NOT_AUTHORIZED);CHECK(navigation.reverse_completed && !navigation.turn_used);
    CHECK(!Motion_NavigationTurn(clock_tick,navigation.session));
    PASS("AUTO-2 production never authorizes a lateral turn even with clear front and rear sensors");
#endif
    a1_advance();clock_tick=0U;navigation.started_tick=mode_entry_tick=UINT32_MAX-100U;
    navigation_odometry.reference_tick=navigation_odometry.last_tick=UINT32_MAX-90U;
    navigation_odometry.progress_tick=clock_tick;
    Odometry_Init(clock_tick);a2_data(0.250f,0.500f);b73_step();
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && navigation.phase_started_tick==0U);expect_zero();
    a2_stabilize();
    a1_sample(0,0,20U);a2_data(0.250f,0.500f);b73_step();
    CHECK(navigation.state==NAV_RETROCEDER_ACOTADO && navigation_odometry.ready);
    CHECK(queued_target.left_speed_cps==-1600);PASS("AUTO-2E wait entry tick zero and subsequent reverse reference are legitimate across wrap");
    a1_begin();clock_tick=UINT32_MAX-10U;navigation.started_tick=navigation.phase_started_tick=mode_entry_tick=clock_tick-1U;
    Odometry_Init(clock_tick);b73_fresh();b73_step();CHECK(Navigation_GetState()==NAV_AVANZAR);
    a1_sample(20,20,30U);b73_step();CHECK(navigation.active);
    navigation_odometry.progress_tick=clock_tick;clock_tick=navigation.started_tick+2000U;
    b73_step();a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-2 post-entry reference, irregular ticks and advance timeout survive FreeRTOS wrap");
}
#endif
