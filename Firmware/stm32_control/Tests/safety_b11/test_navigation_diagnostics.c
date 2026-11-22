/* Observacion de los mismos modulos reales, sin cambiar sus decisiones. */
static NavigationDiagnostics_t nav_diag(void) {
    NavigationDiagnostics_t d;Navigation_GetDiagnostics(&d);return d;
}
#if NAV_AUTO2_ENABLED
static bool diagnostic_after_application, diagnostic_injected;
static void diagnostic_residual_schedule(void) {
    a2_motor_schedule();
    if (!diagnostic_injected && navigation.state==NAV_RETROCEDER_ACOTADO && navigation_odometry.ready &&
        (diagnostic_after_application ? task_context.left_target_cps<0 : target_pending!=0)) {
        diagnostic_injected=true;
        s_odom.left_count=navigation_odometry.left+1;
        s_odom.right_count=navigation_odometry.right;
        s_odom.tick=clock_tick;
    }
}
#endif
static void test_navigation_diagnostics(void) {
    setup();char line[256];NavigationDiagnostics_t d=nav_diag();uint32_t generation;
    CHECK(!(d.flags & NAV_DIAG_SESSION));
    CHECK(Navigation_FormatDiagnostics(line,sizeof line,1U)>0 && strlen(line)<=255U);
    CHECK(!strncmp(line,"NAV,1,1,",8U));expect_zero();
    PASS("AUTO-2D no session is absent; formatting profile does not issue targets");
#if NAV_AUTO1_ODOMETRY
    a1_advance();a1_sample(1800,1783,10U);b73_step();d=nav_diag();
    CHECK(d.reason==NAV_FINISH_TARGET && d.last_phase==NAV_AVANZAR && d.advance_um>=100000);
    CHECK(!(d.flags & NAV_DIAG_REVERSE_REQUESTED));
    uint32_t id=d.session_id, elapsed=d.elapsed_ms;generation=command_generation;
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);
    d=nav_diag();CHECK(d.reason==NAV_FINISH_TARGET && d.session_id==id && d.elapsed_ms==elapsed);
    CHECK(d.state==NAV_DETENIDO && command_generation!=generation);expect_zero();
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET));run_fsm(1);
    d=nav_diag();CHECK(d.reason==NAV_FINISH_TARGET && d.session_id==id && d.elapsed_ms==elapsed);expect_zero();
    PASS("AUTO-2D target and last phase remain retained after later operator STOP and IDLE");
    ++clock_tick;b73_fresh();Odometry_Init(clock_tick);
    CHECK(TaskControlRobot_RequestAutoAt(clock_tick));run_fsm(1);d=nav_diag();
    CHECK(d.session_id==id+1U && d.reason==NAV_FINISH_NONE && !(d.flags & NAV_DIAG_OBSTACLE));
    CHECK(!(d.flags & NAV_DIAG_REVERSE_PUBLISHED));
    PASS("AUTO-2D new AUTO has new local run ID and cannot mix previous diagnostic flags");
#endif
#if NAV_AUTO2_ENABLED
    a1_advance();a2_data(0.274f,0.500f);b73_step();d=nav_diag();
    CHECK((d.flags & (NAV_DIAG_OBSTACLE|NAV_DIAG_REVERSE_CONSIDERED))==6U);
    CHECK(!(d.flags & NAV_DIAG_REVERSE_AUTHORIZED));
    CHECK(!(d.flags & NAV_DIAG_REVERSE_REQUESTED) && d.front_mm==274 && d.rear_mm==500);
    CHECK(d.sensor_flags==7U && d.front_age_ms==1U && d.last_phase==NAV_DETENER_Y_ESPERAR);expect_zero();
    a2_stabilize();
    a1_sample(0,0,20U);a2_data(0.274f,0.500f);b73_step();d=nav_diag();
    CHECK((d.flags & (NAV_DIAG_REVERSE_REQUESTED|NAV_DIAG_REVERSE_PUBLISHED))==48U);
    CHECK(!(d.flags & NAV_DIAG_REVERSE_APPLIED));
    run_motor(1);d=nav_diag();CHECK(d.flags & NAV_DIAG_REVERSE_APPLIED);
    PASS("AUTO-2D obstacle, authorization, request, actual queue publication and backend acknowledgement are distinct");
    a1_advance();a2_data(0.250f,0.250f);b73_step();d=nav_diag();
    CHECK(d.reason==NAV_FINISH_EVASION_NOT_AUTHORIZED && (d.flags & NAV_DIAG_REVERSE_CONSIDERED));
    CHECK(!(d.flags & (NAV_DIAG_REVERSE_AUTHORIZED|NAV_DIAG_REVERSE_REQUESTED|NAV_DIAG_REVERSE_APPLIED)));
    PASS("AUTO-2D rejected rear clearance never reports a reverse publication or application");
    a1_advance();a2_data(0.250f,0.500f);b73_step();
    a2_stabilize();
    a1_sample(0,0,20U);a2_data(0.250f,0.500f);target_speed_queue=NULL;b73_step();d=nav_diag();
    CHECK(d.reason==NAV_FINISH_EMISSION && (d.flags & NAV_DIAG_REVERSE_REQUESTED));
    CHECK(!(d.flags & (NAV_DIAG_REVERSE_PUBLISHED|NAV_DIAG_REVERSE_APPLIED)));
    PASS("AUTO-2D refused publication records request but cannot fabricate queue success or backend acknowledgement");
    a1_advance();a2_data(0.250f,0.500f);b73_step();
    a2_stabilize();
    a1_sample(0,0,20U);a2_data(0.250f,0.500f);overwrite_action=3;b73_step();d=nav_diag();
    CHECK(d.reason==NAV_FINISH_OPERATOR_STOP && (d.flags & NAV_DIAG_REVERSE_PUBLISHED));
    CHECK(!(d.flags & NAV_DIAG_REVERSE_APPLIED));run_motor(1);expect_zero();
    PASS("AUTO-2D late queue success after STOP remains publication only, never application or movement permission");
    for(unsigned applied=0;applied<2U;++applied) {
        setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=diagnostic_residual_schedule;
        a2_with_obstacle=true;a2_hold_navigation=false;diagnostic_injected=false;
        diagnostic_after_application=applied!=0U;run_motor(40);motor_cycle_hook=NULL;
        CHECK(diagnostic_injected);d=nav_diag();
        CHECK(d.reason==NAV_FINISH_ODOMETRY && d.odometry_error==NAV_ODOM_DIRECTION);
        CHECK(d.last_phase==NAV_RETROCEDER_ACOTADO && d.deltas_available && d.left_delta==1 && d.right_delta==0);
        CHECK(d.flags & NAV_DIAG_REVERSE_PUBLISHED);
        CHECK(((d.flags & NAV_DIAG_REVERSE_APPLIED)!=0)==(applied!=0U));
        CHECK(!(d.flags & NAV_DIAG_REVERSE_COMPLETED));expect_zero();
        if (!applied) { CHECK(Navigation_FormatDiagnostics(line,sizeof line,1U)>0);printf("NAV_RESIDUAL_WIRE %s",line); }
        uint32_t run=d.session_id;CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);
        d=nav_diag();CHECK(d.session_id==run && d.reason==NAV_FINISH_ODOMETRY && d.left_delta==1);
        PASS("AUTO-2D real residual +1 count cancels before/after application and diagnostic survives STOP unchanged");
    }
    a2_reverse();a1_sample(-360,-357,10U);a2_data(0.500f,0.500f);b73_step();d=nav_diag();
    CHECK(d.flags & NAV_DIAG_REVERSE_COMPLETED);CHECK(d.reverse_um>=20000);
    PASS("AUTO-2D reverse distance completed is distinct from refused lateral turn");
    a1_advance();clock_tick=navigation.started_tick+2000U;b73_step();d=nav_diag();
    CHECK(d.reason==NAV_FINISH_TIMEOUT && d.last_phase==NAV_AVANZAR);
    a2_reverse();clock_tick=navigation.phase_started_tick+800U;b73_step();d=nav_diag();
    CHECK(d.reason==NAV_FINISH_TIMEOUT && d.last_phase==NAV_RETROCEDER_ACOTADO);
    PASS("AUTO-2D elapsed and last phase distinguish forward timeout from reverse timeout");
    a2_reverse();navigation_odometry.progress_tick=clock_tick-200U;a2_data(0.250f,0.500f);b73_step();
    CHECK(nav_diag().reason==NAV_FINISH_NO_PROGRESS);
    a2_reverse();input.ultrasonido_trasero_valido=0U;b73_step();d=nav_diag();
    CHECK(d.reason==NAV_FINISH_SENSORS && !(d.sensor_flags&4U));
    PASS("AUTO-2D lack of progress and sensor failure preserve separate causes and measurement validity");
    a2_reverse();s_odom.tick=clock_tick-51U;b73_step();d=nav_diag();
    CHECK(d.reason==NAV_FINISH_ODOMETRY && d.odometry_error==NAV_ODOM_STALE);
    a2_reverse();CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));d=nav_diag();CHECK(d.reason==NAV_FINISH_OPERATOR_STOP);
    a2_reverse();TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);d=nav_diag();CHECK(d.reason==NAV_FINISH_SAFETY);
    PASS("AUTO-2D stale odometry, operator STOP and Safety are observable without modifying their precedence");
    const NavigationOdometryError_t errors[]={NAV_ODOM_UNAVAILABLE,NAV_ODOM_STALE,NAV_ODOM_TIMESTAMP,
        NAV_ODOM_DIRECTION,NAV_ODOM_OVERFLOW,NAV_ODOM_NONFINITE,NAV_ODOM_DISAGREEMENT,NAV_ODOM_RANGE};
    for(unsigned i=0;i<8U;++i) {
        a1_advance();a1_sample(0,0,10U);
        if(i==0U)s_odom.state=3U;
        if(i==1U)s_odom.tick=clock_tick-51U;
        if(i==2U){s_odom.tick=navigation_odometry.last_tick;++s_odom.left_count;}
        if(i==3U)--s_odom.left_count;
        if(i==4U){navigation_odometry.left=navigation_odometry.last_left=INT64_MIN;s_odom.left_count=INT64_MAX;}
        if(i==5U)s_odom.x_m=NAN;
        if(i==6U)s_odom.left_count+=300;
        if(i==7U)s_odom.left_count=INT64_MAX;
        b73_step();d=nav_diag();CHECK(d.reason==NAV_FINISH_ODOMETRY && d.odometry_error==errors[i]);expect_zero();
        PASS("AUTO-2D classifies existing odometry failure without replacing its original movement guard");
    }
    for(unsigned phase=0;phase<(NAV_EXPERIMENTAL_TURN_ENABLED ? 3U : 2U);++phase) {
        a2_phase(phase);TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);d=nav_diag();
        CHECK(d.reason==NAV_FINISH_SAFETY && d.last_phase==(phase==0U ? NAV_AVANZAR : phase==1U ? NAV_RETROCEDER_ACOTADO : NAV_GIRAR_ACOTADO));
        expect_zero();PASS("AUTO-2D retains actual last phase when Safety cancels any allowed maneuver");
    }
#endif
    DrvUart_t uart;UART_HandleTypeDef peripheral={0};DrvUart_Init(&uart,&peripheral);
    CHECK(Navigation_FormatDiagnostics(line,sizeof line,42U)>0);
    generation=command_generation;
    b65_uart_command(&uart,"NAV Q=42\n",line);CHECK(command_generation==generation);
    b65_uart_command(&uart,"NAV Q=0\n","ERR_UNKNOWN\n");
    b65_uart_command(&uart,"NAV Q=4294967296\n","ERR_UNKNOWN\n");
    CHECK(!Navigation_FormatDiagnostics(line,sizeof line,0U) || Navigation_FormatDiagnostics(line,sizeof line,0U)<0);
    CHECK(Navigation_FormatDiagnostics(line,8U,1U)<0);
    PASS("AUTO-2D actual UART query validates bounds, formats outside locks and cannot modify motor generation");
    const char *queries[]={"STATUS\n","SENSORS Q=3\n","IMU Q=4\n","MAPPING Q=5 S=0\n"};
    const char *prefixes[]={"STATE=","SENSORS,","IMU,","POSE,"};
    for(unsigned i=0;i<4U;++i) {
        uart_feed(&uart,"NAV Q=6\n");uart_feed(&uart,queries[i]);CHECK(uart_consume(&uart));
        CHECK(!strncmp(uart_response,prefixes[i],strlen(prefixes[i])));
        uart_feed(&uart,queries[i]);uart_feed(&uart,"NAV Q=6\n");CHECK(uart_consume(&uart));
        CHECK(!strncmp(uart_response,prefixes[i],strlen(prefixes[i])));
        PASS("AUTO-2D NAV is subordinate to STATUS/SENSORS/IMU/mapping in both UART burst orders");
    }
    uart_feed(&uart,"NAV Q=6\nSTOP\n");CHECK(uart_consume(&uart));
    CHECK(!strcmp(uart_response,"STOP_QUEUED\n"));expect_zero();
    PASS("AUTO-2D pending NAV cannot displace a higher-priority STOP in UART mailbox");
    /* Extremos del serializador: no depender de printf %lld de newlib-nano. */
    navigation_diagnostics.deltas_available=true;
    navigation_diagnostics.odometry_error=NAV_ODOM_OVERFLOW;
    navigation_diagnostics.left_delta=INT64_MIN;navigation_diagnostics.right_delta=INT64_MAX;
    navigation_diagnostics.session_id=navigation_diagnostics.started_tick=UINT32_MAX;
    navigation_diagnostics.elapsed_ms=navigation_diagnostics.phase_elapsed_ms=UINT32_MAX;
    navigation_diagnostics.snapshot_age_ms=navigation_diagnostics.front_age_ms=navigation_diagnostics.rear_age_ms=UINT32_MAX;
    CHECK(Navigation_FormatDiagnostics(line,sizeof line,UINT32_MAX)>0 && strlen(line)<=255U);
    CHECK(strstr(line,"-9223372036854775808,9223372036854775807"));
    printf("NAV_WIRE %s",line);
    PASS("AUTO-2D bounded wire preserves full signed deltas without float or long-long formatter support");
}
