/* AUTO-2F: simulacion del lazo real, sin alterar PI/driver/odometria. */
#if NAV_AUTO2_ENABLED
static int32_t a2f_reverse_counts;
static bool a2f_late_obstacle, a2f_late_cancelled;
static unsigned a2f_late_cancel;
static TickType_t a2f_reverse_completed_ms;
static void a2f_schedule(void) {
    if(a2f_late_obstacle)a2_with_obstacle=delay_calls>=191;
    NavigationState_t before=navigation.state;
    a2_motor_schedule();
    if(before==NAV_RETROCEDER_ACOTADO && navigation.state!=before && navigation.reverse_completed) {
        a2f_reverse_completed_ms=clock_tick-navigation.reverse_started_tick;
    }
    if(task_context.left_target_cps==-1600 && task_context.right_target_cps==-1600) {
        left_encoder_delta=right_encoder_delta=-a2f_reverse_counts;
    }
    if(a2f_late_obstacle && task_context.left_target_cps==1600 && task_context.right_target_cps==1600) {
        left_encoder_delta=right_encoder_delta=1;
    }
    if(a2f_late_cancel && !a2f_late_cancelled && navigation.state==NAV_RETROCEDER_ACOTADO &&
       (TickType_t)(clock_tick-navigation.reverse_started_tick)>=500U) {
        a2f_late_cancelled=true;
        if(a2f_late_cancel==1U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
        else TaskSafety_ReportFault(SAFETY_FAULT_CLIFF);
    }
}
static void a2f_run(int32_t reverse_counts,bool late,unsigned cancel) {
    setup();robot_state=ROBOT_STATE_IDLE;motor_cycle_hook=a2f_schedule;
    a2f_reverse_counts=reverse_counts;a2f_late_obstacle=late;a2f_late_cancel=cancel;
    a2f_late_cancelled=false;a2f_reverse_completed_ms=0U;
    a2_with_obstacle=true;a2_positive_pwm=a2_negative_pwm=a2_turn_pwm=false;
    run_motor(350);motor_cycle_hook=NULL;
    CHECK(a2_positive_pwm && a2_negative_pwm);expect_zero();
}
#endif
static void test_navigation_auto2f(void) {
#if NAV_AUTO2_ENABLED
    CHECK(NAV_AUTO2_REVERSE_TARGET_MM==20U && MOTION_DRIVE_CPS==1600);
    CHECK(NAV_REVERSE_MAX_MS==800U && NAV_AUTO2_PROGRESS_WINDOW_MS==200U);
    CHECK(NAV_AUTO2_ADVANCE_MAX_MS+NAV_SETTLE_TIMEOUT_MS+NAV_REVERSE_MAX_MS==NAV_SESSION_MAX_MS);
    CHECK(TASK_SAFETY_OBSTACLE_DISTANCE_M==0.20f);
    a2f_run(7,false,0U);NavigationDiagnostics_t d=nav_diag();
    CHECK(navigation.reverse_completed && d.reverse_um>=20000 && d.reverse_um<21000);
    CHECK(a2f_reverse_completed_ms>300U && a2f_reverse_completed_ms<800U);
    CHECK(d.settle_state==2U && d.reverse_reference_available && d.odometry_error==NAV_ODOM_OK);
    CHECK(d.flags & NAV_DIAG_REVERSE_APPLIED);
    PASS("AUTO-2F actual PI/encoder loop reaches 20 mm after 300 ms but before 800 ms and never waits out the new timeout");

    a2f_run(3,false,0U);d=nav_diag();
    CHECK(d.reason==NAV_FINISH_TIMEOUT && d.last_phase==NAV_RETROCEDER_ACOTADO);
    CHECK(d.phase_elapsed_ms==800U && d.reverse_um>0 && d.reverse_um<20000);
    CHECK(d.odometry_error==NAV_ODOM_OK && !navigation.reverse_completed);
    a1_terminal(NAV_FINISH_TIMEOUT);
    PASS("AUTO-2F healthy but slower real encoder progress ends at 800 ms when 20 mm are not reached, without residual commands");
    a2f_run(0,false,0U);d=nav_diag();
    CHECK(d.reason==NAV_FINISH_NO_PROGRESS && d.phase_elapsed_ms<800U && !navigation.reverse_completed);
    PASS("AUTO-2F extending reverse time cannot disable the earlier 1 mm per 200 ms no-progress stop");
    a2f_run(3,true,0U);d=nav_diag();
    CHECK(d.reason==NAV_FINISH_TIMEOUT && d.phase_elapsed_ms==800U && d.elapsed_ms<3000U);
    CHECK(navigation.reverse_started_tick-navigation.started_tick>=1900U);
    PASS("AUTO-2F late forward detection plus real settling and 800 ms reverse fit the unchanged global 3000 ms budget");
    for(unsigned kind=1U;kind<=2U;++kind) {
        a2f_run(3,false,kind);CHECK(a2f_late_cancelled);
        CHECK(nav_diag().reason==(kind==1U?NAV_FINISH_OPERATOR_STOP:NAV_FINISH_SAFETY));
        CHECK(nav_diag().phase_elapsed_ms>=500U && nav_diag().phase_elapsed_ms<800U);
        PASS("AUTO-2F operator STOP and Safety preempt the extended reverse window after the old 300 ms limit");
    }

    const float dists[]={0.170f,0.200f,0.201f};
    for(unsigned side=0;side<2U;++side) {
        for(unsigned i=0;i<3U;++i) {
            setup();a2_data(side==0U?dists[i]:0.500f,side==1U?dists[i]:0.500f);
            run_safety(1);
            if(i<2U) {
                CHECK(TaskSafety_IsStopActive() && (TaskSafety_GetFaults() & SAFETY_FAULT_OBSTACLE));
                clock_tick+=100U;a2_data(side==0U?0.200f:0.500f,side==1U?0.200f:0.500f);
                CHECK(!TaskSafety_RequestReset());
                ++clock_tick;a2_data(0.201f,0.201f);CHECK(TaskSafety_RequestReset());
                CHECK(!TaskSafety_IsStopActive() && TaskSafety_GetFaults()==SAFETY_FAULT_NONE);
            } else { CHECK(!TaskSafety_IsStopActive() && TaskSafety_GetFaults()==SAFETY_FAULT_NONE); }
            expect_zero();
            PASS("AUTO-2F unchanged 200 mm critical guard applies to both sensors and successful rearm still needs safe fresh data");
        }
    }
    for(unsigned invalid=0;invalid<3U;++invalid) {
        setup();a2_data(0.276f,0.251f);
        if(invalid==0U)input.ultrasonido_valido=0U;
        if(invalid==1U)input.timestamp_ultrasonido_trasero=clock_tick-201U;
        if(invalid==2U)queue_available=0;
        run_safety(invalid==2U?6:1);
        CHECK(TaskSafety_IsStopActive() && (TaskSafety_GetFaults() & SAFETY_FAULT_SENSOR));expect_zero();
        PASS("AUTO-2F smaller navigation distances never bypass missing echo, old ultrasound or missing snapshot Safety faults");
    }
#endif
}
