/* Synthetic stimuli, real planner/navigation/FSM/Motion/MotorControl. */
#if NAV_AUTO2_ENABLED
static uint32_t ex_sequence;
static bool ex_peer_up=true;
static bool ex_communication(void){
    uint32_t ack=explore.link_issued?explore.link_query:0U;
    if(++ex_sequence==0U)++ex_sequence;
    return Navigation_ExploreLinkQuery(12345U,ex_sequence,ack,clock_tick);
}
static void ex_begin(void){
    setup();robot_state=ROBOT_STATE_IDLE;Odometry_Init(clock_tick);b73_fresh();ex_sequence=0;ex_peer_up=true;
    CHECK(!TaskControlRobot_RequestAutoProfileAt(clock_tick,NAV_PROFILE_EXPLORE));
    CHECK(TaskControlRobot_RequestExploreAt(clock_tick,12345U));run_fsm(1);
    CHECK(navigation.profile==NAV_PROFILE_EXPLORE && explore.key==12345U);
    a1_sample(0,0,1U);b73_step();CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);
}
static void ex_sample(int left,int right,unsigned dt,float front,float rear){
    a1_sample(left,right,dt);a2_data(front,rear);
    if(ex_peer_up)CHECK(ex_communication());
    Odometry_Observe(0,front,clock_tick,true);Odometry_Observe(1,rear,clock_tick,true);b73_step();
}
static void ex_settle(float front,float rear){
    for(unsigned i=0;i<4U && navigation.state==NAV_DETENER_Y_ESPERAR;++i){ex_sample(0,0,20U,front,rear);expect_zero();}
    CHECK(navigation.state==navigation.settle_target);
    if(navigation.state==NAV_EXPLORAR){
        ex_sample(0,0,20U,front,rear);
        if(navigation.state==NAV_DETENER_Y_ESPERAR && navigation.settle_target!=NAV_EXPLORAR)ex_settle(front,rear);
    }
}
static void ex_turn(float front,float rear){
    if(navigation.state==NAV_EXPLORAR)ex_sample(0,0,20U,front,rear);
    if(navigation.state==NAV_DETENER_Y_ESPERAR)ex_settle(front,rear);
    CHECK(navigation.state==NAV_GIRAR_ACOTADO);
    bool right=explore.right;
    ex_sample(0,0,20U,front,rear);CHECK(target_pending);
    CHECK(queued_target.left_speed_cps==(right?1200:-1200) && queued_target.right_speed_cps==(right?-1200:1200));
    double mm=explore.turn_target_deg*0.01745329252*ODOM_TRACK_M*500.0;
    int l=(int)ceil(mm/navigation_odometry.left_mm_per_count),r=(int)ceil(mm/navigation_odometry.right_mm_per_count);
    ex_sample(right?l:-l,right?-r:r,20U,front,rear);
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);expect_zero();ex_settle(front,rear);
}
/* Physical report reproduced with synthetic counts and current geometry.
 * No test-only change to production references or conversion factors. */
static void ex_diagnostic_turn(void){
    ex_begin();ex_settle(.31f,.8f);CHECK(navigation.state==NAV_GIRAR_ACOTADO && !explore.right);
    ex_sample(0,0,20U,.31f,.8f);CHECK(target_pending && navigation_odometry.ready);
    ex_peer_up=false; /* This short trace remains well inside the independent link timeout. */
}
static void ex_turn_counts(unsigned left,unsigned right,unsigned steps){
    unsigned previous_left=0,previous_right=0;
    for(unsigned i=1U;i<=steps;++i){
        unsigned l=left*i/steps,r=right*i/steps;
        ex_sample(-(int)(l-previous_left),(int)(r-previous_right),20U,.31f,.8f);
        previous_left=l;previous_right=r;
    }
}
static NavigationState_t ex_cut_phase;
static unsigned ex_cut_kind;
static bool ex_cut,ex_near,ex_pwm_seen,ex_zero_seen;
static TickType_t ex_stopped_tick,ex_last_link;
static TaskMotorControlTarget_t ex_older_target;
static void ex_auto_motor_schedule(void){
    if(delay_calls==3U){
        CHECK(TaskControlRobot_RequestExploreAt(clock_tick,12345U));TaskControlRobotEventMessage_t m;
        CHECK(xQueueReceive(event_queue,&m,0U)==pdTRUE);TaskControlRobot_ProcessEventAt(m.event,&m);
    }
    a2_data(ex_near?.31f:.8f,ex_near?.32f:.8f);
    OdometrySnapshot o;Odometry_GetSnapshot(&o);
    if(delay_calls==1U || delay_calls%14U==0U)Odometry_Observe(0,input.distancia_m,o.tick,true);
    if(delay_calls==1U || delay_calls%14U==7U)Odometry_Observe(1,input.distancia_trasera_m,o.tick,true);
    /* Match the real 140-ms/70-ms alternating producer metadata. */
    OdometryRay f,r;Odometry_GetRay(0,&f);Odometry_GetRay(1,&r);
    input.timestamp_ultrasonido_frontal=f.tick;input.timestamp_ultrasonido_trasero=r.tick;
    if(pwm.compare[0] || pwm.compare[1])ex_pwm_seen=true;
    if(!ex_cut && navigation.state==ex_cut_phase && (pwm.compare[0] || pwm.compare[1])){
        ex_cut=true;ex_older_target=queued_target;
        if(ex_cut_kind==0U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
        else if(ex_cut_kind==1U)input.hay_caida=1;
    }
    if(navigation.active && !(ex_cut && ex_cut_kind==2U) && (!explore.link_ready || delay_calls%50U==0U)){
        CHECK(ex_communication());ex_last_link=explore.link_tick;
    }
    if(delay_calls%2U)b73_step();
    if(ex_cut && !navigation.active){
        if(!ex_zero_seen){ex_zero_seen=true;ex_stopped_tick=clock_tick;}
        expect_zero();
    }
    left_encoder_delta=right_encoder_delta=0;
    if(task_context.left_target_cps==1600 && task_context.right_target_cps==1600)left_encoder_delta=right_encoder_delta=8;
    else if(task_context.left_target_cps==-1600 && task_context.right_target_cps==-1600)left_encoder_delta=right_encoder_delta=-7;
    else if(task_context.left_target_cps==-1200 && task_context.right_target_cps==1200){left_encoder_delta=-8;right_encoder_delta=8;}
    else if(task_context.left_target_cps==1200 && task_context.right_target_cps==-1200){left_encoder_delta=8;right_encoder_delta=-8;}
}
#endif
static void test_local_exploration(void){
#if NAV_AUTO2_ENABLED
    LocalEvidence e={0};Local_Reset(&e,0,0);uint32_t evaluated,repeated;
    CHECK(Local_Select(&e,0,0,0,&evaluated,&repeated)==-1);
    for(unsigned i=0;i<36U;++i)CHECK(e.state[i]==0U);
    PASS("LOCAL unknown/no-echo sectors never become free or selectable");
    for(unsigned i=0;i<3U;++i){OdometryRay ray={.sequence=i+1,.tick=clock_tick,.yaw_rad=(i?i==1?0.1745329252f:-0.1745329252f:0),.distance_m=.8f,.state=3};CHECK(Local_Observe(&e,0,&ray,clock_tick));}
    CHECK(Local_Select(&e,0,0,0,&evaluated,&repeated)==0 && evaluated==1U);
    CHECK(Local_Visit(&e,0,0,0));CHECK(Local_Select(&e,0,0,0,&evaluated,&repeated)==-1 && repeated==1U);
    PASS("LOCAL measured adjacent sectors select a low-turn corridor and visited bearing rejects a cycle without trusting unknown areas");
    LocalEvidence narrow={0};Local_Reset(&narrow,0,0);
    for(unsigned i=0;i<3U;++i){OdometryRay n={.sequence=i+1U,.tick=clock_tick,.yaw_rad=((int)i-1)*0.1745329252f,.distance_m=.4f,.state=3};CHECK(Local_Observe(&narrow,0,&n,clock_tick));}
    CHECK(Local_Select(&narrow,0,0,0,&evaluated,&repeated)==-1);
    for(unsigned i=0;i<2U;++i){OdometryRay n={.sequence=i+4U,.tick=clock_tick,.yaw_rad=(i?2:-2)*0.1745329252f,.distance_m=.4f,.state=3};CHECK(Local_Observe(&narrow,0,&n,clock_tick));}
    CHECK(Local_Select(&narrow,0,0,0,&evaluated,&repeated)==0);
    PASS("LOCAL nearer 400-mm corridors require wider contiguous measured valleys for the nominal body and margin; three distant 800-mm bins remain sufficient for selection only");
    OdometryRay ray={.sequence=1,.tick=clock_tick,.yaw_rad=3.14159265358979323846f,.distance_m=.8f,.state=3};
    CHECK(Local_Observe(&e,1,&ray,clock_tick) && e.state[18]==3U);
    CHECK(!Local_Observe(&e,1,&ray,clock_tick));ray.sequence=0;CHECK(!Local_Observe(&e,1,&ray,clock_tick));
    ray.sequence=2;ray.state=1;CHECK(!Local_Observe(&e,1,&ray,clock_tick));
    ray.state=3;ray.tick=clock_tick-201U;CHECK(!Local_Observe(&e,1,&ray,clock_tick));
    ray.tick=clock_tick;ray.distance_m=NAN;CHECK(!Local_Observe(&e,1,&ray,clock_tick));
    PASS("LOCAL front/rear bearings remain independent, with sequence, acquisition time, validity and numeric guards");
    ray=(OdometryRay){.sequence=10,.tick=clock_tick,.distance_m=.3f,.state=3};CHECK(Local_Observe(&e,0,&ray,clock_tick));
    CHECK(e.state[0]==2U && e.state[1]==2U && e.state[35]==2U);
    ray.sequence++;ray.distance_m=.8f;CHECK(Local_Observe(&e,0,&ray,clock_tick));CHECK(e.state[0]==2U);
    Local_Expire(&e,clock_tick+4001U);CHECK(e.state[0]==0U);
    PASS("LOCAL nearby returns inflate chassis and acoustic uncertainty; a distant echo cannot erase a fresh conflicting occupied cone");
    memset(&e,0,sizeof e);ray=(OdometryRay){.sequence=UINT32_MAX,.tick=UINT32_MAX-10U,.distance_m=.8f,.state=3};
    CHECK(Local_Observe(&e,0,&ray,5U));ray.sequence=0;ray.tick=5;CHECK(Local_Observe(&e,0,&ray,5U));
    CHECK(sizeof e<1200U);for(unsigned i=0;i<200U;++i)(void)Local_Visit(&e,i*.06f,0,0);CHECK(e.visit_count==32U);
    PASS("LOCAL producer sequence/tick wrap and bounded sector/history memory remain independent of session duration");
    memset(&e,0,sizeof e);CHECK(Local_Visit(&e,0,0,0));
    CHECK(Local_Visited(&e,-.02f,0,0) && !Local_Visit(&e,-.02f,0,0));
    CHECK(Local_RetreatVisit(&e,0,0,0));
    CHECK(!Local_RetreatAllowed(&e,0,0,0) && !Local_RetreatAllowed(&e,-.009f,0,0));
    CHECK(Local_RetreatAllowed(&e,-LOCAL_RETREAT_PROGRESS_M,0,0));
    CHECK(!Local_RetreatAllowed(&e,0,.02f,0) && !Local_RetreatAllowed(&e,.02f,0,0));
    for(unsigned i=1;i<=4U;++i)CHECK(Local_RetreatVisit(&e,-.02f*i,0,0));
    CHECK(Local_Visited(&e,-.08f,0,0)); /* The general 50-mm guard was not reduced or bypassed for forward. */
    CHECK(!Local_RetreatAllowed(&e,-.06f,0,3.14159265358979323846f));
    CHECK(!Local_RetreatAllowed(&e,-.02f,0,0));
    CHECK(!Local_RetreatAllowed(&e,NAN,0,0));
    PASS("LOCAL retreat origins allow monotonic measured withdrawal, not zero/lateral/forward progress or a return toward previous origins; general 50-mm bearing history remains intact");
    memset(&e,0,sizeof e);
    CHECK(Local_RetreatVisit(&e,0,0,0));CHECK(Local_RetreatVisit(&e,-.08f,0,0));
    CHECK(Local_RetreatVisit(&e,-.08f,-.08f,1.5707963268f));CHECK(Local_RetreatVisit(&e,0,-.08f,3.1415926536f));
    CHECK(!Local_RetreatAllowed(&e,0,0,-1.5707963268f));
    Local_Reset(&e,0,0);CHECK(!Local_RetreatAllowed(&e,0,0,-1.5707963268f));
    memset(&e,0,sizeof e);
    for(unsigned i=0;i<200U;++i)CHECK(Local_RetreatVisit(&e,-.02f*i,0,0));
    CHECK(e.visit_count==LOCAL_VISITS && sizeof e<1200U);
    PASS("LOCAL withdrawal cycle history survives evidence resets, rejects a closed positional loop at a different heading and remains bounded over many successful moves");
    for(unsigned residual=0;residual<3U;++residual){
        ex_begin();TickType_t first_front=clock_tick;
        for(unsigned i=0;i<7U;++i){
            a1_sample(0,0,20U);a2_data(.682f,.8f);
            CHECK(ex_communication());
            if(i==0U){first_front=clock_tick;Odometry_Observe(0,.682f,first_front,true);}
            input.timestamp_ultrasonido_frontal=first_front;
            Odometry_Observe(1,.8f,clock_tick,true);b73_step();expect_zero();
        }
        CHECK(navigation.state==NAV_EXPLORAR && explore.front_confirmations==1U && explore.turns==0U);
        float next_range=residual==2U?.8f:.682f;
        a1_sample(residual==1U?3:0,residual==1U?3:0,20U);a2_data(next_range,.8f);
        CHECK(ex_communication());
        Odometry_Observe(0,next_range,clock_tick,true);Odometry_Observe(1,.8f,clock_tick,true);b73_step();
        if(residual==2U){
            CHECK(explore.front_confirmations==1U && navigation.state==NAV_EXPLORAR && explore.segments==0U);expect_zero();
            ex_sample(0,0,20U,.8f,.8f);
        }
        CHECK(explore.front_confirmations==2U && explore.turns==0U && explore.segments==1U);expect_zero();
        CHECK(navigation.state==(residual==1U?NAV_DETENER_Y_ESPERAR:NAV_AVANZAR));
        CHECK(!target_pending && !navigation.emission_ready);
        PASS("LOCAL 682-mm frontal evidence waits stopped for two distinct captures 140 ms apart, skips a redundant stop only with unchanged encoders and still requires a posterior motion reference");
    }
    for(unsigned mm=349U;mm<=351U;++mm){
        ex_begin();ex_settle((float)mm/1000.0f,.8f);
        CHECK(mm>=350U?navigation.state==NAV_AVANZAR:navigation.state==NAV_GIRAR_ACOTADO);
        expect_zero();
    }
    PASS("LOCAL frontal planning boundary at 350 mm preserves the 275+50+25 margin; shorter clearance triggers exploration rather than direct advance");
    ex_begin();
    for(unsigned i=0;i<4U;++i){a1_sample(0,0,20U);a2_data(.8f,.8f);b73_step();expect_zero();}
    CHECK(navigation.state==NAV_EXPLORAR && !explore.link_ready);
    a1_sample(0,0,20U);b73_step();CHECK(navigation.state==NAV_EXPLORAR);expect_zero();
    uint32_t c=1U;
    CHECK(!Navigation_ExploreRenew(1U,++ex_sequence,0U,clock_tick));
    CHECK(!Navigation_ExploreRenew(12345U,++ex_sequence,c,clock_tick));
    CHECK(ex_communication());CHECK(!explore.link_ready);CHECK(ex_communication());
    CHECK(navigation.state==NAV_EXPLORAR && !navigation.emission_ready);
    CHECK(!Navigation_ExploreRenew(12345U,ex_sequence,0U,clock_tick));
    ex_sample(0,0,20U,.8f,.8f);CHECK(navigation.state==NAV_EXPLORAR);expect_zero();
    ex_sample(0,0,20U,.8f,.8f);CHECK(navigation.state==NAV_AVANZAR);
    CHECK(navigation_settle.window_observations>=4U);
    PASS("LOCAL explicit session consent and an independently proven internal link enable autonomous decisions; all old navigation renewals are rejected");
    ex_sample(0,0,20U,.8f,.8f);CHECK(target_pending);
    ex_sample(900,892,20U,.8f,.8f);
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && !TaskSafety_IsStopActive() && explore.turns==0U);expect_zero();
    PASS("LOCAL independent frontal echoes prioritize a bounded straight segment without an initial scan; motor and completion guards remain intact");
    ex_begin();CHECK(ex_communication());CHECK(ex_communication());ex_peer_up=false;
    ex_settle(.31f,.8f);ex_turn(.31f,.8f);ex_turn(.31f,.8f);ex_turn(.8f,.8f);
    CHECK(navigation.state==NAV_AVANZAR && explore.turns==3U && explore.link_ack==1U);
    ex_sample(0,0,20U,.8f,.8f);ex_sample(900,892,20U,.8f,.8f);expect_zero();
    CHECK(navigation.active && explore.link_ack==1U && (TickType_t)(clock_tick-explore.link_tick)<LOCAL_LINK_TIMEOUT_MS);ex_peer_up=true;
    PASS("LOCAL blocked frontal direction triggers a two-sensor scan; changed clear frontal evidence permits forward recovery under the same live session authorization");
    for(unsigned failure=0;failure<4U;++failure){
        ex_begin();ex_settle(.8f,.8f);
        if(failure==0U){CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);}
        else if(failure==1U){input.hay_caida=1;b73_step();}
        else if(failure==2U){input.ultrasonido_valido=0;b73_step();}
        else{input.distancia_m=.2f;b73_step();}
        CHECK(!navigation.active);expect_zero();
        CHECK(!Navigation_ExploreRenew(12345U,++ex_sequence,c,clock_tick));
        PASS("LOCAL STOP, floor loss, invalid echo and critical 200-mm proximity cancel grants without automatic rearm or restart");
    }
    ex_begin();input.distancia_m=.2f;b73_step();run_safety(1);run_fsm(1);
    CHECK(robot_state==ROBOT_STATE_SAFETY_STOP && TaskSafety_GetFaults()==SAFETY_FAULT_OBSTACLE);
    clock_tick+=100U;b73_fresh();CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_SAFETY_RESET));run_fsm(1);
    CHECK(robot_state==ROBOT_STATE_IDLE && !TaskSafety_IsStopActive() && !navigation.active);
    CHECK(explore.critical_faults & SAFETY_FAULT_OBSTACLE);CHECK(!Navigation_ExploreRenew(12345U,++ex_sequence,0U,clock_tick));
    PASS("LOCAL explicit successful Safety rearm clears only current faults, retains critical session history and cannot restore old lease or motion");
    ex_begin();ex_settle(.8f,.8f);clock_tick=explore.link_tick+LOCAL_LINK_TIMEOUT_MS;s_odom.tick=clock_tick;b73_fresh();
    CHECK(!Navigation_ExploreRenew(12345U,++ex_sequence,0U,clock_tick));
    CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));b73_step();a1_terminal(NAV_FINISH_INTERNAL_LINK_LOST);
    PASS("LOCAL missing internal ESP32 round-trip communication expires in STM32 and cannot be revived by an old or late communication; browser Wi-Fi is independent");
    ex_begin();ex_settle(.31f,.8f);CHECK(navigation.state==NAV_GIRAR_ACOTADO);
    ex_sample(0,0,20U,.31f,.8f);
    for(unsigned i=0;i<10U && navigation.state==NAV_GIRAR_ACOTADO;++i)ex_sample(0,0,20U,.31f,.8f);
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && explore.block==10U);expect_zero();
    PASS("LOCAL no angular progress ends the attempt in stopped stabilization instead of renewing a stalled turn forever");
    ex_begin();ex_settle(.26f,.8f);
    for(unsigned retreat=0;retreat<4U;++retreat){
        CHECK(navigation.state==NAV_RETROCEDER_ACOTADO && explore.retreats==retreat+1U);
        ex_sample(0,0,20U,.26f,.8f);CHECK(target_pending && queued_target.left_speed_cps==-1600 && queued_target.right_speed_cps==-1600);
        ex_sample(-360,-357,20U,.26f,.8f);
        CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && navigation_diagnostics.flags&NAV_DIAG_REVERSE_COMPLETED);expect_zero();
        if(retreat<3U)ex_settle(.26f,.8f);
    }
    CHECK(s_odom.x_m<-.079f && explore.retreats==4U && !TaskSafety_IsStopActive());
    ex_settle(.31f,.8f);CHECK(explore.turns>0U && navigation.state==NAV_GIRAR_ACOTADO);
    PASS("LOCAL four measured 20-mm retreats can continue inside the 50-mm general visit radius with fresh rear clearance and a new stabilization/reference per move");
    PASS("LOCAL fresh evidence after successive withdrawals reopens angular exploration instead of treating the former forward visit as a retreat veto");
    for(unsigned mm=249U;mm<=251U;++mm){
        ex_begin();ex_settle(.26f,mm*.001f);
        if(mm<=250U){CHECK(navigation.state==NAV_EXPLORAR && explore.retreats==0U && explore.action==0U);expect_zero();}
        else CHECK(navigation.state==NAV_RETROCEDER_ACOTADO && explore.retreats==1U);
    }
    PASS("LOCAL withdrawal still requires strictly more than 250-mm recent rear clearance at the unchanged boundary");
    for(unsigned failure=0;failure<5U;++failure){
        ex_begin();ex_settle(.26f,.8f);
        for(unsigned retreat=0;retreat<2U;++retreat){
            ex_sample(0,0,20U,.26f,.8f);ex_sample(-360,-357,20U,.26f,.8f);ex_settle(.26f,.8f);
        }
        CHECK(explore.retreats==3U && navigation.state==NAV_RETROCEDER_ACOTADO);
        ex_sample(0,0,20U,.26f,.8f);CHECK(target_pending);
        if(failure==0U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
        if(failure==1U)input.hay_caida=1;
        if(failure==2U)input.ultrasonido_trasero_valido=0;
        if(failure==3U)input.timestamp_ultrasonido_trasero=clock_tick-1000U;
        if(failure==4U)input.distancia_trasera_m=.249f;
        b73_step();expect_zero();
        if(failure<4U)CHECK(!navigation.active);
        else CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && explore.block==3U && !TaskSafety_IsStopActive());
        PASS("LOCAL STOP, cliff, invalid/stale rear or insufficient clearance cancel a later authorized retreat with no retained target");
    }
    ex_begin();ex_settle(.26f,.8f);ex_sample(0,0,20U,.26f,.8f);
    for(unsigned i=0;i<20U;++i)ex_sample(0,0,20U,.26f,.8f);
    CHECK(explore.retreats==1U && navigation.state==NAV_EXPLORAR && explore.action==0U && explore.block==7U);expect_zero();
    for(unsigned i=0;i<40U;++i)ex_sample(0,0,20U,.26f,.8f);
    CHECK(explore.retreats==1U && navigation.active);expect_zero();
    PASS("LOCAL rear clearance and renewed supervision cannot repeat a withdrawal without actual odometric progress");
    ex_begin();ex_settle(.31f,.32f);
    while(explore.action>=2U && explore.action<=4U)ex_turn(.31f,.32f);
    CHECK(explore.action==5U && explore.scan_deg>160.0f && explore.scan_deg<=180.0f);
    ex_sample(0,0,20U,.31f,.32f);ex_settle(.31f,.32f);CHECK(navigation.state==NAV_RETROCEDER_ACOTADO);
    ex_sample(0,0,20U,.31f,.32f);ex_sample(-360,-357,20U,.33f,.30f);CHECK(navigation.state==NAV_DETENER_Y_ESPERAR);
    ex_settle(.33f,.30f);CHECK(explore.scan_deg==0 && explore.retreated);
    while(explore.action>=2U && explore.action<=4U)ex_turn(.33f,.30f);
    CHECK(navigation.state==NAV_EXPLORAR && explore.action==0 && (explore.block==6U || explore.block==7U) && !TaskSafety_IsStopActive());expect_zero();
    CHECK(explore.retreats==1U); /* A second half-turn would withdraw toward the previous origin, not explore a new position. */
    uint32_t turns=explore.turns;
    for(unsigned i=0;i<30U;++i)ex_sample(0,0,20U,.33f,.30f);
    CHECK(explore.turns==turns && explore.action==0);
    for(unsigned i=0;i<6U;++i)ex_sample(0,0,20U,i%2U?.33f:.8f,i%2U?.30f:.8f);
    CHECK(explore.action==0 && explore.turns==turns);
    for(unsigned i=0;i<3U;++i){
        ex_sample(0,0,20U,.8f,.8f);
        if(i<2U){CHECK(explore.block==8U && navigation.state==NAV_EXPLORAR && explore.turns==turns);expect_zero();}
    }
    CHECK(explore.action==1U && navigation.active && navigation.state==NAV_DETENER_Y_ESPERAR && explore.turns==turns);
    PASS("LOCAL bounded progressive scan, verified retreat and second survey reject withdrawal back toward its origin, then wait for confirmed changed evidence without inventing a critical fault");
    char wire[256];CHECK(Navigation_FormatExplore(wire,sizeof wire,91U,0)>0);printf("LOCAL_BASE_WIRE %s",wire);
    CHECK(Navigation_FormatExplore(wire,sizeof wire,91U,1)>0);printf("LOCAL_SECTORS_WIRE %s",wire);
    CHECK(Navigation_FormatExplore(wire,sizeof wire,90U,1)<0);
    turns=explore.turns;uint32_t sid=navigation_diagnostics.session_id;CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);
    CHECK(Navigation_FormatExplore(wire,sizeof wire,92U,0)>0 && navigation_diagnostics.session_id==sid && explore.turns==turns);
    PASS("LOCAL bounded telemetry freezes matching sector parts and retains prior maneuvers and reason after STOP");
    for(unsigned phase=0;phase<4U;++phase)for(unsigned failure=0;failure<4U;++failure){
        ex_begin();
        if(phase==1U){for(unsigned i=0;i<4U;++i){a1_sample(0,0,20U);b73_step();}CHECK(navigation.state==NAV_EXPLORAR);}
        else if(phase!=0U)ex_settle(.8f,.8f);
        if(phase==2U){
            /* Explicit blocked-front stimulus is needed now that clear front no longer scans. */
            Navigation_Cancel();ex_begin();ex_settle(.31f,.8f);ex_sample(0,0,20U,.31f,.8f);CHECK(navigation.state==NAV_GIRAR_ACOTADO);
        }
        if(phase==3U){CHECK(navigation.state==NAV_AVANZAR);ex_sample(0,0,20U,.8f,.8f);}
        if(failure==0U)CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
        if(failure==1U)input.hay_caida=1;
        if(failure==2U)input.ultrasonido_trasero_valido=0;
        if(failure==3U)input.distancia_trasera_m=.2f;
        b73_step();CHECK(!navigation.active);expect_zero();
        if(failure)CHECK(TaskSafety_IsStopActive());
        CHECK(!Navigation_ExploreRenew(12345U,++ex_sequence,0U,clock_tick));
        PASS("LOCAL STOP/cliff/no-echo/critical rear preserve priority in stabilization, wait, turn and forward with no stale reactivation");
    }
    ex_begin();ex_settle(.23f,.23f);CHECK(explore.action==0U && explore.block==6U);expect_zero();CHECK(!TaskSafety_IsStopActive());
    PASS("LOCAL both directions inhibited above the unchanged critical threshold means stopped wait, not a fictitious hardware fault");
    ex_begin();ex_settle(.27f,.5f);CHECK(explore.action==5U);ex_sample(0,0,20U,.27f,.5f);ex_settle(.27f,.5f);
    ex_sample(0,0,20U,.27f,.5f);ex_sample(-10,-10,20U,.27f,.249f);
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && explore.block==3U);expect_zero();CHECK(!TaskSafety_IsStopActive());
    PASS("LOCAL rear clearance loss cancels an authorized retreat before further negative emission without changing Safety thresholds");
    ex_begin();ex_settle(.8f,.8f);ex_sample(0,0,20U,.8f,.8f);ex_settle(.8f,.8f);ex_sample(0,0,20U,.8f,.8f);
    ex_sample(10,-10,20U,.8f,.8f);a1_terminal(NAV_FINISH_ODOMETRY);CHECK(navigation_diagnostics.odometry_error==NAV_ODOM_DIRECTION);
    PASS("LOCAL wrong-sign encoder increments still stop the session rather than being hidden by a search recovery");
    ex_begin();ex_settle(.8f,.8f);CHECK(navigation.state==NAV_AVANZAR);
    ex_sample(0,0,20U,.8f,.8f);bool hit=false;
    for(unsigned i=0;i<151U && navigation.state==NAV_AVANZAR;++i){
        a1_sample(4,4,20U);a2_data(.8f,.8f);CHECK(ex_communication());
        if((TickType_t)(clock_tick-navigation.phase_started_tick)>=3000U){
            CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));CHECK(Navigation_CompleteCommand(navigation.session,clock_tick));hit=true;
        }
        b73_step();
    }
    CHECK(hit && navigation.active && navigation.state==NAV_DETENER_Y_ESPERAR && explore.block==9U);expect_zero();
    PASS("LOCAL per-motion timeout is enforced at consumption, clears motors and returns to stopped evaluation without renewing a stalled command");
    ex_begin();ex_settle(.8f,.8f);
    for(unsigned segment=0;segment<12U;++segment){
        unsigned guard=0;
        while(navigation.state!=NAV_AVANZAR){
            CHECK(++guard<12U);
            if(navigation.state==NAV_DETENER_Y_ESPERAR)ex_settle(.8f,.8f);
            else if(explore.action>1U)ex_turn(.8f,.8f);
            else ex_sample(0,0,20U,.8f,.8f);
        }
        ex_sample(0,0,20U,.8f,.8f);
        while(navigation.state==NAV_AVANZAR)ex_sample(16,16,20U,.8f,.8f);
        CHECK(navigation.active && navigation.state==NAV_DETENER_Y_ESPERAR);
        ex_settle(.8f,.8f);
    }
    CHECK(explore.segments>=12U && explore.turns==0U && (TickType_t)(clock_tick-navigation.started_tick)>12000U);
    CHECK(!TaskSafety_IsStopActive());
    PASS("LOCAL more than four real segments, 500 mm and 12 seconds without unnecessary turns proceed only with fresh supervision and bounded individual moves");
    setup();robot_state=ROBOT_STATE_IDLE;ex_sequence=0;ex_cut=false;ex_cut_phase=NAV_DETENIDO;ex_cut_kind=0;
    ex_near=false;ex_pwm_seen=ex_zero_seen=false;ex_stopped_tick=ex_last_link=0;motor_cycle_hook=ex_auto_motor_schedule;
    run_motor(2400);motor_cycle_hook=NULL;
    CHECK(navigation.active && !ex_cut && ex_pwm_seen && !TaskSafety_IsStopActive());
    CHECK(explore.segments>4U && explore.turns==0U && (TickType_t)(clock_tick-navigation.started_tick)>12000U);
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);expect_zero();
    PASS("LOCAL healthy continuous MotorControl execution with 140/70-ms sonars and 500-ms diagnostic round trips autonomously advances without scans for over 12 seconds without intermediate approvals");
    const NavigationState_t cut_phases[]={NAV_AVANZAR,NAV_GIRAR_ACOTADO,NAV_RETROCEDER_ACOTADO};
    for(unsigned phase=0;phase<3U;++phase)for(unsigned failure=0;failure<3U;++failure){
        setup();robot_state=ROBOT_STATE_IDLE;ex_sequence=0;ex_cut=false;ex_cut_phase=cut_phases[phase];ex_cut_kind=failure;
        ex_near=phase!=0U;ex_pwm_seen=ex_zero_seen=false;ex_stopped_tick=ex_last_link=0;motor_cycle_hook=ex_auto_motor_schedule;
        run_motor(2400);motor_cycle_hook=NULL;
        CHECK(ex_cut && ex_pwm_seen && ex_zero_seen && !navigation.active);expect_zero();
        CHECK(navigation.finish_reason==(failure==0U?NAV_FINISH_OPERATOR_STOP:failure==1U?NAV_FINISH_SAFETY:NAV_FINISH_INTERNAL_LINK_LOST));
        if(failure==2U)CHECK((TickType_t)(ex_stopped_tick-ex_last_link)<=LOCAL_LINK_TIMEOUT_MS+20U);
        CHECK(!Navigation_ExploreRenew(12345U,++ex_sequence,0U,clock_tick));
        CHECK(xQueueOverwrite(target_speed_queue,&ex_older_target)==pdTRUE);run_motor(2);expect_zero();
        PASS("LOCAL real continuous MotorControl loop autonomously reaches each movement phase with 500-ms diagnostic round trips; STOP, cliff or lost internal communication clears applied PWM and rejects old targets");
    }
    ex_begin();ex_settle(.8f,.8f);TickType_t initial=navigation.started_tick;
    for(unsigned i=0;i<750U;++i)ex_sample(0,0,20U,.8f,.8f);
    CHECK(navigation.active && navigation.state==NAV_EXPLORAR && (TickType_t)(clock_tick-initial)>12000U);
    CHECK(explore.action==0U && (explore.block==6U || explore.block==7U));
    CHECK(explore.turns==2U && explore.retreats==1U); /* Both turn directions and one alternate position, then no progress. */
    expect_zero();PASS("LOCAL autonomous left/right/retreat attempts with zero encoder progress exhaust alternatives and wait without endless retries or per-maneuver confirmations");
    ex_begin();ex_peer_up=false;CHECK(Navigation_SessionLive(clock_tick));
    DrvUart_t uart;UART_HandleTypeDef peripheral={0};DrvUart_Init(&uart,&peripheral);
    uart_feed(&uart,"EXPLORATION Q=1 S=0 K=12345 A=0\n");CHECK(uart_consume(&uart));
    CHECK(!explore.link_ready && !strncmp(uart_response,"EXPL,1,1,",9));
    clock_tick+=20U;uart_feed(&uart,"EXPLORATION Q=2 S=0 K=12345 A=1\n");CHECK(uart_consume(&uart));
    CHECK(explore.link_ready && explore.link_ack==1U && explore.link_query==2U);
    TickType_t proof_tick=explore.link_tick;
    uart_feed(&uart,"EXPLORATION Q=3 S=0\n");CHECK(uart_consume(&uart));CHECK(explore.link_query==2U && explore.link_tick==proof_tick);
    uart_feed(&uart,"EXPLORE K=12345 N=9 P=0\n");CHECK(uart_consume(&uart));CHECK(!strcmp(uart_response,"EXPLORE_REJECTED\n"));
    const char *malformed[]={"EXPLORATION Q=4 S=0 K=12345 A=2 extra\n","EXPLORATION Q=4 S=1 K=12345 A=2\n","EXPLORATION Q=4 S=0 K=4294967296 A=2\n"};
    for(unsigned i=0;i<3U;++i){uart_feed(&uart,malformed[i]);CHECK(uart_consume(&uart));CHECK(!strcmp(uart_response,"ERR_UNKNOWN\n"));CHECK(explore.link_tick==proof_tick);}
    CHECK(!Navigation_ExploreLinkQuery(12345U,2U,1U,clock_tick));
    CHECK(!Navigation_ExploreLinkQuery(1U,4U,2U,clock_tick));
    CHECK(!Navigation_ExploreLinkQuery(12345U,4U,2U,clock_tick-300U));
    CHECK(explore.link_tick==proof_tick);expect_zero();
    PASS("LOCAL real UART query parsing proves a round trip separately from permission; legacy polls, renewals, duplicates, wrong nonce and stale/malformed requests cannot refresh the link");
    for(unsigned i=0;i<5U;++i){clock_tick+=500U;CHECK(Navigation_ExploreLinkQuery(12345U,10U+i,1U,clock_tick));CHECK(explore.link_tick==proof_tick);}
    clock_tick=proof_tick+LOCAL_LINK_TIMEOUT_MS;s_odom.tick=clock_tick;b73_fresh();
    CHECK(Navigation_SessionLive(clock_tick) && !Navigation_LinkLive(clock_tick));
    CHECK(!Navigation_ExploreLinkQuery(12345U,20U,14U,clock_tick));b73_step();a1_terminal(NAV_FINISH_INTERNAL_LINK_LOST);
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);
    CHECK(explore.stop_received && navigation_diagnostics.reason==NAV_FINISH_INTERNAL_LINK_LOST);
    CHECK(!Navigation_ExploreLinkQuery(12345U,21U,20U,clock_tick));expect_zero();
    CHECK(Navigation_FormatExplore(wire,sizeof wire,101U,0)>0);printf("LOCAL_LINK_BASE_WIRE %s",wire);
    CHECK(Navigation_FormatExplore(wire,sizeof wire,101U,1)>0);printf("LOCAL_LINK_DETAIL_WIRE %s",wire);
    PASS("LOCAL incoming requests alone do not prove the return link; expiry cancels with INTERNAL_LINK_LOST, survives STOP history and cannot be revived by late receipt acknowledgements");
    ex_begin();ex_peer_up=false;CHECK(Navigation_ExploreLinkQuery(12345U,1U,0U,clock_tick));
    clock_tick+=LOCAL_LINK_REPLY_MS;CHECK(Navigation_ExploreLinkQuery(12345U,2U,1U,clock_tick));CHECK(!explore.link_ready);
    clock_tick+=20U;CHECK(Navigation_ExploreLinkQuery(12345U,3U,2U,clock_tick));CHECK(explore.link_ready);
    proof_tick=explore.link_tick;CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
    CHECK(!Navigation_ExploreLinkQuery(12345U,4U,3U,clock_tick));CHECK(explore.link_tick==proof_tick);run_fsm(1);expect_zero();
    PASS("LOCAL a delayed diagnostic receipt cannot certify freshness; a genuinely new round trip can recover before timeout, while pending STOP always wins");
    ex_begin();clock_tick=UINT32_MAX-100U;Navigation_BeginExplore(clock_tick,12345U);
    CHECK(Navigation_ExploreLinkQuery(12345U,UINT32_MAX,0U,clock_tick));clock_tick=50U;
    CHECK(Navigation_ExploreLinkQuery(12345U,1U,UINT32_MAX,clock_tick) && explore.link_ready);
    clock_tick+=LOCAL_LINK_TIMEOUT_MS;CHECK(!Navigation_ExploreLinkQuery(12345U,2U,1U,clock_tick));
    PASS("LOCAL diagnostic query and FreeRTOS tick wrap preserve freshness and cannot resurrect an expired internal link");
    for(unsigned critical=0;critical<2U;++critical){
        ex_begin();ex_settle(.682f,.8f);clock_tick=explore.link_tick+LOCAL_LINK_TIMEOUT_MS;s_odom.tick=clock_tick;b73_fresh();
        if(critical)input.hay_caida=1;else CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
        b73_step();a1_terminal(critical?NAV_FINISH_SAFETY:NAV_FINISH_OPERATOR_STOP);expect_zero();
        PASS("LOCAL STOP or cliff concurrent with link timeout retain priority over navigation and cannot be bypassed by diagnostic traffic");
    }
    ex_diagnostic_turn();ex_turn_counts(246U,191U,24U);
    CHECK(navigation.state==NAV_GIRAR_ACOTADO && navigation_diagnostics.turn_mdeg==9370U);
    CHECK(navigation_odometry.progress_mm>12.19 && navigation_odometry.progress_mm<12.21);
    int64_t reference_left=navigation_odometry.left,reference_right=navigation_odometry.right;
    TickType_t reference_tick=navigation.phase_started_tick;
    a1_sample(-5,4,20U);a2_data(.31f,.8f);
    CHECK((TickType_t)(clock_tick-reference_tick)==520U && Navigation_LinkLive(clock_tick));
    CHECK(Navigation_IsCommandCurrent(navigation.session,clock_tick)); /* Real backend authorization path. */
    b73_step();CHECK(navigation.state==NAV_GIRAR_ACOTADO && navigation_diagnostics.turn_mdeg==9563U);
    CHECK(navigation_diagnostics.odometry_error==NAV_ODOM_OK && !TaskSafety_GetFaults());
    OdometrySnapshot accepted_pose;Odometry_GetSnapshot(&accepted_pose);
    CHECK(accepted_pose.state==2U && fabsf(accepted_pose.theta_rad-9.5625247f*.01745329252f)<.00001f);
    double left_travel=251.0*navigation_odometry.left_mm_per_count,right_travel=195.0*navigation_odometry.right_mm_per_count;
    CHECK(left_travel-right_travel>3.0 && left_travel-right_travel<3.011 && Navigation_WheelToleranceMm(left_travel,right_travel)==4.0);
    CHECK(navigation_odometry.left==reference_left && navigation_odometry.right==reference_right && navigation.phase_started_tick==reference_tick);
    for(unsigned i=0;i<3U;++i)ex_sample(-5,4,20U,.31f,.8f);
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && navigation_diagnostics.turn_flags&8U && navigation_diagnostics.turn_mdeg>=10000U);expect_zero();
    CHECK(navigation_diagnostics.turn_elapsed_ms==580U);
    printf("LOCAL_TURN_CORRECTION_REPORT L_MM=%.9f R_MM=%.9f DIFFERENCE_MM=%.9f ACCEPTED_DEG=9.563 COMPLETED_DEG=%.3f PHASE_MS=580\n",left_travel,right_travel,left_travel-right_travel,navigation_diagnostics.turn_mdeg*.001);
    PASS("LOCAL real -251/+195 report remains authorized at 520 ms and completes the remaining angle at 580 ms, without restarting reference, target or deadline");
    ex_diagnostic_turn();ex_turn_counts(268U,195U,24U);
    CHECK(navigation.state==NAV_GIRAR_ACOTADO && Navigation_IsCommandCurrent(navigation.session,clock_tick));
    TaskMotorControlTarget_t before_disagreement=queued_target;
    a1_sample(-1,0,20U);a2_data(.31f,.8f);
    CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));
    CHECK(navigation_diagnostics.odometry_error==NAV_ODOM_DISAGREEMENT && navigation_diagnostics.left_delta==-269 && navigation_diagnostics.right_delta==195);
    b73_step();a1_terminal(NAV_FINISH_ODOMETRY);CHECK(!(navigation_diagnostics.turn_flags&8U));
    PASS("LOCAL neighboring real counts bracket the 4-mm bilateral guard: -268/+195 allowed, -269/+195 rejected by the shared backend and planner criterion");
    /* A later balanced pair cannot erase an out-of-budget discrepancy. */
    a1_sample(0,70,20U);a2_data(.31f,.8f);b73_step();expect_zero();
    CHECK(!navigation.active && navigation.finish_reason==NAV_FINISH_ODOMETRY && navigation_diagnostics.left_delta==-269);
    CHECK(xQueueOverwrite(target_speed_queue,&before_disagreement)==pdTRUE);run_motor(2);expect_zero();
    PASS("LOCAL a later matching pair cannot reset references, revive a terminal turn or consume an old queued target after a genuine discrepancy");
    ex_diagnostic_turn();ex_turn_counts(268U,195U,24U);
    ex_sample(-3,1,20U,.31f,.8f);a1_terminal(NAV_FINISH_ODOMETRY);
    CHECK(navigation_diagnostics.odometry_error==NAV_ODOM_DISAGREEMENT && !(navigation_diagnostics.turn_flags&8U));
    PASS("LOCAL exceeding the 10-degree mean travel cannot bypass the 4-mm guard: invalid turn is rejected before target completion");
    ex_diagnostic_turn();
    CHECK(Navigation_WheelToleranceMm(2.99,4.0)==3.0 && Navigation_WheelToleranceMm(4.0,2.99)==3.0);
    CHECK(Navigation_WheelToleranceMm(3.0,3.0)==4.0);
    for(unsigned profile=NAV_PROFILE_TURN;profile<=NAV_PROFILE_CONTINUOUS;++profile){
        navigation.profile=profile;CHECK(Navigation_WheelToleranceMm(12.0,15.5)==3.0);
    }
    navigation.profile=NAV_PROFILE_EXPLORE;
    navigation.state=NAV_AVANZAR;CHECK(Navigation_WheelToleranceMm(12.0,15.5)==10.0);
    navigation.state=NAV_RETROCEDER_ACOTADO;CHECK(Navigation_WheelToleranceMm(12.0,15.5)==10.0);
    PASS("LOCAL tolerance is restricted to bilateral turn progress; historic turn profiles remain 3 mm and straight motions remain 10 mm");
    ex_diagnostic_turn();ex_turn_counts(55U,55U,4U);
    CHECK(navigation.state==NAV_GIRAR_ACOTADO);
    ex_turn_counts(73U,0U,4U);a1_terminal(NAV_FINISH_ODOMETRY);
    CHECK(navigation_diagnostics.odometry_error==NAV_ODOM_DISAGREEMENT);
    PASS("LOCAL an encoder or wheel that stops after bilateral startup is still rejected when the cumulative difference exceeds 4 mm");
    for(unsigned critical=0;critical<2U;++critical){
        ex_diagnostic_turn();ex_turn_counts(246U,191U,24U);a1_sample(-5,4,20U);a2_data(.31f,.8f);
        if(critical)input.hay_caida=1;else CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));
        b73_step();a1_terminal(critical?NAV_FINISH_SAFETY:NAV_FINISH_OPERATOR_STOP);expect_zero();
        PASS("LOCAL the newly tolerated asymmetry cannot override STOP or cliff Safety");
    }
    for(unsigned bad=0;bad<4U;++bad){
        ex_diagnostic_turn();ex_turn_counts(251U,195U,25U);
        CHECK(navigation.state==NAV_GIRAR_ACOTADO);
        a1_sample(bad==0U?1:0,0,20U);a2_data(.31f,.8f);
        if(bad==1U)s_odom.tick=clock_tick-51U;
        if(bad==2U)s_odom.state=3U;
        if(bad==3U)s_odom.left_count=INT64_MIN;
        CHECK(!Navigation_IsCommandCurrent(navigation.session,clock_tick));
        b73_step();a1_terminal(NAV_FINISH_ODOMETRY);
        CHECK(navigation_diagnostics.odometry_error!=NAV_ODOM_OK);
        PASS("LOCAL tolerating a small bilateral asymmetry never bypasses wrong sign, stale/invalid odometry or extreme encoder counts");
    }
    ex_diagnostic_turn();ex_turn_counts(150U,100U,10U);
    CHECK(navigation.state==NAV_GIRAR_ACOTADO);ex_turn_counts(40U,90U,10U);
    CHECK(navigation.state==NAV_GIRAR_ACOTADO);ex_turn_counts(45U,43U,5U);
    CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && navigation_diagnostics.turn_flags&8U && navigation_diagnostics.turn_mdeg>=10000U);expect_zero();
    PASS("LOCAL a transient asymmetric turn that stays within 3 mm can rebalance and complete normally; asymmetric counts are not categorically invalid");
    ex_diagnostic_turn();ex_sample(-60,0,20U,.31f,.8f);a1_terminal(NAV_FINISH_ODOMETRY);
    CHECK(navigation_diagnostics.odometry_error==NAV_ODOM_DISAGREEMENT);
    for(unsigned i=0;i<20U;++i){a1_sample(-1,0,20U);a2_data(.31f,.8f);b73_step();expect_zero();}
    CHECK(!navigation.active && navigation.finish_reason==NAV_FINISH_ODOMETRY);
    PASS("LOCAL persistent one-wheel motion or missing encoder counts remain terminal and cannot be hidden by stopped snapshots or new diagnostic traffic");
    for(unsigned slip=0;slip<2U;++slip){
        ex_diagnostic_turn();ex_turn_counts(235U,233U,25U);
        OdometrySnapshot estimated;Odometry_GetSnapshot(&estimated);
        CHECK(navigation.state==NAV_DETENER_Y_ESPERAR && navigation_diagnostics.turn_flags&8U);expect_zero();
        /* Ground truth is test metadata only: it is deliberately not an input to firmware. */
        float synthetic_ground_yaw=slip?0.0f:estimated.theta_rad;
        CHECK(slip?fabsf(estimated.theta_rad-synthetic_ground_yaw)>.17f:estimated.theta_rad==synthetic_ground_yaw);
    }
    PASS("LOCAL identical encoder traces with ideal grip or simulated wheel slip produce identical odometry: software cannot certify ground motion without independent evidence");
    ex_diagnostic_turn();ex_sample(-60,0,20U,.31f,.8f);a1_terminal(NAV_FINISH_ODOMETRY);
    uint32_t failed_session=navigation.session;OdometrySnapshot before_restart;Odometry_GetSnapshot(&before_restart);
    a1_sample(0,0,1U); /* Keep the real odometry producer fresh between simulated FSM schedules, without reinitializing it. */
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);expect_zero();
    CHECK(robot_state==ROBOT_STATE_IDLE && !navigation.active);
    a1_sample(0,0,20U);CHECK(TaskControlRobot_RequestExploreAt(clock_tick,54321U));run_fsm(1);
    unsigned new_query=0;
    for(unsigned i=0;i<16U && navigation.state!=NAV_GIRAR_ACOTADO;++i){
        a1_sample(0,0,20U);a2_data(.31f,.8f);
        CHECK(Navigation_ExploreLinkQuery(54321U,++new_query,explore.link_issued?explore.link_query:0U,clock_tick));
        Odometry_Observe(0,.31f,clock_tick,true);Odometry_Observe(1,.8f,clock_tick,true);b73_step();expect_zero();
    }
    CHECK(navigation.state==NAV_GIRAR_ACOTADO && navigation.session!=failed_session && !navigation_odometry.ready);
    a1_sample(0,0,20U);a2_data(.31f,.8f);b73_step();
    CHECK(navigation_odometry.left==before_restart.left_count && navigation_odometry.right==before_restart.right_count && navigation_diagnostics.turn_mdeg==0U);
    CHECK(!Navigation_IsCommandCurrent(failed_session,clock_tick));Navigation_CancelSession(failed_session);
    CHECK(navigation.active && navigation.finish_reason==NAV_FINISH_NONE);
    CHECK(TaskControlRobot_SendEvent(ROBOT_EVENT_STOP));run_fsm(1);expect_zero();
    PASS("LOCAL only a new explicit FSM start with stabilization establishes a new turn reference, preserving global odometry and rejecting the previous session without cancelling the new one");
#endif
}
