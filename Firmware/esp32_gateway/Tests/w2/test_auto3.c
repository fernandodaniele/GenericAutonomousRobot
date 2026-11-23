/* Real HTTP, arbiter, UART parser and serializers, without boards. */
static const char *a3_base="NAV,1,1,3,1,1010,2,4,1,1,450,340,0,0,500,500,7,0,0,0,0,-,-,0,0\n";
static const char *a3_detail="NAVT,1,1,1,1010,31,10640,340,2,80,4,4,60,1,1,1100,-3,-3,1090,0,10000,1\n";
static void a3_native_feed(void) {
    Stm32Navigation_BeginQuery();navigation_feed(a3_base);
    assert(s_navigation.received && s_navigation.profile==3U);
    Stm32Navigation_BeginTurnQuery();navigation_feed(a3_detail);
}
static unsigned a3_uart_seen,a3_uart_bases,a3_uart_waits,a3_uart_turns;
static char a3_uart_reply[256];
static void a3_uart_schedule(void) {
    if(a3_uart_seen==tx_count)return;
    const char *f=frames[tx_count-1];a3_uart_seen=tx_count;unsigned q;char suffix;
    if(!strcmp(f,"STATUS\n"))strcpy(a3_uart_reply,"STATE=0 FAULTS=0 STOP=0\n");
    else if(sscanf(f,"NAV Q=%u S=2%c",&q,&suffix)==2&&suffix=='\n'){
        ++a3_uart_turns;snprintf(a3_uart_reply,sizeof a3_uart_reply,"NAVT,1,%u%s",q,a3_detail+8);
    }else if(sscanf(f,"NAV Q=%u S=1%c",&q,&suffix)==2&&suffix=='\n'){
        ++a3_uart_waits;snprintf(a3_uart_reply,sizeof a3_uart_reply,"NAVW,1,%u,1,1010,0,0,0,0,0,0,0,-,-,-,-,100\n",q);
    }else if(sscanf(f,"NAV Q=%u%c",&q,&suffix)==2&&suffix=='\n'){
        ++a3_uart_bases;snprintf(a3_uart_reply,sizeof a3_uart_reply,"NAV,1,%u%s",q,a3_base+7);
    }else return;
    rx_data=a3_uart_reply;rx_size=strlen(rx_data);
}
static DWORD WINAPI a3_navigation_writer(void *arg) {
    (void)arg;
    for(unsigned i=0;i<3000U;++i){
        uint32_t q=Stm32Navigation_BeginQuery();char line[256];
        snprintf(line,sizeof line,"NAV,1,%u,3,%u,1010,2,4,1,1,450,340,0,0,500,500,7,0,0,0,0,-,-,0,0\n",q,i+1U);navigation_feed(line);
        Stm32Navigation_BeginTurnQuery();
        snprintf(line,sizeof line,"NAVT,1,%u,%u,1010,31,10640,340,2,80,4,4,60,1,1,1100,-3,-3,1090,0,10000,1\n",q,i+1U);navigation_feed(line);
    }
    return 0;
}
static void test_auto3(void) {
    const char *names[]={"AUTO_TURN","AUTO_EVADE"};
    const UartControlCommand commands[]={UART_CONTROL_AUTO_TURN,UART_CONTROL_AUTO_EVADE};
    for(unsigned i=0;i<2U;++i){
        auto_ready();post(names[i],strlen(names[i]));assert(!strncmp(response_status,"202",3)&&tx_count==0);
        post(names[i],strlen(names[i]));assert(strstr(reply,"\"duplicate\":true"));
        WebCommands_ProcessPending();char expected[16];snprintf(expected,sizeof expected,"%s\n",names[i]);
        assert(frame_count(expected)==1U);post(names[i],strlen(names[i]));WebCommands_ProcessPending();assert(frame_count(expected)==1U);
        assert(s_receipts[s_latest].command==commands[i]);
        pass("AUTO-3 explicit closed command is deferred, idempotent and transmitted exactly once");
        auto_ready();post(names[i],strlen(names[i]));request_id="bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
        post("STOP",4);WebCommands_ProcessPending();assert(frame_count(expected)==0U&&frame_count("STOP\n")==1U);
        auto_ready();post(names[i],strlen(names[i]));wait_hook=unsafe_before_auto;WebCommands_ProcessPending();assert(tx_count==0);
        reset();feed("STATE=2 FAULTS=0 STOP=0\n");post(names[i],strlen(names[i]));assert(!strncmp(response_status,"412",3));
        pass("AUTO-3 STOP and unsafe/mode changes revoke experimental requests at admission and under the TX gate");
    }
    auto_ready();post("AUTO_TURN\n",10);assert(!strncmp(response_status,"400",3));
    auto_ready();post("AUTO_EVADE\nSTOP",15);assert(!strncmp(response_status,"400",3));
    auto_ready();post("AUTO_CONTINUOUS\nSTOP",20);assert(!strncmp(response_status,"413",3));
    pass("AUTO-3 accepts no arbitrary UART suffix, newline or command sequence");
    reset();feed("STATE=0 FAULTS=0 STOP=0\n");a3_native_feed();navigation_api();
    assert(s_navigation.turn_received && s_navigation.turn_flags==31U && s_navigation.turn_stabilization.reference_left==-3);
    assert(strstr(reply,"\"profile\":\"AUTO_3_TURN\"") && strstr(reply,"\"progress_deg\":10.640") && strstr(reply,"\"turn_stabilization\":{\"available\":true"));
    printf("AUTO3_JSON %s\n",reply);
    assert(reply_length<NAVIGATION_JSON_CAPACITY && !tx_count);
    pass("AUTO-3 complete real UART packet creates compatible JSON with measured angle and independent turn stabilization");
    for(unsigned i=0;i<2U;++i) {
        const char *base=getenv(i==0U?"GAR_AUTO3_BASE_0":"GAR_AUTO3_BASE_1");
        const char *detail=getenv(i==0U?"GAR_AUTO3_TURN_0":"GAR_AUTO3_TURN_1");
        if(!base || !detail)continue;
        reset();unsigned q;assert(sscanf(base,"NAV,1,%u,",&q)==1 && q);
        s_navigation_counter=q-1U;assert(Stm32Navigation_BeginQuery()==q);navigation_feed(base);navigation_feed("\n");
        assert(s_navigation.received && s_navigation.profile==3U+i);
        assert(Stm32Navigation_BeginTurnQuery()==q);navigation_feed(detail);navigation_feed("\n");navigation_api();
        assert(s_navigation.turn_received && s_navigation.turn_flags==31U && s_navigation.turn_mdeg>=10000U);
        pass("AUTO-3 actual STM32 formatter output is accepted by real ESP32 parser and HTTP for each profile");
    }
    for(size_t split=0;split<=strlen(a3_detail);++split){
        reset();Stm32Navigation_BeginQuery();navigation_feed(a3_base);Stm32Navigation_BeginTurnQuery();
        Stm32Navigation_Feed((const uint8_t*)a3_detail,split);Stm32Navigation_Feed((const uint8_t*)a3_detail+split,strlen(a3_detail)-split);
        assert(s_navigation.turn_received);
    }
    pass("AUTO-3 turn packet accepts every UART fragmentation boundary");
    const char *bad[]={
        "NAVT,2,1,1,1010,31,10640,340,2,80,4,4,60,1,1,1100,-3,-3,1090,0,10000,1\n",
        "NAVT,1,2,1,1010,31,10640,340,2,80,4,4,60,1,1,1100,-3,-3,1090,0,10000,1\n",
        "NAVT,1,1,2,1010,31,10640,340,2,80,4,4,60,1,1,1100,-3,-3,1090,0,10000,1\n",
        "NAVT,1,1,1,1010,32,10640,340,2,80,4,4,60,1,1,1100,-3,-3,1090,0,10000,1\n",
        "NAVT,1,1,1,1010,31,9999,340,2,80,4,4,60,1,1,1100,-3,-3,1090,0,10000,1\n",
        "NAVT,1,1,1,1010,31,10640,340,2,80,3,3,60,1,1,1100,-3,-3,1090,0,10000,1\n",
        "NAVT,1,1,1,1010,31,10640,340,2,80,4,4,59,1,1,1100,-3,-3,1090,0,10000,1\n",
        "NAVT,1,1,1,1010,31,10640,340,2,80,4,4,60,1,1,1100,-3,-3,1090,0,10000,2\n",
        "NAVT,1,1,1,1010,31,10640,340,2,80,4,4,60,1,1,1100,-3,-3,1090,0,10000\n"
    };
    for(unsigned i=0;i<sizeof bad/sizeof bad[0];++i){reset();Stm32Navigation_BeginQuery();navigation_feed(a3_base);
        Stm32Navigation_BeginTurnQuery();navigation_feed(bad[i]);assert(!s_navigation.turn_received);}
    reset();Stm32Navigation_BeginQuery();navigation_feed(a3_base);Stm32Navigation_BeginTurnQuery();clock_us+=200000;
    navigation_feed(a3_detail);assert(!s_navigation.turn_received);
    pass("AUTO-3 malformed, inconsistent, old-query, foreign-session and expired diagnostic packets are rejected");
    reset();a3_native_feed();clock_us=1000000;uint32_t q=Stm32Navigation_BeginQuery();char frame[256];
    snprintf(frame,sizeof frame,"NAV,1,%u%s",q,a3_base+7);navigation_feed(frame);navigation_api();assert(s_navigation.turn_received);
    clock_us+=4000000;navigation_api();assert(strstr(reply,"\"historical\":true")&&strstr(reply,"\"stale\":true"));
    Stm32Navigation_BeginQuery();navigation_feed("NAV,1,3,3,2,9000,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,-,-,0,0\n");
    assert(!s_navigation.turn_received);
    pass("AUTO-3 terminal diagnostic survives STOP/IDLE and stale links but is never inherited by another session");
    reset();a3_native_feed();s_navigation.turn_stabilization.reference_left=INT64_MIN;s_navigation.turn_stabilization.reference_right=INT64_MAX;
    s_navigation.turn_stabilization.source_age=UINT32_MAX;s_navigation.stabilization=s_navigation.turn_stabilization;
    s_navigation.front_age=s_navigation.rear_age=s_navigation.event_age=s_navigation.decision_age=UINT32_MAX;
    navigation_api();assert(reply_length<NAVIGATION_JSON_CAPACITY&&strstr(reply,"-9223372036854775808"));
    char tiny[16];assert(Stm32Navigation_Json(tiny,sizeof tiny,true,false)==-1);
    printf("AUTO3_LIMIT_JSON %s\n",reply);
    pass("AUTO-3 expanded static HTTP buffer fits full int64 references for both stabilizations and rejects truncation");
    reset();HANDLE writer=CreateThread(NULL,0,a3_navigation_writer,NULL,0,NULL);assert(writer);
    for(unsigned i=0;i<3000U;++i){navigation_api();assert(reply_length<NAVIGATION_JSON_CAPACITY);}
    assert(WaitForSingleObject(writer,10000)==WAIT_OBJECT_0);CloseHandle(writer);
    assert(s_navigation.turn_received && s_navigation.session_id==3000U);
    pass("AUTO-3 concurrent UART snapshot updates and HTTP copies remain bounded and consistent under the shared lock");
    reset();a3_uart_seen=a3_uart_bases=a3_uart_waits=a3_uart_turns=0;
    rx_prepare_hook=a3_uart_schedule;rx_data_delay=100000;loop_limit=100U;
    if(!setjmp(task_end))app_main();
    assert(a3_uart_bases>=3U && a3_uart_waits>=2U && a3_uart_turns>=2U && s_navigation.turn_received);
    assert(frame_count("AUTO_TURN\n")==0U&&frame_count("AUTO_EVADE\n")==0U);
    pass("AUTO-3 actual app_main retains STATUS/sensors and services all navigation parts without starting any movement");
}
