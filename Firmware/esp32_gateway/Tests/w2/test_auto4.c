/* Native HTTP/UART/mapping and AUTO-4 diagnostics; synthetic fixtures only. */
static const char *a4_base="NAV,1,1,5,1,1010,1,1,0,257,200,100,10000,0,500,500,7,0,0,0,0,-,-,0,0\n";
static const char *a4_detail="NAVC,1,1,1,1010,1,0,0,0,10000,10000,10000,0,7,0,0,0,0,0,0,0,0,0,12000,4,2,500,0\n";
static void a4_native_feed(void) {
    Stm32Navigation_BeginQuery();navigation_feed(a4_base);assert(s_navigation.profile==5U);
    Stm32Navigation_BeginContinuousQuery();navigation_feed(a4_detail);assert(s_navigation.continuous.received);
}
static unsigned a4_uart_seen,a4_uart_counters[6];
static bool a4_uart_terminal;
static char a4_uart_reply[256];
static void a4_uart_schedule(void) {
    if(a4_uart_seen==tx_count)return;
    const char *f=frames[tx_count-1U];a4_uart_seen=tx_count;unsigned q,part;char suffix;
    if(!strcmp(f,"STATUS\n"))strcpy(a4_uart_reply,"STATE=1 FAULTS=0 STOP=0\n");
    else if(sscanf(f,"NAV Q=%u S=%u%c",&q,&part,&suffix)==3 && suffix=='\n') {
        if(part==3U){++a4_uart_counters[3];snprintf(a4_uart_reply,sizeof a4_uart_reply,"NAVC,1,%u%s",q,a4_uart_terminal ? ",1,1010,4,4,0,0,400000,400000,400000,0,0,2,80,4,4,60,0,0,0,0,12000,4,2,500,0\n" : a4_detail+8);}
        else if(part==2U){++a4_uart_counters[2];snprintf(a4_uart_reply,sizeof a4_uart_reply,"NAVT,1,%u,1,1010,0,0,0,0,0,0,0,0,0,0,-,-,-,-,200,10000,1\n",q);}
        else {++a4_uart_counters[1];snprintf(a4_uart_reply,sizeof a4_uart_reply,"NAVW,1,%u,1,1010,0,0,0,0,0,0,0,-,-,-,-,100\n",q);}
    }else if(sscanf(f,"NAV Q=%u%c",&q,&suffix)==2&&suffix=='\n'){
        ++a4_uart_counters[0];snprintf(a4_uart_reply,sizeof a4_uart_reply,"NAV,1,%u%s",q,a4_uart_terminal ? ",5,1,1010,2,1,14,257,6400,1540,100000,0,500,500,7,0,1,1,0,-,-,0,0\n" : a4_base+7);
    }else if(sscanf(f,"MAPPING Q=%u S=%u%c",&q,&part,&suffix)==3&&suffix=='\n'){
        unsigned x=a4_uart_counters[4]*25U;uint32_t tick=(uint32_t)(clock_us/1000);
        if(part==0U){++a4_uart_counters[4];x=a4_uart_counters[4]*25U;
            snprintf(a4_uart_reply,sizeof a4_uart_reply,"POSE,2,%u,7,%u,2,1,%u,%u,%u,0,0,25,0,0\n",q,tick,a4_uart_counters[4]*450U,a4_uart_counters[4]*445U,x);
        }else {++a4_uart_counters[5];snprintf(a4_uart_reply,sizeof a4_uart_reply,"RAY,1,%u,7,%u,%u,%u,3,0,500,%u,0,%u,785\n",q,tick,part-1U,a4_uart_counters[5],x,part==1U?0U:3142U);}
    }else strcpy(a4_uart_reply,"ERR_UNKNOWN\n");
    rx_data=a4_uart_reply;rx_size=strlen(rx_data);
}
static DWORD WINAPI a4_snapshot_writer(void *arg) {
    (void)arg;char line[256];
    for(unsigned i=0;i<1000U;++i){
        uint32_t q=Stm32Navigation_BeginQuery();
        snprintf(line,sizeof line,"NAV,1,%u,5,%u,1010,2,1,14,257,6400,1540,100000,0,500,500,7,0,1,1,0,-,-,0,0\n",q,i+1U);navigation_feed(line);
        Stm32Navigation_BeginContinuousQuery();
        snprintf(line,sizeof line,"NAVC,1,%u,%u,1010,4,4,0,0,%u,%u,%u,0,0,2,80,4,4,60,0,0,0,0,12000,4,2,500,0\n",q,i+1U,400000U+i,400000U+i,400000U+i);navigation_feed(line);
    }
    return 0;
}
static void test_auto4(void) {
    auto_ready();post("AUTO_CONTINUOUS",15);assert(!strncmp(response_status,"202",3) && !tx_count);
    post("AUTO_CONTINUOUS",15);assert(strstr(reply,"\"duplicate\":true"));
    WebCommands_ProcessPending();assert(frame_count("AUTO_CONTINUOUS\n")==1U);
    api();assert(strstr(reply,"\"name\":\"AUTO_CONTINUOUS\"") && strstr(reply,"\"status_after_send\":false"));
    pass("AUTO-4 explicit one-shot command has a closed HTTP body, deferred delivery and no false operational confirmation");
    auto_ready();post("AUTO_CONTINUOUS",15);request_id="bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    post("STOP",4);WebCommands_ProcessPending();assert(frame_count("AUTO_CONTINUOUS\n")==0 && frame_count("STOP\n")==1U);
    auto_ready();post("AUTO_CONTINUOUS",15);wait_hook=unsafe_before_auto;WebCommands_ProcessPending();assert(tx_count==0);
    auto_ready();post("AUTO_CONTINUOUS",15);clock_us+=500000;WebCommands_ProcessPending();assert(tx_count==0);
    reset();feed("STATE=2 FAULTS=0 STOP=0\n");post("AUTO_CONTINUOUS",15);assert(!strncmp(response_status,"412",3));
    reset();feed("STATE=1 FAULTS=0 STOP=0\n");post("AUTO_CONTINUOUS",15);assert(!strncmp(response_status,"412",3));
    pass("AUTO-4 STOP, mode change, stale request or ESP32 restart cannot transmit an old continuous-profile permission");
    reset();a4_native_feed();navigation_api();assert(strstr(reply,"\"profile\":\"AUTO_4_CONTINUOUS\"") && strstr(reply,"\"path_mm\":10.000"));
    assert(strstr(reply,"\"supervised_turn_eligible\":true") && strstr(reply,"\"session_ms\":12000"));
    assert(reply_length<NAVIGATION_JSON_CAPACITY && !tx_count);printf("AUTO4_JSON %s\n",reply);
    pass("AUTO-4 coherent NAVC diagnostics serialize actual controller limits, progress and sensor eligibility without issuing commands");
    for(size_t split=0;split<=strlen(a4_detail);++split){
        reset();Stm32Navigation_BeginQuery();navigation_feed(a4_base);Stm32Navigation_BeginContinuousQuery();
        Stm32Navigation_Feed((const uint8_t*)a4_detail,split);Stm32Navigation_Feed((const uint8_t*)a4_detail+split,strlen(a4_detail)-split);assert(s_navigation.continuous.received);
    }
    pass("AUTO-4 supplementary packet preserves strict framing at all UART fragmentation boundaries");
    for(unsigned fault=0;fault<8U;++fault){
        reset();Stm32Navigation_BeginQuery();navigation_feed(a4_base);Stm32Navigation_BeginContinuousQuery();
        char line[256];snprintf(line,sizeof line,"NAVC,%u,%u,%u,1010,%u,0,%u,0,10000,10000,10000,0,7,0,0,0,0,0,0,0,0,0,%u,4,2,500,0\n",
            fault==0U?2U:1U,fault==1U?2U:1U,fault==2U?2U:1U,fault==3U?5U:1U,fault==4U?3U:0U,fault==5U?60001U:12000U);
        if(fault==6U)clock_us+=200000;
        if(fault==7U)line[strlen(line)-1U]=',';
        navigation_feed(line);assert(!s_navigation.continuous.received);
    }
    pass("AUTO-4 malformed versions, queries, sessions, counters, limits, incomplete and expired replies are rejected");
    for(unsigned i=0;i<3U;++i){
        const char *base=getenv(i==0U?"GAR_AUTO4_BASE_0":i==1U?"GAR_AUTO4_BASE_1":"GAR_AUTO4_BASE_2");
        const char *detail=getenv(i==0U?"GAR_AUTO4_DETAIL_0":i==1U?"GAR_AUTO4_DETAIL_1":"GAR_AUTO4_DETAIL_2");
        if(!base||!detail)continue;
        reset();unsigned q;assert(sscanf(base,"NAV,1,%u,",&q)==1);s_navigation_counter=q-1U;
        assert(Stm32Navigation_BeginQuery()==q);navigation_feed(base);navigation_feed("\n");assert(s_navigation.profile==5U);
        Stm32Navigation_BeginContinuousQuery();navigation_feed(detail);navigation_feed("\n");navigation_api();
        assert(s_navigation.continuous.received && s_navigation.continuous.evasions==i && strstr(reply,"LIMIT_REACHED"));
        pass("AUTO-4 real STM32 formatter output for zero/one/two recoveries is parsed by the ESP32 and HTTP endpoint");
    }
    reset();Stm32Navigation_BeginQuery();navigation_feed("NAV,1,1,5,1,1010,2,1,13,1,500,100,0,0,270,240,7,0,0,0,0,-,-,0,0\n");
    Stm32Navigation_BeginContinuousQuery();navigation_feed("NAVC,1,1,1,1010,1,0,1,0,10000,10000,10000,2,0,0,0,0,0,0,0,0,0,0,12000,4,2,500,0\n");
    assert(s_navigation.continuous.received);clock_us=1000000;uint32_t q=Stm32Navigation_BeginQuery();char line[256];
    snprintf(line,sizeof line,"NAV,1,%u,5,1,1010,2,1,13,1,500,100,0,0,270,240,7,0,0,0,0,-,-,0,0\n",q);navigation_feed(line);
    feed("STATE=0 FAULTS=0 STOP=0\n");navigation_api();assert(s_navigation.continuous.received && strstr(reply,"BLOCKED_NO_SAFE_PATH"));
    assert(strstr(reply,"\"critical_faults\":0")&&strstr(reply,"\"historical\":true"));clock_us+=4000000;navigation_api();assert(strstr(reply,"\"stale\":true"));
    Stm32Navigation_BeginQuery();navigation_feed("NAV,1,3,5,2,9000,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,-,-,0,0\n");assert(!s_navigation.continuous.received);
    pass("AUTO-4 terminal blocked state needs no invented hardware fault; STOP retains ageing history and a new session never inherits it");
    reset();a4_native_feed();s_navigation.continuous.path_um=UINT32_MAX;s_navigation.continuous.useful_um=UINT32_MAX;
    s_navigation.continuous.source_age=UINT32_MAX;s_navigation.continuous.received_us=0;clock_us=INT64_MAX;
    navigation_api();assert(reply_length<NAVIGATION_JSON_CAPACITY);printf("AUTO4_LIMIT_JSON %s\n",reply);
    char tiny[8];assert(Stm32Navigation_Json(tiny,sizeof tiny,true,false)==-1);
    pass("AUTO-4 maximum numeric ages and progress fit the static HTTP buffer; insufficient buffers cannot return truncated JSON");
    reset();HANDLE writer=CreateThread(NULL,0,a4_snapshot_writer,NULL,0,NULL);assert(writer);
    for(unsigned i=0;i<1000U;++i){
        navigation_api();assert(reply_length<NAVIGATION_JSON_CAPACITY);
        if(strstr(reply,"\"continuous\":{\"available\":true")){
            unsigned id,whole,fraction;const char *session=strstr(reply,"\"session_id\":")+13;
            const char *path=strstr(reply,"\"path_mm\":")+10;
            assert(sscanf(session,"%u",&id)==1 && sscanf(path,"%u.%u",&whole,&fraction)==2);
            assert(whole==400U && fraction==id-1U);
        }
    }
    assert(WaitForSingleObject(writer,10000)==WAIT_OBJECT_0);CloseHandle(writer);
    pass("AUTO-4 multipart publications concurrent with HTTP retain one coherent session/progress tuple and never perform network I/O under the lock");
    reset();a4_uart_seen=0;memset(a4_uart_counters,0,sizeof a4_uart_counters);
    rx_prepare_hook=a4_uart_schedule;rx_data_delay=100000;loop_limit=200U;
    if(!setjmp(task_end))app_main();
    printf("AUTO4_POLL %u %u %u %u POSE=%u RAY=%u STATUS=%u\n",a4_uart_counters[0],a4_uart_counters[1],a4_uart_counters[2],a4_uart_counters[3],a4_uart_counters[4],a4_uart_counters[5],frame_count("STATUS\n"));
    assert(a4_uart_counters[0]>=3U && a4_uart_counters[1]>=2U && a4_uart_counters[2]>=2U && a4_uart_counters[3]>=2U);
    assert(a4_uart_counters[4]>=2U && a4_uart_counters[5]>=2U && s_map_work.grid.path_count>=2U);
    unsigned occupied=0,unknown=0;for(unsigned i=0;i<MAP_CELLS;++i){occupied+=s_map_work.grid.cells[i]==2U;unknown+=s_map_work.grid.cells[i]==0U;}
    assert(occupied>0U && unknown>0U && s_map_work.grid.revision>0U);httpd_req_t req={0};assert(WebServer_Map(&req)==ESP_OK);
    assert(strstr(reply,"\"trajectory_mm\":[[") && !frame_count("AUTO_CONTINUOUS\n"));
    pass("AUTO-4 real app_main keeps STATUS, all diagnostic parts, trajectory and echo-based occupancy operational without automatic commands");    reset();a4_uart_terminal=true;a4_uart_seen=0;memset(a4_uart_counters,0,sizeof a4_uart_counters);
    rx_prepare_hook=a4_uart_schedule;rx_data_delay=100000;loop_limit=150U;
    if(!setjmp(task_end))app_main();
    assert(a4_uart_counters[3]>=2U && s_navigation.continuous.received && s_navigation.continuous.completed_segments==4U);
    navigation_api();assert(strstr(reply,"LIMIT_REACHED") && strstr(reply,"\"historical\":true"));
    a4_uart_terminal=false;
    pass("AUTO-4 terminal/STOP history still fetches all supplementary parts without starving NAVC or requiring an active motion session");

}
