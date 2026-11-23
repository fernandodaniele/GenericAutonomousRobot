/* Misma recepcion UART, parser y HTTP reales; sin hardware ni E/S desde HTTP. */
static const char *navigation_line="NAV,1,1,2,7,1000,2,3,7,319,150,30,50000,0,298,450,7,0,20,90,4,1,0,10,0\r\n";
static void navigation_feed(const char *line) { Stm32Navigation_Feed((const uint8_t *)line,strlen(line)); }
static void navigation_api(void) { httpd_req_t request={0};assert(WebServer_Navigation(&request)==ESP_OK); }
static DWORD WINAPI navigation_writer(void *arg) {
    (void)arg;
    for(unsigned i=0;i<10000U;++i) {
        char line[256];uint32_t q=Stm32Navigation_BeginQuery();
        snprintf(line,sizeof line,"NAV,1,%u,2,%u,1000,2,3,7,319,150,30,%u,0,298,450,7,0,20,90,4,1,0,10,0\n",q,i+1U,(i+1U)*1000U);
        navigation_feed(line);
    }
    return 0;
}
/* Replay sintetico: UART con 100 ms por lectura y consultas reales app_main. */
static unsigned navigation_f_seen, navigation_f_bases, navigation_f_parts, navigation_f_retained;
static char navigation_f_reply[256];
static void navigation_f_uart(void) {
    if(navigation_f_seen==tx_count) { return; }
    const char *frame=frames[tx_count-1U];navigation_f_seen=tx_count;
    unsigned q;char suffix;
    if(!strcmp(frame,"STATUS\n")) {
        strcpy(navigation_f_reply,"STATE=0 FAULTS=0 STOP=0\n");
    } else if(sscanf(frame,"NAV Q=%u S=1%c",&q,&suffix)==2 && suffix=='\n') {
        ++navigation_f_parts;
        snprintf(navigation_f_reply,sizeof navigation_f_reply,"NAVW,1,%u,7,1000,2,80,4,4,60,0,0,1220,123,456,1200,100\n",q);
    } else if(sscanf(frame,"NAV Q=%u%c",&q,&suffix)==2 && suffix=='\n') {
        if(s_navigation.stabilization.received)++navigation_f_retained;
        ++navigation_f_bases;
        snprintf(navigation_f_reply,sizeof navigation_f_reply,"NAV,1,%u,2,7,1000,2,3,4,895,500,300,56058,11759,295,487,7,0,1,1,0,-,-,0,0\n",q);
    } else { return; }
    rx_data=navigation_f_reply;rx_size=strlen(navigation_f_reply);
}
static void test_navigation(void) {
    reset();navigation_api();
    assert(strstr(reply,"\"received\":false") && strstr(reply,"\"profile\":null") && strstr(reply,"\"applied\":null"));
    assert(!tx_count && !rx_count);
    pass("AUTO-2D no UART data are unknown; HTTP cannot send commands or claim application");
    reset();Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,2,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,-,-,0,0\n");navigation_api();
    assert(strstr(reply,"\"snapshot_available\":false") && strstr(reply,"\"decision\":{\"age_ms\":null"));
    assert(strstr(reply,"\"front_mm\":null") && strstr(reply,"\"front_valid\":null"));
    reset();feed("STATE=1 FAULTS=0 STOP=0\n");Stm32Navigation_BeginQuery();navigation_feed(navigation_line);navigation_api();
    assert(s_navigation.received && s_navigation.session_id==7 && s_navigation.left_delta==1);
    assert(strstr(reply,"\"connected\":true") && strstr(reply,"\"profile\":\"AUTO_2\"") && strstr(reply,"\"last_phase\":\"RETROCEDER_ACOTADO\""));
    assert(strstr(reply,"\"finish_reason\":\"ODOMETRY\"") && strstr(reply,"\"odometry_error\":\"wrong_direction\""));
    assert(strstr(reply,"\"published\":true,\"applied\":false") && strstr(reply,"\"advance_progress_mm\":50.000"));
    assert(strstr(reply,"\"front_mm\":298") && strstr(reply,"\"left_delta_counts\":\"1\""));
    printf("NAV_JSON %s\n",reply);
    pass("AUTO-2D real API reports retained terminal cause, exact signed diagnostic deltas and separate request/publication/application");
    reset();feed("STATE=1 FAULTS=0 STOP=0\n");Stm32Navigation_BeginQuery();
    const char *stm_wire=getenv("GAR_NAVIGATION_WIRE");
    navigation_feed(stm_wire ? stm_wire : navigation_line);navigation_feed("\n");navigation_api();
    assert(s_navigation.received && s_navigation.reason==7U && s_navigation.odometry_error==4U && s_navigation.left_delta==1);
    assert(strstr(reply,"\"published\":true,\"applied\":false"));
    printf("NAV_STM_JSON %s\n",reply);
    pass("AUTO-2D STM32 residual-format wire feeds actual ESP32 parser and API without inferring physical reverse");
    for(size_t split=0;split<=strlen(navigation_line);++split) {
        reset();Stm32Navigation_BeginQuery();
        Stm32Navigation_Feed((const uint8_t *)navigation_line,split);
        Stm32Navigation_Feed((const uint8_t *)navigation_line+split,strlen(navigation_line)-split);
        assert(s_navigation.received && s_navigation.reason==7U);
    }
    reset();Stm32Navigation_BeginQuery();
    for(size_t i=0;i<strlen(navigation_line);++i)Stm32Navigation_Feed((const uint8_t *)navigation_line+i,1U);
    assert(s_navigation.received);
    pass("AUTO-2D all UART fragmentation boundaries including one-byte CRLF preserve one complete reply");
    const char *bad[]={
        "NAV,2,1,2,7,1000,2,3,7,319,150,30,50000,0,298,450,7,0,20,90,4,1,0,10,0\n",
        "NAV,1,1,7,7,1000,2,3,7,319,150,30,50000,0,298,450,7,0,20,90,4,1,0,10,0\n",
        "NAV,1,1,2,7,1000,5,3,7,319,150,30,50000,0,298,450,7,0,20,90,4,1,0,10,0\n",
        "NAV,1,1,2,7,1000,2,3,17,319,150,30,50000,0,298,450,7,0,20,90,4,1,0,10,0\n",
        "NAV,1,1,2,7,1000,2,3,16,319,150,30,50000,0,298,450,7,0,20,90,4,1,0,10,0\n",
        "NAV,1,1,2,7,1000,2,3,7,1024,150,30,50000,0,298,450,7,0,20,90,4,1,0,10,0\n",
        "NAV,1,1,2,7,1000,2,3,7,319,150,30,-1,0,298,450,7,0,20,90,4,1,0,10,0\n",
        "NAV,1,1,2,7,1000,2,3,7,319,150,30,50000,0,0,450,7,0,20,90,4,1,0,10,0\n",
        "NAV,1,1,2,7,1000,2,3,7,319,150,30,50000,0,298,450,7,0,20,90,9,1,0,10,0\n",
        "NAV,1,1,2,7,1000,2,3,7,319,150,30,50000,0,298,450,7,0,20,90,4,9223372036854775808,0,10,0\n",
        "NAV,1,1,2,7,1000,2,3,7,319,150,30,50000,0,298,450,7,0,20,90,4,-9223372036854775809,0,10,0\n",
        "NAV,1,1,2,7,1000,2,3,7,319,150,30,50000,0,298,450,7,0,20,90,4,-,0,10,0\n",
        "NAV,1,4294967296,2,7,1000,2,3,7,319,150,30,50000,0,298,450,7,0,20,90,4,1,0,10,0\n",
        "NAV,1,1,2,7,1000,2,3,7,319,150,30,50000,0,298,450,7,0,20,90,4,1,0,10,0,extra\n"
    };
    for(unsigned i=0;i<sizeof bad/sizeof bad[0];++i) {
        reset();Stm32Navigation_BeginQuery();navigation_feed(bad[i]);assert(!s_navigation.received);
    }
    reset();Stm32Navigation_BeginQuery();navigation_feed("NAV,1,1,2,7,1000,2");assert(!s_navigation.received);
    navigation_feed("\n");char huge[300];memset(huge,'X',sizeof huge);Stm32Navigation_Feed((uint8_t *)huge,sizeof huge);navigation_feed("\n");
    const uint8_t nul[]={'N','A','V',0,',','1','\n'};Stm32Navigation_Feed(nul,sizeof nul);
    navigation_feed(navigation_line);assert(s_navigation.received);
    pass("AUTO-2D strict bounds, version, incomplete/unknown fields, overflow/NUL and resynchronization reject malformed frames");
    reset();Stm32Navigation_BeginQuery();clock_us=200000;navigation_feed(navigation_line);assert(!s_navigation.received);
    reset();Stm32Navigation_BeginQuery();navigation_feed(navigation_line);clock_us=100000;navigation_feed(navigation_line);
    navigation_api();assert(strstr(reply,"\"age_ms\":100"));
    Stm32Navigation_BeginQuery();navigation_feed(navigation_line);assert(s_navigation.session_id==7U);
    pass("AUTO-2D reply correlation rejects late, duplicate and old query frames without rejuvenating data");
    reset();feed("STATE=1 FAULTS=0 STOP=0\n");Stm32Navigation_BeginQuery();navigation_feed(navigation_line);
    clock_us=3000000;navigation_api();assert(strstr(reply,"\"stale\":true") && strstr(reply,"\"finish_reason\":\"ODOMETRY\""));
    feed("STATE=0 FAULTS=0 STOP=0\n");navigation_api();assert(strstr(reply,"\"stm32_connected\":true") && strstr(reply,"\"connected\":false"));
    Stm32Navigation_BeginQuery();navigation_feed("NAV,1,2,2,8,4000,1,1,0,257,10,10,0,0,500,500,7,0,1,1,0,-,-,0,0\n");
    navigation_api();assert(strstr(reply,"\"connected\":true") && strstr(reply,"\"session_id\":8") && strstr(reply,"\"published\":false"));
    clock_us+=3000000;navigation_api();assert(strstr(reply,"\"stm32_connected\":false"));
    pass("AUTO-2D telemetry/STATUS freshness are independent; historical terminal data remain labelled and new session replaces flags atomically");
    reset();feed("STATE=0 FAULTS=0 STOP=0\n");Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,1,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-,-,0,0\n");navigation_api();
    assert(strstr(reply,"\"profile\":\"AUTO_1\"") && strstr(reply,"\"available\":false") && strstr(reply,"\"state\":null"));
    pass("AUTO-2D profile is reported by STM32 even before any AUTO, not inferred from ESP32 HTML");
    reset();feed("STATE=1 FAULTS=0 STOP=0\n");Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,2,4294967295,4294967295,2,3,7,1023,4294967295,4294967295,200000000,200000000,4000,4000,15,4294967295,4294967295,4294967295,5,-9223372036854775808,9223372036854775807,4294967295,4294967295\n");
    navigation_api();assert(strstr(reply,"-9223372036854775808") && strstr(reply,"9223372036854775807"));
    assert(strstr(reply,"200000.000") && strlen(reply)<NAVIGATION_JSON_CAPACITY);
    printf("NAV_MAX_JSON %s\n",reply);
    pass("AUTO-2D maximum-size actual JSON retains signed int64 as strings and fits HTTP buffer");
    reset();Uart_Init();clock_us=1000000;Uart_PollNavigation();assert(tx_count==1 && !strcmp(frames[0],"NAV Q=1\n"));
    Uart_PollNavigation();assert(tx_count==1);clock_us=5000000;Uart_PollNavigation();Uart_PollNavigation();assert(tx_count==2);
    reset();Uart_Init();post("STOP",4);clock_us=1000000;Uart_PollNavigation();assert(!tx_count);
    WebCommands_ProcessPending();assert(tx_count==1 && !strcmp(frames[0],"STOP\n"));
    reset();Uart_Init();clock_us=1000000;wait_hook=stop_during_send;Uart_PollNavigation();assert(!tx_count);
    WebCommands_ProcessPending();assert(tx_count==1 && !strcmp(frames[0],"STOP\n"));
    pass("AUTO-2D one-Hz polling is nonaccumulating, STOP wins and HTTP never writes UART");
    reset();loop_limit=100U;if(!setjmp(task_end))app_main();
    assert(task_count==1U && rx_count==100U && frame_count("STATUS\n")==10U);
    assert(s_navigation_counter>=5U && s_navigation_counter<=10U);
    pass("AUTO-2D actual app_main services NAV alongside existing queries with 100-ms UART waits and no new task");
    manual_ready();Uart_Init();clock_us=1000000;Uart_PollNavigation();assert(!tx_count && !s_navigation_pending);
    navigation_api();assert(strstr(reply,"\"paused\":true"));
    reset();Uart_Init();clock_us=1000000;tx_error=1;Uart_PollNavigation();assert(tx_count==1 && !s_navigation_pending);
    tx_error=0;Uart_PollNavigation();assert(tx_count==1);
    reset();Uart_Init();clock_us=1000000;feed("STATE=3 FAULTS=4 STOP=1\n");Uart_PollNavigation();
    assert(tx_count==1 && !strcmp(frames[0],"NAV Q=1\n"));
    pass("AUTO-2D manual lease pauses diagnostic TX, failed TX does not retry and Safety state does not suppress read-only queries");
    reset();clock_us=INT64_C(4294967296)*1000;feed("STATE=1 FAULTS=0 STOP=0\n");
    Stm32Navigation_BeginQuery();navigation_feed(navigation_line);clock_us+=2000000;navigation_api();
    assert(strstr(reply,"\"age_ms\":2000") && strstr(reply,"\"connected\":true"));
    clock_us-=3000000;navigation_api();assert(strstr(reply,"\"age_ms\":null") && strstr(reply,"\"connected\":false"));
    pass("AUTO-2D ESP32 monotonic age is 64-bit, independent of STM ticks, and clock regression cannot look fresh");
    reset();feed("STATE=1 FAULTS=0 STOP=0\n");Stm32Navigation_BeginQuery();
    rx_data=navigation_line;rx_size=strlen(navigation_line);Uart_ReadResponse();assert(s_navigation.received && rx_count==1);
    assert(WebServer_Start()==ESP_OK && !strcmp(routes[18].uri,"/api/navigation") && routes[18].method==HTTP_GET);
    for(int e=1;e<=3;++e){http_error=e;httpd_req_t r={0};assert(WebServer_Navigation(&r)==ESP_FAIL);}http_error=0;
    pass("AUTO-2D sole real UART reader feeds navigation; extra GET route preserves LRU and HTTP errors, without commands");
    reset();HANDLE writer=CreateThread(NULL,0,navigation_writer,NULL,0,NULL);assert(writer);
    for(unsigned i=0;i<10000U;++i) {
        Stm32NavigationSnapshot s;taskENTER_CRITICAL(&s_navigation_lock);s=s_navigation;taskEXIT_CRITICAL(&s_navigation_lock);
        if(s.received)assert(s.advance_um==s.session_id*1000U);
        assert(Stm32Navigation_Json(reply,sizeof reply,true,false)>0);
    }
    assert(WaitForSingleObject(writer,10000)==WAIT_OBJECT_0);CloseHandle(writer);
    pass("AUTO-2D 10000 parser publications concurrent with HTTP snapshots never mix session IDs and metrics");
    /* AUTO-2D.1: mismo handler y generador, sin sustituir datos ausentes por cero. */
    reset();assert(WebServer_Start()==ESP_OK);
    httpd_req_t r={0};
    assert(routes[18].handler(&r)==ESP_OK && response_count==1U);
    assert(reply_length==strlen(reply) && reply[0]=='{' && reply[reply_length-1U]=='}');
    assert(!strcmp(reply_type,"application/json; charset=utf-8") && !strcmp(reply_cache,"no-store"));
    assert(strstr(reply,"\"received\":false") && strstr(reply,"\"session_id\":null"));
    printf("NAV_NO_DATA_JSON %s\n",reply);
    Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,2,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-,-,0,0\n");
    assert(routes[18].handler(&r)==ESP_OK && response_count==2U);
    assert(strstr(reply,"\"received\":true") && strstr(reply,"\"available\":false"));
    pass("AUTO-2D.1 missing UART data and pre-AUTO snapshot produce complete JSON responses with unknown fields");
    reset();feed("STATE=1 FAULTS=0 STOP=0\n");Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,2,9,1000,1,1,0,257,40,20,3000,0,500,500,7,0,1,1,0,-,-,0,0\n");
    navigation_api();assert(strstr(reply,"\"terminal\":false") && strstr(reply,"\"active_reported\":true"));
    assert(strstr(reply,"\"advance_progress_mm\":3.000") && reply_length==strlen(reply));
    Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,2,2,9,1000,2,1,1,257,1200,1180,100000,0,500,500,7,0,1,1,0,-,-,10,0\n");
    navigation_api();assert(strstr(reply,"\"finish_reason\":\"TARGET\"") && strstr(reply,"\"terminal\":true"));
    feed("STATE=0 FAULTS=0 STOP=0\n");navigation_api();
    assert(strstr(reply,"\"session_id\":9") && strstr(reply,"\"finish_reason\":\"TARGET\""));
    feed("RESET_QUEUED\nSTATE=0 FAULTS=0 STOP=0\n");navigation_api();
    assert(strstr(reply,"\"session_id\":9") && strstr(reply,"\"finish_reason\":\"TARGET\""));
    Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,3,2,10,3000,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,-,-,0,0\n");
    navigation_api();assert(strstr(reply,"\"session_id\":10") && strstr(reply,"\"finish_reason\":\"NONE\""));
    pass("AUTO-2D.1 live/terminal HTTP data survive later STOP/RESET status and are replaced only by a new NAV session");
    reset();Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,2,4294967295,4294967295,2,3,10,1023,4294967295,4294967295,200000000,200000000,4000,4000,15,4294967295,4294967295,4294967295,7,-9223372036854775808,9223372036854775807,4294967295,4294967295\n");
    clock_us=INT64_MAX;navigation_api();
    assert(strstr(reply,"EVASION_NOT_AUTHORIZED") && strstr(reply,"wheel_disagreement"));
    assert(strstr(reply,"-9223372036854775808") && strstr(reply,"9223372036854775807"));
    assert(reply_length==strlen(reply) && reply_length<NAVIGATION_JSON_CAPACITY && reply[reply_length-1U]=='}');
    printf("NAV_HTTP_LIMIT_JSON %s\n",reply);
    pass("AUTO-2D.1 long terminal strings plus int64/timestamp limits still send one complete bounded HTTP JSON");
    reset();navigation_format_error=true;navigation_api();
    assert(response_count==1U && !strcmp(response_status,"500 Internal Server Error"));
    assert(!strcmp(reply,"{\"error\":\"navigation_unavailable\"}") && reply_length==strlen(reply));
    assert(!strcmp(reply_type,"application/json; charset=utf-8") && !strcmp(reply_cache,"no-store"));
    navigation_format_error=false;navigation_api();
    assert(response_count==2U && strstr(reply,"\"received\":false"));
    pass("AUTO-2D.1 serializer failure sends a bounded HTTP 500 JSON body instead of silently closing the request");
    reset();assert(WebServer_Start()==ESP_OK);
    const char *endpoints[]={"/health","/api/status","/api/sensors","/api/map","/api/imu","/api/navigation"};
    for(unsigned round=0;round<100U;++round) {
        for(unsigned i=0;i<sizeof endpoints/sizeof endpoints[0];++i) {
            unsigned route=0U;while(route<route_count && strcmp(routes[route].uri,endpoints[i]))++route;
            assert(route<route_count && routes[route].method==HTTP_GET && routes[route].handler(&r)==ESP_OK);
            assert(reply_length==strlen(reply) && reply[0]=='{' && reply[reply_length-1U]=='}');
        }
    }
    assert(response_count==600U && !tx_count && !rx_count);
    char tiny[8];assert(Stm32Navigation_Json(tiny,sizeof tiny,false,false)<0);
    pass("AUTO-2D.1 serialized HTTP handlers coexist with IMU/map buffers, exact lengths and no UART or truncated JSON success");
    reset();feed("STATE=1 FAULTS=0 STOP=0\n");Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,2,17,1000,5,5,0,263,100,40,20000,0,295,487,7,0,1,1,0,-,-,0,0\n");
    navigation_api();assert(strstr(reply,"\"state\":\"DETENER_Y_ESPERAR\"") && strstr(reply,"\"stabilization\":{\"available\":false"));
    Stm32Navigation_BeginWaitQuery();
    navigation_feed("NAVW,1,1,17,1000,1,40,2,2,20,1,0,-,-,-,-,100\n");navigation_api();
    assert(strstr(reply,"\"stabilization\":{\"available\":true,\"state\":\"waiting\"") && strstr(reply,"\"reference_tick\":null"));
    assert(strstr(reply,"\"observations\":2") && strstr(reply,"\"window_ms\":20"));
    pass("AUTO-2E new phase is visible while old NAV fields remain compatible and missing details stay unknown");
    Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,2,2,17,1000,3,3,0,831,160,20,20000,0,295,487,7,0,1,1,0,-,-,0,0\n");
    navigation_api();assert(strstr(reply,"\"stabilization\":{\"available\":false"));
    Stm32Navigation_BeginWaitQuery();
    const char *confirmed="NAVW,1,2,17,1000,2,80,4,4,60,0,0,1100,3,2,1080,100\r\n";
    for(size_t i=0;i<strlen(confirmed);++i)Stm32Navigation_Feed((const uint8_t *)confirmed+i,1U);
    navigation_api();assert(strstr(reply,"\"reference_left_counts\":\"3\"") && strstr(reply,"\"reference_right_counts\":\"2\""));
    assert(strstr(reply,"\"state\":\"confirmed\"") && strstr(reply,"\"reverse_entry_tick\":1080") && strstr(reply,"\"reference_tick\":1100"));
    feed("STATE=0 FAULTS=0 STOP=0\n");navigation_api();assert(strstr(reply,"\"state\":\"confirmed\""));
    pass("AUTO-2E fragmented details share one source snapshot and retain signed reference, confirmation and separate reverse-entry tick");
    reset();Stm32Navigation_BeginQuery();navigation_feed(navigation_line);Stm32Navigation_BeginWaitQuery();
    const char *invalid_wait[]={
        "NAVW,2,1,7,1000,2,80,4,4,60,0,0,0,3,2,0,0\n",
        "NAVW,1,2,7,1000,2,80,4,4,60,0,0,0,3,2,0,0\n",
        "NAVW,1,1,8,1000,2,80,4,4,60,0,0,0,3,2,0,0\n",
        "NAVW,1,1,7,1000,5,80,4,4,60,0,0,0,3,2,0,0\n",
        "NAVW,1,1,7,1000,2,80,3,3,60,0,0,0,3,2,0,0\n",
        "NAVW,1,1,7,1000,2,80,4,4,60,2,0,0,3,2,0,0\n",
        "NAVW,1,1,7,1000,2,80,4,4,60,0,0,-1,3,2,0,0\n",
        "NAVW,1,1,7,1000,2,80,4,4,60,0,0,0,-,2,0,0\n",
        "NAVW,1,1,7,1000,2,80,4,4,60,0,0,0,9223372036854775808,2,0,0\n"
    };
    for(unsigned i=0;i<sizeof invalid_wait/sizeof invalid_wait[0];++i) {
        s_navigation_wait_pending=1U;navigation_feed(invalid_wait[i]);assert(!s_navigation.stabilization.received);
    }
    s_navigation_wait_pending=1U;clock_us=200000;
    navigation_feed("NAVW,1,1,7,1000,2,80,4,4,60,0,0,0,3,2,0,0\n");
    assert(!s_navigation.stabilization.received);
    pass("AUTO-2E malformed, mismatched, incomplete, overflowing and expired wait details cannot contaminate a cached session");
    reset();Uart_Init();clock_us=1000000;Uart_PollNavigation();navigation_feed(navigation_line);
    clock_us=1099999;Uart_PollNavigation();assert(tx_count==1U);
    clock_us=1100000;Uart_PollNavigation();assert(tx_count==2U && !strcmp(frames[1],"NAV Q=1 S=1\n"));
    Uart_PollNavigation();assert(tx_count==2U);
    clock_us=2000000;Uart_PollNavigation();assert(tx_count==3U && !strcmp(frames[2],"NAV Q=2\n"));
    pass("AUTO-2E optional detail query is one-shot after 100 ms, without bursts or changing the one-Hz base poll");
    reset();Uart_Init();clock_us=1000000;Uart_PollNavigation();navigation_feed(navigation_line);
    clock_us=1100000;wait_hook=stop_during_send;Uart_PollNavigation();assert(tx_count==1U);
    WebCommands_ProcessPending();assert(tx_count==2U && !strcmp(frames[1],"STOP\n"));
    manual_ready();Uart_Init();Stm32Navigation_BeginQuery();navigation_feed(navigation_line);
    clock_us+=100000;Uart_PollNavigation();assert(!tx_count && !s_navigation_wait_pending);
    pass("AUTO-2E STOP after TX wait and active manual lease also suppress the complementary query");
    reset();feed("STATE=1 FAULTS=0 STOP=0\n");s_navigation_counter=90U;Stm32Navigation_BeginQuery();
    const char *base_wire=getenv("GAR_SETTLE_BASE"),*detail_wire=getenv("GAR_SETTLE_DETAIL");
    navigation_feed(base_wire?base_wire:"NAV,1,91,2,9,1000,3,3,0,831,160,20,20000,0,295,487,7,0,1,1,0,-,-,0,0\n");navigation_feed("\n");
    Stm32Navigation_BeginWaitQuery();
    navigation_feed(detail_wire?detail_wire:"NAVW,1,91,9,1000,2,80,4,4,60,0,0,1100,3,2,1080,100\n");navigation_feed("\n");
    navigation_api();assert(s_navigation.stabilization.received && strstr(reply,"\"state\":\"confirmed\""));
    printf("NAV_SETTLE_JSON %s\n",reply);
    pass("AUTO-2E actual STM32 paired frames reach the real ESP32 parser and complete JSON without changing old fields");
    reset();Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,2,4294967295,4294967295,2,5,4,263,4294967295,4294967295,200000000,0,4000,4000,15,4294967295,4294967295,4294967295,0,-,-,4294967295,4294967295\n");
    Stm32Navigation_BeginWaitQuery();
    navigation_feed("NAVW,1,1,4294967295,4294967295,3,4294967295,65535,65535,4294967295,1,1,-,-,-,-,4294967295\n");
    navigation_api();assert(strstr(reply,"\"state\":\"timeout\"") && strstr(reply,"\"last_phase\":\"DETENER_Y_ESPERAR\""));
    assert(reply_length<NAVIGATION_JSON_CAPACITY && reply_length==strlen(reply));
    pass("AUTO-2E wait timeout and retained numeric limits fit the existing static HTTP buffer");
    reset();Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,2,4294967295,4294967295,2,3,10,1023,4294967295,4294967295,200000000,200000000,4000,4000,15,4294967295,4294967295,4294967295,7,-9223372036854775808,9223372036854775807,4294967295,4294967295\n");
    Stm32Navigation_BeginWaitQuery();
    navigation_feed("NAVW,1,1,4294967295,4294967295,2,4294967295,65535,65535,4294967295,1,1,4294967295,-9223372036854775808,9223372036854775807,4294967295,4294967295\n");
    clock_us=INT64_MAX;navigation_api();
    assert(strstr(reply,"\"reference_left_counts\":\"-9223372036854775808\"") && reply_length<NAVIGATION_JSON_CAPACITY);
    printf("NAV_SETTLE_LIMIT_JSON %s\n",reply);
    pass("AUTO-2E all fields and full int64 reference with maximum numeric ages fit without truncating the old JSON contract");
    reset();feed("STATE=1 FAULTS=0 STOP=0\n");Stm32Navigation_BeginQuery();navigation_feed(navigation_line);
    Stm32Navigation_BeginWaitQuery();
    navigation_feed("NAVW,1,1,7,1000,2,80,4,4,60,0,0,1100,3,2,1080,100\n");
    assert(s_navigation.stabilization.received);clock_us=1000000;
    char terminal_again[256];uint32_t next_query=Stm32Navigation_BeginQuery();
    snprintf(terminal_again,sizeof terminal_again,"NAV,1,%u%s",next_query,navigation_line+7);
    navigation_feed(terminal_again);navigation_api();
    assert(s_navigation.stabilization.received && strstr(reply,"\"stabilization\":{\"available\":true"));
    assert(s_navigation.stabilization.received_us==0 && strstr(reply,"\"snapshot_age_ms\":1100"));
    pass("AUTO-2F repeated terminal NAV cannot erase real stabilization details or rejuvenate their acquisition age");
    clock_us=2000000;feed("STATE=0 FAULTS=0 STOP=0\n");next_query=Stm32Navigation_BeginQuery();
    snprintf(terminal_again,sizeof terminal_again,"NAV,1,%u%s",next_query,navigation_line+7);
    navigation_feed(terminal_again);Stm32Navigation_QueryFailed();navigation_api();
    assert(s_navigation.stabilization.received && strstr(reply,"\"snapshot_age_ms\":2100"));
    clock_us+=3000000;navigation_api();
    assert(strstr(reply,"\"stale\":true") && s_navigation.stabilization.received);
    Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,4,2,8,2000,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,-,-,0,0\n");navigation_api();
    assert(!s_navigation.stabilization.received && strstr(reply,"\"stabilization\":{\"available\":false"));
    pass("AUTO-2F lost detail replies and stale links keep labelled historical data, while new sessions clear it");
    reset();Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,2,17,1000,3,3,0,831,100,20,20000,0,270,487,7,0,1,1,0,-,-,0,0\n");
    Stm32Navigation_BeginWaitQuery();
    navigation_feed("NAVW,1,1,17,1000,2,80,4,4,60,0,0,1100,3,2,1080,100\n");
    Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,2,2,17,1000,3,3,0,831,300,220,20000,5000,270,487,7,0,1,1,0,-,-,0,0\n");
    assert(s_navigation.stabilization.received && s_navigation.reverse_um==5000U);
    Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,3,2,17,1000,2,3,4,831,880,800,20000,12000,270,487,7,0,1,1,0,-,-,0,0\n");
    assert(s_navigation.stabilization.received && s_navigation.reason==4U && s_navigation.stabilization.reference_left==3);
    Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,4,2,17,1000,3,3,0,831,100,20,20000,0,270,487,7,0,1,1,0,-,-,0,0\n");
    assert(!s_navigation.stabilization.received); /* terminal -> activo no es la misma maniobra */
    pass("AUTO-2F confirmed fixed reverse reference survives progress updates and transition to terminal state in the same session");
    reset();Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,1,2,7,1000,2,5,4,263,200,200,20000,0,270,487,7,0,1,1,0,-,-,0,0\n");
    Stm32Navigation_BeginWaitQuery();navigation_feed("NAVW,1,1,7,1000,3,200,9,1,0,0,0,-,-,-,-,100\n");
    Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,2,2,7,1000,2,5,4,263,200,200,20000,0,270,487,7,0,1,1,0,-,-,0,0\n");
    assert(s_navigation.stabilization.received && s_navigation.stabilization.state==3U);
    reset();Stm32Navigation_BeginQuery();navigation_feed(navigation_line);Stm32Navigation_BeginQuery();
    navigation_feed("NAV,1,2,2,7,1000,2,3,7,319,150,30,50000,0,298,450,7,0,20,90,4,1,0,10,0\n");
    assert(!s_navigation.stabilization.received);
    pass("AUTO-2F immutable wait timeouts are retained but absent NAVW never becomes an invented confirmation");
    reset();navigation_f_seen=navigation_f_bases=navigation_f_parts=navigation_f_retained=0;
    rx_prepare_hook=navigation_f_uart;rx_data_delay=100000;loop_limit=100U;
    if(!setjmp(task_end))app_main();
    assert(navigation_f_bases>=3U && navigation_f_parts>=2U && navigation_f_retained>=1U);
    navigation_api();assert(s_navigation.stabilization.received && strstr(reply,"\"available\":true"));
    assert(s_navigation.reason==4U && s_navigation.reverse_um==11759U);
    pass("AUTO-2F real app_main UART polling and HTTP preserve stabilization through repeated 100-ms reply cycles");
}
