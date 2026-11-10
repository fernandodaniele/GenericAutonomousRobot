/* Grilla y parser reales; geometria y paquetes sinteticos solo en pruebas. */
static void map_reply_version(unsigned version,unsigned part,const char *payload) {
    s_map_part=part;unsigned selected;uint32_t query=Stm32Mapping_BeginQuery(&selected);
    assert(selected==part);char line[192];
    int n=snprintf(line,sizeof line,"%s,%u,%" PRIu32 ",%s\n",part?"RAY":"POSE",version,query,payload);
    assert(n>0&&n<(int)sizeof line);Stm32Mapping_Feed((const uint8_t *)line,(size_t)n);
}
static void map_reply(unsigned part,const char *payload) { map_reply_version(1U,part,payload); }
static unsigned map_occupied(const MappingGrid *g) {unsigned n=0;for(unsigned i=0;i<MAP_CELLS;++i)n+=g->cells[i]==2U;return n;}
static void map_json(void){httpd_req_t req={0};assert(WebServer_Map(&req)==ESP_OK);}
static DWORD WINAPI map_writer(void *arg) {
    (void)arg;
    for(unsigned i=0;i<1000U;++i) {
        char payload[128];snprintf(payload,sizeof payload,"7,%u,2,1,%u,%u,%u,%u,0,0,0,0",i,i,i,i,i);
        map_reply(0,payload);
    }
    return 0;
}
static void test_mapping(void) {
    reset();map_json();assert(strstr(reply,"\"connected\":false")&&strstr(reply,"\"calibrated\":false")&&strstr(reply,"\"x_mm\":null"));
    assert(!map_occupied(&s_map_work.grid)&&s_map_work.grid.path_count==0U);
    pass("W7 no sample produces an unknown bounded grid, null pose and no fictitious path");
    reset();map_reply(0,"7,0,1,0,123,-456,-,-,-,-,-,0");map_json();
    assert(strstr(reply,"\"state\":\"uncalibrated\"")&&strstr(reply,"\"left_counts\":\"123\"")&&strstr(reply,"\"x_mm\":null"));
    map_reply(1,"7,0,0,1,2,10,500,-,-,-,-");assert(!map_occupied(&s_map_work.grid));
    pass("W7 real counts and unlocalized ultrasound survive without inventing a calibrated pose or occupancy");
    reset();map_reply(0,"7,0,2,1,10,10,0,0,0,0,0,0");
    map_reply(1,"7,0,0,1,3,10,500,100,0,0,250");
    assert(map_occupied(&s_map_work.grid)>0U&&s_map_work.grid.path_count==1U);
    unsigned centre=(MAP_HEIGHT/2U)*MAP_WIDTH+MAP_WIDTH/2U;
    assert(s_map_work.grid.cells[centre+5U]==1U&&s_map_work.grid.cells[centre+12U]==2U);
    pass("W7 localized front echo clears a narrow central ray and adds conservative uncertain occupancy arc");
    uint32_t revision=s_map_work.grid.revision;
    map_reply(1,"7,0,0,1,3,10,500,100,0,0,250");assert(s_map_work.grid.revision==revision);
    map_reply(2,"7,0,1,1,3,10,500,-100,0,3142,250");
    assert(s_map_work.grid.cells[centre-12U]==2U);
    pass("W7 per-sensor sequences avoid double insertion; rear origin and yaw project behind the robot");
    revision=s_map_work.grid.revision;
    map_reply(1,"7,10,0,2,1,0,-,-,-,-,-");assert(s_map_work.grid.revision==revision);
    map_reply(1,"7,20,0,3,3,301,1000,0,0,0,250");assert(s_map_work.grid.revision==revision);
    map_reply(1,"7,30,0,4,3,300,1000,0,0,0,250");assert(s_map_work.grid.revision>revision);
    pass("W7 invalid attempts never add free cells; exact 300-ms age accepted and 301 ms rejected");
    uint32_t newest=s_map_work.rays[0].sequence;
    map_reply(1,"7,5,0,2,3,0,700,0,0,0,250");assert(s_map_work.rays[0].sequence==newest);
    pass("W7 old acquisition sequence cannot reactivate earlier occupancy even in a fresh query response");
    map_reply(0,"8,0,1,0,0,0,-,-,-,-,-,0");
    assert(s_map_work.epoch==8U&&!map_occupied(&s_map_work.grid)&&!s_map_work.grid.path_count);
    pass("W7 new STM32 epoch clears old frame, map, path and per-sensor dedup state");
    const char *bad_pose[]={"8,1,2,0,0,0,0,0,0,0,0,0","8,1,1,0,0,0,0,0,0,0,0,0",
        "8,1,2,1,9223372036854775808,0,0,0,0,0,0,0","8,1,2,1,0,0,100001,0,0,0,0,0",
        "8,1,2,1,0,0,0,0,3143,0,0,0","8,1,2,1,0,0,0,0,0,251,0,0","8,1,9,1,0,0,0,0,0,0,0,0",
        "8,1,2,1,+1,0,0,0,0,0,0,0","8,1,2,1,00000000000000000001,0,0,0,0,0,0,0",
        "8,1,2,1,0,0,0,0,0,0,0,0,EXTRA"};
    for(unsigned i=0;i<sizeof bad_pose/sizeof bad_pose[0];++i) {
        uint32_t before=s_map_work.pose.tick;map_reply(0,bad_pose[i]);assert(s_map_work.pose.tick==before&&s_map_pending);
    }
    pass("W7 strict pose parsing rejects inconsistent calibration, overflow, bounds, extra or signed-invalid fields");
    const char *bad_ray[]={"8,0,0,1,3,0,19,0,0,0,250","8,0,0,1,3,0,500,-,-,-,-",
        "8,0,0,0,3,0,500,0,0,0,250","8,0,0,1,3,0,500,0,0,0,0","8,0,1,1,3,0,500,0,0,0,250"};
    for(unsigned i=0;i<sizeof bad_ray/sizeof bad_ray[0];++i) {map_reply(1,bad_ray[i]);assert(s_map_work.rays[0].sequence==0U);}
    pass("W7 range, beam, part, sequence and mandatory ray coordinates validated as a tuple");
    reset();unsigned part;uint32_t q=Stm32Mapping_BeginQuery(&part);char packet[192];
    snprintf(packet,sizeof packet,"STATUS=bad\nPOSE,1,%" PRIu32 ",7,0,2,1,-9223372036854775808,9223372036854775807,0,0,0,0,0,0\r\n",q);
    for(size_t i=0;i<strlen(packet);++i)Stm32Mapping_Feed((const uint8_t *)&packet[i],1U);
    assert(s_map_work.pose.state==2U&&!strcmp(s_map_work.pose.left,"-9223372036854775808"));
    Stm32Mapping_Feed((const uint8_t *)packet,strlen(packet));assert(!s_map_pending);
    pass("W7 complete fragmented CRLF lines, multiple messages and signed 64-bit string counts; duplicate query does not refresh");
    reset();Uart_Init();q=Stm32Mapping_BeginQuery(&part);
    snprintf(packet,sizeof packet,"POSE,1,%" PRIu32 ",7,10,2,1,4,5,0,0,0,0,0,0\nSTATE=2 FAULTS=0 STOP=0\n",q);
    rx_data=packet;rx_size=12;Uart_ReadResponse();assert(!s_map_work.pose.available);
    rx_data=packet+12;rx_size=strlen(packet)-12;Uart_ReadResponse();
    assert(s_map_work.pose.available&&snapshot().state==2U&&rx_count==2U&&!tx_count);
    pass("W7 sole real UART receiver feeds fragmented mapping and STATUS together, preserving the console and parser isolation");
    reset();q=Stm32Mapping_BeginQuery(&part);clock_us=200000;
    snprintf(packet,sizeof packet,"POSE,1,%" PRIu32 ",7,0,2,1,1,1,0,0,0,0,0,0\n",q);
    Stm32Mapping_Feed((const uint8_t *)packet,strlen(packet));assert(!s_map_work.pose.available);
    clock_us=199999;Stm32Mapping_Feed((const uint8_t *)packet,strlen(packet));assert(s_map_work.pose.available&&s_map_work.pose.age==200U);
    pass("W7 reply deadline exact 200 ms rejects; accepted RTT is added conservatively to source age");
    reset();q=Stm32Mapping_BeginQuery(&part);memset(packet,'X',sizeof packet);Stm32Mapping_Feed((const uint8_t *)packet,sizeof packet);
    Stm32Mapping_Feed((const uint8_t *)"\n",1);assert(!s_map_work.pose.available);
    snprintf(packet,sizeof packet,"POSE,1,%" PRIu32 ",7,4294967295,2,1,1,1,0,0,0,0,0,0\n",q);
    Stm32Mapping_Feed((const uint8_t *)packet,strlen(packet));assert(s_map_work.pose.tick==UINT32_MAX);
    map_reply(0,"7,0,2,1,2,2,0,0,0,0,0,0");assert(s_map_work.pose.tick==0U);
    pass("W7 oversized line drains to LF and recovers; source tick wrap preserves the same map epoch");
    clock_us=0;feed("STATE=2 FAULTS=0 STOP=0\n");map_json();assert(strstr(reply,"\"connected\":true"));
    clock_us=501000;map_json();assert(strstr(reply,"\"stale\":true")&&strstr(reply,"\"x_mm\":0"));
    clock_us=1500000;map_json();assert(strstr(reply,"\"connected\":false"));
    clock_us=1500001;map_reply(0,"7,10,2,1,3,3,0,0,0,0,0,0");feed("STATE=2 FAULTS=0 STOP=0\n");map_json();assert(strstr(reply,"\"connected\":true"));
    feed("STATE=2 FAULTS=0 STOP=1\n"); /* Link is still valid; map is never movement permission. */
    assert(Stm32Mapping_Json(reply,sizeof reply,false)>0&&strstr(reply,"\"connected\":false"));
    pass("W7 age, disconnected STATUS and recovery preserve historical geometry without movement or zero substitutions");
    reset();assert(!Stm32Mapping_PollDue());clock_us=100000;assert(Stm32Mapping_PollDue());
    Stm32Mapping_DeferPoll();assert(!Stm32Mapping_PollDue());clock_us=100000000;assert(Stm32Mapping_PollDue());
    Stm32Mapping_DeferPoll();assert(!Stm32Mapping_PollDue());
    pass("W7 optional 100-ms polling is single-flight and never catches up in bursts");
    manual_ready();WebManual_Renew(manual_token,1,UART_MOTION_FORWARD);assert(!WebCommands_CanPollMapping());
    WebManual_Process();assert(WebCommands_CanPollMapping());clock_us+=150000;assert(!WebCommands_CanPollMapping());
    manual_ready();clock_us+=100000;Uart_PollMapping();assert(tx_count==1&&strstr(frames[0],"MAPPING Q=")==frames[0]);
    WebManual_Renew(manual_token,1,UART_MOTION_FORWARD);WebManual_Process();assert(frame_count("AVANZAR\n")==1);
    manual_ready();clock_us+=100000;wait_hook=stop_during_send;Uart_PollMapping();assert(!tx_count);
    WebCommands_ProcessPending();assert(frame_count("STOP\n")==1);
    pass("W7 mapping runs in held MANUAL only with lease margin; direction and STOP win before and after TX gate wait");
    reset();Uart_Init();clock_us=100000;tx_error=1;Uart_PollMapping();assert(!s_map_pending&&tx_count==1);
    tx_error=0;Uart_PollMapping();assert(tx_count==1);clock_us+=100000;Uart_PollMapping();assert(tx_count==2);
    pass("W7 failed UART poll does not accumulate or retry immediately, create leases or move motors");
    reset();Uart_Init();clock_us=100000;feed("STATE=1 FAULTS=0 STOP=0\n");
    Uart_PollMapping();assert(tx_count==1&&strstr(frames[0],"MAPPING Q=")==frames[0]&&!s_manual.active);
    unsigned before_http=tx_count;map_json();assert(tx_count==before_http);
    pass("W7 mapping and HTTP remain readable in AUTO without manual lease or movement emission");
    struct { uint32_t before;MappingGrid grid;uint32_t after; } guarded={.before=0x12345678,.after=0xabcdef12};
    for(int i=-110000;i<=110000;i+=10000)for(int a=-3142;a<=3142;a+=628)
        (void)MappingGrid_Ray(&guarded.grid,i,-i,a,4000,785,0);
    assert(guarded.before==0x12345678&&guarded.after==0xabcdef12);
    for(unsigned i=0;i<MAP_CELLS;++i)assert(guarded.grid.cells[i]<=2U);
    MappingGrid_Clear(&guarded.grid);MappingGrid_Ray(&guarded.grid,100000,100000,0,4000,250,0);
    assert(!map_occupied(&guarded.grid));
    pass("W7 projection clips both map edges without false occupied border, OOB writes or unbounded storage");
    MappingGrid_Clear(&guarded.grid);assert(!MappingGrid_Ray(&guarded.grid,0,0,0,0,250,0));
    MappingGrid_Ray(&guarded.grid,0,0,0,500,250,0);unsigned occupied=map_occupied(&guarded.grid);
    MappingGrid_Ray(&guarded.grid,0,0,0,1500,250,0);assert(map_occupied(&guarded.grid)>=occupied);
    for(unsigned i=0;i<1000;++i)MappingGrid_AddPose(&guarded.grid,(int)i*50,0);
    assert(guarded.grid.path_count==128U&&guarded.grid.path_next<128U);
    pass("W7 free evidence never erases occupied hypotheses, invalid ranges clear nothing and trajectory remains bounded");
    reset();clock_us=10000;feed("STATE=0 FAULTS=0 STOP=0\n");
    HANDLE thread=CreateThread(NULL,0,map_writer,NULL,0,NULL);assert(thread);
    for(unsigned i=0;i<1000U;++i) {
        map_json(); /* JSON is copied/formatted with no network I/O under lock. */
        const char *x=strstr(reply,"\"x_mm\":")+7,*y=strstr(reply,"\"y_mm\":")+7;
        if(strncmp(x,"null",4)) assert(strtol(x,NULL,10)==strtol(y,NULL,10));
    }
    assert(WaitForSingleObject(thread,10000)==WAIT_OBJECT_0);CloseHandle(thread);
    assert(s_map_work.pose.tick==999U);pass("W7 1000 real parser/map publications concurrent with HTTP snapshots; no mixed pose tuples or locked I/O");
    unsigned sent=tx_count;reset();WebServer_Start();assert(!strcmp(routes[13].uri,"/api/map")&&!strcmp(routes[14].uri,"/map.js"));
    httpd_req_t req={0};assert(routes[13].handler(&req)==ESP_OK);assert(routes[14].handler(&req)==ESP_OK);
    assert(tx_count==0&&sent==0);char tiny_buffer[8];assert(Stm32Mapping_Json(tiny_buffer,sizeof tiny_buffer,true)<0);
    pass("W7 GET endpoints are read-only, assets embedded locally, JSON buffers reject truncation");
    reset();feed("STATE=0 FAULTS=0 STOP=0\n");
    map_reply_version(2,0,"7,0,1,0,123,456,-,-,-,-,-,0");map_json();
    assert(strstr(reply,"\"calibration\":\"incomplete\"")&&strstr(reply,"\"calibrated\":false")&&strstr(reply,"\"x_mm\":null"));
    assert(!map_occupied(&s_map_work.grid)&&!s_map_work.grid.path_count);
    pass("W7.4 POSE v2 incomplete geometry exposes real counts, no metric pose and no occupancy");
    map_reply_version(2,0,"7,10,2,1,123,456,0,0,0,0,0,0");map_json();
    assert(strstr(reply,"\"calibration\":\"provisional\"")&&strstr(reply,"\"calibrated\":false")&&strstr(reply,"\"valid\":true"));
    map_reply(1,"7,0,0,1,3,10,500,85,0,0,785");
    assert(map_occupied(&s_map_work.grid)>0U);
    pass("W7.4 complete provisional geometry permits actual metric occupancy but never claims physical calibration");
    map_reply_version(2,0,"7,20,2,2,124,457,10,0,0,10,0,0");map_json();
    assert(strstr(reply,"\"calibration\":\"calibrated\"")&&strstr(reply,"\"calibrated\":true"));
    assert(!map_occupied(&s_map_work.grid)&&s_map_work.grid.path_count==1U);
    pass("W7.4 calibrated status follows explicit class 2 and clears evidence from a previous calibration class");
    const char *bad_class[]={"7,30,2,3,0,0,0,0,0,0,0,0","7,30,2,0,0,0,0,0,0,0,0,0","7,30,1,2,0,0,-,-,-,-,-,0"};
    for(unsigned i=0;i<3U;++i) {map_reply_version(2,0,bad_class[i]);assert(s_map_work.pose.tick==20U);}
    map_reply_version(3,0,"7,30,2,2,0,0,0,0,0,0,0,0");assert(s_map_work.pose.tick==20U);
    pass("W7.4 unknown protocol/classes or inconsistent metric states never refresh or promote calibration");
    map_reply(0,"7,30,2,1,125,458,20,0,0,10,0,0");map_json();
    assert(strstr(reply,"\"calibration\":\"provisional\"")&&strstr(reply,"\"calibrated\":false"));
    pass("W7.4 old POSE v1 remains readable conservatively as provisional, never as full physical calibration");
    clock_us+=501000;map_json();
    assert(strstr(reply,"\"calibration\":\"provisional\"")&&strstr(reply,"\"stale\":true")&&strstr(reply,"\"x_mm\":20"));
    pass("W7.4 approximate calibration labels do not hide stale data or replace historical coordinates");
}
