/* Real new endpoint/parser/arbiter; all UART and clock inputs are synthetic. */
static const char *ex_fixture="EXPL,1,1,1,1010,6,2,1,4,0,0,0,0,0,0,0,1400,1,800,800,1,0,2,4,80\n";
static const char *ex_owner="123456789abcdef0123456789abcdef0";
static esp_err_t ex_post(const char *body){request_body=body;body_offset=0;httpd_req_t r={.content_len=strlen(body)};return WebServer_ExplorationWatch(&r);}
static void ex_empty_sectors(uint32_t q){
    char frame[256];int n=snprintf(frame,sizeof frame,"EXPS,1,%u,%u,%u,",q,exploration_staged.v[2],exploration_staged.v[3]);
    memset(frame+n,'0',36);n+=36;frame[n++]=',';memset(frame+n,'0',108);n+=108;frame[n++]='\n';
    Stm32Exploration_Feed((const uint8_t*)frame,(size_t)n);
}
static void ex_native_feed(void){
    unsigned part;Stm32Exploration_Query(&part);assert(part==0U);
    Stm32Exploration_Feed((const uint8_t*)ex_fixture,strlen(ex_fixture));assert(exploration_staged.received);
    uint32_t q=Stm32Exploration_Query(&part);assert(part==1U);ex_empty_sectors(q);assert(exploration.received);
}
/* Real parser inputs; the peer below models only UART/STM32 communication, not motion. */
static uint32_t ex_manager_sid=1U;
static void ex_manager_reply(uint32_t key,uint32_t proof,unsigned state,unsigned faults){
    unsigned part;uint32_t q=Stm32Exploration_Query(&part);assert(part==0U);char line[256];
    int n=snprintf(line,sizeof line,"EXPL,1,%u,%u,1010,%u,0,1,0,0,0,0,0,0,0,0,0,0,682,800,1,%u,2,4,80,%u,0,%u,%u,%u,0\n",q,ex_manager_sid,state,faults,key,proof,proof?1U:0U,state==2U?16U:0U);
    assert(n>0 && n<256);Stm32Exploration_Feed((const uint8_t*)line,(size_t)n);
    Stm32Exploration_Query(&part);assert(part==1U);ex_empty_sectors(q);
}
static void ex_manager_start(void){
    auto_ready();ex_manager_sid=1U;request_explore_owner=ex_owner;assert(post("AUTO_EXPLORE",12)==ESP_OK);
    WebCommands_ProcessPending();assert(exploration_owner.key && !exploration_owner.active);
    feed("STATE=1 FAULTS=0 STOP=0\n");ex_manager_reply(exploration_owner.key,0,6,0);assert(exploration_owner.active);
}
static void ex_stop_during_start(void){request_id="bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";assert(post("STOP",4)==ESP_OK);}
static unsigned ex_uart_seen,ex_uart_status,ex_uart_queries,ex_uart_drop;
static uint32_t ex_uart_key,ex_uart_previous,ex_uart_proof;
static int64_t ex_uart_tick,ex_uart_issued;
static bool ex_uart_ready,ex_uart_cancelled;
static char ex_uart_reply[256];
static void ex_uart_schedule(void){
    if(ex_uart_key && clock_us-ex_uart_tick>=3000000)ex_uart_cancelled=true;
    unsigned q,part,key,ack;size_t used=0;
    while(ex_uart_seen<tx_count){
        const char *f=frames[ex_uart_seen++];assert(strncmp(f,"EXPLORE K=",10U)!=0);
        if(sscanf(f,"AUTO_EXPLORE K=%u",&key)==1){ex_uart_key=key;ex_uart_tick=clock_us;ex_uart_previous=ex_uart_proof=0;ex_uart_ready=ex_uart_cancelled=false;}
        else if(!strcmp(f,"STATUS\n")){++ex_uart_status;used+=(size_t)snprintf(ex_uart_reply+used,sizeof ex_uart_reply-used,"STATE=1 FAULTS=0 STOP=0\n");}
        else if(sscanf(f,"EXPLORATION Q=%u S=%u",&q,&part)==2){
            bool dropped=ex_uart_drop && iterations>=ex_uart_drop && iterations<900U;
            if(!part){
                assert(sscanf(f,"EXPLORATION Q=%u S=%u K=%u A=%u",&q,&part,&key,&ack)==4);++ex_uart_queries;
                if(!ex_uart_cancelled && key && key==ex_uart_key){
                    if(ack && ack==ex_uart_previous && clock_us-ex_uart_issued<1000000){ex_uart_tick=clock_us;ex_uart_proof=ack;ex_uart_ready=true;}
                    ex_uart_previous=q;ex_uart_issued=clock_us;
                }
                if(!dropped)used+=(size_t)snprintf(ex_uart_reply+used,sizeof ex_uart_reply-used,"EXPL,1,%u,1,1010,%u,1,1,%u,5,0,0,0,0,0,0,0,0,682,800,1,0,2,4,80,%u,%u,%u,%u,%u,0\n",q,ex_uart_cancelled?2U:1U,ex_uart_cancelled?12U:0U,ex_uart_key,(unsigned)((clock_us-ex_uart_tick)/1000),ex_uart_proof,ex_uart_ready?1U:0U,ex_uart_cancelled?16U:0U);
            }else if(!dropped){
                used+=(size_t)snprintf(ex_uart_reply+used,sizeof ex_uart_reply-used,"EXPS,1,%u,1,1010,",q);
                memset(ex_uart_reply+used,'0',36);used+=36;ex_uart_reply[used++]=',';memset(ex_uart_reply+used,'0',108);used+=108;ex_uart_reply[used++]='\n';
            }
        }
    }
    assert(used<sizeof ex_uart_reply);rx_data=ex_uart_reply;rx_size=used;
}
static DWORD WINAPI ex_writer(void *arg){
    (void)arg;
    for(unsigned i=1;i<=1000U;++i){
        exploration_second=false;unsigned part;uint32_t q=Stm32Exploration_Query(&part);char line[256];
        snprintf(line,sizeof line,"EXPL,1,%u,1,1010,6,2,1,4,%u,0,0,0,0,0,0,1400,%u,800,800,1,0,2,4,80\n",q,i,i);
        Stm32Exploration_Feed((const uint8_t*)line,strlen(line));Stm32Exploration_Query(&part);ex_empty_sectors(q);
    }return 0;
}
static void test_exploration(void){
    auto_ready();request_explore_owner=NULL;assert(post("AUTO_EXPLORE",12)==ESP_OK && !strncmp(response_status,"400",3));assert(!tx_count);
    request_explore_owner=ex_owner;assert(post("AUTO_EXPLORE",12)==ESP_OK && !strncmp(response_status,"202",3));
    assert(!tx_count);WebCommands_ProcessPending();assert(tx_count==1U && !strncmp(frames[0],"AUTO_EXPLORE K=",15U));
    assert(exploration_owner.key && !strcmp(exploration_owner.owner,ex_owner));api();assert(!strstr(reply,ex_owner));
    pass("LOCAL explicit closed POST with private owner capability, deferred UART and no token exposure in status receipts");
    uint32_t key,ack;Stm32Exploration_LinkRequest(&key,&ack);assert(key==exploration_owner.key && !ack);
    ex_manager_reply(key,0,6,0);assert(exploration_owner.active);uint32_t received_query=exploration.v[1];
    Stm32Exploration_LinkRequest(&key,&ack);assert(ack==received_query);
    clock_us+=500000;Uart_PollExploration();assert(tx_count==2U && strstr(frames[1]," S=0 K=") && strstr(frames[1]," A=1\n"));
    for(unsigned i=0;i<10U;++i)assert(ex_post("WATCH 1 0")==ESP_OK && !strncmp(response_status,"412",3));
    assert(tx_count==2U);request_explore_owner="ffffffffffffffffffffffffffffffff";assert(ex_post("WATCH 2 0")==ESP_OK && !strncmp(response_status,"412",3));
    pass("LOCAL explicit session has no periodic permission frames; diagnostic requests acknowledge only a complete correlated reply and legacy WATCH is inert");
    const char *bad[]={"WATCH -1 0","WATCH 4294967296 0","WATCH 2 0 STOP","WATCH 2 0\n","WATCH 2","AUTO_EXPLORE","WATCH 2 -1"};
    request_explore_owner=ex_owner;for(unsigned i=0;i<sizeof bad/sizeof bad[0];++i){ex_post(bad[i]);assert(!strncmp(response_status,"400",3));}
    request_origin="http://foreign";assert(ex_post("WATCH 2 0")==ESP_FAIL && !strncmp(response_status,"403",3));request_origin=request_host="http://bad";
    pass("LOCAL strict Origin/Host, size and unsigned decimal grammar reject arbitrary UART, overflow and malformed renewals");
    for(size_t split=0;split<=strlen(ex_fixture);++split){
        reset();unsigned part;Stm32Exploration_Query(&part);
        Stm32Exploration_Feed((const uint8_t*)ex_fixture,split);Stm32Exploration_Feed((const uint8_t*)ex_fixture+split,strlen(ex_fixture)-split);assert(exploration_staged.received && !exploration.received);
    }
    pass("LOCAL status packet tolerates all fragmentation boundaries and preserves framed numeric fields");
    reset();unsigned part;Stm32Exploration_Query(&part);Stm32Exploration_Feed((const uint8_t*)ex_fixture,strlen(ex_fixture));char sectors[256];Stm32Exploration_Query(&part);assert(part==1U);
    strcpy(sectors,"EXPS,1,1,1,1010,");unsigned offset=(unsigned)strlen(sectors);
    memset(sectors+offset,'3',36);offset+=36;sectors[offset++]=',';
    for(unsigned i=0;i<36;++i){memcpy(sectors+offset,"050",3);offset+=3;}sectors[offset++]='\r';sectors[offset++]='\n';sectors[offset]=0;
    Stm32Exploration_Feed((const uint8_t*)sectors,offset);assert(exploration.sectors_received);
    char json[2048];feed("STATE=1 FAULTS=0 STOP=0\n");int n=Stm32Exploration_Json(json,sizeof json);assert(n>0 && n<2048 && strstr(json,"\"connected\":true") && strstr(json,"800,800"));
    assert(Stm32Exploration_Json(json,100)<0);httpd_req_t request={0};assert(WebServer_Exploration(&request)==ESP_OK);printf("LOCAL_JSON %s\n",reply);
    pass("LOCAL coherent sector snapshot and bounded read-only JSON serialize real measured cells, no motor I/O and no stack response buffer");
    for(unsigned fault=0;fault<7U;++fault){
        reset();unsigned p;Stm32Exploration_Query(&p);char line[256];strcpy(line,ex_fixture);
        if(fault==0)line[5]='2';
        if(fault==1)line[7]='2';
        if(fault==2)strcpy(line,"EXPL,1,1,1,1010,7,2,1,4,0,0,0,0,0,0,0,1400,1,800,800,1,0\n");
        if(fault==3)strcpy(line,"EXPL,1,1,1,1010,6,2,1,4,0,0,0,0,0,0,0,1501,1,800,800,1,0\n");
        if(fault==4)clock_us+=300000;
        if(fault==5)line[strlen(line)-1U]=',';
        if(fault==6){memset(line,'X',255);line[255]=0;Stm32Exploration_Feed((const uint8_t*)line,255);}
        Stm32Exploration_Feed((const uint8_t*)line,strlen(line));assert(!exploration.received);
    }
    pass("LOCAL strict version/query/ranges/deadline/overflow guards do not rejuvenate malformed or incomplete evidence");
    ex_manager_start();uint32_t manager_key=exploration_owner.key;
    clock_us+=10000000;feed("STATE=1 FAULTS=0 STOP=0\n");
    assert(Stm32Exploration_Fast() && exploration_owner.active && exploration_owner.key==manager_key);
    pass("LOCAL ESP32 has no 1200/1500-ms authorization expiry; only the independent STM32 link watchdog can cancel on communication loss");
    request_id="bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";assert(post("STOP",4)==ESP_OK && !exploration_owner.key);
    WebCommands_ProcessPending();assert(frame_count("STOP\n")==1U);
    ex_manager_reply(manager_key,1,6,0);Stm32Exploration_LinkRequest(&key,&ack);assert(!key && !ack);
    pass("LOCAL STOP after reconnection clears diagnostic peer identity immediately; old replies cannot recreate a cancelled session");
    for(unsigned fault=0;fault<3U;++fault){
        ex_manager_start();manager_key=exploration_owner.key;
        if(fault==0U)ex_manager_reply(manager_key,1,2,0);
        if(fault==1U)ex_manager_reply(manager_key+1U,1,6,0);
        if(fault==2U){ex_manager_sid=2U;ex_manager_reply(manager_key,1,6,0);}
        assert(!exploration_owner.key);ex_manager_reply(manager_key,2,6,0);Stm32Exploration_LinkRequest(&key,&ack);assert(!key && !ack);
    }
    pass("LOCAL STM32 terminal state or changed session removes correlation; replayed data cannot resume it");
    ex_manager_start();clock_us+=500000;tx_error=1;Uart_PollExploration();assert(!exploration_owner.key);
    pass("LOCAL short UART writes discard peer correlation without retrying control; subsequent diagnosis cannot fabricate a received round trip");
    auto_ready();request_explore_owner=ex_owner;post("AUTO_EXPLORE",12);tx_hook=ex_stop_during_start;WebCommands_ProcessPending();
    assert(!exploration_owner.key);WebCommands_ProcessPending();assert(frame_count("STOP\n")==1U);
    pass("LOCAL STOP racing the start write still invalidates its epoch before creating local correlation");
    ex_manager_start();manager_key=exploration_owner.key;reset();feed("STATE=1 FAULTS=0 STOP=0\n");
    ex_manager_reply(manager_key,1,6,0);Stm32Exploration_LinkRequest(&key,&ack);assert(!key && !ack);
    pass("LOCAL ESP32 reboot never adopts a nonce from telemetry: diagnosis is readable, but old session communication is not resumed");
    for(unsigned loss=0;loss<2U;++loss){
        auto_ready();request_explore_owner=ex_owner;post("AUTO_EXPLORE",12);
        ex_uart_seen=ex_uart_status=ex_uart_queries=0;ex_uart_key=ex_uart_previous=ex_uart_proof=0;ex_uart_drop=loss?300U:0U;ex_uart_ready=ex_uart_cancelled=false;
        rx_prepare_hook=ex_uart_schedule;rx_data_delay=20000;loop_limit=1000U;
        if(!setjmp(task_end))app_main();
        assert(ex_uart_queries>10U && ex_uart_status>10U && task_count==1U);
        assert(loss?(ex_uart_cancelled && !exploration_owner.key):(!ex_uart_cancelled && exploration_owner.active));
        for(unsigned i=0;i<tx_count;++i)assert(strncmp(frames[i],"EXPLORE K=",10U));
        pass("LOCAL real app_main navigational diagnostics use bidirectional proof without permission renewals or HTTP; losing only reply traffic cancels in the simulated STM32 peer, with no restart on recovery");
    }
    auto_ready();request_explore_owner=ex_owner;post("AUTO_EXPLORE",12);ex_uart_seen=ex_uart_status=ex_uart_queries=ex_uart_drop=0;ex_uart_key=ex_uart_previous=ex_uart_proof=0;ex_uart_ready=ex_uart_cancelled=false;
    rx_prepare_hook=ex_uart_schedule;rx_data_delay=20000;loop_limit=200U;if(!setjmp(task_end))app_main();assert(exploration_owner.active);
    int64_t reboot_time=clock_us;reset();clock_us=reboot_time;ex_uart_seen=0;
    char recovered[2048];Stm32Exploration_Json(recovered,sizeof recovered); /* Reconnected GET enables read-only discovery. */
    rx_prepare_hook=ex_uart_schedule;rx_data_delay=20000;loop_limit=400U;if(!setjmp(task_end))app_main();
    assert(ex_uart_cancelled && !exploration_owner.key);
    for(unsigned i=0;i<tx_count;++i)assert(strncmp(frames[i],"AUTO_EXPLORE K=",15U));
    assert(Stm32Exploration_Json(recovered,sizeof recovered)>0 && strstr(recovered,"INTERNAL_LINK_LOST"));
    pass("LOCAL simulated ESP32 reboot can rediscover diagnostics but never recovers the old peer nonce or resumes navigation; the independent STM32 watchdog cancels the session");
    reset();ex_native_feed();feed("STATE=1 FAULTS=0 STOP=0\n");clock_us+=3000000;Stm32Exploration_Json(json,sizeof json);assert(strstr(json,"\"connected\":false"));
    reset();ex_native_feed();unsigned part2;uint32_t query2=Stm32Exploration_Query(&part2);assert(query2==2U && part2==0U);
    char next[256];strcpy(next,ex_fixture);next[7]='2';Stm32Exploration_Feed((const uint8_t*)next,strlen(next));
    assert(exploration.v[1]==1U && exploration.sectors_received && exploration_staged.v[1]==2U);
    Stm32Exploration_Query(&part2);ex_empty_sectors(query2);assert(exploration.v[1]==2U && exploration.sectors_received);
    pass("LOCAL polling a new basic packet retains the last complete sector snapshot until atomic publication of matching supplementary data");
    reset();ex_native_feed();feed("STATE=1 FAULTS=0 STOP=0\n");exploration.v[8]=exploration.v[16];
    HANDLE writer=CreateThread(NULL,0,ex_writer,NULL,0,NULL);assert(writer);
    for(unsigned i=0;i<10000U;++i){
        assert(Stm32Exploration_Json(json,sizeof json)>0);unsigned segments,sequence;
        assert(sscanf(strstr(json,"\"segments\":"),"\"segments\":%u",&segments)==1);
        assert(sscanf(strstr(json,"\"sequence\":"),"\"sequence\":%u",&sequence)==1);assert(segments==sequence);
    }
    assert(WaitForSingleObject(writer,10000)==WAIT_OBJECT_0);CloseHandle(writer);
    pass("LOCAL 1000 real UART publications concurrent with 10000 JSON snapshots preserve atomic field tuples and keep network sends outside locks");
    reset();Stm32Navigation_BeginQuery();navigation_feed("NAV,1,1,6,1,1010,6,6,0,1,400,200,0,0,800,800,7,0,0,0,0,-,-,0,0\n");
    navigation_api();assert(strstr(reply,"\"profile\":\"LOCAL_EXPLORE\"") && strstr(reply,"EXPLORAR"));
    pass("LOCAL existing navigation endpoint accepts the new profile/wait state while preserving earlier profile contracts");
    reset();Stm32Navigation_BeginQuery();navigation_feed("NAV,1,1,6,1,1010,2,1,16,1,3000,0,0,0,682,800,7,0,0,0,0,-,-,0,0\n");
    navigation_api();assert(strstr(reply,"\"finish_reason\":\"INTERNAL_LINK_LOST\"") && strstr(reply,"\"historical\":true"));
    pass("LOCAL navigation HTTP accepts the new communication cause without reusing the legacy supervision-loss meaning");
    const char *base=getenv("GAR_LOCAL_BASE"),*detail=getenv("GAR_LOCAL_SECTORS");
    if(base && detail){
        reset();unsigned p;exploration_counter=90;Stm32Exploration_Query(&p);
        Stm32Exploration_Feed((const uint8_t*)base,strlen(base));Stm32Exploration_Feed((const uint8_t*)"\n",1);assert(exploration_staged.received);
        Stm32Exploration_Query(&p);Stm32Exploration_Feed((const uint8_t*)detail,strlen(detail));Stm32Exploration_Feed((const uint8_t*)"\n",1);assert(exploration.sectors_received);
        assert(Stm32Exploration_Json(json,sizeof json)>0);pass("LOCAL actual STM32 formatter packets interoperate with the real ESP32 parser and JSON");
    }
    base=getenv("GAR_LOCAL_LINK_BASE");detail=getenv("GAR_LOCAL_LINK_DETAIL");
    if(base && detail){
        reset();unsigned p;exploration_counter=100U;Stm32Exploration_Query(&p);
        Stm32Exploration_Feed((const uint8_t*)base,strlen(base));Stm32Exploration_Feed((const uint8_t*)"\n",1);
        Stm32Exploration_Query(&p);Stm32Exploration_Feed((const uint8_t*)detail,strlen(detail));Stm32Exploration_Feed((const uint8_t*)"\n",1);
        feed("STATE=1 FAULTS=0 STOP=0\n");
        assert(Stm32Exploration_Json(json,sizeof json)>0 && strstr(json,"\"link_connected\":true") && strstr(json,"\"link_confirmed\":false") && strstr(json,"\"session_cancelled\":true") && strstr(json,"\"stop_received\":true") && strstr(json,"\"terminal_reason\":\"INTERNAL_LINK_LOST\""));
        pass("LOCAL actual STM32 terminal link-loss packets retain STOP and the original communication cause through ESP32 and HTTP");
    }
}
