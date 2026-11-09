/* Included in test_status.c: real parsers, main, HTTP and UART; SDK mocked. */
static void sensors_feed(const char *line) { Stm32Sensors_Feed((const uint8_t *)line,strlen(line)); }
static Stm32SensorsSnapshot sensors_snapshot(void) { Stm32SensorsSnapshot s; Stm32Sensors_GetSnapshot(&s); return s; }
static void sensors_api(void) { httpd_req_t r={0}; assert(WebServer_Sensors(&r)==ESP_OK); printf("W5_JSON %s\n",reply); }
static const char *sensor_line="SENSORS,1,1,1,0,1,345,0,2,-,30,15,0,0\r\n";
static DWORD WINAPI sensors_writer(void *arg) {
    (void)arg;
    for(unsigned i=0;i<10000;++i) {
        char line[96]; uint32_t q=Stm32Sensors_BeginQuery(), mm=100+i%100;
        snprintf(line,sizeof line,"SENSORS,1,%u,1,0,1,%u,0,1,%u,0,15,0,0\n",q,mm,mm+100);
        sensors_feed(line);
    }
    return 0;
}
static void test_sensors(void) {
    reset(); sensors_api();
    assert(strstr(reply,"\"valid\":false") && strstr(reply,"\"distance_mm\":null") && strstr(reply,"\"floor\":[null,null,null,null]"));
    pass("W5 no telemetry is unknown, not zero distance or healthy IR");
    reset(); feed("STATE=2 FAULTS=0 STOP=0\n"); Stm32Sensors_BeginQuery(); sensors_feed(sensor_line); sensors_api();
    assert(strstr(reply,"\"distance_mm\":345") && strstr(reply,"\"state\":\"invalid\"") && strstr(reply,"\"failed\":true"));
    assert(sensors_snapshot().front_age==0 && sensors_snapshot().rear_age==30 && sensors_snapshot().connected);
    pass("W5 valid first tick zero, rear failed attempt independent, no fabricated rear distance");
    for(size_t split=0;split<=strlen(sensor_line);++split) {
        reset(); Stm32Sensors_BeginQuery();
        Stm32Sensors_Feed((const uint8_t *)sensor_line,split);
        Stm32Sensors_Feed((const uint8_t *)sensor_line+split,strlen(sensor_line)-split);
        assert(sensors_snapshot().valid && sensors_snapshot().front_mm==345);
    }
    reset(); Stm32Sensors_BeginQuery();
    for(size_t i=0;i<strlen(sensor_line);++i) Stm32Sensors_Feed((const uint8_t *)sensor_line+i,1);
    assert(sensors_snapshot().valid);
    pass("W5 all fragmentation boundaries and one-byte CRLF reception");
    const char *bad[]={"SENSORS,2,1,1,0,1,345,0,2,-,30,15,0,0\n", "SENSORS,1,1,1,0,1,0,0,2,-,30,15,0,0\n",
        "SENSORS,1,1,1,0,0,-,0,2,-,30,15,0,0\n", "SENSORS,1,1,1,0,1,345,0,2,0,30,15,0,0\n",
        "SENSORS,1,1,1,4294967296,1,345,0,2,-,30,15,0,0\n", "SENSORS,1,1,1,0,1,345,0,2,-,30,16,0,0\n",
        "SENSORS,1,1,1,0,1,345,0,2,-,30,15,0,1\n", "SENSORS,1,1,0,-,0,-,-,0,-,-,0,-,-\n",
        "SENSORS,1,1,1,0,1,345,0,2,-,30,15,0,0,extra\n", "SENSORS,1,1,1,0,1,+345,0,2,-,30,15,0,0\n"};
    for(unsigned i=0;i<sizeof bad/sizeof bad[0];++i) { reset(); Stm32Sensors_BeginQuery(); sensors_feed(bad[i]); assert(!sensors_snapshot().valid); }
    reset(); Stm32Sensors_BeginQuery(); sensors_feed("SENSORS,1,1,1,0,1,345"); assert(!sensors_snapshot().valid);
    char huge[160]; memset(huge,'x',sizeof huge); Stm32Sensors_Feed((uint8_t *)huge,sizeof huge); sensors_feed("\n");
    const uint8_t nul[]={ 'S','E','N','S','O','R','S',0,',','1','\n' }; Stm32Sensors_Feed(nul,sizeof nul);
    sensors_feed(sensor_line); assert(sensors_snapshot().valid);
    pass("W5 malformed/overflow/NUL/partial fields rejected, bounded parser resynchronizes at LF");
    reset(); Stm32Sensors_BeginQuery(); sensors_feed("STATE=2 FAULTS=0 STOP=0\nPONG\nMOVE_ACCEPTED\n"); assert(!sensors_snapshot().valid);
    Stm32Status_Feed((const uint8_t *)sensor_line,strlen(sensor_line)); assert(!snapshot().valid);
    sensors_feed(sensor_line); clock_us=50000; sensors_feed(sensor_line); assert(sensors_snapshot().age_ms==50);
    Stm32Sensors_BeginQuery(); sensors_feed(sensor_line); assert(sensors_snapshot().query==1);
    pass("W5 STATUS/acks are not sensor data; duplicate and wrong-query replies never rejuvenate cache");
    reset(); Stm32Sensors_BeginQuery(); clock_us=200000; sensors_feed(sensor_line); assert(!sensors_snapshot().valid);
    reset(); clock_us=100000; Stm32Sensors_BeginQuery(); clock_us+=19000; sensors_feed(sensor_line);
    assert(sensors_snapshot().front_age==19 && sensors_snapshot().rear_age==49);
    clock_us+=100000; assert(sensors_snapshot().front_age==119);
    pass("W5 200-ms reply deadline, transport RTT and cache age added without cross-MCU clock subtraction");
    for(unsigned age=300;age<=900;age+=300) {
        reset(); feed("STATE=2 FAULTS=0 STOP=0\n"); Stm32Sensors_BeginQuery();
        char line[96]; snprintf(line,sizeof line,"SENSORS,1,1,1,%u,1,345,%u,1,330,%u,15,%u,0\n",age,age,age,age);
        sensors_feed(line); sensors_api();
        assert(!strstr(reply,"\"stale\":true") && strstr(reply,"\"limits_ms\":{\"snapshot\":1500,\"ultrasound\":1500,\"ir\":1500}"));
    }
    pass("W5.8 typical 300/600/900-ms sensor ages remain current under centralized monitor limits");
    reset(); feed("STATE=2 FAULTS=0 STOP=0\n"); Stm32Sensors_BeginQuery(); sensors_feed(sensor_line);
    clock_us=1499999; assert(sensors_snapshot().connected); clock_us=1500000; assert(!sensors_snapshot().connected);
    sensors_api(); assert(strstr(reply,"\"distance_mm\":345") && strstr(reply,"\"connected\":false"));
    pass("W5.8 unchanged link deadline at 1500 ms, old readings retained explicitly historical");
    Stm32Sensors_BeginQuery(); sensors_feed("SENSORS,1,2,1,0,1,345,0,1,330,0,15,0,0\n"); sensors_api();
    assert(sensors_snapshot().connected && !strstr(reply,"\"stale\":true"));
    pass("W5.8 new correlated telemetry recovers monitoring without discarding or inventing readings");
    reset(); feed("STATE=2 FAULTS=0 STOP=0\n"); Stm32Sensors_BeginQuery();
    sensors_feed("SENSORS,1,1,1,0,1,345,1500,1,330,1501,15,1500,0\n"); sensors_api();
    assert(strstr(reply,"\"front\":{\"state\":\"valid\",\"valid\":true,\"failed\":false,\"stale\":false") &&
           strstr(reply,"\"rear\":{\"state\":\"valid\",\"valid\":true,\"failed\":false,\"stale\":true") &&
           strstr(reply,"\"ir\":{\"available\":true,\"stale\":false"));
    Stm32Sensors_BeginQuery(); sensors_feed("SENSORS,1,2,1,1501,1,345,0,1,330,0,15,0,0\n"); sensors_api();
    assert(strstr(reply,"\"front\":{\"state\":\"valid\",\"valid\":true,\"failed\":false,\"stale\":true") &&
           strstr(reply,"\"ir\":{\"available\":true,\"stale\":true"));
    Stm32Sensors_BeginQuery(); sensors_feed("SENSORS,1,3,1,4294967295,1,4000,4294967295,1,4000,4294967295,15,4294967295,0\n");
    clock_us=50000; sensors_api(); assert(strstr(reply,"\"age_ms\":4294967295"));
    char tiny[10]; assert(Stm32Sensors_Json(tiny,sizeof tiny,true,false)==-1);
    pass("W5 exact per-sensor age boundaries and saturated 32-bit ages fit valid bounded JSON; truncation rejected");
    manual_ready(); Stm32Sensors_BeginQuery(); sensors_feed(sensor_line); sensors_api();
    assert(strstr(reply,"\"paused\":true") && !strstr(reply,"\"stale\":true"));
    clock_us+=1500000; sensors_api(); assert(strstr(reply,"\"paused\":true") && strstr(reply,"\"stale\":true"));
    s_status.valid=false; sensors_api();
    assert(strstr(reply,"\"stm32_connected\":false") && strstr(reply,"\"paused\":true") && strstr(reply,"\"stale\":true"));
    pass("W5.8 pause does not erase historical ages or hide loss detected by STATUS");
    reset(); Stm32Sensors_BeginQuery(); sensors_feed("SENSORS,1,1,0,-,0,-,-,0,-,-,-,-,-\n"); sensors_api();
    assert(strstr(reply,"\"snapshot_available\":false") && strstr(reply,"\"age_ms\":null"));
    for(unsigned mask=0;mask<16;++mask) {
        uint32_t q=Stm32Sensors_BeginQuery(); char line[96];
        snprintf(line,sizeof line,"SENSORS,1,%u,1,0,0,-,-,0,-,-,%u,0,%u\n",q,mask,mask!=15);
        sensors_feed(line); assert(sensors_snapshot().ir_mask==mask && sensors_snapshot().cliff==(mask!=15));
    }
    pass("W5 no source snapshot vs no individual result; all 16 floor masks preserve TCRT logic");
    reset(); clock_us=500000; Uart_PollSensors(); assert(tx_count==1 && !strcmp(frames[0],"SENSORS Q=1\n"));
    Uart_PollSensors(); clock_us+=499999; Uart_PollSensors(); assert(tx_count==1);
    ++clock_us; Uart_PollSensors(); assert(tx_count==2); clock_us+=10000000; Uart_PollSensors(); Uart_PollSensors(); assert(tx_count==3);
    pass("W5 500-ms polling, no catch-up bursts or retry of a lost response");
    manual_ready(); s_sensor_poll_us=clock_us-500000; Uart_PollSensors(); assert(tx_count==0);
    WebManual_Renew(manual_token,1,UART_MOTION_FORWARD); WebManual_Process(); assert(tx_count==1 && !strcmp(frames[0],"AVANZAR\n"));
    WebManual_End(manual_token); clock_us+=500000; Uart_PollSensors(); assert(tx_count==1);
    WebManual_Process(); assert(tx_count==2 && !strcmp(frames[1],"DETENER\n"));
    reset(); clock_us=500000; stop_during_send(); Uart_PollSensors(); assert(tx_count==0); WebCommands_ProcessPending(); assert(!strcmp(frames[0],"STOP\n"));
    reset(); clock_us=500000; wait_hook=stop_during_send; Uart_PollSensors(); assert(tx_count==0); WebCommands_ProcessPending(); assert(!strcmp(frames[0],"STOP\n"));
    pass("W5 manual active, DETENER and STOP suppress polling, including STOP concurrent with TX-gate wait");
    reset(); clock_us=500000; gate_fail=true; Uart_PollSensors(); gate_fail=false; Uart_PollSensors(); assert(tx_count==0);
    clock_us+=500000; tx_error=1; Uart_PollSensors(); assert(tx_count==1 && !s_sensor_pending); Uart_PollSensors(); assert(tx_count==1);
    pass("W5 UART error does not renew manual authority or retry continuously");
    reset(); feed("STATE=2 FAULTS=0 STOP=0\n"); Stm32Sensors_BeginQuery();
    rx_data=sensor_line; rx_size=strlen(sensor_line); Uart_ReadResponse(); assert(sensors_snapshot().valid && rx_count==1 && snapshot().valid);
    sensors_api(); assert(!tx_count); unsigned before=tx_count;
    for(int error=1;error<=3;++error) { http_error=error; httpd_req_t r={0}; assert(WebServer_Sensors(&r)==ESP_FAIL); }
    assert(tx_count==before); pass("W5 sole real UART reader feeds both parsers; HTTP errors/read-only API never transmit UART");
    reset(); HANDLE thread=CreateThread(NULL,0,sensors_writer,NULL,0,NULL); assert(thread);
    for(unsigned i=0;i<10000;++i) { Stm32SensorsSnapshot s=sensors_snapshot(); if(s.valid) assert(s.rear_mm==s.front_mm+100); }
    assert(WaitForSingleObject(thread,10000)==WAIT_OBJECT_0); CloseHandle(thread);
    assert(sensors_snapshot().rear_mm==sensors_snapshot().front_mm+100);
    pass("W5 10000 concurrent real parser/cache copies have consistent front/rear tuples without locked I/O");
}
