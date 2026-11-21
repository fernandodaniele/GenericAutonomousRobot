static void imu_feed(const char *text) { Stm32Imu_Feed((const uint8_t *)text,strlen(text)); }
static void imu_api(void) { httpd_req_t r={0}; assert(WebServer_Imu(&r)==ESP_OK); }
static const char *imu_line="IMU,1,1,2,0,0,0,0,0,16384,131,-131,0\r\n";
/* Exercise actual app_main, TX, sole RX and all real parsers at the UART
 * driver's allowed 100-ms read duration, with valid mapping replies. */
static unsigned loop_replied, loop_imu_mode, loop_imu_count;
static int64_t loop_imu_times[32];
static char loop_response[192];
static void loop_reply(void) {
    if (loop_replied==tx_count) return;
    loop_replied=tx_count;
    const char *frame=frames[tx_count-1U]; uint32_t query; unsigned part; int n=0;
    if (!strcmp(frame,"STATUS\n")) n=snprintf(loop_response,sizeof loop_response,"STATE=3 FAULTS=4 STOP=1\n");
    else if (sscanf(frame,"SENSORS Q=%" SCNu32,&query)==1)
        n=snprintf(loop_response,sizeof loop_response,"SENSORS,1,%" PRIu32 ",1,0,2,-,0,2,-,0,15,0,0\n",query);
    else if (sscanf(frame,"MAPPING Q=%" SCNu32 " S=%u",&query,&part)==2) {
        if (!part) n=snprintf(loop_response,sizeof loop_response,"POSE,2,%" PRIu32 ",7,0,2,1,0,0,0,0,0,0,0,0\n",query);
        else n=snprintf(loop_response,sizeof loop_response,"RAY,1,%" PRIu32 ",7,0,%u,0,0,0,-,-,-,-,-\n",query,part-1U);
    } else if (sscanf(frame,"IMU3 Q=%" SCNu32,&query)==1 || sscanf(frame,"IMU2 Q=%" SCNu32,&query)==1 || sscanf(frame,"IMU Q=%" SCNu32,&query)==1) {
        assert(loop_imu_count<32U); loop_imu_times[loop_imu_count++]=clock_us;
        if (loop_imu_mode==1U)
            n=snprintf(loop_response,sizeof loop_response,"IMU,1,%" PRIu32 ",2,0,0,0,0,0,16384,0,0,0\n",query);
        else if (loop_imu_mode==2U)
            n=snprintf(loop_response,sizeof loop_response,"IMU,1,%" PRIu32 ",3,1,4,-,-,-,-,-,-,-\n",query);
        else if (loop_imu_mode==4U)
            n=snprintf(loop_response,sizeof loop_response,"IMU,1,%" PRIu32 ",4,3,32,500,-,-,-,-,-,-\n",query);
        /* Mode 3 deliberately simulates no IMU UART response at all. */
    }
    assert(n>=0 && (size_t)n<sizeof loop_response);
    if(n) { rx_data=loop_response; rx_size=(size_t)n; }
}
static void run_imu_loop(unsigned mode) {
    reset(); loop_replied=loop_imu_count=0; loop_imu_mode=mode;
    loop_limit=60U; rx_data_delay=100000; rx_prepare_hook=loop_reply;
    if(!setjmp(task_end)) app_main();
    rx_prepare_hook=NULL;
    assert(snapshot().connected && snapshot().state==3U && snapshot().faults==4U && snapshot().stop==1U);
    assert(s_map_work.pose.available && s_sensor_data.valid);
    printf("IMU schedule: %u requests in 60 cycles; mode=%u\n",loop_imu_count,mode);
    assert(loop_imu_count>=8U);
    int64_t min_gap=INT64_MAX, max_gap=0;
    for(unsigned i=1;i<loop_imu_count;++i) {
        int64_t gap=loop_imu_times[i]-loop_imu_times[i-1U];
        assert(gap>=500000 && gap<=880000); /* No bursts; bounded priority delay. */
        if (gap<min_gap) min_gap=gap;
        if (gap>max_gap) max_gap=gap;
    }
    printf("IMU intervals: min=%" PRId64 " max=%" PRId64 " us\n",min_gap,max_gap);
    assert(frame_count("STATUS\n")>=6U);
}
static DWORD WINAPI imu_writer(void *arg) {
    (void)arg;
    for(unsigned n=0;n<10000;++n) {
        uint32_t q=Stm32Imu_BeginQuery(); char line[128]; int sign=n%2?1:-1;
        snprintf(line,sizeof line,"IMU,1,%u,2,0,0,0,%d,%d,%d,%d,%d,%d\n",q,sign*16384,sign*16384,sign*16384,sign*131,sign*131,sign*131);
        imu_feed(line);
    }
    return 0;
}
static void test_imu(void) {
    reset(); imu_api();
    assert(strstr(reply,"\"state\":\"no_data\"") && strstr(reply,"\"sample_age_ms\":null") && strstr(reply,"\"accel_g\":[null,null,null]"));
    pass("IMU API starts without data, invented zeros, I2C operations or UART writes");
    feed("STATE=0 FAULTS=0 STOP=0\n"); Stm32Imu_BeginQuery();
    for(size_t i=0;i<strlen(imu_line);++i)Stm32Imu_Feed((const uint8_t *)imu_line+i,1);
    imu_api();
    assert(strstr(reply,"\"valid\":true") && strstr(reply,"\"accel_g\":[0.000000,0.000000,1.000000]") &&
        strstr(reply,"\"gyro_dps\":[1.000000,-1.000000,0.000000]") && !tx_count);
    pass("IMU strict correlated fragmented CRLF raw axes convert using 16384 LSB/g and 131 LSB/(degree/s)");
    clock_us=100000; imu_feed(imu_line); imu_api(); assert(strstr(reply,"\"sample_age_ms\":100"));
    Stm32Imu_BeginQuery(); imu_feed("STATUS\nIMU,1,2,4,3,32,100,-,-,-,-,-,-\n"); imu_api();
    assert(strstr(reply,"\"state\":\"read_error\"") && strstr(reply,"\"valid\":false") && strstr(reply,"\"i2c_error\":32") && strstr(reply,"\"gyro_dps\":[null,null,null]"));
    pass("IMU duplicate does not rejuvenate data; read failure hides previous measurements and reports explicit bus error");
    const char *bad[]={
        "IMU,2,1,2,0,0,0,0,0,0,0,0,0\n", "IMU,1,2,2,0,0,0,0,0,0,0,0,0\n",
        "IMU,1,1,5,0,0,0,0,0,0,0,0,0\n", "IMU,1,1,2,1,0,0,0,0,0,0,0,0\n",
        "IMU,1,1,2,0,1,0,0,0,0,0,0,0\n", "IMU,1,1,2,0,0,-,0,0,0,0,0,0\n",
        "IMU,1,1,2,0,0,0,32768,0,0,0,0,0\n", "IMU,1,1,2,0,0,0,-32769,0,0,0,0,0\n",
        "IMU,1,1,2,0,0,0,+1,0,0,0,0,0\n", "IMU,1,1,2,0,0,0,0,0,0,0,0\n",
        "IMU,1,1,3,0,0,-,-,-,-,-,-,-\n", "IMU,1,1,3,1,0,-,0,-,-,-,-,-\n",
        "IMU,1,1,3,1,4294967296,-,-,-,-,-,-,-\n", "MOVE_ACCEPTED\n"
    };
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) { reset(); Stm32Imu_BeginQuery(); imu_feed(bad[i]); assert(!s_imu.received); }
    reset(); Stm32Imu_BeginQuery(); char oversized[sizeof(s_imu_line)+1U]; memset(oversized,'x',sizeof oversized);
    Stm32Imu_Feed((const uint8_t *)oversized,sizeof oversized); imu_feed(imu_line); assert(!s_imu.received);
    imu_feed(imu_line); assert(s_imu.received);
    reset(); Stm32Imu_BeginQuery(); const char nul[]="IMU,1,1,2,0,0,0,0,0,0,0,0,0\0\n";
    Stm32Imu_Feed((const uint8_t *)nul,sizeof nul-1); assert(!s_imu.received);
    pass("IMU malformed, incomplete, unknown, mismatched, out-of-range and overflowing responses cannot replace valid state");
    for(unsigned state=0;state<=4;++state) {
        if(state==2)continue;
        reset(); Stm32Imu_BeginQuery(); char line[128];
        snprintf(line,sizeof line,"IMU,1,1,%u,%u,0,-,-,-,-,-,-,-\n",state,state>=3?1:0);
        imu_feed(line); imu_api(); assert(s_imu.received && !strstr(reply,"\"valid\":true") && strstr(reply,"\"sample_age_ms\":null"));
    }
    pass("IMU initialization, ready, wrong identity/config and failed communication remain distinguishable from a valid zero reading");
    reset(); feed("STATE=0 FAULTS=0 STOP=0\n"); Stm32Imu_BeginQuery(); clock_us=199999; imu_feed(imu_line);
    imu_api(); assert(strstr(reply,"\"sample_age_ms\":200"));
    reset(); Stm32Imu_BeginQuery(); clock_us=200000; imu_feed(imu_line); assert(!s_imu.received);
    pass("IMU original sample age includes conservative round-trip transport and rejects responses at the 200-ms deadline");
    reset(); feed("STATE=0 FAULTS=0 STOP=0\n"); Stm32Imu_BeginQuery(); imu_feed(imu_line);
    clock_us=1500000; imu_api(); assert(strstr(reply,"\"valid\":false") && strstr(reply,"\"sample_age_ms\":1500") && strstr(reply,"\"accel_g\":[null,null,null]"));
    feed("STATE=0 FAULTS=0 STOP=0\n"); Stm32Imu_BeginQuery(); imu_feed("IMU,1,2,2,0,0,1500,0,0,0,0,0,0\n");
    imu_api(); assert(strstr(reply,"\"valid\":true")); ++clock_us; imu_api();
    clock_us+=1000; imu_api(); assert(strstr(reply,"\"valid\":false"));
    Stm32Imu_BeginQuery(); imu_feed("IMU,1,3,2,0,0,0,0,0,16384,0,0,0\n"); imu_api(); assert(strstr(reply,"\"valid\":true"));
    s_status.valid=false; imu_api(); assert(strstr(reply,"\"stm32_connected\":false") && strstr(reply,"\"valid\":false"));
    pass("IMU link and sample ages expire independently; new samples recover, STATUS loss immediately hides measurements");
    reset(); clock_us=500000; Uart_PollImu(); Uart_PollImu();
    assert(tx_count==1 && !strcmp(frames[0],"IMU3 Q=1\n")); clock_us+=10000000; Uart_PollImu(); Uart_PollImu(); assert(tx_count==2);
    manual_ready(); clock_us+=500000; Uart_PollImu(); assert(!tx_count);
    reset(); clock_us=500000; wait_hook=stop_during_send; Uart_PollImu(); assert(!tx_count);
    WebCommands_ProcessPending(); assert(!strcmp(frames[0],"STOP\n"));
    reset(); clock_us=500000; tx_error=1; Uart_PollImu(); assert(!s_imu_pending); Uart_PollImu(); assert(tx_count==1);
    pass("IMU 500-ms polling pauses during manual/control, revalidates STOP at TX gate and never retries or catches up");
    reset(); feed("STATE=2 FAULTS=0 STOP=0\n"); Stm32Imu_BeginQuery(); rx_data=imu_line; rx_size=strlen(imu_line); Uart_ReadResponse();
    assert(s_imu.received && rx_count==1 && snapshot().valid);
    assert(WebServer_Start()==ESP_OK && !strcmp(routes[15].uri,"/api/imu") && routes[15].method==HTTP_GET);
    httpd_req_t r={0}; assert(routes[15].handler(&r)==ESP_OK && routes[16].handler(&r)==ESP_OK && !tx_count);
    char tiny[10]; assert(Stm32Imu_Json(tiny,sizeof tiny,true,false)==-1);
    for(int e=1;e<=3;++e) { http_error=e; assert(WebServer_Imu(&r)==ESP_FAIL); }
    pass("real sole UART receiver, read-only IMU endpoint and embedded script preserve W1-W7 routes, LRU and HTTP error propagation");
    reset(); feed("STATE=0 FAULTS=0 STOP=0\n"); HANDLE thread=CreateThread(NULL,0,imu_writer,NULL,0,NULL); assert(thread);
    for(unsigned n=0;n<10000;++n) {
        imu_api();
        if(strstr(reply,"\"valid\":true")) assert(strstr(reply,"\"accel_g\":[1.000000,1.000000,1.000000]") || strstr(reply,"\"accel_g\":[-1.000000,-1.000000,-1.000000]"));
    }
    assert(WaitForSingleObject(thread,10000)==WAIT_OBJECT_0); CloseHandle(thread);
    pass("concurrent actual IMU parser and HTTP reader obtain coherent copies without locks during network formatting");
    run_imu_loop(1U); imu_api(); assert(strstr(reply,"\"received\":true") && strstr(reply,"\"valid\":true"));
    pass("actual scheduler polls IMU despite 100-ms UART reads and continuous successful mapping; Safety fault does not inhibit diagnostics");
    run_imu_loop(2U); imu_api(); assert(strstr(reply,"\"state\":\"init_error\"") && strstr(reply,"\"received\":true") && strstr(reply,"\"valid\":false"));
    pass("actual end-to-end main UART parser and HTTP distinguish sensor I2C initialization failure from missing telemetry");
    run_imu_loop(3U); imu_api(); assert(strstr(reply,"\"state\":\"no_data\"") && strstr(reply,"\"received\":false"));
    pass("actual scheduler transmits IMU requests even when only IMU responses are absent; STATUS and other telemetry continue");
    run_imu_loop(4U); imu_api(); assert(strstr(reply,"\"state\":\"read_error\"") && strstr(reply,"\"valid\":false") && strstr(reply,"\"gyro_dps\":[null,null,null]"));
    pass("actual IMU read-error reply propagates through sole UART reader and HTTP with no old valid measurements");
}

static void imu2_reply(const char *calibration) {
    uint32_t q=Stm32Imu_BeginQuery(); char line[192];
    int n=snprintf(line,sizeof(line),"IMU,2,%" PRIu32 ",2,0,0,0,0,0,16384,131,-262,66,%s\n",q,calibration);
    assert(n>0 && (size_t)n<sizeof(line));imu_feed(line);
}
static esp_err_t imu_cal_post(const char *body) {
    request_body=body;body_offset=0;httpd_req_t r={.content_len=strlen(body)};
    return WebServer_ImuCalibrate(&r);
}
static void imu_cal_ready(void) {
    reset();feed("STATE=3 FAULTS=4 STOP=1\n");imu2_reply("0,0,0,0,-,-,-,-");
}

static void imu3_reply(const char *orientation) {
    uint32_t q=Stm32Imu_BeginQuery();char line[256];
    int n=snprintf(line,sizeof line,"IMU,3,%" PRIu32 ",2,0,0,0,0,0,16384,131,-262,66,2,0,250,7,100,32750,-65500,16500,%s\r\n",q,orientation);
    assert(n>0 && (size_t)n<sizeof line);
    for(int i=0;i<n;++i)Stm32Imu_Feed((const uint8_t *)line+i,1);
}
static DWORD WINAPI imu3_writer(void *arg) {
    (void)arg;
    for(unsigned n=0;n<10000;++n) {
        uint32_t q=Stm32Imu_BeginQuery();char line[256];int sign=n%2?1:-1;
        snprintf(line,sizeof line,"IMU,3,%u,2,0,0,0,0,0,16384,0,0,%d,2,0,250,7,100,0,0,0,1,0,0,0,%d,1,20,80,%d,%d,0,0,1,7\n",
            q,sign*131,sign*90000,sign*88000,sign*88000);
        imu_feed(line);
    }
    return 0;
}
static void test_imu_orientation(void) {
    imu_cal_ready();imu_api();
    assert(strstr(reply,"\"orientation\":{\"available\":false") && strstr(reply,"\"degrees\":null"));
    imu3_reply("1,0,0,0,90000,1,20,80,88000,87000,1000,-30000,1,7");imu_api();
    assert(s_imu.yaw_known && s_imu.yaw_reference && s_imu.yaw_reference_tick==0);
    assert(strstr(reply,"\"degrees\":90.000") && strstr(reply,"\"reference_tick\":0") && strstr(reply,"\"reference_request\":7") &&
        strstr(reply,"\"gyro_delta_deg\":88.000,\"encoder_delta_deg\":87.000,\"difference_deg\":1.000") &&
        strstr(reply,"\"encoder_reference_deg\":-30.000,\"encoder_calibration\":1"));
    pass("IMU-3 fragmented v3 carries unwrapped yaw and timestamp-associated common deltas; legacy v2 remains available without invented angles");
    imu3_reply("1,0,0,0,-90000,2,-,-,-,-,-,-,-,7");imu_api();
    assert(strstr(reply,"\"degrees\":-90.000") && strstr(reply,"\"state\":\"odometry_unavailable\"") && strstr(reply,"\"gyro_delta_deg\":null"));
    pass("IMU-3 negative yaw remains usable when encoder reference/history is unavailable; no subtraction of unrelated origins");
    const char *bad[]={
        "1,0,-,0,90000,2,-,-,-,-,-,-,-,7", "1,0,0,0,90000,2,-,-,-,-,-,-,-,8",
        "1,0,0,0,2147483647,2,-,-,-,-,-,-,-,7", "1,3,0,0,90000,2,-,-,-,-,-,-,-,7",
        "2,0,0,0,-,3,-,-,-,-,-,-,-,7", "0,0,0,0,-,0,-,-,-,-,-,-,-,7",
        "1,0,0,0,90000,1,20,80,180001,87000,1000,0,1,7",
        "1,0,0,0,90000,1,20,80,88000,87000,1000,0,0,7",
        "1,0,0,0,90000,3,-,-,-,-,-,-,-,7", "1,0,0,0,-,2,-,-,-,-,-,-,-,7",
        "1,0,0,0,90000,1,20,80,88000,87000,1000,0,1"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) {
        imu_cal_ready();imu3_reply(bad[i]);assert(!s_imu.yaw_known);
    }
    pass("IMU-3 strict v3 rejects missing references, mismatched calibration IDs, overflow, malformed validity and impossible comparisons");
    imu_cal_ready();imu3_reply("2,3,0,500,-,3,-,-,-,-,-,-,-,7");imu_api();
    assert(strstr(reply,"\"requires_calibration\":true,\"reason\":\"timing\"") && strstr(reply,"\"degrees\":null") && strstr(reply,"\"sample_age_ms\":500"));
    imu3_reply("1,0,5000,0,0,2,-,-,-,-,-,-,-,7");imu_api();
    assert(strstr(reply,"\"degrees\":0.000") && strstr(reply,"\"reference_tick\":5000"));
    pass("IMU-3 discontinuity explicitly requires recalibration and a new reference zero is distinguishable from a hidden angle reset");
    imu_cal_ready();uint32_t q=Stm32Imu_BeginQuery();char line[256];
    snprintf(line,sizeof line,"IMU,3,%" PRIu32 ",2,0,0,0,0,0,16384,0,0,0,2,0,250,7,100,0,0,0,1,0,0,0,90000,1,20,80,88000,87000,1000,0,2,7\n",q);
    clock_us+=199999;imu_feed(line);imu_api();
    assert(strstr(reply,"\"sample_age_ms\":200") && strstr(reply,"\"sample_age_ms\":220") && strstr(reply,"\"degrees\":90.000"));
    clock_us+=100000;imu_feed(line);imu_api();assert(strstr(reply,"\"sample_age_ms\":300"));
    s_status.valid=false;imu_api();assert(strstr(reply,"\"degrees\":null") && strstr(reply,"\"difference_deg\":null"));
    pass("IMU-3 conservatively ages the original integration/comparison samples, duplicate replies cannot rejuvenate them, STATUS loss hides angles");
    imu_cal_ready();imu3_reply("1,0,0,0,90000,1,20,80,88000,87000,1000,0,1,7");clock_us+=1500000;imu_api();
    assert(strstr(reply,"\"degrees\":null") && strstr(reply,"\"difference_deg\":null") && strstr(reply,"\"requires_calibration\":false"));
    feed("STATE=3 FAULTS=4 STOP=1\n");imu3_reply("1,0,0,0,91000,2,-,-,-,-,-,-,-,7");imu_api();assert(strstr(reply,"\"degrees\":91.000"));
    pass("IMU-3 telemetry expiration does not fabricate calibration loss or integrate web gaps; fresh source data alone restores display");
    imu_cal_ready();q=Stm32Imu_BeginQuery();
    snprintf(line,sizeof line,"IMU,3,%" PRIu32 ",4,3,32,100,-,-,-,-,-,-,3,2,250,7,-,-,-,-,2,2,0,100,-,3,-,-,-,-,-,-,-,7\n",q);
    imu_feed(line);imu_api();assert(s_imu.yaw_state==2 && s_imu.cal_state==3 && strstr(reply,"\"requires_calibration\":true,\"reason\":\"i2c\""));
    pass("IMU-3 real API distinguishes received I2C failure and invalid angular reference from missing UART response");
    imu_cal_ready();q=Stm32Imu_BeginQuery();
    snprintf(line,sizeof line,"IMU,3,%" PRIu32 ",2,0,0,0,-32768,-32768,-32768,-32768,-32768,-32768,2,0,250,4294967295,4294967295,-8192000,-8192000,-8192000,1,0,0,0,-2147483646,1,20,0,-180000,-180000,0,-180000,2,4294967295\n",q);
    rx_data=line;rx_size=strlen(line);Uart_ReadResponse();imu_api();
    assert(s_imu.yaw_reference_request==UINT32_MAX && strstr(reply,"\"degrees\":-2147483.646") && strlen(reply)<1792);
    printf("IMU-3 largest-value JSON: %u/1792 bytes\n",(unsigned)strlen(reply));
    pass("IMU-3 large signed fields and full 32-bit reference pass the actual sole UART reader and bounded HTTP response");
    imu_cal_ready();HANDLE thread=CreateThread(NULL,0,imu3_writer,NULL,0,NULL);assert(thread);
    for(unsigned n=0;n<10000;++n) {
        imu_api();
        if(strstr(reply,"\"degrees\":90.000"))assert(strstr(reply,"\"gyro_delta_deg\":88.000,\"encoder_delta_deg\":88.000"));
        if(strstr(reply,"\"degrees\":-90.000"))assert(strstr(reply,"\"gyro_delta_deg\":-88.000,\"encoder_delta_deg\":-88.000"));
    }
    assert(WaitForSingleObject(thread,10000)==WAIT_OBJECT_0);CloseHandle(thread);
    pass("IMU-3 parser and concurrent HTTP obtain coherent orientation/reference/comparison copies using the existing lock");
}
static void test_imu_calibration(void) {
    imu_cal_ready();imu_api();
    assert(strstr(reply,"\"state\":\"pending\"") && strstr(reply,"\"gyro_raw\":[131,-262,66]") && strstr(reply,"\"gyro_corrected_dps\":[null,null,null]"));
    imu2_reply("1,0,100,7,-,-,-,-");imu_api();
    assert(strstr(reply,"\"state\":\"running\"") && strstr(reply,"\"samples\":100") && strstr(reply,"\"offset_dps\":[null,null,null]"));
    imu2_reply("2,0,250,7,0,25125,-65500,16500");imu_api();
    assert(strstr(reply,"\"gyro_dps\":[1.000000,-2.000000,0.503817]") &&
        strstr(reply,"\"gyro_corrected_dps\":[0.232824,0.000000,0.000000]") &&
        strstr(reply,"\"offset_dps\":[0.767176,-2.000000,0.503817]"));
    pass("IMU-2 v2 preserves IMU-1 units and raw axes while exposing pending, running and exact fractional mean correction");
    imu2_reply("3,1,120,7,-,-,-,-");imu_api();
    assert(strstr(reply,"\"state\":\"failed\"") && strstr(reply,"\"reason\":\"motion\"") && strstr(reply,"\"valid\":true") && strstr(reply,"\"gyro_corrected_dps\":[null,null,null]"));
    uint32_t q=Stm32Imu_BeginQuery();char line[192];
    snprintf(line,sizeof line,"IMU,2,%" PRIu32 ",4,3,32,500,-,-,-,-,-,-,3,2,120,7,-,-,-,-\n",q);imu_feed(line);imu_api();
    assert(strstr(reply,"\"reason\":\"i2c\"") && strstr(reply,"\"valid\":false") && strstr(reply,"\"gyro_raw\":[null,null,null]"));
    imu2_reply("3,2,120,7,-,-,-,-");imu_api();
    assert(strstr(reply,"\"valid\":true") && strstr(reply,"\"gyro_corrected_dps\":[null,null,null]"));
    pass("IMU-2 calibration failure never hides useful uncorrected data, and I2C recovery cannot revive invalid offsets");
    const char *bad[]={"2,0,249,7,0,1,2,3","2,0,250,0,0,1,2,3","2,0,250,7,-,1,2,3","2,0,250,7,0,8192001,2,3",
        "1,0,250,7,-,-,-,-","0,0,1,0,-,-,-,-","3,0,1,7,-,-,-,-","3,6,1,7,-,-,-,-","1,0,10,7,0,-,-,-"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) {
        imu_cal_ready();uint32_t before=s_imu.cal_samples;imu2_reply(bad[i]);
        assert(s_imu.cal_state==0 && s_imu.cal_samples==before);
    }
    pass("IMU-2 parser rejects incomplete, contradictory, out-of-range or falsely calibrated v2 metadata");
    imu_cal_ready();imu2_reply("2,0,250,7,0,32750,-65500,16500");clock_us+=1500000;imu_api();
    assert(strstr(reply,"\"gyro_corrected_dps\":[null,null,null]") && strstr(reply,"\"offset_dps\":[null,null,null]"));
    feed("STATE=3 FAULTS=4 STOP=1\n");imu2_reply("2,0,250,7,1500,32750,-65500,16500");imu_api();assert(strstr(reply,"\"gyro_corrected_dps\":[0.000000,0.000000,0.000000]"));
    s_status.valid=false;imu_api();assert(strstr(reply,"\"gyro_corrected_dps\":[null,null,null]"));
    pass("IMU-2 corrected measurements require fresh source data and STATUS, independently of a previously valid calibration");
    imu_cal_ready();assert(WebServer_Start()==ESP_OK && routes[17].method==HTTP_POST && !strcmp(routes[17].uri,"/api/imu/calibrate"));
    assert(imu_cal_post("CALIBRATE")==ESP_OK && !strncmp(response_status,"202",3) && !tx_count);
    assert(imu_cal_post("CALIBRATE")==ESP_OK && !strncmp(response_status,"409",3) && strstr(reply,"replay"));
    request_id="bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";assert(imu_cal_post("CALIBRATE")==ESP_OK && strstr(reply,"busy"));
    assert(Uart_SendImuCalibration() && tx_count==1 && !strncmp(frames[0],"IMUCAL N=",9));
    uint32_t number=s_imu_cal_request.number;
    snprintf(line,sizeof line,"IMUCAL,1,%" PRIu32 ",1\n",number);imu_feed(line);imu_api();
    assert(strstr(reply,"\"request_phase\":\"accepted\"") && strstr(reply,"\"state\":\"pending\""));
    snprintf(line,sizeof line,"1,0,25,%" PRIu32 ",-,-,-,-",number);imu2_reply(line);imu_api();
    assert(strstr(reply,"\"state\":\"running\"") && strstr(reply,"\"samples\":25"));
    pass("IMU-2 explicit POST queues one UART request, deduplicates IDs, rejects concurrent starts and does not confuse ACK with calibration success");
    imu_cal_ready();assert(Stm32Imu_RequestCalibration("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")==0);
    feed("STATE=2 FAULTS=0 STOP=0\n");assert(!Uart_SendImuCalibration() && !tx_count);imu_api();assert(strstr(reply,"\"request_phase\":\"rejected\""));
    imu_cal_ready();assert(Stm32Imu_RequestCalibration("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")==0);
    wait_hook=stop_during_send;assert(!Uart_SendImuCalibration() && !tx_count);WebCommands_ProcessPending();assert(!strcmp(frames[0],"STOP\n"));
    pass("IMU-2 revalidates mode and pending STOP inside UART gate; calibration never displaces operational commands");
    imu_cal_ready();assert(Stm32Imu_RequestCalibration("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")==0);clock_us+=500000;
    assert(!Uart_SendImuCalibration() && !tx_count);imu_api();assert(strstr(reply,"\"request_phase\":\"expired\""));
    imu_cal_ready();assert(Stm32Imu_RequestCalibration("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")==0);tx_error=1;
    assert(!Uart_SendImuCalibration());unsigned sent=tx_count;assert(!Uart_SendImuCalibration() && tx_count==sent);imu_api();assert(strstr(reply,"\"request_phase\":\"tx_error\""));
    imu_cal_ready();assert(Stm32Imu_RequestCalibration("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")==0);assert(Uart_SendImuCalibration());clock_us+=500000;
    imu_api();assert(strstr(reply,"\"request_phase\":\"expired\""));
    snprintf(line,sizeof line,"1,0,25,%" PRIu32 ",-,-,-,-",s_imu_cal_request.number);imu2_reply(line);imu_api();
    assert(strstr(reply,"\"request_phase\":\"accepted\"") && strstr(reply,"\"state\":\"running\"") && strstr(reply,"\"gyro_corrected_dps\":[null,null,null]"));
    pass("IMU-2 stale requests, UART short writes and missing ACK expire without automatic retries or movement");
    imu_cal_ready();request_origin="http://foreign";imu_cal_post("CALIBRATE");assert(!strncmp(response_status,"403",3) && !tx_count);
    request_origin="http://192.168.4.1";imu_cal_post("AVANZAR");assert(!strncmp(response_status,"400",3) && !tx_count);
    feed("STATE=2 FAULTS=0 STOP=0\n");imu_cal_post("CALIBRATE");assert(!strncmp(response_status,"412",3) && !tx_count);
    pass("IMU-2 POST uses closed operation, Origin/Host/body checks and refuses calibration in MANUAL without changing its behavior");
}
