/* W6: rutas reales HTTP/arbitro/UART, sin hardware ni otro receptor. */
static void auto_ready(void)
{
    reset(); Uart_Init(); clock_us=10000;
    feed("STATE=0 FAULTS=0 STOP=0\n");
}
static void reset_before_auto(void) { Uart_SendReset(); }
static void halt_before_auto(void) { WebManual_RequestHalt(); }
static void manual_before_auto(void) { Uart_SendManual(); }
static void unsafe_before_auto(void) { feed("STATE=3 FAULTS=1 STOP=1\n"); }

static void test_auto(void)
{
    const char *bad_status[]={NULL,"STATE=2 FAULTS=0 STOP=0\n","STATE=1 FAULTS=0 STOP=0\n",
        "STATE=3 FAULTS=1 STOP=1\n","STATE=0 FAULTS=4 STOP=0\n","STATE=0 FAULTS=0 STOP=1\n"};
    for(unsigned i=0;i<sizeof bad_status/sizeof bad_status[0];++i) {
        reset(); if(bad_status[i]) feed(bad_status[i]);
        post("AUTO",4); assert(!strncmp(response_status,"412",3));
        WebCommands_ProcessPending(); assert(!tx_count && s_pending<0);
    }
    pass("W6 AUTO requires known safe IDLE; MANUAL/AUTO/Safety/fault/STOP cannot admit it");
    auto_ready(); clock_us+=1999999; post("AUTO",4); assert(!strncmp(response_status,"202",3));
    auto_ready(); clock_us+=2000000; post("AUTO",4); assert(!strncmp(response_status,"412",3));
    pass("W6 AUTO STATUS deadline is strictly below 2000 ms, independent of visual freshness");

    auto_ready(); post("AUTO",4); assert(!tx_count && strstr(reply,"\"confirmed\":false"));
    post("AUTO",4); assert(strstr(reply,"\"duplicate\":true"));
    WebCommands_ProcessPending(); assert(frame_count("AUTO\n")==1);
    post("AUTO",4); WebCommands_ProcessPending(); assert(frame_count("AUTO\n")==1);
    api(); assert(strstr(reply,"\"status_after_send\":false"));
    ++clock_us; feed("AUTO_QUEUED\n"); api(); assert(strstr(reply,"\"status_after_send\":false"));
    feed("STATE=1 FAULTS=0 STOP=0\n"); api();
    assert(strstr(reply,"\"state\":1") && strstr(reply,"\"status_after_send\":true"));
    pass("W6 AUTO POST is idempotent, deferred, delivered once, observed only with later STATUS");

    for(unsigned age=499;age<=500;++age) {
        auto_ready(); post("AUTO",4); clock_us+=(int64_t)age*1000; WebCommands_ProcessPending();
        assert(frame_count("AUTO\n")== (age<500));
    }
    auto_ready(); post("AUTO",4); wait_delay=500000; WebCommands_ProcessPending();
    assert(!frame_count("AUTO\n"));
    pass("W6 AUTO age checked again after UART gate wait; 500 ms cannot be rejuvenated");
    void (*hooks[])(void)={stop_during_send,reset_before_auto,manual_before_auto,halt_before_auto,unsafe_before_auto};
    for(unsigned i=0;i<sizeof hooks/sizeof hooks[0];++i) {
        auto_ready(); post("AUTO",4); wait_hook=hooks[i]; WebCommands_ProcessPending();
        assert(!frame_count("AUTO\n")); WebCommands_ProcessPending(); WebManual_Process();
        assert(!frame_count("AUTO\n") && !s_manual.active);
    }
    pass("W6 STOP RESET MANUAL DETENER and Safety during UART wait invalidate pending AUTO");

    auto_ready(); post("AUTO",4); request_id="11111111111111111111111111111111";
    post("STOP",4); WebCommands_ProcessPending();
    assert(frame_count("STOP\n")==1 && !frame_count("AUTO\n"));
    request_id="0123456789abcdef0123456789abcdef"; post("AUTO",4); WebCommands_ProcessPending();
    assert(strstr(reply,"\"phase\":\"cancelled\"") && !frame_count("AUTO\n"));
    pass("W6 STOP replaces pending AUTO, and reusing its ID cannot resurrect the cancelled request");
    auto_ready(); post("AUTO",4); tx_hook=stop_during_send; WebCommands_ProcessPending();
    WebCommands_ProcessPending(); assert(tx_count==2 && !strcmp(frames[0],"AUTO\n") && !strcmp(frames[1],"STOP\n"));
    pass("W6 STOP concurrent with already committed AUTO frame follows it through the same TX gate");

    auto_ready(); post("AUTO",4); tx_error=1; WebCommands_ProcessPending(); WebCommands_ProcessPending();
    assert(frame_count("AUTO\n")==1 && s_receipts[s_latest].phase==WEB_CMD_FAILED);
    gate_fail=true; Uart_SendAuto(); WebCommands_ProcessPending(); assert(frame_count("AUTO\n")==1);
    pass("W6 failed AUTO delivery has no retry or reactivation from the previous STATUS");

    auto_ready(); post("AUTO",4); Uart_SendReset(); WebCommands_ProcessPending(); assert(!frame_count("AUTO\n"));
    request_id="22222222222222222222222222222222"; post("AUTO",4); assert(!strncmp(response_status,"412",3));
    feed("STATE=0 FAULTS=0 STOP=0\n"); post("AUTO",4); assert(!strncmp(response_status,"412",3));
    ++clock_us; feed("STATE=0 FAULTS=0 STOP=0\n"); post("AUTO",4); assert(!strncmp(response_status,"202",3));
    WebCommands_ProcessPending(); assert(frame_count("AUTO\n")==1);
    pass("W6 reset barrier rejects old and same-instant IDLE; only later STATUS and a fresh request can enable AUTO");

    manual_ready(); assert(WebManual_Renew(manual_token,1,UART_MOTION_FORWARD)==WEB_MANUAL_OK);
    ++clock_us; feed("STATE=0 FAULTS=0 STOP=0\n"); post("AUTO",4);
    assert(!s_manual.active && !s_manual.pending); WebCommands_ProcessPending();
    assert(tx_count==2 && !strcmp(frames[0],"DETENER\n") && !strcmp(frames[1],"AUTO\n"));
    assert(WebManual_Renew(manual_token,2,UART_MOTION_FORWARD)==WEB_MANUAL_EXPIRED);
    feed("STATE=1 FAULTS=0 STOP=0\n");
    char token[33]; assert(WebManual_Open("33333333333333333333333333333333",token)==WEB_MANUAL_UNSAFE);
    WebManual_Process(); assert(!frame_count("AVANZAR\n"));
    pass("W6 AUTO revokes any old manual lease, drains DETENER first and forbids manual renewal in AUTO");

    auto_ready(); assert(!Uart_SendControl(UART_CONTROL_AUTO) && !tx_count);
    ++clock_us; feed("STATE=0 FAULTS=0 STOP=0\n"); Uart_SendAuto(); assert(!tx_count && s_pending>=0);
    WebCommands_ProcessPending(); assert(frame_count("AUTO\n")==1);
    pass("W6 console n shares deferred validated AUTO path; raw control call cannot bypass its one-use permission");
    auto_ready(); post("AUTO",4); clock_us+=3000001; WebCommands_ProcessPending();
    feed("STATE=0 FAULTS=0 STOP=0\n"); WebCommands_ProcessPending(); assert(!tx_count);
    reset(); WebCommands_ProcessPending(); assert(!tx_count && !s_manual.active);
    pass("W6 lost communication, later recovery and ESP32 restart never replay or generate AUTO");
    auto_ready(); post("AUTO",4); WebCommands_ProcessPending();
    ++clock_us; feed("STATE=1 FAULTS=0 STOP=0\n");
    Stm32Sensors_DeferPoll(); clock_us+=500000; Uart_PollSensors();
    assert(frame_count("SENSORS Q=1\n")==1); Uart_SendStatus();
    assert(frame_count("STATUS\n")==1 && frame_count("AUTO\n")==1);
    pass("W6 STATUS and SENSORS remain available in AUTO without autonomous command renewal");
}
