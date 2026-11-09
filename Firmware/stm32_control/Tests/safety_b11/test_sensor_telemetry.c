/* Real TaskComms and UART mailbox, included in the permanent integration suite. */
static void test_sensor_telemetry(void) {
    DrvUart_t uart; UART_HandleTypeDef peripheral;
    setup(); DrvUart_Init(&uart,&peripheral); queue_available=0;
    b65_uart_command(&uart,"SENSORS\n","SENSORS,1,0,0,-,0,-,-,0,-,-,-,-,-\n");
    PASS("W5 missing source snapshot is explicit, no zero distance substitution");
    queue_available=1; input=(SensoresDatos_t){0}; clock_tick=10;
    input.ir=(TCRT_Lecturas_t){1,1,1,1};
    b65_uart_command(&uart,"SENSORS Q=1\n","SENSORS,1,1,1,10,0,-,-,0,-,-,15,10,0\n");
    PASS("W5 first snapshot IR available while both ultrasound results are absent");
    input.resultado_frontal_disponible=1; input.ultrasonido_valido=1; input.distancia_m=0.345f;
    input.resultado_trasero_disponible=1; input.ultrasonido_trasero_valido=0;
    input.timestamp_resultado_frontal=0; input.timestamp_resultado_trasero=5;
    b65_uart_command(&uart,"SENSORS Q=2\n","SENSORS,1,2,1,10,1,345,10,2,-,5,15,10,0\n");
    CHECK(input.timestamp_ultrasonido_frontal==0 && !target_pending);
    PASS("W5 valid tick zero front and failed rear serialized independently without motion");
    input.ultrasonido_trasero_valido=1; input.distancia_trasera_m=0.29427f;
    b65_uart_command(&uart,"SENSORS Q=3\n","SENSORS,1,3,1,10,1,345,10,1,294,5,15,10,0\n");
    PASS("W5 rear recovery and integer-millimeter rounding preserve front sample");
    clock_tick=5; input.timestamp=input.timestamp_ir=input.timestamp_resultado_frontal=UINT32_MAX-4;
    input.timestamp_resultado_trasero=0;
    b65_uart_command(&uart,"SENSORS Q=4294967295\n","SENSORS,1,4294967295,1,10,1,345,10,1,294,5,15,10,0\n");
    PASS("W5 snapshot, acquisition and IR ages use unsigned tick wrap; max query identifier");
    for(unsigned mask=0;mask<16;++mask) {
        input.ir=(TCRT_Lecturas_t){(mask&1)!=0,(mask&2)!=0,(mask&4)!=0,(mask&8)!=0};
        input.hay_caida=mask!=15;
        char expected[96]; snprintf(expected,sizeof expected,"SENSORS,1,4,1,10,1,345,10,1,294,5,%u,10,%u\n",mask,mask!=15);
        b65_uart_command(&uart,"SENSORS Q=4\n",expected);
    }
    PASS("W5 all 16 digital IR combinations preserve 1=floor, 0=no floor");
    clock_tick=UINT32_MAX; input.timestamp=input.timestamp_ir=input.timestamp_resultado_frontal=input.timestamp_resultado_trasero=0;
    input.distancia_m=input.distancia_trasera_m=4.0f;
    b65_uart_command(&uart,"SENSORS Q=4294967295\n","SENSORS,1,4294967295,1,4294967295,1,4000,4294967295,1,4000,4294967295,15,4294967295,0\n");
    CHECK(strlen(uart_response)<96);
    PASS("W5 maximum ages and identifiers fit a single bounded response, no float printf");
    const char *bad[]={"SENSORS Q=0\n","SENSORS Q=4294967296\n","SENSORS Q=-1\n","SENSORS Q=1 extra\n","SENSORS Q=\n"};
    for(unsigned i=0;i<sizeof bad/sizeof bad[0];++i) b65_uart_command(&uart,bad[i],"ERR_UNKNOWN\n");
    PASS("W5 malformed queries rejected without acquisition or motor commands");
    setup(); DrvUart_Init(&uart,&peripheral);
    uart_feed(&uart,"SENSORS\nSTATUS\nAVANZAR\nDETENER\nSTOP\nSENSORS\n");
    CHECK(uart_consume(&uart) && strcmp(uart_response,"STOP_QUEUED\n")==0); expect_zero();
    CHECK(!uart_consume(&uart)); PASS("W5 STOP wins full telemetry/status/movement/halt burst");
    setup(); DrvUart_Init(&uart,&peripheral); uart_feed(&uart,"SENSORS\nSTATUS\n");
    CHECK(uart_consume(&uart) && strncmp(uart_response,"STATE=",6)==0);
    uart_feed(&uart,"SENSORS\nAVANZAR\n"); CHECK(uart_consume(&uart) && strcmp(uart_response,"MOVE_ACCEPTED\n")==0);
    uart_feed(&uart,"AVANZAR\nSENSORS\n"); CHECK(uart_consume(&uart) && strcmp(uart_response,"MOVE_ACCEPTED\n")==0);
    uart_feed(&uart,"SENSORS\nDETENER\n"); CHECK(uart_consume(&uart) && strcmp(uart_response,"MOVE_ACCEPTED\n")==0); expect_zero();
    PASS("W5 telemetry never displaces STATUS, movement or DETENER in the one-slot mailbox");
}
