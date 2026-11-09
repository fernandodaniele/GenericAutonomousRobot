/* Rutas reales; solo se sustituyen SDK, reloj, UART fisica y HTTP. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include "sdk.h"
#include "stm32_status.c"
#define TAG uart_tag
#include "uart_comm.c"
#undef TAG
#include "web_commands.c"
#define TAG web_tag
#include "web_server.c"
#undef TAG
static int console_getchar(void);
#define getchar console_getchar
#include "main.c"
#undef getchar

static LONG64 clock_us;
static __thread int lock_depth;
static unsigned cases, tx_count, rx_count, init_count, task_count, iterations;
static char frames[4096][32], reply[16384];
static CRITICAL_SECTION tx_lock;
static int tx_error;
static void (*tx_hook)(void);
static const char *request_type="text/plain", *request_origin="http://192.168.4.1", *request_host="192.168.4.1", *request_id;
static const char *request_body, *response_status;
static size_t body_offset, body_chunk=6;
static int body_error;
static int64_t body_delay;
static const char *rx_data, *keys;
static size_t rx_size;
static httpd_uri_t routes[6];
static unsigned route_count, server_count, server_stops, fail_route;
static int http_error, network_error;
static jmp_buf task_end;

static void pass(const char *name) { printf("PASS %02u %s\n", ++cases, name); }
void test_enter(portMUX_TYPE *lock) { assert(lock_depth == 0); EnterCriticalSection(&lock->native); ++lock_depth; }
void test_exit(portMUX_TYPE *lock) { assert(lock_depth == 1); --lock_depth; LeaveCriticalSection(&lock->native); }
int64_t esp_timer_get_time(void) { assert(!lock_depth); return clock_us; }
void test_log(const char *tag, const char *fmt, ...) { (void)tag; (void)fmt; assert(!lock_depth); }
const char *esp_err_to_name(esp_err_t err) { (void)err; return "mock"; }
int uart_driver_install(int p,int rx,int tx,int q,void *queue,int flags)
{ assert(p==2 && rx==256 && tx==0 && q==0 && !queue && !flags && !lock_depth); ++init_count; return 0; }
int uart_param_config(int p,const uart_config_t *c)
{ assert(p==2 && c->baud_rate==115200 && c->data_bits==8 && !c->parity && c->stop_bits==1 && !c->flow_ctrl); return 0; }
int uart_set_pin(int p,int tx,int rx,int rts,int cts)
{ assert(p==2 && tx==17 && rx==16 && rts==-1 && cts==-1); return 0; }
int uart_write_bytes(int port,const void *bytes,size_t size)
{
    EnterCriticalSection(&tx_lock); /* Mismo contrato por llamada que tx_mux del SDK. */
    assert(port==2 && !lock_depth && tx_count<4096 && size<sizeof frames[0]);
    memcpy(frames[tx_count],bytes,size); frames[tx_count++][size]='\0';
    assert(((const char *)bytes)[size-1]=='\n');
    if(tx_hook) { void (*hook)(void)=tx_hook; tx_hook=NULL; hook(); }
    LeaveCriticalSection(&tx_lock); return tx_error ? (int)size-1 : (int)size;
}
int uart_read_bytes(int port,void *bytes,uint32_t size,unsigned timeout)
{
    assert(port==2 && size==255 && timeout==pdMS_TO_TICKS(100) && !lock_depth);
    ++rx_count; if (!rx_size) { clock_us+=100000; return 0; }
    assert(rx_size<=size); memcpy(bytes,rx_data,rx_size); int result=(int)rx_size; rx_size=0; return result;
}
esp_err_t WifiAp_Start(void) { return network_error ? ESP_FAIL : ESP_OK; }
esp_err_t httpd_start(httpd_handle_t *out,const httpd_config_t *cfg)
{ assert(cfg->server_port==80 && !lock_depth); ++server_count; *out=(void *)1; return ESP_OK; }
esp_err_t httpd_stop(httpd_handle_t handle) { assert(handle); ++server_stops; return ESP_OK; }
esp_err_t httpd_register_uri_handler(httpd_handle_t handle,const httpd_uri_t *r)
{ assert(handle && route_count<6); assert(r->method==HTTP_GET || (!strcmp(r->uri,"/api/command") && r->method==HTTP_POST)); routes[route_count++]=*r; return route_count==fail_route ? ESP_FAIL : ESP_OK; }
esp_err_t httpd_resp_set_type(httpd_req_t *r,const char *type)
{ assert(r && type && !lock_depth); return http_error==1 ? ESP_FAIL : ESP_OK; }
esp_err_t httpd_resp_set_hdr(httpd_req_t *r,const char *key,const char *value)
{ assert(r && !lock_depth); assert((!strcmp(key,"Cache-Control") && !strcmp(value,"no-store")) || (!strcmp(key,"Connection") && !strcmp(value,"close"))); return http_error==2 ? ESP_FAIL : ESP_OK; }
esp_err_t httpd_resp_set_status(httpd_req_t *r,const char *value) { assert(r && !lock_depth); response_status=value; return ESP_OK; }
static const char *header(const char *name)
{
    if(!strcmp(name,"Content-Type")) return request_type;
    if(!strcmp(name,"Origin")) return request_origin;
    if(!strcmp(name,"Host")) return request_host;
    if(!strcmp(name,"X-Request-ID")) return request_id;
    assert(0); return NULL;
}
size_t httpd_req_get_hdr_value_len(httpd_req_t *r,const char *name) { assert(r); const char *s=header(name); return s ? strlen(s) : 0; }
esp_err_t httpd_req_get_hdr_value_str(httpd_req_t *r,const char *name,char *out,size_t size)
{ assert(r); const char *s=header(name); if(!s || strlen(s)>=size) return ESP_FAIL; strcpy(out,s); return ESP_OK; }
int httpd_req_recv(httpd_req_t *r,char *out,size_t size)
{ assert(r && !lock_depth); clock_us+=body_delay; if(body_error) return -1; size_t count=size<body_chunk ? size : body_chunk; memcpy(out,request_body+body_offset,count); body_offset+=count; return (int)count; }
esp_err_t httpd_resp_send(httpd_req_t *r,const char *data,ptrdiff_t size)
{ assert(r && size>=0 && (size_t)size<sizeof reply && !lock_depth); memcpy(reply,data,size); reply[size]='\0'; return http_error==3 ? ESP_FAIL : ESP_OK; }
BaseType_t xTaskCreate(void (*fn)(void *),const char *name,unsigned stack,void *arg,unsigned priority,void *out)
{ assert(fn==ConsoleInputTask && !strcmp(name,"ConsoleInput") && stack==4096 && !arg && priority==5 && !out); ++task_count; return pdPASS; }
void vTaskDelay(unsigned ticks)
{
    if (ticks==pdMS_TO_TICKS(50)) longjmp(task_end,1);
    assert(ticks==pdMS_TO_TICKS(10)); clock_us+=10000;
    if (++iterations==15) longjmp(task_end,1);
}
static int console_getchar(void) { return *keys ? *keys++ : EOF; }
static void reset(void)
{
    memset(&s_status,0,sizeof s_status); s_length=0; s_discard_line=false; s_poll_started=false; s_last_poll_us=0;
    clock_us=0; tx_count=rx_count=init_count=task_count=iterations=0; rx_size=0;
    route_count=server_count=server_stops=fail_route=0; http_error=network_error=0;
    s_attempted=false; s_result=ESP_ERR_INVALID_STATE; s_server=NULL;
    memset(s_receipts,0,sizeof s_receipts); s_pending=s_sending=s_latest=-1; s_next_receipt=0; tx_error=0; tx_hook=NULL;
    request_type="text/plain"; request_origin="http://192.168.4.1"; request_host="192.168.4.1";
    request_id="0123456789abcdef0123456789abcdef"; body_offset=0; body_chunk=6; body_error=0; body_delay=0;
}
static void feed(const char *text) { Stm32Status_Feed((const uint8_t *)text,strlen(text)); }
static Stm32StatusSnapshot snapshot(void) { Stm32StatusSnapshot s; Stm32Status_GetSnapshot(&s); return s; }
static void api(void) { httpd_req_t r={0}; assert(WebServer_Status(&r)==ESP_OK); }

static DWORD WINAPI writer(void *arg)
{
    (void)arg;
    for (unsigned n=0;n<20000;++n) { char line[64]; unsigned state=n%4;
        snprintf(line,sizeof line,"STATE=%u FAULTS=%u STOP=%u\n",state,100+state,state%2); feed(line); }
    return 0;
}

static esp_err_t post(const char *body,size_t length)
{
    request_body=body; body_offset=0;
    httpd_req_t req={.content_len=length};
    return WebServer_Command(&req);
}
static void stop_during_send(void)
{
    WebCommandReceipt receipt;
    assert(WebCommands_Submit("11111111111111111111111111111111",UART_CONTROL_STOP,&receipt)==WEB_CMD_ACCEPTED);
}
static DWORD WINAPI tx_writer(void *arg)
{
    for(unsigned i=0;i<500;++i) {
        if(arg) assert(Uart_SendControl(UART_CONTROL_STOP));
        else Uart_SendStatus();
    }
    return 0;
}
static DWORD WINAPI submit_writer(void *arg)
{
    (void)arg;
    for(unsigned i=0;i<1000;++i) {
        WebCommandReceipt r;
        WebCommandResult result=WebCommands_Submit("22222222222222222222222222222222",UART_CONTROL_STOP,&r);
        assert((result==WEB_CMD_ACCEPTED || result==WEB_CMD_DUPLICATE) && r.command==UART_CONTROL_STOP);
    }
    return 0;
}
static DWORD WINAPI command_worker(void *arg)
{
    (void)arg;
    for(unsigned i=0;i<2000;++i) {
        WebCommands_ProcessPending(); WebCommandReceipt r;
        if(WebCommands_GetLatest(&r)) assert(!strcmp(r.id,"22222222222222222222222222222222") && r.command==UART_CONTROL_STOP);
    }
    return 0;
}

int main(void)
{
    InitializeCriticalSection(&s_lock.native);
    InitializeCriticalSection(&s_command_lock.native);
    InitializeCriticalSection(&tx_lock);
    reset(); assert(!snapshot().valid && !snapshot().connected); api();
    assert(!strcmp(reply,"{\"valid\":false,\"connected\":false,\"state\":null,\"faults\":null,\"stop\":null,\"age_ms\":null}"));
    pass("boot/no response: unknown fields are null, not safe defaults");
    feed("STATE=3 FAULTS=4 STOP=1\n"); assert(snapshot().valid && snapshot().state==3 && snapshot().faults==4 && snapshot().stop==1 && snapshot().age_ms==0);
    pass("valid exact STATUS at time zero");
    const char *line="STATE=2 FAULTS=15 STOP=0\r\n";
    for (size_t split=0;split<=strlen(line);++split) {
        reset(); Stm32Status_Feed((const uint8_t *)line,split);
        if(split<strlen(line)) assert(!snapshot().valid);
        Stm32Status_Feed((const uint8_t *)line+split,strlen(line)-split);
        assert(snapshot().valid && snapshot().state==2 && snapshot().faults==15 && !snapshot().stop);
    }
    pass("every fragmentation boundary including CR/LF split");
    reset(); for(size_t i=0;i<strlen(line);++i) Stm32Status_Feed((const uint8_t *)line+i,1);
    assert(snapshot().state==2); pass("one-byte fragments");
    reset(); feed("PONG\nRESET_QUEUED\r\nSTATE=1 FAULTS=2 STOP=1\nMOVE_ACCEPTED\nSTATE=0 FAULTS=0 STOP=0\r\n");
    assert(snapshot().valid && !snapshot().state && !snapshot().faults && !snapshot().stop);
    pass("multiple complete lines and unrelated UART responses");
    const char *invalid[]={"STATE=1 FAULTS=2\n","STATE=4 FAULTS=0 STOP=0\n","STATE=-1 FAULTS=0 STOP=0\n",
      "STATE=1 FAULTS=4294967296 STOP=0\n","STATE=1 FAULTS=0 STOP=2\n","STATE=1 FAULTS=0 STOP=0junk\n",
      "prefix STATE=1 FAULTS=0 STOP=0\n","STATE=1  FAULTS=0 STOP=0\n","STATE=1 FAULTS=+2 STOP=0\n",
      "STATE=1 FAULTS=0 STOP=0\r\r\n","STATE=1\r FAULTS=0 STOP=0\n","STATE=4294967296 FAULTS=0 STOP=0\n",
      "STOP_QUEUED\n","ERR_UNKNOWN\n","\n"};
    for(size_t i=0;i<sizeof invalid/sizeof invalid[0];++i) {
        reset(); feed("STATE=3 FAULTS=4 STOP=1\n"); clock_us=1000000; feed(invalid[i]);
        assert(snapshot().state==3 && snapshot().faults==4 && snapshot().stop==1 && snapshot().age_ms==1000);
    }
    pass("15 malformed/other responses never replace or rejuvenate valid data");
    reset(); feed("STATE=0 FAULTS=0 STOP=0"); assert(!snapshot().valid);
    feed("\n"); assert(snapshot().valid); pass("incomplete response accepted only after LF");
    reset(); char long_line[200]; memset(long_line,'X',sizeof long_line); Stm32Status_Feed((uint8_t *)long_line,sizeof long_line);
    feed("STATE=1 FAULTS=0 STOP=0\n"); assert(!snapshot().valid); feed(line); assert(snapshot().valid);
    pass("overflow discards entire line and resynchronizes at LF");
    reset(); const uint8_t nul_line[]="STATE=1 FAULTS=0 STOP=0\0STATE=2 FAULTS=0 STOP=0\n";
    Stm32Status_Feed(nul_line,sizeof nul_line-1); assert(!snapshot().valid); feed(line); assert(snapshot().valid);
    pass("embedded NUL cannot turn a prefix into a valid STATUS");
    reset(); feed("STATE=1 FAULTS=4294967295 STOP=1\n"); assert(snapshot().faults==UINT32_MAX); api(); assert(strstr(reply,"4294967295"));
    pass("unsigned mask preserved including unknown bits; JSON remains numeric");
    reset(); feed(line); clock_us=2999999; assert(snapshot().connected && snapshot().age_ms==2999);
    clock_us=3000000; assert(!snapshot().connected && snapshot().valid); api(); assert(strstr(reply,"\"connected\":false") && strstr(reply,"\"state\":2"));
    feed("PONG\n"); assert(!snapshot().connected); feed(line); assert(snapshot().connected && snapshot().age_ms==0);
    pass("stale boundary 3000 ms retains values; only valid STATUS restores freshness");
    reset(); clock_us=((LONG64)UINT32_MAX+100)*1000; feed(line); clock_us+=3000000;
    assert(snapshot().age_ms==3000 && !snapshot().connected); pass("64-bit monotonic age across 32-bit milliseconds wrap");
    reset(); assert(Stm32Status_PollDue() && !Stm32Status_PollDue()); clock_us=999999; assert(!Stm32Status_PollDue());
    clock_us=1000000; assert(Stm32Status_PollDue()); clock_us=100000000; assert(Stm32Status_PollDue() && !Stm32Status_PollDue());
    clock_us+=999999; assert(!Stm32Status_PollDue()); ++clock_us; assert(Stm32Status_PollDue());
    assert(!tx_count); pass("poll schedule has no UART side effects, catch-up or duplicate requests");
    reset(); Uart_Init(); rx_data=line; rx_size=9; Uart_ReadResponse(); assert(!snapshot().valid);
    rx_data=line+9; rx_size=strlen(line)-9; Uart_ReadResponse(); assert(snapshot().state==2 && rx_count==2);
    pass("actual sole UART reader feeds fragmented responses and retains console printing");
    reset(); keys="psrtmwxade?"; if(!setjmp(task_end)) ConsoleInputTask(NULL);
    const char *expected[]={"PING\n","STATUS\n","RESET\n","STOP\n","MANUAL\n","AVANZAR\n","RETROCEDER\n","GIRAR_IZQUIERDA\n","GIRAR_DERECHA\n","DETENER\n"};
    assert(tx_count==10); for(unsigned i=0;i<10;++i) assert(!strcmp(frames[i],expected[i]));
    pass("all original console keys unchanged, one complete uart_write_bytes per frame, unknown key ignored");
    for(unsigned mode=0;mode<2;++mode) {
        reset(); network_error=(int)mode; if(!setjmp(task_end)) app_main();
        assert(init_count==1 && task_count==1 && rx_count==15 && tx_count==2);
        assert(!strcmp(frames[0],"STATUS\n") && !strcmp(frames[1],"STATUS\n"));
    }
    pass("actual app_main polls at 1 Hz without new tasks, including AP failure");
    reset(); assert(WebServer_Start()==ESP_OK && WebServer_Start()==ESP_OK && server_count==1 && route_count==6);
    const char *paths[]={"/","/style.css","/app.js","/health","/api/status"};
    for(unsigned i=0;i<5;++i) {
        assert(!strcmp(routes[i].uri,paths[i])); httpd_req_t r={.user_ctx=routes[i].user_ctx}; assert(routes[i].handler(&r)==ESP_OK);
        if(i<3) assert(!memcmp(reply,resources[i].start,(size_t)(resources[i].end-resources[i].start)));
        if(i==3) assert(!strcmp(reply,"{\"status\":\"ok\",\"service\":\"esp32-web\"}"));
    }
    assert(!tx_count && !rx_count); pass("all five GET routes, W1 resources/health unchanged, HTTP never accesses UART");
    for(int err=1;err<=3;++err) { http_error=err; httpd_req_t r={0}; assert(WebServer_Status(&r)==ESP_FAIL); }
    pass("status HTTP propagates header/send errors without UART or held locks");
    reset(); fail_route=5; assert(WebServer_Start()==ESP_FAIL && server_stops==1 && WebServer_Start()==ESP_FAIL && server_count==1);
    pass("new route failure cleans server and does not retry initialization");
    reset(); feed("STATE=0 FAULTS=100 STOP=0\n"); HANDLE thread=CreateThread(NULL,0,writer,NULL,0,NULL); assert(thread);
    for(unsigned n=0;n<20000;++n) { Stm32StatusSnapshot s=snapshot(); assert(s.valid && s.faults==100+s.state && s.stop==s.state%2);
        api(); unsigned state,faults,stop; assert(sscanf(reply,"{\"valid\":true,\"connected\":true,\"state\":%u,\"faults\":%u,\"stop\":%u",&state,&faults,&stop)==3);
        assert(faults==100+state && stop==state%2); }
    assert(WaitForSingleObject(thread,10000)==WAIT_OBJECT_0); CloseHandle(thread);
    pass("20,000 real parser publications concurrent with 20,000 snapshot/HTTP reads; no mixed tuples or locked E/S");
    const char *commands[]={"STOP","RESET","MANUAL"};
    for(unsigned i=0;i<3;++i) {
        reset(); assert(post(commands[i],strlen(commands[i]))==ESP_OK);
        assert(!strcmp(response_status,"202 Accepted") && strstr(reply,"\"accepted\":true") && strstr(reply,"\"confirmed\":false"));
        assert(tx_count==0); api(); assert(strstr(reply,"\"phase\":\"queued\""));
        WebCommands_ProcessPending(); assert(tx_count==1);
        char frame[16]; snprintf(frame,sizeof frame,"%s\n",commands[i]); assert(!strcmp(frames[0],frame));
        api(); assert(strstr(reply,"\"phase\":\"transmitted\"") && strstr(reply,"\"status_after_send\":false"));
        WebCommands_ProcessPending(); assert(tx_count==1);
    }
    pass("three valid POST commands: API acceptance, one later UART dispatch, no operational confirmation");
    reset(); body_chunk=1; assert(post("MANUAL",6)==ESP_OK && body_offset==6 && !tx_count);
    WebCommands_ProcessPending(); assert(tx_count==1 && !strcmp(frames[0],"MANUAL\n"));
    pass("fragmented POST body reassembled within fixed six-byte bound");
    const char *bad[]={"AUTO","PING","STATUS","stop","STOP\n","RESET ","w","","STOP\nMANUAL","AVANZAR"};
    for(unsigned i=0;i<sizeof bad/sizeof bad[0];++i) {
        reset(); post(bad[i],strlen(bad[i])); WebCommands_ProcessPending(); assert(tx_count==0 && strstr(reply,"\"accepted\":false"));
    }
    reset(); const char nul_body[]={'S','T','O','P','\0'}; post(nul_body,5); WebCommands_ProcessPending(); assert(!tx_count);
    pass("unknown, AUTO, movement, injected frames, whitespace, empty and NUL bodies never reach UART");
    reset(); request_origin="http://foreign.example"; post("STOP",4); assert(!strcmp(response_status,"403 Forbidden") && !body_offset && !tx_count);
    reset(); request_origin=NULL; post("STOP",4); assert(!strcmp(response_status,"403 Forbidden"));
    reset(); request_type="application/json"; post("STOP",4); assert(!strncmp(response_status,"415",3));
    reset(); request_id=NULL; post("STOP",4); assert(!strncmp(response_status,"400",3));
    reset(); request_id="short"; post("STOP",4); assert(!strncmp(response_status,"400",3));
    reset(); request_id="XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"; post("STOP",4); assert(!strncmp(response_status,"400",3));
    WebCommands_ProcessPending(); assert(!tx_count);
    pass("Origin/content-type/id required, foreign origin and malformed identifiers rejected; no CORS headers");
    reset(); body_error=1; post("STOP",4); assert(!strncmp(response_status,"408",3)); WebCommands_ProcessPending(); assert(!tx_count);
    reset(); body_chunk=1; body_delay=1000000; post("MANUAL",6); assert(!strncmp(response_status,"408",3) && body_offset==1);
    WebCommands_ProcessPending(); assert(!tx_count); pass("incomplete or slow POST times out without waiting on UART or retrying");
    reset(); post("RESET",5); post("RESET",5); assert(strstr(reply,"\"duplicate\":true"));
    WebCommands_ProcessPending(); post("RESET",5); WebCommands_ProcessPending(); assert(tx_count==1 && strstr(reply,"\"duplicate\":true"));
    post("MANUAL",6); assert(!strncmp(response_status,"400",3)); assert(tx_count==1);
    pass("same request ID is idempotent before/after dispatch and cannot change command");
    reset(); post("RESET",5); request_id="11111111111111111111111111111111"; post("MANUAL",6);
    assert(!strncmp(response_status,"409",3)); WebCommands_ProcessPending(); assert(tx_count==1 && !strcmp(frames[0],"RESET\n"));
    pass("one pending slot: conflicting mode request rejected without extra frame");
    reset(); post("RESET",5); request_id="11111111111111111111111111111111"; post("STOP",4);
    WebCommands_ProcessPending(); assert(tx_count==1 && !strcmp(frames[0],"STOP\n"));
    request_id="0123456789abcdef0123456789abcdef"; post("RESET",5); assert(strstr(reply,"\"phase\":\"cancelled\""));
    WebCommands_ProcessPending(); assert(tx_count==1); pass("STOP replaces pending RESET; cancelled ID cannot reactivate it");
    reset(); post("RESET",5); tx_hook=stop_during_send; WebCommands_ProcessPending();
    assert(tx_count==1 && !strcmp(frames[0],"RESET\n")); WebCommands_ProcessPending(); assert(tx_count==2 && !strcmp(frames[1],"STOP\n"));
    pass("STOP concurrent with in-flight dispatch remains pending and follows it; no lock held across UART");
    reset(); post("RESET",5); tx_error=1; WebCommands_ProcessPending(); api(); assert(strstr(reply,"\"phase\":\"failed\""));
    WebCommands_ProcessPending(); assert(tx_count==1); post("RESET",5); WebCommands_ProcessPending(); assert(tx_count==1);
    assert(!Uart_SendControl((UartControlCommand)99) && tx_count==1); pass("short UART write and invalid enum fail without automatic retries");
    reset(); feed("STATE=0 FAULTS=0 STOP=0\n"); post("RESET",5); WebCommands_ProcessPending();
    api(); assert(strstr(reply,"\"status_after_send\":false"));
    clock_us=1000; feed("RESET_QUEUED\n"); api(); assert(strstr(reply,"\"status_after_send\":false"));
    feed("STATE=3 FAULTS=4 STOP=1\n"); api(); assert(strstr(reply,"\"status_after_send\":true") && strstr(reply,"\"faults\":4"));
    clock_us=3001000; api(); assert(strstr(reply,"\"status_after_send\":false"));
    pass("old/same-tick STATUS and RESET_QUEUED cannot confirm; later observation may still report blocked Safety; stale unconfirmed");
    reset(); post("STOP",4); if(!setjmp(task_end)) app_main(); assert(tx_count==3 && !strcmp(frames[0],"STOP\n") && !strcmp(frames[1],"STATUS\n"));
    assert(rx_count==15 && task_count==1); pass("actual main services STOP before periodic STATUS while retaining sole receiver and console task");
    reset(); assert(WebServer_Start()==ESP_OK && routes[5].method==HTTP_POST && !strcmp(routes[5].uri,"/api/command"));
    fail_route=6; s_attempted=false; s_server=NULL; route_count=server_count=0;
    assert(WebServer_Start()==ESP_FAIL && server_stops==1); pass("command route only POST; registration errors stop partial server");
    reset(); HANDLE a=CreateThread(NULL,0,tx_writer,NULL,0,NULL), b=CreateThread(NULL,0,tx_writer,(void *)1,0,NULL); assert(a && b);
    assert(WaitForSingleObject(a,10000)==WAIT_OBJECT_0 && WaitForSingleObject(b,10000)==WAIT_OBJECT_0); CloseHandle(a); CloseHandle(b);
    assert(tx_count==1000); for(unsigned i=0;i<tx_count;++i) assert(!strcmp(frames[i],"STATUS\n") || !strcmp(frames[i],"STOP\n"));
    pass("1000 concurrent real UART API sends retain whole frames under SDK tx_mux contract mock");
    reset(); a=CreateThread(NULL,0,submit_writer,NULL,0,NULL); b=CreateThread(NULL,0,command_worker,NULL,0,NULL); assert(a && b);
    assert(WaitForSingleObject(a,10000)==WAIT_OBJECT_0 && WaitForSingleObject(b,10000)==WAIT_OBJECT_0); CloseHandle(a); CloseHandle(b);
    WebCommands_ProcessPending(); assert(tx_count==1 && !strcmp(frames[0],"STOP\n"));
    pass("1000 concurrent duplicate submissions vs 2000 dispatch/receipt reads: exactly one frame, consistent receipt, no deadlock");
    printf("RESULT %u native W1/W2/W3 test groups passed; no hardware validation\n",cases); return 0;
}
