#include "stm32_navigation.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static portMUX_TYPE s_navigation_lock=portMUX_INITIALIZER_UNLOCKED;
static Stm32NavigationSnapshot s_navigation;
static char s_navigation_line[256];
static size_t s_navigation_length;
static bool s_navigation_discard;
static uint32_t s_navigation_counter, s_navigation_pending;
static int64_t s_navigation_sent, s_navigation_poll, s_navigation_received;
static bool s_navigation_wait_ready;
static uint32_t s_navigation_wait_query, s_navigation_wait_pending;
static int64_t s_navigation_wait_sent;

bool Stm32Navigation_WaitQueryDue(void) {
    int64_t age=esp_timer_get_time()-s_navigation_poll;
    return s_navigation_wait_ready && age>=100000 && age<(int64_t)NAVIGATION_POLL_MS*1000;
}

bool Stm32Navigation_PollDue(void) {
    return Stm32Navigation_WaitQueryDue() || esp_timer_get_time()-s_navigation_poll >= (int64_t)NAVIGATION_POLL_MS*1000;
}
void Stm32Navigation_DeferPoll(void) {
    if (Stm32Navigation_WaitQueryDue()) { s_navigation_wait_ready=false; }
    else { s_navigation_poll=esp_timer_get_time(); }
}
uint32_t Stm32Navigation_BeginQuery(void) {
    s_navigation_wait_ready=false;s_navigation_wait_pending=0U;
    if (++s_navigation_counter==0U) { ++s_navigation_counter; }
    s_navigation_pending=s_navigation_counter;
    s_navigation_sent=s_navigation_poll=esp_timer_get_time();
    return s_navigation_pending;
}
uint32_t Stm32Navigation_BeginWaitQuery(void) {
    s_navigation_wait_ready=false;
    s_navigation_wait_pending=s_navigation_wait_query;
    s_navigation_wait_sent=esp_timer_get_time();
    return s_navigation_wait_pending;
}
void Stm32Navigation_QueryFailed(void) {
    s_navigation_pending=s_navigation_wait_pending=0U;s_navigation_wait_ready=false;
}

static bool Navigation_Field(const char **cursor, int64_t *value, bool *present, bool delta, bool last) {
    const char *p=*cursor;
    *present=true;
    if (delta && *p=='-' && (p[1]==',' || p[1]=='\0')) {
        *present=false; *value=0; ++p;
    } else {
        bool negative=delta && *p=='-';
        if (negative) { ++p; }
        if (*p<'0' || *p>'9') { return false; }
        uint64_t number=0, limit=delta ? (uint64_t)INT64_MAX+(negative ? 1U : 0U) : UINT32_MAX;
        do {
            uint32_t digit=(uint32_t)(*p-'0');
            if (number>(limit-digit)/10U) { return false; }
            number=number*10U+digit; ++p;
        } while (*p>='0' && *p<='9');
        *value=negative ? (number==(uint64_t)INT64_MAX+1U ? INT64_MIN : -(int64_t)number) : (int64_t)number;
    }
    if (*p!=(last ? '\0' : ',')) { return false; }
    *cursor=last ? p : p+1;
    return true;
}

static void Navigation_Accept(void) {
    if (strncmp(s_navigation_line,"NAV,",4U)!=0) { return; }
    const char *p=s_navigation_line+4;
    int64_t v[24]; bool has[24];
    for (unsigned i=0;i<24U;++i) {
        if (!Navigation_Field(&p,&v[i],&has[i],i==20U || i==21U,i==23U)) { return; }
    }
    if (v[0]!=1U || !v[1] || v[2]>2U || v[5]>5U || v[6]>5U || v[7]>12U || v[8]>1023U ||
        v[11]>200000000U || v[12]>200000000U || v[15]>15U || v[19]>8U || has[20]!=has[21]) { return; }
    if (!(v[8]&1U) && (v[3] || v[7] || v[8] || v[9] || v[10])) { return; }
    if ((v[8]&1U) && ((v[5]==2U)!=(v[7]!=0U))) { return; }
    if (!(v[8]&256U) && v[11]) { return; }
    if (!(v[8]&512U) && v[12]) { return; }
    if (!(v[15]&1U) && v[15]) { return; }
    if ((v[15]&2U) ? (v[13]<20U || v[13]>4000U) : (v[13] || v[17])) { return; }
    if ((v[15]&4U) ? (v[14]<20U || v[14]>4000U) : (v[14] || v[18])) { return; }
    if (has[20] && !v[19]) { return; }
    int64_t now=esp_timer_get_time();
    if (s_navigation_pending!=(uint32_t)v[1] || now<s_navigation_sent ||
        now-s_navigation_sent >= (int64_t)NAVIGATION_REPLY_MS*1000) { return; }
    Stm32NavigationSnapshot s={
        .received=true,.deltas_available=has[20],.profile=(uint32_t)v[2],.session_id=(uint32_t)v[3],
        .started_tick=(uint32_t)v[4],.state=(uint32_t)v[5],.last_phase=(uint32_t)v[6],.reason=(uint32_t)v[7],
        .flags=(uint32_t)v[8],.elapsed_ms=(uint32_t)v[9],.phase_elapsed_ms=(uint32_t)v[10],
        .advance_um=(uint32_t)v[11],.reverse_um=(uint32_t)v[12],.front_mm=(uint32_t)v[13],.rear_mm=(uint32_t)v[14],
        .sensor_flags=(uint32_t)v[15],.snapshot_age=(uint32_t)v[16],.front_age=(uint32_t)v[17],.rear_age=(uint32_t)v[18],
        .odometry_error=(uint32_t)v[19],.left_delta=v[20],.right_delta=v[21],
        .decision_age=(uint32_t)v[22],.event_age=(uint32_t)v[23]
    };
    /* El RTT solo aumenta las edades: no se comparan relojes de dos MCU. */
    uint64_t rtt=(uint64_t)(now-s_navigation_sent)/1000U;
    s.decision_age=UINT32_MAX-s.decision_age<rtt ? UINT32_MAX : s.decision_age+(uint32_t)rtt;
    s.event_age=UINT32_MAX-s.event_age<rtt ? UINT32_MAX : s.event_age+(uint32_t)rtt;
    taskENTER_CRITICAL(&s_navigation_lock);
    /* La base se consulta antes que NAVW: no borrar en cada periodo los
     * metadatos YA recibidos que son inmutables en esta misma sesion.
     * No rejuvenecer su reloj, inventar una parte ausente ni heredar otra
     * ventana activa/referencia todavia incompleta. */
    bool same_session=s_navigation.received && (s.flags&1U) && (s_navigation.flags&1U) &&
        s.profile==s_navigation.profile && s.session_id==s_navigation.session_id &&
        s.started_tick==s_navigation.started_tick;
    bool fixed_reference=(s_navigation.state==3U || s_navigation.state==4U) &&
        s.elapsed_ms>=s_navigation.elapsed_ms &&
        s_navigation.stabilization.state==2U && s_navigation.stabilization.reference_available &&
        (s.flags&8U) && (s.state==3U || s.state==4U || (s.state==2U && (s.last_phase==3U || s.last_phase==4U)));
    bool same_terminal=s.state==2U && s_navigation.state==2U && s.last_phase==s_navigation.last_phase &&
        s.reason==s_navigation.reason && s.elapsed_ms==s_navigation.elapsed_ms &&
        s.phase_elapsed_ms==s_navigation.phase_elapsed_ms;
    if (same_session && s_navigation.stabilization.received && (fixed_reference || same_terminal)) {
        s.stabilization=s_navigation.stabilization;
    }
    s_navigation=s; s_navigation_received=now;
    taskEXIT_CRITICAL(&s_navigation_lock);
    s_navigation_pending=0U;
    s_navigation_wait_query=(uint32_t)v[1];s_navigation_wait_ready=true;
}

static void Navigation_AcceptWait(void) {
    if (strncmp(s_navigation_line,"NAVW,",5U)!=0) { return; }
    const char *p=s_navigation_line+5;
    int64_t v[16];bool has[16];
    for (unsigned i=0;i<16U;++i) {
        if (!Navigation_Field(&p,&v[i],&has[i],i>=11U && i<=14U,i==15U)) { return; }
    }
    if (v[0]!=1U || !v[1] || v[4]>4U || v[6]>UINT16_MAX || v[7]>v[6] || v[9]>1U || v[10]>1U ||
        (v[4]==2U && (v[6]<4U || v[7]<4U || v[8]<60U || !has[14])) ||
        has[11]!=has[12] || has[12]!=has[13] ||
        (has[11] && (v[11]<0 || v[11]>UINT32_MAX || !has[14] || v[4]!=2U)) ||
        (has[14] && (v[14]<0 || v[14]>UINT32_MAX || v[4]!=2U))) { return; }
    int64_t now=esp_timer_get_time();
    if (s_navigation_wait_pending!=(uint32_t)v[1] || now<s_navigation_wait_sent ||
        now-s_navigation_wait_sent>=(int64_t)NAVIGATION_REPLY_MS*1000) { return; }
    Stm32NavigationWaitSnapshot w={ .received=true,.state=(uint32_t)v[4],.elapsed_ms=(uint32_t)v[5],
        .observations=(uint32_t)v[6],.window_observations=(uint32_t)v[7],.window_ms=(uint32_t)v[8],
        .left_span=(uint32_t)v[9],.right_span=(uint32_t)v[10],.reference_available=has[11],
        .reference_tick=(uint32_t)v[11],.reference_left=v[12],.reference_right=v[13],
        .reverse_entered=has[14],.entry_tick=(uint32_t)v[14],.received_us=now };
    uint64_t age=(uint64_t)v[15]+(uint64_t)(now-s_navigation_wait_sent)/1000U;
    w.source_age=age>UINT32_MAX ? UINT32_MAX : (uint32_t)age;
    taskENTER_CRITICAL(&s_navigation_lock);
    if (s_navigation.received && s_navigation.session_id==(uint32_t)v[2] &&
        s_navigation.started_tick==(uint32_t)v[3]) { s_navigation.stabilization=w; }
    taskEXIT_CRITICAL(&s_navigation_lock);
    s_navigation_wait_pending=0U;
}

void Stm32Navigation_Feed(const uint8_t *bytes, size_t length) {
    if (!bytes) { return; }
    for (size_t i=0;i<length;++i) {
        uint8_t c=bytes[i];
        if (c=='\n') {
            if (!s_navigation_discard) {
                if (s_navigation_length && s_navigation_line[s_navigation_length-1U]=='\r') { --s_navigation_length; }
                s_navigation_line[s_navigation_length]='\0'; Navigation_Accept();Navigation_AcceptWait();
            }
            s_navigation_length=0; s_navigation_discard=false;
        } else if (!s_navigation_discard) {
            if (c==0U || (c<32U && c!='\r') || c>126U || s_navigation_length>=sizeof s_navigation_line-1U) {
                s_navigation_discard=true;
            } else { s_navigation_line[s_navigation_length++]=(char)c; }
        }
    }
}

static const char *Navigation_Number(char text[24], bool present, uint64_t value) {
    if (!present) { return "null"; }
    (void)snprintf(text,24,"%" PRIu64,value); return text;
}
static const char *Navigation_Millimeters(char text[24], bool present, uint32_t um) {
    if (!present) { return "null"; }
    (void)snprintf(text,24,"%" PRIu32 ".%03" PRIu32,um/1000U,um%1000U); return text;
}
static const char *Navigation_Boolean(bool present, bool value) {
    return !present ? "null" : value ? "true" : "false";
}
static int Navigation_WaitJson(char *out,size_t capacity,const Stm32NavigationWaitSnapshot *w,
                               bool available,int64_t now,char nums[16][24]) {
    static const char *const states[]={"not_requested","waiting","confirmed","timeout","cancelled"};
    char left[24]="null",right[24]="null",state[24]="null";
    bool present=available && w->received;
    if (present) { (void)snprintf(state,sizeof state,"\"%s\"",states[w->state]); }
    if (present && w->reference_available) {
        (void)snprintf(left,sizeof left,"\"%" PRId64 "\"",w->reference_left);
        (void)snprintf(right,sizeof right,"\"%" PRId64 "\"",w->reference_right);
    }
    uint64_t age=now>=w->received_us ? (uint64_t)(now-w->received_us)/1000U+w->source_age : 0U;
    return snprintf(out,capacity,",\"stabilization\":{\"available\":%s,\"state\":%s,\"elapsed_ms\":%s,\"observations\":%s,"
        "\"window_observations\":%s,\"window_ms\":%s,\"left_span_counts\":%s,\"right_span_counts\":%s,"
        "\"snapshot_age_ms\":%s,\"reverse_entry_tick\":%s,\"reference_tick\":%s,\"reference_left_counts\":%s,\"reference_right_counts\":%s}}",
        present?"true":"false",state,Navigation_Number(nums[0],present,w->elapsed_ms),
        Navigation_Number(nums[1],present,w->observations),Navigation_Number(nums[2],present,w->window_observations),
        Navigation_Number(nums[3],present,w->window_ms),Navigation_Number(nums[4],present,w->left_span),
        Navigation_Number(nums[5],present,w->right_span),Navigation_Number(nums[6],present&&now>=w->received_us,age),
        Navigation_Number(nums[7],present&&w->reverse_entered,w->entry_tick),
        Navigation_Number(nums[8],present&&w->reference_available,w->reference_tick),left,right);
}
int Stm32Navigation_Json(char *out, size_t capacity, bool stm32_connected, bool paused) {
    Stm32NavigationSnapshot s;
    int64_t received, now=esp_timer_get_time();
    taskENTER_CRITICAL(&s_navigation_lock); s=s_navigation; received=s_navigation_received; taskEXIT_CRITICAL(&s_navigation_lock);
    uint64_t age=s.received && now>=received ? (uint64_t)(now-received)/1000U : 0U;
    bool connected=s.received && now>=received && stm32_connected && age<NAVIGATION_LINK_MS;
    bool available=s.received && (s.flags&1U);
    bool terminal=available && s.state==2U;
    static const char *const states[]={"ESPERAR_DATOS","AVANZAR","DETENIDO","RETROCEDER_ACOTADO","GIRAR_ACOTADO","DETENER_Y_ESPERAR"};
    static const char *const reasons[]={"NONE","TARGET","CANCELLED","SAFETY","TIMEOUT","OBSTACLE","SENSORS","ODOMETRY","EMISSION","EVASION_COMPLETED","EVASION_NOT_AUTHORIZED","OPERATOR_STOP","NO_PROGRESS"};
    static const char *const errors[]={"none","unavailable","stale_or_future","timestamp","wrong_direction","overflow","nonfinite","wheel_disagreement","numeric_range"};
    const char *yes="true", *no="false";
    char nums[16][24], left[24]="null", right[24]="null";
    char phase[32]="null", reason[40]="null", error[32]="null";
    if (available) {
        (void)snprintf(phase,sizeof phase,"\"%s\"",states[s.last_phase]);
        (void)snprintf(reason,sizeof reason,"\"%s\"",reasons[s.reason]);
        (void)snprintf(error,sizeof error,"\"%s\"",errors[s.odometry_error]);
    }
    if (available && s.deltas_available) {
        (void)snprintf(left,sizeof left,"\"%" PRId64 "\"",s.left_delta);
        (void)snprintf(right,sizeof right,"\"%" PRId64 "\"",s.right_delta);
    }
    int n=snprintf(out,capacity,
        "{\"received\":%s,\"connected\":%s,\"stm32_connected\":%s,\"paused\":%s,\"stale\":%s,\"age_ms\":%s,"
        "\"available\":%s,\"historical\":%s,\"profile\":%s,\"session_id\":%s,\"started_tick\":%s,"
        "\"state\":%s,\"state_code\":%s,\"last_phase\":%s,\"finish_reason\":%s,\"finish_code\":%s,\"terminal\":%s,\"active_reported\":%s,"
        "\"obstacle_detected\":%s,\"reverse\":{\"considered\":%s,\"authorized\":%s,\"requested\":%s,\"published\":%s,\"applied\":%s,\"completed\":%s,\"progress_mm\":%s},"
        "\"advance_progress_mm\":%s,\"elapsed_ms\":%s,\"phase_elapsed_ms\":%s,\"event_age_ms\":%s,"
        "\"decision\":{\"age_ms\":%s,\"snapshot_available\":%s,\"snapshot_age_at_decision_ms\":%s,\"front_mm\":%s,\"front_valid\":%s,\"front_age_at_decision_ms\":%s,"
        "\"rear_mm\":%s,\"rear_valid\":%s,\"rear_age_at_decision_ms\":%s,\"cliff\":%s},"
        "\"odometry_error\":%s,\"left_delta_counts\":%s,\"right_delta_counts\":%s}",
        s.received?yes:no,connected?yes:no,stm32_connected?yes:no,paused?yes:no,connected?no:yes,
        Navigation_Number(nums[0],s.received&&now>=received,age),available?yes:no,(available&&(!connected||terminal))?yes:no,
        !s.received?"null":s.profile==2U?"\"AUTO_2\"":s.profile==1U?"\"AUTO_1\"":"\"B7_LEGACY\"",
        Navigation_Number(nums[1],available,s.session_id),Navigation_Number(nums[2],available,s.started_tick),
        available? (s.state==0U?"\"ESPERAR_DATOS\"":s.state==1U?"\"AVANZAR\"":s.state==2U?"\"DETENIDO\"":s.state==3U?"\"RETROCEDER_ACOTADO\"":s.state==4U?"\"GIRAR_ACOTADO\"":"\"DETENER_Y_ESPERAR\""):"null",
        Navigation_Number(nums[3],available,s.state),
        phase,reason,Navigation_Number(nums[4],available,s.reason),Navigation_Boolean(available,terminal),Navigation_Boolean(available,!terminal),
        Navigation_Boolean(available,s.flags&2U),Navigation_Boolean(available,s.flags&4U),Navigation_Boolean(available,s.flags&8U),
        Navigation_Boolean(available,s.flags&16U),Navigation_Boolean(available,s.flags&32U),Navigation_Boolean(available,s.flags&64U),
        Navigation_Boolean(available,s.flags&128U),
        Navigation_Millimeters(nums[5],available&&(s.flags&512U),s.reverse_um),
        Navigation_Millimeters(nums[6],available&&(s.flags&256U),s.advance_um),
        Navigation_Number(nums[7],available,s.elapsed_ms),Navigation_Number(nums[8],available,s.phase_elapsed_ms),
        Navigation_Number(nums[9],available,age+s.event_age),Navigation_Number(nums[10],available&&(s.sensor_flags&1U),age+s.decision_age),
        Navigation_Boolean(available,s.sensor_flags&1U),
        Navigation_Number(nums[11],available&&(s.sensor_flags&1U),s.snapshot_age),
        Navigation_Number(nums[12],available&&(s.sensor_flags&2U),s.front_mm),Navigation_Boolean(available&&(s.sensor_flags&1U),s.sensor_flags&2U),
        Navigation_Number(nums[13],available&&(s.sensor_flags&2U),s.front_age),
        Navigation_Number(nums[14],available&&(s.sensor_flags&4U),s.rear_mm),Navigation_Boolean(available&&(s.sensor_flags&1U),s.sensor_flags&4U),
        Navigation_Number(nums[15],available&&(s.sensor_flags&4U),s.rear_age),
        Navigation_Boolean(available&&(s.sensor_flags&1U),s.sensor_flags&8U),error,left,right);
    if (n<=0 || (size_t)n>=capacity) { return -1; }
    int extra=Navigation_WaitJson(out+n-1,capacity-(size_t)n+1U,&s.stabilization,available,now,nums);
    return extra>=0 && (size_t)extra<capacity-(size_t)n+1U ? n-1+extra : -1;
}
