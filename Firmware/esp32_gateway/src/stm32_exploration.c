#include "stm32_exploration.h"
#include "stm32_status.h"
#include "stm32_navigation.h"
#include "web_commands.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
static portMUX_TYPE exploration_lock=portMUX_INITIALIZER_UNLOCKED;
typedef struct {uint32_t v[30];char sectors[37];uint16_t cm[36];bool received,sectors_received,link_fields;int64_t at;} ExplorationSnapshot;
static ExplorationSnapshot exploration,exploration_staged;
static uint32_t exploration_revoked;
#define EXPLORE_LINK_MS 3000U /* Diagnostic projection only; STM32 owns the sole link watchdog. */
/* Volatile correlation only, not an expiring authorization or lease manager. */
static struct {char owner[33];uint32_t key,ack,session,start;bool active;} exploration_owner;
static char exploration_line[256];static size_t exploration_length;static bool exploration_discard;
static uint32_t exploration_counter,exploration_query;static unsigned exploration_part;
static int64_t exploration_poll,exploration_sent;static bool exploration_wanted,exploration_second;
void Stm32Exploration_Start(uint32_t key,const char *owner){
    int64_t now=esp_timer_get_time();uint32_t epoch;char current[33];
    taskENTER_CRITICAL(&exploration_lock);epoch=exploration_revoked;taskEXIT_CRITICAL(&exploration_lock);
    if(!WebCommands_ExploreOwner(current) || strcmp(current,owner))return;
    taskENTER_CRITICAL(&exploration_lock);
    if(epoch!=exploration_revoked){taskEXIT_CRITICAL(&exploration_lock);return;}
    memset(&exploration,0,sizeof exploration);memset(&exploration_staged,0,sizeof exploration_staged);
    memset(&exploration_owner,0,sizeof exploration_owner);exploration_owner.key=key;
    memcpy(exploration_owner.owner,owner,33);
    exploration_wanted=true;exploration_poll=now-500000;taskEXIT_CRITICAL(&exploration_lock);
}
void Stm32Exploration_Revoke(void){
    taskENTER_CRITICAL(&exploration_lock);++exploration_revoked;memset(&exploration_owner,0,sizeof exploration_owner);taskEXIT_CRITICAL(&exploration_lock);
}
void Stm32Exploration_LinkRequest(uint32_t *key,uint32_t *ack){
    taskENTER_CRITICAL(&exploration_lock);*key=exploration_owner.key;*ack=exploration_owner.ack;taskEXIT_CRITICAL(&exploration_lock);
}
bool Stm32Exploration_Fast(void){
    taskENTER_CRITICAL(&exploration_lock);bool active=exploration_owner.key!=0U;taskEXIT_CRITICAL(&exploration_lock);return active;
}
bool Stm32Exploration_PollDue(void){
    int64_t now=esp_timer_get_time();taskENTER_CRITICAL(&exploration_lock);
    bool due=exploration_wanted && (exploration_second || now-exploration_poll>=500000);
    taskEXIT_CRITICAL(&exploration_lock);return due;
}
uint32_t Stm32Exploration_Query(unsigned *part){
    int64_t now=esp_timer_get_time();
    if(exploration_second){*part=1;exploration_second=false;}
    else{*part=0;exploration_poll=now;if(++exploration_counter==0)++exploration_counter;exploration_query=exploration_counter;}
    exploration_part=*part;exploration_sent=now;return exploration_query;
}
static bool ExploreNumber(const char **p,uint32_t *v,bool last){
    *v=0;if(**p<'0'||**p>'9')return false;
    do{unsigned n=(unsigned)(**p-'0');if(*v>(UINT32_MAX-n)/10U)return false;*v=*v*10U+n;++*p;}while(**p>='0'&&**p<='9');
    if(**p!=(last?'\0':','))return false;
    if(!last)++*p;
    return true;
}
static int ExploreHex(char c){return c>='0'&&c<='9'?c-'0':c>='a'&&c<='f'?c-'a'+10:-1;}
static void ExploreAccept(void){
    int64_t now=esp_timer_get_time();if(!exploration_query || now<exploration_sent || now-exploration_sent>=300000)return;
    if(!strncmp(exploration_line,"EXPL,",5U) && exploration_part==0U){
        uint32_t v[30]={0};const char *p=exploration_line+5;unsigned count=0;
        for(;;){
            if(count==30U)return;
            bool last=strchr(p,',')==NULL;
            if(!ExploreNumber(&p,&v[count++],last))return;
            if(last)break;
        }
        if(count!=24U && count!=25U && count!=30U)return;
        if(count==30U && (v[15] || v[16] || v[27]>1U || v[28]>16U || v[29]>1U))return;
        if(v[0]!=1U || v[1]!=exploration_query || v[4]>6U || v[5]>5U || v[7]>12U || v[11]>200000U ||
           v[12]>36U || v[13]>36U || v[14]>1152U || v[15]>1500U || v[17]>4000U || v[18]>4000U || v[19]>1U || v[20]>15U || v[21]>4U || v[22]>UINT16_MAX || (v[23]>1000U && v[4]!=2U) ||
           (v[19] && (!v[2] || (v[4]==2U && v[15]))))return;
        taskENTER_CRITICAL(&exploration_lock);memcpy(exploration_staged.v,v,sizeof v);exploration_staged.received=true;exploration_staged.link_fields=count==30U;
        exploration_staged.at=now;exploration_staged.sectors_received=false;taskEXIT_CRITICAL(&exploration_lock);exploration_second=true;
    }else if(!strncmp(exploration_line,"EXPS,",5U) && exploration_part==1U){
        const char *p=exploration_line+5;uint32_t v[4];
        for(unsigned i=0;i<4U;++i)if(!ExploreNumber(&p,&v[i],false))return;
        if(v[0]!=1U || v[1]!=exploration_query || strlen(p)!=145U || p[36]!=',')return;
        uint16_t cm[36];char sectors[37];
        for(unsigned i=0;i<36U;++i){
            if(p[i]<'0'||p[i]>'3')return;
            sectors[i]=p[i];int a=ExploreHex(p[37+3*i]),b=ExploreHex(p[38+3*i]),c=ExploreHex(p[39+3*i]);
            if(a<0||b<0||c<0)return;
            cm[i]=(uint16_t)(a*256+b*16+c);if(cm[i]>400U || (p[i]=='0' && cm[i]))return;
        }sectors[36]='\0';
        taskENTER_CRITICAL(&exploration_lock);
        bool matched=exploration_staged.received && v[1]==exploration_staged.v[1] && v[2]==exploration_staged.v[2] && v[3]==exploration_staged.v[3] &&
            now>=exploration_staged.at && now-exploration_staged.at<1000000;
        if(matched){
            memcpy(exploration_staged.sectors,sectors,sizeof sectors);memcpy(exploration_staged.cm,cm,sizeof cm);exploration_staged.sectors_received=true;
            exploration=exploration_staged; /* Publish both parts atomically. Retain last full snapshot while the next one is pending. */
        }
        taskEXIT_CRITICAL(&exploration_lock);
        taskENTER_CRITICAL(&exploration_lock);
        if(matched && exploration_owner.key){
            bool same=exploration.v[24]==exploration_owner.key;
            bool live=exploration.v[19] && exploration.v[4]!=2U && !exploration.v[20] && !exploration.v[28];
            if(same && live && exploration.link_fields){
                if(!exploration_owner.active){
                    exploration_owner.active=true;exploration_owner.session=exploration.v[2];exploration_owner.start=exploration.v[3];
                }
                if(exploration.v[2]==exploration_owner.session && exploration.v[3]==exploration_owner.start)
                    exploration_owner.ack=exploration.v[1]; /* Receipt of the complete matching reply, not a movement permit. */
                else memset(&exploration_owner,0,sizeof exploration_owner);
            }else if(exploration_owner.active || (same && !live))memset(&exploration_owner,0,sizeof exploration_owner);
        }
        bool ended=matched && (!exploration.v[19] || exploration.v[4]==2U) && !exploration_owner.key;
        if(ended)exploration_wanted=false;
        taskEXIT_CRITICAL(&exploration_lock);
    }
}
void Stm32Exploration_Feed(const uint8_t *p,size_t n){
    for(size_t i=0;i<n;++i){
        if(p[i]=='\n'){
            if(!exploration_discard){if(exploration_length && exploration_line[exploration_length-1]=='\r')--exploration_length;
                exploration_line[exploration_length]='\0';ExploreAccept();}
            exploration_length=0;exploration_discard=false;
        }else if(!p[i] || exploration_length+1U>=sizeof exploration_line)exploration_discard=true;
        else if(!exploration_discard)exploration_line[exploration_length++]=(char)p[i];
    }
}
int Stm32Exploration_Json(char *out,size_t cap){
    Stm32StatusSnapshot status;Stm32Status_GetSnapshot(&status);int64_t now=esp_timer_get_time();
    taskENTER_CRITICAL(&exploration_lock);exploration_wanted=true; /* Diagnostic discovery only. */
    ExplorationSnapshot s=exploration;
    bool owner_ready=exploration_owner.key && exploration_owner.active;
    bool session_pending=exploration_owner.key && !exploration_owner.active;
    taskEXIT_CRITICAL(&exploration_lock);
    bool valid=s.received && s.v[19];bool wire_connected=s.received && status.connected && now>=s.at && now-s.at<1500000;
    bool connected=valid && wire_connected;
    uint64_t link_age=(uint64_t)s.v[25]+(uint64_t)(now>=s.at?(now-s.at)/1000:0);
    bool link_up=connected && s.link_fields && s.v[4]!=2U && s.v[27] && link_age<EXPLORE_LINK_MS;
    bool terminal=valid && s.v[4]==2U;
    char link_age_json[24]="null";if(s.link_fields)(void)snprintf(link_age_json,sizeof link_age_json,"%" PRIu64,link_age);
    static const char *blocks[]={"none","need_sector_evidence","front_inhibited","rear_inhibited","maneuver_evaluation","rotation_budget","alternatives_exhausted","cycle","environment_change_pending","motion_timeout","no_progress","supervision_lost","internal_link_lost"};
    int n=snprintf(out,cap,"{\"session_pending\":%s,\"session_active\":%s,\"renewal_source\":\"none\",\"renewals_required\":false,\"authorization_mode\":\"session\",\"session_started\":%s,\"session_cancelled\":%s,\"stop_received\":%s,\"link_connected\":%s,\"link_confirmed\":%s,\"link_monitoring\":%s,\"link_age_ms\":%s,\"link_watchdog_ms\":3000,\"link_sequence\":%" PRIu32 ",\"terminal_reason_code\":%u,\"terminal_reason\":\"%s\",\"control_ready\":%s,\"received\":%s,\"connected\":%s,\"available\":%s,\"historical\":%s,\"age_ms\":%" PRIu64 ",\"profile\":\"LOCAL_EXPLORE\",\"session_id\":%" PRIu32 ",\"started_tick\":%" PRIu32 ",\"state_code\":%" PRIu32 ",\"action_code\":%" PRIu32 ",\"challenge\":%" PRIu32 ",\"block_reason\":\"%s\",\"segments\":%" PRIu32 ",\"turns\":%" PRIu32 ",\"retreats\":%" PRIu32 ",\"scan_deg\":%.3f,\"selected_sector\":%" PRIu32 ",\"candidates\":%" PRIu32 ",\"repeated\":%" PRIu32 ",\"lease_ms\":%" PRIu32 ",\"sequence\":%" PRIu32 ",\"critical_faults\":%" PRIu32 ",\"stabilization_state\":%" PRIu32 ",\"stabilization_observations\":%" PRIu32 ",\"stabilization_ms\":%" PRIu32 ",\"sector_step_deg\":10,\"sectors_available\":%s,\"sectors\":\"%s\",\"range_mm\":[",
        session_pending?"true":"false",valid&&!terminal?"true":"false",valid?"true":"false",terminal?"true":"false",s.v[29]?"true":"false",wire_connected?"true":"false",link_up?"true":"false",valid&&!terminal&&s.link_fields?"true":"false",link_age_json,s.v[26],(unsigned)s.v[28],s.link_fields?Stm32Navigation_ReasonName(s.v[28]):"UNKNOWN",owner_ready?"true":"false",s.received?"true":"false",connected?"true":"false",valid?"true":"false",valid&&s.v[4]==2U?"true":"false",(uint64_t)(s.received&&now>=s.at?(now-s.at)/1000:0),
        s.v[2],s.v[3],s.v[4],s.v[5],s.v[6],blocks[s.v[7]],s.v[8],s.v[9],s.v[10],s.v[11]/1000.0,s.v[12],s.v[13],s.v[14],s.v[15],s.v[16],s.v[20],s.v[21],s.v[22],s.v[23],s.sectors_received?"true":"false",s.sectors_received?s.sectors:"");
    if(n<0 || (size_t)n>=cap)return -1;
    for(unsigned i=0;i<36U;++i){int m=snprintf(out+n,cap-(size_t)n,s.sectors_received&&s.sectors[i]!='0'?"%s%u":"%snull",i?",":"",(unsigned)s.cm[i]*10U);
        if(m<0 || (size_t)m>=cap-(size_t)n)return -1;
        n+=m;}
    if((size_t)n+3U>=cap)return -1;
    memcpy(out+n,"]}",3);return n+2;
}
