#include "stm32_mapping.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdarg.h>
#include <limits.h>
typedef struct {
    bool available,calibrated;
    uint8_t calibration; /* 0 incompleta, 1 provisional, 2 fisicamente calibrada */
    uint32_t tick,age,state;
    int32_t x,y,theta,ds,da;
    char left[22],right[22];
    int64_t received;
} MapPose;
typedef struct { uint32_t sequence,state,age;int64_t received; } MapRayStatus;
typedef struct { MappingGrid grid; MapPose pose;MapRayStatus rays[2];uint32_t epoch; } MapCache;
static portMUX_TYPE s_map_lock=portMUX_INITIALIZER_UNLOCKED;
/* Solo UART modifica working. HTTP nunca lo lee sin una copia publicada. */
static MapCache s_map_work,s_map_published;
static uint32_t s_map_counter,s_map_pending;
static unsigned s_map_part,s_map_expected;
static int64_t s_map_sent,s_map_poll;
static char s_map_line[192];
static size_t s_map_length;
static bool s_map_discard;
static void Map_Publish(void) {
    taskENTER_CRITICAL(&s_map_lock);s_map_published=s_map_work;taskEXIT_CRITICAL(&s_map_lock);
}
bool Stm32Mapping_PollDue(void) {
    int64_t now=esp_timer_get_time();
    if(s_map_pending && now-s_map_sent<(int64_t)MAPPING_REPLY_MS*1000) return false;
    return now-s_map_poll>=(int64_t)MAPPING_POLL_MS*1000;
}
void Stm32Mapping_DeferPoll(void) { s_map_poll=esp_timer_get_time(); }
uint32_t Stm32Mapping_BeginQuery(unsigned *part) {
    if(!s_map_counter) esp_fill_random(&s_map_counter,sizeof(s_map_counter));
    if(++s_map_counter==0U) ++s_map_counter;
    s_map_pending=s_map_counter;s_map_expected=s_map_part;
    s_map_part=(s_map_part+1U)%3U;*part=s_map_expected;
    s_map_sent=s_map_poll=esp_timer_get_time();return s_map_pending;
}
void Stm32Mapping_QueryFailed(void) { s_map_pending=0U; }
static bool Map_Unsigned(const char *s,uint32_t *out) {
    uint32_t v=0;if(!s||*s<'0'||*s>'9') return false;
    do { uint32_t d=(uint32_t)(*s-'0');if(v>(UINT32_MAX-d)/10U)return false;v=v*10U+d;++s; }
    while(*s>='0'&&*s<='9');
    if(*s) return false;
    *out=v;return true;
}
static bool Map_Signed(const char *s,int32_t *out,int32_t limit) {
    bool minus=*s=='-';if(minus) ++s;
    uint32_t v;if(!Map_Unsigned(s,&v)||v>(uint32_t)limit) return false;
    *out=minus?-(int32_t)v:(int32_t)v;return true;
}
static bool Map_Count(const char *s) {
    bool minus=*s=='-';if(minus) ++s;
    if(strlen(s)>19U) return false;
    uint64_t v=0,limit=minus ? (uint64_t)INT64_MAX+1U:(uint64_t)INT64_MAX;
    if(*s<'0'||*s>'9') return false;
    do { uint64_t d=(uint64_t)(*s-'0');if(v>(limit-d)/10U) return false;v=v*10U+d;++s; }
    while(*s>='0'&&*s<='9');return *s=='\0';
}
static uint32_t Map_Age(uint32_t base,int64_t received,int64_t now) {
    uint64_t elapsed=now>received ? (uint64_t)(now-received)/1000U : 0U;
    return elapsed>UINT32_MAX-base ? UINT32_MAX:base+(uint32_t)elapsed;
}
static void Map_Accept(void) {
    bool pose=strncmp(s_map_line,"POSE,",5U)==0;
    if(!pose&&strncmp(s_map_line,"RAY,",4U)!=0) return;
    char *fields[15];unsigned count=0;char *p=s_map_line;
    fields[count++]=p;
    for(;*p;++p) if(*p==',') { if(count>=15U)return;*p='\0';fields[count++]=p+1; }
    if(count!=(pose?15U:14U)) return;
    uint32_t version,q,epoch,tick;
    if(!Map_Unsigned(fields[1],&version)||(version!=1U&&!(pose&&version==2U))||!Map_Unsigned(fields[2],&q)||!q||
       !Map_Unsigned(fields[3],&epoch)||!epoch||!Map_Unsigned(fields[4],&tick)) return;
    int64_t now=esp_timer_get_time();
    if(q!=s_map_pending||now<s_map_sent||now-s_map_sent>=(int64_t)MAPPING_REPLY_MS*1000||
       pose!=(s_map_expected==0U)) return;
    uint32_t rtt=(uint32_t)((now-s_map_sent+999)/1000U);
    MapPose next={0};MapRayStatus ray={0};unsigned sensor=0;
    int32_t rx=0,ry=0,heading=0;uint32_t range=0,beam=0;
    if(pose) {
        uint32_t cal;
        if(!Map_Unsigned(fields[5],&next.state)||next.state>3U||!Map_Unsigned(fields[6],&cal)||cal>(version==2U?2U:1U)||
           !Map_Count(fields[7])||strlen(fields[7])>=22U||!Map_Count(fields[8])||strlen(fields[8])>=22U||
           !Map_Unsigned(fields[14],&next.age)) return;
        if((next.state==2U&&!cal)||(next.state<=1U&&cal)) return;
        int32_t *metrics[]={&next.x,&next.y,&next.theta,&next.ds,&next.da};
        const int32_t limits[]={100000,100000,3142,250,3142};
        for(unsigned i=0;i<5U;++i) {
            if(next.state==2U) { if(!Map_Signed(fields[9+i],metrics[i],limits[i]))return; }
            else if(strcmp(fields[9+i],"-")) return;
        }
        /* v1 solo acreditaba ruedas. Sin prueba del montaje no elevarlo a
         * calibracion fisica completa; conservar metrica como provisional. */
        next.calibration=(uint8_t)cal;
        next.calibrated=cal==2U;next.available=true;next.tick=tick;
        strcpy(next.left,fields[7]);strcpy(next.right,fields[8]);next.received=now;
        next.age=next.age>UINT32_MAX-rtt?UINT32_MAX:next.age+rtt;
    } else {
        uint32_t si;
        if(!Map_Unsigned(fields[5],&si)||si>1U||si+1U!=s_map_expected||
           !Map_Unsigned(fields[6],&ray.sequence)||!Map_Unsigned(fields[7],&ray.state)||ray.state>3U||
           !Map_Unsigned(fields[8],&ray.age)||(ray.state==0U)!=(ray.sequence==0U))return;
        sensor=si;
        if(ray.state>=2U) { if(!Map_Unsigned(fields[9],&range)||range<20U||range>4000U)return; }
        else if(strcmp(fields[9],"-")) return;
        if(ray.state==3U) {
            if(!Map_Signed(fields[10],&rx,101000)||!Map_Signed(fields[11],&ry,101000)||
               !Map_Signed(fields[12],&heading,3142)||!Map_Unsigned(fields[13],&beam)||beam<50U||beam>785U)return;
        } else for(unsigned i=10U;i<14U;++i) if(strcmp(fields[i],"-"))return;
        ray.received=now;ray.age=ray.age>UINT32_MAX-rtt?UINT32_MAX:ray.age+rtt;
    }
    if(s_map_work.epoch==epoch) {
        if(pose && s_map_work.pose.available &&
            (uint32_t)(next.tick-s_map_work.pose.tick)>INT32_MAX) return;
        if(!pose && s_map_work.rays[sensor].sequence &&
            (uint32_t)(ray.sequence-s_map_work.rays[sensor].sequence)>INT32_MAX) return;
    }
    s_map_pending=0U;
    if(s_map_work.epoch!=epoch) { memset(&s_map_work,0,sizeof(s_map_work));s_map_work.epoch=epoch; }
    if(pose) {
        if(s_map_work.pose.available&&s_map_work.pose.calibration!=next.calibration) {
            MappingGrid_Clear(&s_map_work.grid);
            memset(s_map_work.rays,0,sizeof(s_map_work.rays));
        }
        s_map_work.pose=next;
        if(next.state==2U&&next.age<=500U) MappingGrid_AddPose(&s_map_work.grid,next.x,next.y);
    } else {
        if(ray.sequence!=s_map_work.rays[sensor].sequence&&ray.state==3U&&
            s_map_work.pose.calibration!=0U&&s_map_work.pose.state==2U)
            (void)MappingGrid_Ray(&s_map_work.grid,rx,ry,heading,range,beam,ray.age);
        s_map_work.rays[sensor]=ray;
    }
    Map_Publish();
}
void Stm32Mapping_Feed(const uint8_t *bytes,size_t n) {
    if(!bytes) return;
    for(size_t i=0;i<n;++i) {
        uint8_t c=bytes[i];
        if(c=='\n') {
            if(!s_map_discard) {
                if(s_map_length&&s_map_line[s_map_length-1U]=='\r') --s_map_length;
                s_map_line[s_map_length]='\0';Map_Accept();
            }
            s_map_length=0;s_map_discard=false;
        } else if(!s_map_discard) {
            if(c==0U||s_map_length>=sizeof(s_map_line)-1U) s_map_discard=true;
            else s_map_line[s_map_length++]=(char)c;
        }
    }
}
static bool Map_Print(char *out,size_t capacity,size_t *used,const char *format,...) {
    if(*used>=capacity) return false;
    va_list ap;va_start(ap,format);int n=vsnprintf(out+*used,capacity-*used,format,ap);va_end(ap);
    if(n<0||(size_t)n>=capacity-*used)return false;
    *used+=(size_t)n;return true;
}
int Stm32Mapping_Json(char *out,size_t capacity,bool stm32_connected) {
    if(!out||!capacity) return -1;
    static MapCache snapshot; /* HTTP unico: no poner la grilla en el stack. */
    taskENTER_CRITICAL(&s_map_lock);snapshot=s_map_published;taskEXIT_CRITICAL(&s_map_lock);
    int64_t now=esp_timer_get_time();MapPose *p=&snapshot.pose;
    uint32_t age=Map_Age(p->age,p->received,now);
    bool connected=stm32_connected&&p->available&&age<MAPPING_LINK_MS;
    bool valid=connected&&p->state==2U&&age<=500U;
    static const char *states[]={"no_sample","uncalibrated","valid","invalid"};
    static const char *calibrations[]={"incomplete","provisional","calibrated"};
    size_t used=0;
#define MAP_PRINT(...) do { if(!Map_Print(out,capacity,&used,__VA_ARGS__)) return -1; } while(0)
    MAP_PRINT("{\"connected\":%s,\"calibrated\":%s,\"calibration\":\"%s\",\"epoch\":%" PRIu32 ",\"pose\":{\"valid\":%s,\"state\":\"%s\",\"stale\":%s,",
        connected?"true":"false",p->calibrated?"true":"false",calibrations[p->calibration],snapshot.epoch,valid?"true":"false",
        states[p->state],!connected||age>500U?"true":"false");
    if(p->available) MAP_PRINT("\"tick\":%" PRIu32 ",\"age_ms\":%" PRIu32 ",\"left_counts\":\"%s\",\"right_counts\":\"%s\",",
        p->tick,age,p->left,p->right);
    else MAP_PRINT("\"tick\":null,\"age_ms\":null,\"left_counts\":null,\"right_counts\":null,");
    if(p->state==2U) MAP_PRINT("\"x_mm\":%" PRId32 ",\"y_mm\":%" PRId32 ",\"theta_mrad\":%" PRId32 ",\"increment_mm\":%" PRId32 ",\"increment_mrad\":%" PRId32 "},",p->x,p->y,p->theta,p->ds,p->da);
    else MAP_PRINT("\"x_mm\":null,\"y_mm\":null,\"theta_mrad\":null,\"increment_mm\":null,\"increment_mrad\":null},");
    MAP_PRINT("\"rays\":[");
    for(unsigned i=0;i<2U;++i) MAP_PRINT("%s{\"state\":%" PRIu32 ",\"age_ms\":%" PRIu32 "}",i?",":"",snapshot.rays[i].state,Map_Age(snapshot.rays[i].age,snapshot.rays[i].received,now));
    MAP_PRINT("],\"grid\":{\"width\":%u,\"height\":%u,\"resolution_mm\":%u,\"origin_x_mm\":%" PRId32 ",\"origin_y_mm\":%" PRId32 ",\"revision\":%" PRIu32 ",\"cells\":\"",MAP_WIDTH,MAP_HEIGHT,MAP_CELL_MM,MAP_ORIGIN_X_MM,MAP_ORIGIN_Y_MM,snapshot.grid.revision);
    if(used+MAP_CELLS+1U>=capacity) return -1;
    for(unsigned i=0;i<MAP_CELLS;++i) out[used++]=(char)('0'+snapshot.grid.cells[i]);
    MAP_PRINT("\"},\"trajectory_mm\":[");
    for(unsigned i=0;i<snapshot.grid.path_count;++i) {
        unsigned index=(snapshot.grid.path_next+MAP_PATH_LENGTH-snapshot.grid.path_count+i)%MAP_PATH_LENGTH;
        MAP_PRINT("%s[%" PRId32 ",%" PRId32 "]",i?",":"",snapshot.grid.path[index][0],snapshot.grid.path[index][1]);
    }
    MAP_PRINT("]}");
#undef MAP_PRINT
    return (int)used;
}
