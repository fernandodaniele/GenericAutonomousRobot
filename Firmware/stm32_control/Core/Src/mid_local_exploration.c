#include "mid_local_exploration.h"
#include "odometry_config.h"
#include <math.h>
#include <string.h>
#define LOCAL_PI 3.14159265358979323846f
float Local_Difference(float a){return remainderf(a,2.0f*LOCAL_PI);}
void Local_Reset(LocalEvidence *e,float x,float y){
    memset(e->state,0,sizeof e->state);memset(e->mm,0,sizeof e->mm);
    /* Keep sequence barriers and bounded visited-pose history on translation. */
    e->origin_set=true;e->x=x;e->y=y;
}
void Local_Expire(LocalEvidence *e,TickType_t now){
    for(unsigned i=0;i<LOCAL_SECTORS;++i)
        if(e->state[i] && (TickType_t)(now-e->tick[i])>pdMS_TO_TICKS(LOCAL_EVIDENCE_MS))e->state[i]=0;
}
bool Local_Observe(LocalEvidence *e,unsigned sensor,const OdometryRay *r,TickType_t now){
    Local_Expire(e,now);
    if(sensor>1 || !r || r->state!=3U || !isfinite(r->yaw_rad) || !isfinite(r->x_m) || !isfinite(r->y_m) ||
       !isfinite(r->distance_m) || r->distance_m<0.02f || r->distance_m>4.0f ||
       (TickType_t)(now-r->tick)>pdMS_TO_TICKS(200U) || (e->seen[sensor] && ((uint32_t)(r->sequence-e->sequence[sensor])==0U || (uint32_t)(r->sequence-e->sequence[sensor])>=0x80000000U)))return false;
    e->seen[sensor]=true;e->sequence[sensor]=r->sequence;
    float a=Local_Difference(r->yaw_rad);if(a<0)a+=2.0f*LOCAL_PI;
    unsigned s=(unsigned)floorf(a/(LOCAL_PI/18.0f)+0.5f)%LOCAL_SECTORS;
    uint16_t mm=(uint16_t)(r->distance_m*1000.0f);
    if(mm>=LOCAL_CLEAR_MM && e->state[s]==2U)return true; /* A far echo does not disprove a fresh nearby cone. */
    e->mm[s]=mm;e->tick[s]=r->tick;e->state[s]=mm>=LOCAL_CLEAR_MM?3U:1U;
    /* Inflate a nearby return by chassis radius + margin and the provisional
     * beam uncertainty. Never mark the rest of a cone free from one echo. */
    if(mm<LOCAL_CLEAR_MM){
        float radius=hypotf(ODOM_CHASSIS_LENGTH_M,ODOM_CHASSIS_WIDTH_M)*0.5f+LOCAL_MARGIN_MM*0.001f;
        float spread=asinf(fminf(1.0f,radius/r->distance_m))+ODOM_BEAM_HALF_ANGLE_RAD;
        unsigned n=(unsigned)ceilf(spread/(LOCAL_PI/18.0f));if(n>18U)n=18U;
        for(int j=-(int)n;j<=(int)n;++j){unsigned k=(unsigned)((int)s+j+36)%36U;e->state[k]=2U;e->tick[k]=r->tick;e->mm[k]=mm;}
    }
    return true;
}
static unsigned Local_Repeats(const LocalEvidence *e,float x,float y,float a){
    unsigned n=0;
    for(unsigned i=0;i<e->visit_count;++i)
        if(hypotf(x-e->visits[i].x,y-e->visits[i].y)<LOCAL_HISTORY_RADIUS_M && fabsf(Local_Difference(a-e->visits[i].yaw))<LOCAL_PI/9.0f)++n;
    return n;
}
int Local_Select(const LocalEvidence *e,float x,float y,float yaw,uint32_t *evaluated,uint32_t *repeated){
    int best=-1;float best_cost=1e9f;*evaluated=0;*repeated=0;
    uint8_t history[36]={0};
    /* At most 32 position checks and five angular neighbors per visit, not
     * 36*32 hypot calls while Navigation owns the control transaction. */
    for(unsigned v=0;v<e->visit_count;++v){
        if(hypotf(x-e->visits[v].x,y-e->visits[v].y)>=LOCAL_HISTORY_RADIUS_M)continue;
        float a=Local_Difference(e->visits[v].yaw);if(a<0)a+=2.0f*LOCAL_PI;
        unsigned center=(unsigned)floorf(a/(LOCAL_PI/18.0f)+0.5f)%36U;
        for(int j=-2;j<=2;++j){unsigned k=(unsigned)((int)center+j+36)%36U;
            if(fabsf(Local_Difference(k*(LOCAL_PI/18.0f)-e->visits[v].yaw))<LOCAL_PI/9.0f)++history[k];}
    }
    for(unsigned i=0;i<36U;++i){
        if(e->state[i]!=3U || e->state[(i+35U)%36U]!=3U || e->state[(i+1U)%36U]!=3U)continue;
        unsigned distance=e->mm[i];if(e->mm[(i+35U)%36U]<distance)distance=e->mm[(i+35U)%36U];
        if(e->mm[(i+1U)%36U]<distance)distance=e->mm[(i+1U)%36U];
        float radius=500.0f*hypotf(ODOM_CHASSIS_LENGTH_M,ODOM_CHASSIS_WIDTH_M)+LOCAL_MARGIN_MM;
        unsigned half=(unsigned)ceilf(fmaxf(1.0f,(atan2f(radius,(float)distance)*180.0f/LOCAL_PI-5.0f)/10.0f));
        bool corridor=true;
        for(int j=-(int)half;j<=(int)half;++j)if(e->state[(unsigned)((int)i+j+36)%36U]!=3U)corridor=false;
        if(!corridor)continue;
        ++*evaluated;float angle=i*(LOCAL_PI/18.0f);unsigned repeats=history[i];
        *repeated+=repeats;
        /* Repeated failed bearings are exhausted until evidence changes. */
        if(repeats)continue;
        float cost=fabsf(Local_Difference(angle-yaw))*100.0f-0.01f*e->mm[i];
        if(cost<best_cost){best_cost=cost;best=(int)i;}
    }
    return best;
}
bool Local_Visited(const LocalEvidence *e,float x,float y,float yaw){return Local_Repeats(e,x,y,yaw)!=0U;}
static void Local_RecordVisit(LocalEvidence *e,float x,float y,float yaw,bool retreat){
    unsigned n=e->visit_next;e->visits[n].x=x;e->visits[n].y=y;e->visits[n].yaw=yaw;e->visits[n].retreat=retreat;
    e->visit_next=(uint8_t)((n+1U)%LOCAL_VISITS);if(e->visit_count<LOCAL_VISITS)++e->visit_count;
}
bool Local_Visit(LocalEvidence *e,float x,float y,float yaw){
    if(Local_Repeats(e,x,y,yaw))return false;
    Local_RecordVisit(e,x,y,yaw,false);
    return true;
}
bool Local_RetreatAllowed(const LocalEvidence *e,float x,float y,float yaw){
    if(!isfinite(x) || !isfinite(y) || !isfinite(yaw))return false;
    float c=cosf(yaw),s=sinf(yaw);
    /* General visits still exclude repeated forward bearings within 50 mm.
     * Withdrawal is different: all nearby withdrawal origins must be behind
     * us along the current backward direction, with measured net progress.
     * Changing heading or revisiting an origin cannot manufacture progress. */
    for(unsigned i=0;i<e->visit_count;++i){
        if(!e->visits[i].retreat)continue;
        float dx=x-e->visits[i].x,dy=y-e->visits[i].y;
        if(hypotf(dx,dy)<LOCAL_HISTORY_RADIUS_M && -(dx*c+dy*s)<LOCAL_RETREAT_PROGRESS_M)return false;
    }
    return true;
}
bool Local_RetreatVisit(LocalEvidence *e,float x,float y,float yaw){
    if(!Local_RetreatAllowed(e,x,y,yaw))return false;
    Local_RecordVisit(e,x,y,yaw,true);
    return true;
}
