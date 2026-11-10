#include "mid_odometry.h"
#include "odometry_config.h"
#include "task.h"
#include <math.h>
#include <string.h>
#include <limits.h>
#define ODOM_PI 3.141592654f
typedef struct { TickType_t tick; float x, y, theta; bool valid; } OdomPoint;
static OdometryConfig s_odom_config;
static OdometrySnapshot s_odom;
static OdomPoint s_odom_history[ODOM_HISTORY_LENGTH];
static unsigned s_odom_next, s_odom_count;
static OdometryRay s_odom_rays[2];
static float Odom_Angle(float a) { return remainderf(a, 2.0f * ODOM_PI); }
static bool Odom_Positive(float v) { return isfinite(v) && v > 0.0f; }
static bool Odom_GeometryComplete(const OdometryConfig *c) {
    if(!c || !Odom_Positive(c->left_radius_m) || !Odom_Positive(c->right_radius_m) ||
       !Odom_Positive(c->left_cpr) || !Odom_Positive(c->right_cpr) || !Odom_Positive(c->track_m) ||
       c->left_radius_m>1.0f || c->right_radius_m>1.0f || c->track_m>5.0f ||
       !(c->beam_half_angle>=0.05f && c->beam_half_angle<=0.7854f)) return false;
    for(unsigned i=0;i<2U;++i)
        if(!isfinite(c->sensor_x[i]) || !isfinite(c->sensor_y[i]) || !isfinite(c->sensor_yaw[i]) ||
           fabsf(c->sensor_x[i])>1.0f || fabsf(c->sensor_y[i])>1.0f) return false;
    return true;
}
static void Odom_Publish(const OdometrySnapshot *s) {
    taskENTER_CRITICAL();
    s_odom = *s;
    s_odom_history[s_odom_next] = (OdomPoint){s->tick,s->x_m,s->y_m,s->theta_rad,s->state==2U};
    s_odom_next = (s_odom_next+1U)%ODOM_HISTORY_LENGTH;
    if(s_odom_count<ODOM_HISTORY_LENGTH) ++s_odom_count;
    taskEXIT_CRITICAL();
}
void Odometry_InitConfig(const OdometryConfig *c, TickType_t tick) {
    OdometrySnapshot initial = {.tick=tick,.state=1U};
    taskENTER_CRITICAL();
    memset(&s_odom_config,0,sizeof(s_odom_config));
    if(c) s_odom_config=*c;
    memset(s_odom_history,0,sizeof(s_odom_history));
    memset(s_odom_rays,0,sizeof(s_odom_rays));
    s_odom_next=s_odom_count=0U;
    taskEXIT_CRITICAL();
    if(Odom_GeometryComplete(c)) {
        initial.calibration = c->confirmed && c->sensors_confirmed ?
            ODOM_GEOMETRY_CALIBRATED : ODOM_GEOMETRY_PROVISIONAL;
        initial.state=2U;
    }
    initial.calibrated = initial.calibration==ODOM_GEOMETRY_CALIBRATED;
    Odom_Publish(&initial);
}
void Odometry_Init(TickType_t tick) {
    const OdometryConfig config = {
        ODOM_LEFT_RADIUS_M,ODOM_RIGHT_RADIUS_M,ODOM_LEFT_WHEEL_CPR,ODOM_RIGHT_WHEEL_CPR,ODOM_TRACK_M,
        ODOM_CALIBRATION_CONFIRMED!=0,ODOM_SENSOR_GEOMETRY_CONFIRMED!=0,
        {ODOM_FRONT_X_M,ODOM_REAR_X_M},{ODOM_FRONT_Y_M,ODOM_REAR_Y_M},
        {ODOM_FRONT_YAW_RAD,ODOM_REAR_YAW_RAD},ODOM_BEAM_HALF_ANGLE_RAD
    };
    Odometry_InitConfig(&config,tick);
}
void Odometry_GetSnapshot(OdometrySnapshot *out) {
    if(!out) return;
    taskENTER_CRITICAL(); *out=s_odom; taskEXIT_CRITICAL();
}
static bool Odom_AddCount(int64_t *total,int32_t delta) {
    if((delta>0 && *total>INT64_MAX-delta) || (delta<0 && *total<INT64_MIN-delta)) return false;
    *total+=delta; return true;
}
void Odometry_Update(int32_t left,int32_t right,TickType_t tick) {
    OdometrySnapshot next; Odometry_GetSnapshot(&next);
    if(next.state==0U) return;
    TickType_t dt=(TickType_t)(tick-next.tick);
    bool counts=Odom_AddCount(&next.left_count,left);
    counts=Odom_AddCount(&next.right_count,right) && counts;
    next.increment_m=next.increment_rad=0.0f;
    if(!counts || dt==0U || dt>pdMS_TO_TICKS(ODOM_MAX_INTERVAL_MS)) next.state=3U;
    if(next.state==2U) {
        float dl=(float)left*(2.0f*ODOM_PI*s_odom_config.left_radius_m/s_odom_config.left_cpr);
        float dr=(float)right*(2.0f*ODOM_PI*s_odom_config.right_radius_m/s_odom_config.right_cpr);
        float ds=(dl+dr)*0.5f, da=(dr-dl)/s_odom_config.track_m;
        /* Guardas numericas, no limites de seguridad ni velocidades calibradas. */
        if(!isfinite(ds)||!isfinite(da)||fabsf(ds)>0.25f||fabsf(da)>ODOM_PI) next.state=3U;
        else {
            float half=da*0.5f, sinc=fabsf(half)<0.0001f ? 1.0f-half*half/6.0f : sinf(half)/half;
            float x=next.x_m+ds*sinc*cosf(next.theta_rad+half);
            float y=next.y_m+ds*sinc*sinf(next.theta_rad+half);
            if(!isfinite(x)||!isfinite(y)||fabsf(x)>ODOM_POSITION_LIMIT_M||fabsf(y)>ODOM_POSITION_LIMIT_M) next.state=3U;
            else { next.x_m=x;next.y_m=y;next.theta_rad=Odom_Angle(next.theta_rad+da);
                   next.increment_m=ds;next.increment_rad=da; }
        }
    }
    next.tick=tick; Odom_Publish(&next);
}
bool Odometry_PoseAt(TickType_t tick,float *x,float *y,float *theta) {
    if(!x||!y||!theta) return false;
    OdomPoint newer={0},older={0}; bool found=false;
    taskENTER_CRITICAL();
    if(s_odom.state==2U) {
        TickType_t age=(TickType_t)(s_odom.tick-tick);
        for(unsigned n=0;n<s_odom_count;++n) {
            OdomPoint point=s_odom_history[(s_odom_next+ODOM_HISTORY_LENGTH-1U-n)%ODOM_HISTORY_LENGTH];
            TickType_t p_age=(TickType_t)(s_odom.tick-point.tick);
            if(p_age>=age) { older=point; if(n==0U) newer=point;found=point.valid&&newer.valid;break; }
            newer=point;
        }
    }
    taskEXIT_CRITICAL();
    if(!found) return false; /* Nunca extrapolar al presente ni fuera del historial. */
    TickType_t span=(TickType_t)(newer.tick-older.tick);
    if(span>pdMS_TO_TICKS(ODOM_MAX_INTERVAL_MS)) return false;
    float f=span ? (float)(TickType_t)(tick-older.tick)/(float)span : 0.0f;
    *x=older.x+f*(newer.x-older.x); *y=older.y+f*(newer.y-older.y);
    *theta=Odom_Angle(older.theta+f*Odom_Angle(newer.theta-older.theta)); return true;
}
void Odometry_Observe(unsigned sensor,float distance,TickType_t tick,bool valid) {
    if(sensor>1U) return;
    OdometryRay ray={.tick=tick,.state=1U};
    if(valid && isfinite(distance) && distance>=0.02f && distance<=4.0f) {
        float x,y,a;
        ray.distance_m=distance;ray.state=2U;
        float sx=s_odom_config.sensor_x[sensor],sy=s_odom_config.sensor_y[sensor];
        float yaw=s_odom_config.sensor_yaw[sensor], beam=s_odom_config.beam_half_angle;
        if(isfinite(sx)&&isfinite(sy)&&isfinite(yaw)&&
            fabsf(sx)<=1.0f&&fabsf(sy)<=1.0f&&beam>=0.05f&&beam<=0.7854f&&
            Odometry_PoseAt(tick,&x,&y,&a)) {
            ray.x_m=x+cosf(a)*sx-sinf(a)*sy;ray.y_m=y+sinf(a)*sx+cosf(a)*sy;
            ray.yaw_rad=Odom_Angle(a+yaw);ray.half_angle=beam;ray.state=3U;
        }
    }
    taskENTER_CRITICAL();
    ray.sequence=s_odom_rays[sensor].sequence+1U;
    if(!ray.sequence) ++ray.sequence;
    s_odom_rays[sensor]=ray;
    taskEXIT_CRITICAL();
}
void Odometry_GetRay(unsigned sensor,OdometryRay *out) {
    if(sensor>1U||!out) return;
    taskENTER_CRITICAL(); *out=s_odom_rays[sensor];taskEXIT_CRITICAL();
}
