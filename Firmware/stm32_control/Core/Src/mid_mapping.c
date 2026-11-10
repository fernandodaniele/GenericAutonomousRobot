#include "mid_mapping.h"
#include "mid_odometry.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* Primer query tras arranque identifica la instancia STM32. ESP32 usa una
 * secuencia no nula; cambios de epoch borran mapa/path, no confundir tick wrap. */
static uint32_t s_mapping_epoch;
bool Mapping_Query(const char *message,uint32_t *query,unsigned *part) {
    if(!message||!query||!part||strncmp(message,"MAPPING Q=",10U)!=0) return false;
    const char *p=message+10;uint32_t value=0;
    if(*p<'0'||*p>'9') return false;
    do { uint32_t d=(uint32_t)(*p-'0');if(value>(UINT32_MAX-d)/10U)return false;
         value=value*10U+d;++p; } while(*p>='0'&&*p<='9');
    if(!value||strncmp(p," S=",3U)!=0||p[3]<'0'||p[3]>'2'||p[4]!='\0') return false;
    *query=value;*part=(unsigned)(p[3]-'0');return true;
}
static const char *Mapping_Count(char text[22],int64_t value) {
    /* Newlib nano no garantiza %lld: conversion acotada incluso INT64_MIN. */
    uint64_t magnitude=value<0 ? (uint64_t)(-(value+1))+1U : (uint64_t)value;
    unsigned n=21U;text[n]='\0';
    do { text[--n]=(char)('0'+magnitude%10U);magnitude/=10U; } while(magnitude);
    if(value<0) text[--n]='-';
    return text+n;
}
static const char *Mapping_Metric(char text[12],bool present,float v) {
    if(!present||!isfinite(v)||fabsf(v)>101.0f) return "-";
    (void)snprintf(text,12U,"%ld",(long)lroundf(v*1000.0f));return text;
}
int Mapping_Format(char *out,size_t capacity,uint32_t query,unsigned part,TickType_t now) {
    if(!out||!capacity||!query||part>2U) return -1;
    if(!s_mapping_epoch) s_mapping_epoch=query;
    char metric[5][12];int n;
    if(part==0U) {
        OdometrySnapshot s;Odometry_GetSnapshot(&s);char left[22],right[22];
        uint64_t age=(uint64_t)(TickType_t)(now-s.tick)*1000U/configTICK_RATE_HZ;
        if(age>UINT32_MAX) age=UINT32_MAX;
        /* v2: CAL es clase 0/1/2; un receptor v1 no debe tomar provisional=1
         * como confirmacion fisica. El contrato RAY permanece en version 1. */
        n=snprintf(out,capacity,"POSE,2,%lu,%lu,%lu,%u,%u,%s,%s,%s,%s,%s,%s,%s,%lu\n",
            (unsigned long)query,(unsigned long)s_mapping_epoch,(unsigned long)s.tick,s.state,(unsigned)s.calibration,
            Mapping_Count(left,s.left_count),Mapping_Count(right,s.right_count),
            Mapping_Metric(metric[0],s.state==2U,s.x_m),Mapping_Metric(metric[1],s.state==2U,s.y_m),
            Mapping_Metric(metric[2],s.state==2U,s.theta_rad),Mapping_Metric(metric[3],s.state==2U,s.increment_m),
            Mapping_Metric(metric[4],s.state==2U,s.increment_rad),(unsigned long)age);
    } else {
        OdometryRay ray;Odometry_GetRay(part-1U,&ray);
        uint64_t age=(uint64_t)(TickType_t)(now-ray.tick)*1000U/configTICK_RATE_HZ;
        if(age>UINT32_MAX) age=UINT32_MAX;
        n=snprintf(out,capacity,"RAY,1,%lu,%lu,%lu,%u,%lu,%u,%lu,%s,%s,%s,%s,%s\n",
            (unsigned long)query,(unsigned long)s_mapping_epoch,(unsigned long)ray.tick,part-1U,
            (unsigned long)ray.sequence,ray.state,(unsigned long)age,
            Mapping_Metric(metric[0],ray.state>=2U,ray.distance_m),Mapping_Metric(metric[1],ray.state==3U,ray.x_m),
            Mapping_Metric(metric[2],ray.state==3U,ray.y_m),Mapping_Metric(metric[3],ray.state==3U,ray.yaw_rad),
            Mapping_Metric(metric[4],ray.state==3U,ray.half_angle));
    }
    return n>0&&(size_t)n<capacity ? n : -1;
}
