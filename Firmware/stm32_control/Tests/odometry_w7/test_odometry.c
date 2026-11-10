/* Datos sinteticos SOLO en tests; fuentes reales y encoder real. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../Core/Src/mid_odometry.c"
#include "../../Core/Src/mid_mapping.c"
#include "../../Core/Src/drv_encoder.c"
static unsigned cases,depth;
#define CHECK(v) do {if(!(v)){fprintf(stderr,"FAIL %u: %s\n",(unsigned)__LINE__,#v);exit(1);}}while(0)
#define PASS(s) do{CHECK(!depth);printf("PASS %02u %s\n",++cases,s);}while(0)
#define NEAR(a,b,t) CHECK(fabsf((a)-(b))<(t))
void test_enter(void){CHECK(!depth);++depth;}
void test_exit(void){CHECK(depth==1U);--depth;}
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *t,uint32_t c){CHECK(t&&c==TIM_CHANNEL_ALL);return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *t,uint32_t c){CHECK(t&&c==TIM_CHANNEL_ALL);return HAL_OK;}
static OdometryConfig config(void){return (OdometryConfig){0.05f,0.05f,1000,1000,0.2f,true,true,{0.1f,-0.1f},{0,0},{0,3.141592654f},0.25f};}
static OdometrySnapshot read_pose(void){OdometrySnapshot p;Odometry_GetSnapshot(&p);return p;}
int main(void){
    OdometrySnapshot p;OdometryRay ray;float x,y,a;
    Odometry_GetSnapshot(&p);CHECK(p.state==0U);Odometry_Update(10,10,0);CHECK(read_pose().state==0U);
    PASS("no encoder baseline means no odometry sample");
    Odometry_Init(0);Odometry_Update(100,-50,10);p=read_pose();
    CHECK(p.state==1U&&!p.calibrated&&p.left_count==100&&p.right_count==-50);
    CHECK(!Odometry_PoseAt(0,&x,&y,&a));Odometry_Observe(0,0.5f,0,true);Odometry_GetRay(0,&ray);
    CHECK(ray.state==2U&&ray.sequence==1U);PASS("production defaults expose real signed counts, no invented metric pose or mapped echo");
    OdometryConfig c=config();Odometry_InitConfig(&c,0);Odometry_Update(100,100,10);p=read_pose();
    NEAR(p.x_m,0.031415927f,1e-6f);NEAR(p.y_m,0,1e-6f);NEAR(p.theta_rad,0,1e-6f);
    CHECK(p.state==2U&&p.calibrated&&p.tick==10U);NEAR(p.increment_m,p.x_m,1e-6f);
    PASS("differential exact arc integration: equal positive counts advance in +X meters");
    Odometry_Update(-100,-100,20);p=read_pose();NEAR(p.x_m,0,1e-6f);CHECK(p.left_count==0&&p.right_count==0);
    PASS("reverse and signed cumulative counts return to the relative origin");
    Odometry_InitConfig(&c,0);Odometry_Update(-100,100,10);p=read_pose();
    NEAR(p.x_m,0,1e-6f);NEAR(p.y_m,0,1e-6f);NEAR(p.theta_rad,0.31415927f,1e-6f);
    Odometry_Update(100,-100,20);NEAR(read_pose().theta_rad,0,1e-6f);
    PASS("opposite encoder signs rotate CCW/CW without translating");
    Odometry_InitConfig(&c,0);for(unsigned i=1;i<=200U;++i)Odometry_Update(0,20,i*10U);
    p=read_pose();NEAR(p.x_m,0,0.00002f);NEAR(p.y_m,0,0.00002f);NEAR(p.theta_rad,0,0.00002f);
    PASS("200 exact-arc steps close a synthetic circle with bounded floating-point error");
    Odometry_InitConfig(&c,0);for(unsigned i=1;i<=1000U;++i)Odometry_Update(10,10,i*10U);
    p=read_pose();NEAR(p.x_m,3.14159265f,0.00005f);CHECK(s_odom_count==ODOM_HISTORY_LENGTH);
    PASS("long straight accumulation and fixed 64-entry history remain bounded");
    CHECK(!Odometry_PoseAt(0,&x,&y,&a)&&!Odometry_PoseAt(10001,&x,&y,&a));
    CHECK(Odometry_PoseAt(9995,&x,&y,&a));NEAR(x,3.1400219f,0.00006f);
    PASS("interpolate at acquisition tick; reject expired history and future pose without extrapolation");
    Odometry_InitConfig(&c,UINT32_MAX-4U);Odometry_Update(100,100,5);
    CHECK(read_pose().state==2U&&Odometry_PoseAt(0,&x,&y,&a));NEAR(x,0.015707964f,1e-6f);
    PASS("tick zero and interpolation across FreeRTOS unsigned wrap are legitimate");
    Odometry_InitConfig(&c,0);p=read_pose();p.theta_rad=3.13f;Odom_Publish(&p);Odometry_Update(-10,10,10);
    CHECK(Odometry_PoseAt(5,&x,&y,&a));CHECK(fabsf(a)>3.0f);
    PASS("heading interpolation takes the short angular arc across plus/minus pi");
    Odometry_InitConfig(&c,0);Odometry_Update(0,0,10);Odometry_Observe(0,0.5f,0,true);Odometry_GetRay(0,&ray);
    CHECK(ray.state==3U);NEAR(ray.x_m,0.1f,1e-6f);NEAR(ray.yaw_rad,0,1e-6f);
    Odometry_Observe(1,0.5f,0,true);Odometry_GetRay(1,&ray);CHECK(ray.state==3U);
    NEAR(ray.x_m,-0.1f,1e-6f);NEAR(fabsf(ray.yaw_rad),3.141592654f,1e-6f);
    PASS("front/rear use independent measured extrinsics and acquisition pose, not current pose");
    Odometry_InitConfig(&c,0);s_odom.theta_rad=1.570796327f;Odometry_Update(0,0,10);
    Odometry_Observe(0,1,10,true);Odometry_GetRay(0,&ray);NEAR(ray.x_m,0,1e-6f);NEAR(ray.y_m,0.1f,1e-6f);
    PASS("sensor offset rotates into world frame at robot orientation");
    Odometry_Observe(0,0,10,false);Odometry_GetRay(0,&ray);CHECK(ray.state==1U&&ray.sequence==2U);
    Odometry_Observe(0,NAN,10,true);Odometry_GetRay(0,&ray);CHECK(ray.state==1U);
    Odometry_Observe(0,0.5f,11,true);Odometry_GetRay(0,&ray);CHECK(ray.state==2U);
    Odometry_Observe(0,0.5f,10,true);Odometry_GetRay(0,&ray);CHECK(ray.state==3U);
    PASS("failed, nonfinite and unassociated results never project; later valid capture recovers independently");
    c.sensors_confirmed=false;Odometry_InitConfig(&c,0);Odometry_Update(0,0,10);
    Odometry_Observe(0,1,0,true);Odometry_GetRay(0,&ray);
    CHECK(ray.state==3U&&read_pose().state==2U&&!read_pose().calibrated&&read_pose().calibration==ODOM_GEOMETRY_PROVISIONAL);
    PASS("complete unconfirmed mounting permits only provisional projection, never physical calibration");
    for(unsigned i=0;i<5U;++i){c=config();
        if(i==0)c.left_cpr=0;
        if(i==1)c.track_m=NAN;
        if(i==2)c.right_radius_m=INFINITY;
        if(i==3)c.confirmed=false;
        if(i==4)c.track_m=-0.2f;
        Odometry_InitConfig(&c,0);CHECK(!read_pose().calibrated);}
    PASS("missing, nonfinite, negative or unconfirmed geometry is explicitly uncalibrated");
    c=config();Odometry_InitConfig(&c,0);Odometry_Update(0,0,50);CHECK(read_pose().state==2U);
    Odometry_InitConfig(&c,0);Odometry_Update(0,0,51);CHECK(read_pose().state==3U);
    Odometry_Update(1,1,61);CHECK(read_pose().state==3U&&!Odometry_PoseAt(61,&x,&y,&a));
    Odometry_InitConfig(&c,0);Odometry_Update(0,0,0);CHECK(read_pose().state==3U);
    PASS("missed sampling interval and ambiguous same-tick increments latch invalid until explicit restart");
    Odometry_InitConfig(&c,0);s_odom.left_count=INT64_MAX;Odometry_Update(1,0,10);
    CHECK(read_pose().state==3U&&read_pose().left_count==INT64_MAX);
    Odometry_InitConfig(&c,0);s_odom.right_count=INT64_MIN;Odometry_Update(0,-1,10);
    CHECK(read_pose().state==3U&&read_pose().right_count==INT64_MIN);
    PASS("signed 64-bit accumulation never wraps at either numerical limit");
    Odometry_InitConfig(&c,0);Odometry_Update(INT32_MAX,INT32_MIN,10);CHECK(read_pose().state==3U);
    Odometry_InitConfig(&c,0);s_odom.x_m=100.0f;Odometry_Update(10,10,10);CHECK(read_pose().state==3U);
    PASS("unreasonable step and world-coordinate numerical bounds invalidate instead of drawing garbage");
    TIM_HandleTypeDef l={.Init={UINT16_MAX},.counter=65534},r={.Init={UINT32_MAX},.counter=1};
    EncoderHandle_t le,re;Encoder_Init(&le,&l);Encoder_Init(&re,&r);CHECK(Encoder_Start(&le)==HAL_OK);
    l.counter=2;r.counter=UINT32_MAX-2U;int32_t dl=Encoder_ReadDelta(&le),dr=Encoder_ReadDelta(&re);
    CHECK(dl==4&&dr==-4);Odometry_InitConfig(&c,0);Odometry_Update(dl,dr,10);
    CHECK(read_pose().left_count==4&&read_pose().right_count==-4&&le.last_count==2U);
    CHECK(Encoder_ReadDelta(&le)==0&&Encoder_ReadDelta(&re)==0&&Encoder_Stop(&re)==HAL_OK);
    PASS("real TIM1 16-bit and TIM2 32-bit wrap/sign deltas feed one observer without rereading counts");
    char line[160];uint32_t q;unsigned part;
    CHECK(Mapping_Query("MAPPING Q=4294967295 S=2",&q,&part)&&q==UINT32_MAX&&part==2U);
    const char *bad[]={"MAPPING Q=0 S=0","MAPPING Q=-1 S=0","MAPPING Q=4294967296 S=0","MAPPING Q=1 S=3","MAPPING Q=1 S=0 extra","MAPPING Q=1"};
    for(unsigned i=0;i<sizeof bad/sizeof bad[0];++i)CHECK(!Mapping_Query(bad[i],&q,&part));
    PASS("mapping request parser rejects overflow, suffixes, invalid parts and zero query");
    Odometry_InitConfig(&c,0);Odometry_Update(100,100,10);int n=Mapping_Format(line,sizeof line,1,0,10);
    CHECK(n>0&&strstr(line,"POSE,2,1,1,10,2,2,100,100,31,0,0,31,0,0\n"));
    Odometry_Observe(0,0.5f,0,true);n=Mapping_Format(line,sizeof line,2,1,10);
    CHECK(n>0&&strstr(line,"RAY,1,2,1,0,0,1,3,10,500,100,0,0,250\n"));
    CHECK(Mapping_Format(line,8,3,0,10)<0&&Mapping_Format(line,sizeof line,0,0,10)<0);
    PASS("POSE/RAY preserve source tick, original pose, query/boot epoch and bounded integer units");
    s_odom.left_count=INT64_MIN;s_odom.right_count=INT64_MAX;
    n=Mapping_Format(line,sizeof line,UINT32_MAX,0,UINT32_MAX);
    CHECK(n>0&&n<160&&strstr(line,"-9223372036854775808,9223372036854775807,"));
    Odometry_Init(0);n=Mapping_Format(line,sizeof line,4,0,0);CHECK(n>0&&strstr(line,",0,1,0,0,0,-,-,-,-,-,0\n"));
    PASS("full signed counts do not depend on newlib-nano long-long printf; absent metrics are dashes");

    Odometry_Init(0);OdometryConfig nominal=s_odom_config;
    NEAR(ODOM_CHASSIS_LENGTH_M,0.170f,1e-7f);NEAR(ODOM_CHASSIS_WIDTH_M,0.170f,1e-7f);
    NEAR(nominal.left_radius_m,0.01325f,1e-7f);NEAR(nominal.right_radius_m,0.01325f,1e-7f);
    NEAR(nominal.track_m,0.150f,1e-7f);NEAR(nominal.sensor_x[0],0.085f,1e-7f);NEAR(nominal.sensor_x[1],-0.085f,1e-7f);
    NEAR(nominal.sensor_y[0],0,1e-7f);NEAR(nominal.sensor_y[1],0,1e-7f);
    NEAR(nominal.sensor_yaw[0],0,1e-7f);NEAR(nominal.sensor_yaw[1],ODOM_PI,1e-7f);
    CHECK(nominal.left_cpr==0&&nominal.right_cpr==0&&!nominal.confirmed&&!nominal.sensors_confirmed);
    CHECK(read_pose().calibration==ODOM_GEOMETRY_INCOMPLETE);
    PASS("W7.4 production geometry is 170-mm chassis, 26.5-mm wheels, 150-mm base and +/-85-mm sensor offsets; no invented CPR");
    /* CPR conocidos SOLO sinteticos: se verifica escala independiente por rueda. */
    nominal.left_cpr=1000;nominal.right_cpr=2000;
    Odometry_InitConfig(&nominal,0);Odometry_Update(500,1000,10);p=read_pose();
    CHECK(p.state==2U&&p.calibration==ODOM_GEOMETRY_PROVISIONAL&&!p.calibrated);
    NEAR(p.x_m,ODOM_PI*0.0265f*0.5f,1e-7f);NEAR(p.y_m,0,1e-7f);NEAR(p.theta_rad,0,1e-7f);
    PASS("W7.4 known separate CPRs enable nominal-distance conversion labelled provisional, not calibrated");
    Odometry_Observe(0,0.5f,0,true);Odometry_GetRay(0,&ray);
    CHECK(ray.state==3U);NEAR(ray.x_m,0.085f,1e-7f);NEAR(ray.yaw_rad,0,1e-7f);
    Odometry_Observe(1,0.5f,0,true);Odometry_GetRay(1,&ray);
    CHECK(ray.state==3U);NEAR(ray.x_m,-0.085f,1e-7f);NEAR(fabsf(ray.yaw_rad),ODOM_PI,1e-7f);
    CHECK(Mapping_Format(line,sizeof line,8,0,10)>0&&strstr(line,",2,1,500,1000,42,0,0,42,0,0\n"));
    PASS("W7.4 provisional front/rear extrinsics project from acquisition pose and POSE v2 carries class 1");
    for(unsigned mask=0;mask<4U;++mask) {
        nominal.confirmed=(mask&1U)!=0U;nominal.sensors_confirmed=(mask&2U)!=0U;
        Odometry_InitConfig(&nominal,0);p=read_pose();
        CHECK(p.state==2U&&p.calibrated==(mask==3U));
        CHECK(p.calibration==(mask==3U?ODOM_GEOMETRY_CALIBRATED:ODOM_GEOMETRY_PROVISIONAL));
    }
    PASS("W7.4 physical calibration requires both confirmations and never follows automatically from complete numbers");
    for(unsigned which=0;which<6U;++which) {
        c=nominal;
        if(which==0)c.left_cpr=0;
        if(which==1)c.right_cpr=0;
        if(which==2)c.left_cpr=-1;
        if(which==3)c.right_cpr=NAN;
        if(which==4)c.left_cpr=INFINITY;
        if(which==5)c.right_cpr=-1;
        Odometry_InitConfig(&c,0);Odometry_Update(10,20,10);p=read_pose();
        CHECK(p.state==1U&&p.calibration==ODOM_GEOMETRY_INCOMPLETE&&!p.calibrated);
        CHECK(!Odometry_PoseAt(0,&x,&y,&a));Odometry_Observe(0,0.5f,0,true);Odometry_GetRay(0,&ray);CHECK(ray.state==2U);
    }
    PASS("W7.4 absent, negative or nonfinite CPR on either motor blocks metric pose and occupancy even with confirmation flags set");
    c=nominal;c.beam_half_angle=0;Odometry_InitConfig(&c,0);CHECK(read_pose().calibration==ODOM_GEOMETRY_INCOMPLETE);
    c=nominal;c.sensor_x[1]=NAN;Odometry_InitConfig(&c,0);CHECK(read_pose().calibration==ODOM_GEOMETRY_INCOMPLETE);
    c=nominal;c.track_m=0;Odometry_InitConfig(&c,0);CHECK(read_pose().calibration==ODOM_GEOMETRY_INCOMPLETE);
    PASS("W7.4 incomplete or incoherent indispensable mounting, base or uncertainty parameters cannot be promoted to calibrated");
    printf("RESULT %u W7 odometry/protocol groups; synthetic calibration only; no hardware validation\n",cases);return 0;
}
