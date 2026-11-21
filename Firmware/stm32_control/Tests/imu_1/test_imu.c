/* Real task and MPU6050 driver, substituted RTOS/I2C. No physical claims. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <setjmp.h>
#include "main.h" /* RTOS/HAL fixture guard before real i2c.h includes main.h. */
#include "task_imu.h"
#include "task.h"
#include "queue.h"
#include "../../Core/Src/mpu6050.c"
#include "../../Core/Src/task_imu.c"
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL %u: %s\n",__LINE__,#x); exit(1); } } while(0)
static TickType_t tick;
static I2C_HandleTypeDef bus;
static TaskIMUSnapshot_t saved, history[300];
static unsigned writes, stop_after, reads, init_reads, init_fails, read_fail_at, groups, retry_delays;
static uint8_t identity=0x68;
static HAL_StatusTypeDef read_error=HAL_TIMEOUT;
static unsigned write_fail_at, register_writes;
static jmp_buf finish;
static uint8_t raw[14];
static int available;
static RobotControlState_t test_mode=ROBOT_STATE_SAFETY_STOP;
static unsigned critical_depth, request_after_reads;
static unsigned request_after_recovery;
static bool request_done, recovery_request_done;
RobotControlState_t TaskControlRobot_GetState(void) { return test_mode; }
void test_enter(void) { ++critical_depth; }
void test_exit(void) { CHECK(critical_depth>0); --critical_depth; }
static void pass(const char *s) { printf("PASS %02u %s\n",++groups,s); }
void Error_Handler(void) { CHECK(0); }
TickType_t xTaskGetTickCount(void) { return tick; }
uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *b) { CHECK(b==&bus); return b->ErrorCode; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *b,uint16_t address,uint16_t reg,uint16_t size,uint8_t *out,uint16_t n,uint32_t timeout) {
    CHECK(b==&bus && address==0xD0 && size==I2C_MEMADD_SIZE_8BIT && timeout==100U);
    if (reg==0x75) {
        CHECK(n==1); ++init_reads;
        if(init_reads<=init_fails) { b->ErrorCode=4; return HAL_ERROR; }
        *out=identity; b->ErrorCode=0; return HAL_OK;
    }
    CHECK(reg==0x3B && n==14); ++reads;
    if(reads==read_fail_at) { b->ErrorCode=32; return read_error; }
    memcpy(out,raw,n); b->ErrorCode=0; return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *b,uint16_t address,uint16_t reg,uint16_t size,uint8_t *data,uint16_t n,uint32_t timeout) {
    static const uint8_t regs[]={0x6B,0x19,0x1C,0x1B}, values[]={0,7,0,0};
    CHECK(b==&bus && address==0xD0 && n==1 && size==1 && timeout==100);
    unsigned i=register_writes++%4;
    CHECK(reg==regs[i] && *data==values[i]);
    return register_writes==write_fail_at ? HAL_BUSY : HAL_OK;
}
QueueHandle_t xQueueCreate(UBaseType_t length,UBaseType_t bytes) { CHECK(length==1 && bytes==sizeof(saved)); return (void *)1; }
BaseType_t xQueueOverwrite(QueueHandle_t q,const void *p) {
    CHECK(q==(void *)1); saved=*(const TaskIMUSnapshot_t *)p; available=1;
    CHECK(!critical_depth);
    if(stop_after) { CHECK(writes<300); history[writes++]=saved; if(writes==stop_after) longjmp(finish,1); }
    return pdPASS;
}
BaseType_t xQueuePeek(QueueHandle_t q,void *p,TickType_t wait) { CHECK(q==(void *)1 && wait==0); if(!available)return pdFALSE; *(TaskIMUSnapshot_t *)p=saved; return pdTRUE; }
void vQueueDelete(QueueHandle_t q) { (void)q; }
BaseType_t xTaskCreate(void (*fn)(void *),const char *name,unsigned stack,void *arg,UBaseType_t priority,TaskHandle_t *handle) {
    CHECK(fn==TaskIMU_Run && !strcmp(name,"IMU") && stack==512 && priority==4 && arg==&bus && handle==NULL);
    return pdPASS;
}
void vTaskDelayUntil(TickType_t *last,TickType_t period) {
    CHECK(period==20); *last+=period; tick=*last;
    if(request_after_reads && !request_done && reads==request_after_reads) {
        request_done=true; CHECK(TaskIMU_RequestCalibration(7U,tick));
    }
    if(request_after_recovery && !recovery_request_done && reads==request_after_recovery) {
        recovery_request_done=true;CHECK(TaskIMU_RequestCalibration(8U,tick));
    }
}
void vTaskDelay(TickType_t delay) { CHECK(delay==1000); ++retry_delays; tick+=delay; }
static void axis(unsigned index,int16_t value) { raw[index]=(uint16_t)value>>8; raw[index+1]=(uint8_t)value; }
static void reset(void) {
    imu_data_queue=NULL; available=0; tick=0; memset(&bus,0,sizeof(bus));
    writes=stop_after=reads=init_reads=init_fails=read_fail_at=retry_delays=write_fail_at=register_writes=0;
    identity=0x68; memset(raw,0,sizeof(raw)); axis(4,16384); read_error=HAL_TIMEOUT;
    calibration_request=last_calibration_request=0;calibration_issued=0;critical_depth=0;
    request_after_reads=0;request_done=false;test_mode=ROBOT_STATE_SAFETY_STOP;
    request_after_recovery=0;recovery_request_done=false;
    TaskIMU_Create(&bus);
}
static void run(unsigned events) { writes=0; stop_after=events; if(!setjmp(finish))TaskIMU_Run(&bus); stop_after=0; }
static TaskIMUSnapshot_t stationary(void) {
    return (TaskIMUSnapshot_t){ .state=IMU_VALID,.has_sample=true,
        .data={.Accel_Z_RAW=16384,.Gyro_X_RAW=131,.Gyro_Y_RAW=-262,.Gyro_Z_RAW=66} };
}
static void test_calibration(void) {
    reset(); TaskIMUSnapshot_t s=stationary();
    CHECK(saved.calibration.state==IMU_CAL_PENDING && !TaskIMU_RequestCalibration(1U,tick));
    IMU_CalibrationBegin(&s,1U,0U,0U);
    for(unsigned i=1;i<=249U;++i)IMU_CalibrationObserve(&s,i*20U);
    CHECK(s.calibration.state==IMU_CAL_RUNNING && s.calibration.samples==249U);
    IMU_CalibrationObserve(&s,5000U);
    CHECK(s.calibration.state==IMU_CAL_VALID && s.calibration.samples==250U && s.calibration.completed_tick==5000U);
    CHECK(s.calibration.sum[0]==32750 && s.calibration.sum[1]==-65500 && s.calibration.sum[2]==16500);
    CHECK(s.data.Gyro_X_RAW==131 && s.data.Gyro_Y_RAW==-262);
    pass("IMU-2 exactly 250 actual stationary samples over 5 seconds produce measured signed biases without altering raw values");
    s=stationary(); IMU_CalibrationBegin(&s,2U,0U,0U);
    for(unsigned i=1;i<=250;++i) { s.data.Gyro_X_RAW=(int16_t)(100+i%2);IMU_CalibrationObserve(&s,i*20U); }
    CHECK(s.calibration.state==IMU_CAL_VALID && s.calibration.sum[0]==25125);
    pass("IMU-2 fractional raw-count average is preserved exactly, not rounded to an arbitrary offset");
    for(unsigned kind=0;kind<4;++kind) {
        s=stationary(); IMU_CalibrationBegin(&s,3U,0U,0U);IMU_CalibrationObserve(&s,20U);
        if(kind==0)s.data.Gyro_X_RAW=656;
        if(kind==1)s.data.Gyro_X_RAW=263;
        if(kind==2)s.data.Accel_X_RAW=820;
        if(kind==3)s.data.Accel_Z_RAW=0;
        IMU_CalibrationObserve(&s,40U);
        CHECK(s.calibration.state==IMU_CAL_FAILED && s.calibration.reason==IMU_CAL_MOTION);
        CHECK(s.calibration.sum[0]==0 && s.calibration.sum[1]==0 && s.calibration.sum[2]==0);
        s.data=stationary().data; IMU_CalibrationObserve(&s,60U);CHECK(s.calibration.state==IMU_CAL_FAILED);
    }
    pass("IMU-2 angular motion, gyro variation, acceleration variation and absent gravity fail without automatic restart");
    s=stationary();IMU_CalibrationBegin(&s,4U,0U,0U);
    for(unsigned i=1;i<=250;++i) { s.data.Gyro_X_RAW=i%2?27:-27;IMU_CalibrationObserve(&s,i*20U); }
    CHECK(s.calibration.state==IMU_CAL_FAILED && s.calibration.reason==IMU_CAL_MOTION);
    pass("IMU-2 excessive per-axis variance rejects a window even when peak-to-peak variation passes");
    s=stationary();IMU_CalibrationBegin(&s,5U,0U,0U);
    IMU_CalibrationObserve(&s,20U);IMU_CalibrationObserve(&s,20U);
    CHECK(s.calibration.state==IMU_CAL_FAILED && s.calibration.reason==IMU_CAL_TIMING);
    s=stationary();IMU_CalibrationBegin(&s,6U,0U,0U);
    for(unsigned i=1;i<=250;++i)IMU_CalibrationObserve(&s,i*10U);
    CHECK(s.calibration.state==IMU_CAL_FAILED && s.calibration.reason==IMU_CAL_TIMING);
    s=stationary();IMU_CalibrationBegin(&s,7U,0U,0U);IMU_CalibrationObserve(&s,20U);IMU_CalibrationObserve(&s,61U);
    CHECK(s.calibration.state==IMU_CAL_FAILED && s.calibration.reason==IMU_CAL_TIMING);
    s=stationary();IMU_CalibrationBegin(&s,8U,0U,0U);IMU_CalibrationObserve(&s,6001U);
    CHECK(s.calibration.state==IMU_CAL_FAILED && s.calibration.reason==IMU_CAL_INSUFFICIENT);
    pass("IMU-2 duplicate ticks, catch-up bursts, missing intervals and insufficient samples cannot validate a short window");
    s=stationary();TickType_t start=UINT32_MAX-4999U;IMU_CalibrationBegin(&s,9U,start,start);
    for(unsigned i=1;i<=250;++i)IMU_CalibrationObserve(&s,(TickType_t)(start+i*20U));
    CHECK(s.calibration.state==IMU_CAL_VALID && s.calibration.completed_tick==0U);
    pass("IMU-2 calibration crosses FreeRTOS wrap and accepts legitimate completion tick zero");
    test_mode=ROBOT_STATE_MANUAL;s=stationary();IMU_CalibrationBegin(&s,10U,0U,0U);
    CHECK(s.calibration.state==IMU_CAL_FAILED && s.calibration.reason==IMU_CAL_MODE);
    test_mode=ROBOT_STATE_IDLE;IMU_CalibrationBegin(&s,11U,0U,0U);IMU_CalibrationObserve(&s,20U);
    test_mode=ROBOT_STATE_AUTONOMOUS;IMU_CalibrationObserve(&s,40U);
    CHECK(s.calibration.state==IMU_CAL_FAILED && s.calibration.reason==IMU_CAL_MODE);
    pass("IMU-2 mode checks reject calibration in MANUAL/AUTO without changing robot mode or motor behavior");
    reset();axis(8,131);axis(10,-262);axis(12,66);request_after_reads=1;run(253);
    CHECK(request_done && saved.calibration.state==IMU_CAL_VALID && saved.calibration.request==7U && saved.calibration.samples==250U);
    CHECK(saved.calibration.sum[0]==32750 && saved.sample_tick==5020U && saved.state==IMU_VALID);
    CHECK(!TaskIMU_RequestCalibration(7U,tick));
    pass("IMU-2 real TaskIMU and I2C path honor one explicit request and acquire the entire 250-sample window");
    reset();request_after_reads=1;read_fail_at=4;run(6);
    CHECK(saved.state==IMU_READ_ERROR && saved.calibration.state==IMU_CAL_FAILED && saved.calibration.reason==IMU_CAL_I2C);
    CHECK(saved.calibration.sum[0]==0);
    pass("IMU-2 real I2C failure during calibration immediately discards partial offsets");
    reset();request_after_reads=1;read_fail_at=4;run(8);
    CHECK(saved.state==IMU_VALID && saved.calibration.state==IMU_CAL_FAILED);
    CHECK(TaskIMU_RequestCalibration(8U,tick));CHECK(!TaskIMU_RequestCalibration(9U,tick));
    CHECK(!TaskIMU_RequestCalibration(10U,tick-500U));
    s=stationary();IMU_CalibrationBegin(&s,8U,tick,tick);
    for(unsigned i=1;i<=250;++i)IMU_CalibrationObserve(&s,tick+i*20U);
    CHECK(s.calibration.state==IMU_CAL_VALID);IMU_CalibrationI2CError(&s);
    CHECK(s.calibration.state==IMU_CAL_FAILED && s.calibration.reason==IMU_CAL_I2C);
    pass("IMU-2 recovery restores raw data but requires a fresh explicit request; stale or concurrent requests and reuse after I2C failure are rejected");
    reset();request_after_reads=1;read_fail_at=4;request_after_recovery=5;run(259);
    CHECK(saved.calibration.state==IMU_CAL_VALID && saved.calibration.request==8U && saved.calibration.samples==250U);
    CHECK(recovery_request_done && saved.calibration.completed_tick==6100U && retry_delays==1U);
    pass("IMU-2 real task recovers from I2C failure and accepts a fresh full 250-sample explicit recalibration without restart");
    reset();request_after_reads=1;read_fail_at=252;run(254);
    CHECK(history[252].calibration.state==IMU_CAL_VALID && saved.state==IMU_READ_ERROR && saved.calibration.state==IMU_CAL_FAILED && saved.calibration.reason==IMU_CAL_I2C);
    pass("IMU-2 an I2C fault after successful calibration revokes previously valid offsets on the real task path");
}
int main(void) {
    TaskIMUSnapshot_t s; MPU6050_t d;
    CHECK(!TaskIMU_GetSnapshot(NULL) && !TaskIMU_GetLatest(NULL));
    CHECK(!TaskIMU_GetSnapshot(&s)); pass("no task or null output cannot expose a valid sample");
    reset(); CHECK(TaskIMU_GetSnapshot(&s) && s.state==IMU_INITIALIZING && !s.has_sample && !TaskIMU_GetLatest(&d));
    pass("existing single 512-word priority-4 task publishes initialization without a second acquisition task");
    axis(0,-16384); axis(8,131); axis(10,-131); axis(12,32767); run(3);
    CHECK(history[0].state==IMU_READY && history[1].state==IMU_VALID && history[2].state==IMU_VALID);
    CHECK(saved.data.Accel_X_RAW==-16384 && saved.data.Accel_Z_RAW==16384 && saved.data.Gyro_Y_RAW==-131);
    CHECK(saved.has_sample && saved.sample_tick==40 && TaskIMU_GetLatest(&d));
    CHECK(d.Ax==-1 && d.Gy==-1 && reads==2 && register_writes==4);
    pass("real driver verifies identity and config, reads one coherent 14-byte burst every 20 ms with signed axes");
    reset(); init_fails=1; run(3);
    CHECK(history[0].state==IMU_INIT_ERROR && history[0].hal_status==HAL_ERROR && history[0].i2c_error==4 && !history[0].has_sample);
    CHECK(history[2].state==IMU_VALID && retry_delays==1 && init_reads==2);
    pass("absent unpowered IMU reports init failure, retries at one second and recovers without restarting STM32");
    reset(); identity=0x67; run(1);
    CHECK(saved.state==IMU_INIT_ERROR && saved.hal_status==HAL_ERROR && !TaskIMU_GetLatest(&d));
    pass("wrong WHO_AM_I is an explicit initialization error without invented readings");
    for(unsigned i=1;i<=4;++i) {
        reset(); write_fail_at=i; run(1); CHECK(saved.state==IMU_INIT_ERROR && saved.hal_status==HAL_BUSY && reads==0);
    }
    pass("each failed configuration register write prevents valid acquisition");
    reset(); read_fail_at=2; run(3);
    CHECK(history[1].state==IMU_VALID && saved.state==IMU_READ_ERROR && saved.has_sample && saved.sample_tick==20);
    CHECK(saved.hal_status==HAL_TIMEOUT && saved.i2c_error==32 && !TaskIMU_GetLatest(&d));
    pass("read timeout immediately withdraws validity, preserves original sample tick and reports HAL/I2C error");
    reset(); read_fail_at=2; run(5);
    CHECK(history[2].state==IMU_READ_ERROR && history[3].state==IMU_READY && saved.state==IMU_VALID);
    CHECK(retry_delays==1 && saved.sample_tick==1060 && saved.hal_status==HAL_OK && saved.i2c_error==0);
    pass("read-error recovery is bounded to one-second retries and needs a fresh successful burst to become valid");
    reset(); tick=UINT32_MAX-19U; run(2); CHECK(saved.sample_tick==0U && saved.has_sample && saved.state==IMU_VALID);
    pass("successful acquisition at tick zero after wrap is legitimate");
    reset(); axis(0,INT16_MIN); axis(4,INT16_MIN); run(2);
    CHECK(saved.data.Accel_X_RAW==INT16_MIN && saved.data.Accel_Z_RAW==INT16_MIN && isfinite(saved.data.KalmanAngleX));
    pass("extreme raw acceleration cannot overflow the inherited roll norm before square root");
    test_calibration();
    printf("RESULT %u IMU acquisition/driver/calibration groups passed; no hardware validation\n",groups);
    return 0;
}
