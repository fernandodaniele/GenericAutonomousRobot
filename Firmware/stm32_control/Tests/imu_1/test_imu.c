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
static TaskIMUSnapshot_t saved, history[12];
static unsigned writes, stop_after, reads, init_reads, init_fails, read_fail_at, groups, retry_delays;
static uint8_t identity=0x68;
static HAL_StatusTypeDef read_error=HAL_TIMEOUT;
static unsigned write_fail_at, register_writes;
static jmp_buf finish;
static uint8_t raw[14];
static int available;
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
    if(stop_after) { CHECK(writes<12); history[writes++]=saved; if(writes==stop_after) longjmp(finish,1); }
    return pdPASS;
}
BaseType_t xQueuePeek(QueueHandle_t q,void *p,TickType_t wait) { CHECK(q==(void *)1 && wait==0); if(!available)return pdFALSE; *(TaskIMUSnapshot_t *)p=saved; return pdTRUE; }
void vQueueDelete(QueueHandle_t q) { (void)q; }
BaseType_t xTaskCreate(void (*fn)(void *),const char *name,unsigned stack,void *arg,UBaseType_t priority,TaskHandle_t *handle) {
    CHECK(fn==TaskIMU_Run && !strcmp(name,"IMU") && stack==512 && priority==4 && arg==&bus && handle==NULL);
    return pdPASS;
}
void vTaskDelayUntil(TickType_t *last,TickType_t period) { CHECK(period==20); *last+=period; tick=*last; }
void vTaskDelay(TickType_t delay) { CHECK(delay==1000); ++retry_delays; tick+=delay; }
static void axis(unsigned index,int16_t value) { raw[index]=(uint16_t)value>>8; raw[index+1]=(uint8_t)value; }
static void reset(void) {
    imu_data_queue=NULL; available=0; tick=0; memset(&bus,0,sizeof(bus));
    writes=stop_after=reads=init_reads=init_fails=read_fail_at=retry_delays=write_fail_at=register_writes=0;
    identity=0x68; memset(raw,0,sizeof(raw)); axis(4,16384); read_error=HAL_TIMEOUT;
    TaskIMU_Create(&bus);
}
static void run(unsigned events) { writes=0; stop_after=events; if(!setjmp(finish))TaskIMU_Run(&bus); stop_after=0; }
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
    printf("RESULT %u IMU acquisition/driver groups passed; no hardware validation\n",groups);
    return 0;
}
