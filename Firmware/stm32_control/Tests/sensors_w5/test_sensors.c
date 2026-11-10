/* Actual acquisition loop with deterministic RTOS, GPIO and ultrasound APIs. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include "../../Core/Src/task_sensores.c"
#include "../../Core/Src/mid_odometry.c"
static TickType_t tick, origin;
static unsigned writes, limit, ir_reads, triggers[2], modes[2], cases;
static TickType_t trigger_tick[2], trigger_history[32];
static unsigned trigger_sensors[32], total_triggers;
static SensoresDatos_t history[64], latest;
static int queue_has_data;
static jmp_buf stop;
#define CHECK(v) do { if(!(v)) { fprintf(stderr,"FAIL %u: %s\n",(unsigned)__LINE__,#v);exit(1); } } while(0)
#define PASS(s) printf("PASS %02u %s\n",++cases,s)
static unsigned observer_lock;
void test_enter(void) { ++observer_lock; }
void test_exit(void) { CHECK(observer_lock); --observer_lock; }
void Error_Handler(void) { CHECK(0); }
TickType_t xTaskGetTickCount(void) { return tick; }
BaseType_t xTaskCreate(void (*fn)(void *),const char *n,unsigned stack,void *arg,UBaseType_t prio,TaskHandle_t *h) {
    CHECK(fn==vTaskSensores && !strcmp(n,"Sensores") && stack==320 && !arg && prio==4 && !h);return pdPASS;
}
QueueHandle_t xQueueCreate(UBaseType_t n,UBaseType_t size) { CHECK(n==1 && size==sizeof(SensoresDatos_t));return (void *)1; }
BaseType_t xQueueOverwrite(QueueHandle_t q,const void *v) { CHECK(q && writes<64);latest=*(const SensoresDatos_t *)v;history[writes++]=latest;queue_has_data=1;return pdPASS; }
BaseType_t xQueuePeek(QueueHandle_t q,void *v,TickType_t wait) { CHECK(q && !wait);if(!queue_has_data)return pdFALSE;*(SensoresDatos_t *)v=latest;return pdTRUE; }
void vTaskDelayUntil(TickType_t *last,TickType_t step) { CHECK(step==10);if(writes==limit)longjmp(stop,1);*last+=step;tick=*last; }
void TCRT_Init(void) { }
TCRT_Lecturas_t TCRT_LeerTodos(void) { unsigned mask=(ir_reads++)%16;return (TCRT_Lecturas_t){(mask&1)!=0,(mask&2)!=0,(mask&4)!=0,(mask&8)!=0}; }
static HAL_StatusTypeDef trigger(unsigned s) {
    CHECK(total_triggers<32);trigger_history[total_triggers]=tick;trigger_sensors[total_triggers++]=s;
    ++triggers[s];trigger_tick[s]=tick;return modes[s]==2 && triggers[s]==1 ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HCSR04_Trigger(void) { return trigger(0); }
HAL_StatusTypeDef HCSR04_Rear_Trigger(void) { return trigger(1); }
uint8_t HCSR04_IsReady(void) { return !(modes[0] && triggers[0]==1) && (TickType_t)(tick-trigger_tick[0])>=10; }
uint8_t HCSR04_Rear_IsReady(void) { return !(modes[1] && triggers[1]==1) && (TickType_t)(tick-trigger_tick[1])>=10; }
float HCSR04_GetDistance(void) { return 0.345f; }
float HCSR04_Rear_GetDistance(void) { return 0.33f; }
HAL_StatusTypeDef HCSR04_Cancel(void) { return HAL_OK; }
HAL_StatusTypeDef HCSR04_Rear_Cancel(void) { return HAL_OK; }
static void run(unsigned count,TickType_t start,unsigned front,unsigned rear) {
    tick=origin=start;writes=ir_reads=total_triggers=0;triggers[0]=triggers[1]=0;modes[0]=front;modes[1]=rear;
    limit=count;queue_has_data=0;memset(history,0,sizeof history);sensores_queue=NULL;TaskSensores_Create();
    Odometry_Init(start);
    SensoresDatos_t copy;CHECK(TaskSensores_GetData(&copy)==pdFALSE && TaskSensores_GetData(NULL)==pdFALSE);
    if(!setjmp(stop))vTaskSensores(NULL);
    CHECK(TaskSensores_GetData(&copy)==pdTRUE && !memcmp(&copy,&latest,sizeof copy));
}
int main(void) {
    run(20,0,0,0);
    CHECK(!history[0].resultado_frontal_disponible && !history[0].resultado_trasero_disponible && history[0].timestamp_ir==0);
    CHECK(history[1].resultado_frontal_disponible && history[1].ultrasonido_valido && history[1].timestamp_ultrasonido_frontal==0 && history[1].timestamp_resultado_frontal==0);
    CHECK(history[8].resultado_trasero_disponible && history[8].timestamp_resultado_trasero==70 && history[8].ultrasonido_trasero_valido);
    PASS("first real loop snapshots distinguish no result from valid acquisition at tick zero");
    CHECK(ir_reads==10 && total_triggers==3);
    for(unsigned i=1;i<total_triggers;++i)CHECK(trigger_sensors[i]!=trigger_sensors[i-1] && (TickType_t)(trigger_history[i]-trigger_history[i-1])>=70);
    for(unsigned i=0;i<20;++i)CHECK(history[i].timestamp==i*10 && history[i].timestamp_ir==(i/2)*20);
    PASS("unchanged 10-ms publication, 20-ms IR, alternating 70-ms triggers and 140-ms per sensor");
    run(20,0,1,0);
    CHECK(!history[4].resultado_frontal_disponible && history[5].resultado_frontal_disponible && !history[5].ultrasonido_valido && history[5].timestamp_resultado_frontal==0);
    CHECK(history[8].ultrasonido_trasero_valido && !history[8].ultrasonido_valido);
    CHECK(history[15].ultrasonido_valido && history[15].timestamp_ultrasonido_frontal==140 && history[15].timestamp_resultado_frontal==140);
    PASS("50-ms front timeout is failed result; rear valid independently; next front capture recovers");
    run(20,0,0,1);CHECK(history[12].resultado_trasero_disponible && !history[12].ultrasonido_trasero_valido && history[12].ultrasonido_valido);
    CHECK(history[12].timestamp_resultado_trasero==70 && history[12].timestamp_ultrasonido_trasero==0);
    PASS("rear timeout preserves its failed-attempt tick without changing front validity or accepted timestamp");
    run(18,0,2,0);CHECK(history[0].resultado_frontal_disponible && !history[0].ultrasonido_valido && history[15].ultrasonido_valido);
    PASS("failed DMA arm represented as failed acquisition, never valid zero distance");
    run(20,UINT32_MAX-39U,0,0);
    CHECK(history[1].timestamp_resultado_frontal==origin && history[8].timestamp_resultado_trasero==(TickType_t)(origin+70U));
    for(unsigned i=1;i<total_triggers;++i)CHECK((TickType_t)(trigger_history[i]-trigger_history[i-1])>=70);
    PASS("metadata and acquisition scheduling retain unsigned tick wrap behavior");
    run(32,0,0,0);CHECK(ir_reads==16);
    for(unsigned i=0;i<16;++i) { SensoresDatos_t *s=&history[i*2];CHECK(s->ir.del_izq==((i&1)!=0) && s->ir.del_der==((i&2)!=0) && s->ir.tras_izq==((i&4)!=0) && s->ir.tras_der==((i&8)!=0));CHECK(s->hay_caida==(i!=15)); }
    PASS("all four GPIO-derived states and aggregate cliff unchanged across all 16 masks");
    OdometryRay front,rear;Odometry_GetRay(0,&front);Odometry_GetRay(1,&rear);
    CHECK(front.state==2U&&rear.state==2U&&front.sequence==3U&&rear.sequence==2U);
    CHECK(front.tick==280U&&rear.tick==210U);
    CHECK(fabsf(front.distance_m-0.345f)<1e-6f&&fabsf(rear.distance_m-0.33f)<1e-6f);
    PASS("W7 real acquisition observer preserves independent original trigger ticks without changing the 70-ms schedule");
    printf("RESULT %u real acquisition-loop groups passed; no hardware validation\n",cases);return 0;
}
