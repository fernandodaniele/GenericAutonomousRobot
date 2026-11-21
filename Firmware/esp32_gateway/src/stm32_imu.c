#include "stm32_imu.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stm32_status.h"
#include "web_commands.h"
#include "esp_random.h"

typedef struct {
    bool received, has_sample;
    uint32_t state, hal_status, error, sample_age;
    int16_t raw[6];
    int64_t received_us;
    bool cal_known;
    uint32_t cal_state, cal_reason, cal_samples, cal_request, cal_age;
    int32_t cal_sum[3];
} ImuSnapshot;
static portMUX_TYPE s_imu_lock = portMUX_INITIALIZER_UNLOCKED;
static ImuSnapshot s_imu;
static char s_imu_line[192];
static size_t s_imu_length;
static bool s_imu_discard;
static uint32_t s_imu_counter, s_imu_pending;
static int64_t s_imu_sent, s_imu_poll;
enum { CAL_REQ_IDLE, CAL_REQ_QUEUED, CAL_REQ_SENT, CAL_REQ_ACCEPTED,
       CAL_REQ_REJECTED, CAL_REQ_TX_ERROR, CAL_REQ_EXPIRED };
static struct { uint32_t number; unsigned phase; int64_t queued_us, sent_us; } s_imu_cal_request;
static char s_imu_cal_ids[8][33];
static unsigned s_imu_cal_next;
static uint32_t s_imu_cal_counter;

static bool ImuCalibrationAllowed(void) {
    Stm32StatusSnapshot status; Stm32Status_GetSnapshot(&status);
    ImuSnapshot s;
    taskENTER_CRITICAL(&s_imu_lock); s=s_imu; taskEXIT_CRITICAL(&s_imu_lock);
    int64_t now=esp_timer_get_time();
    return status.valid && status.connected && status.age_ms<2000U &&
        (status.state==0U || status.state==3U) && WebCommands_CanPollSensors() &&
        s.received && s.cal_known && s.state==2U && s.cal_state!=1U &&
        now>=s.received_us && now-s.received_us<IMU_LINK_MS*1000 &&
        (uint64_t)s.sample_age+(uint64_t)(now-s.received_us)/1000U<=IMU_SAMPLE_MAX_AGE_MS;
}
int Stm32Imu_RequestCalibration(const char *id) {
    if(!id || strlen(id)!=32U)return 3;
    for(unsigned i=0;i<32U;++i)if(!((id[i]>='0'&&id[i]<='9')||(id[i]>='a'&&id[i]<='f')))return 3;
    if(!ImuCalibrationAllowed())return 3;
    int64_t now=esp_timer_get_time();
    uint32_t seed;esp_fill_random(&seed,sizeof(seed));
    taskENTER_CRITICAL(&s_imu_lock);
    for(unsigned i=0;i<8U;++i)if(!strcmp(s_imu_cal_ids[i],id)){taskEXIT_CRITICAL(&s_imu_lock);return 1;}
    if(s_imu_cal_request.phase==CAL_REQ_SENT && now-s_imu_cal_request.sent_us>=500000)s_imu_cal_request.phase=CAL_REQ_EXPIRED;
    if(s_imu_cal_request.phase==CAL_REQ_QUEUED || s_imu_cal_request.phase==CAL_REQ_SENT){taskEXIT_CRITICAL(&s_imu_lock);return 2;}
    if(!s_imu_cal_counter)s_imu_cal_counter=seed;
    if(++s_imu_cal_counter==0U)++s_imu_cal_counter;
    s_imu_cal_request.number=s_imu_cal_counter; s_imu_cal_request.phase=CAL_REQ_QUEUED; s_imu_cal_request.queued_us=now;
    memcpy(s_imu_cal_ids[s_imu_cal_next],id,33U);s_imu_cal_next=(s_imu_cal_next+1U)%8U;
    taskEXIT_CRITICAL(&s_imu_lock);
    return 0;
}
bool Stm32Imu_CalibrationPending(void) {
    taskENTER_CRITICAL(&s_imu_lock); bool pending=s_imu_cal_request.phase==CAL_REQ_QUEUED;taskEXIT_CRITICAL(&s_imu_lock);return pending;
}
bool Stm32Imu_ClaimCalibration(uint32_t *request) {
    bool allowed=ImuCalibrationAllowed();int64_t now=esp_timer_get_time();
    taskENTER_CRITICAL(&s_imu_lock);
    bool queued=s_imu_cal_request.phase==CAL_REQ_QUEUED;
    if(queued && (!allowed || now-s_imu_cal_request.queued_us>=500000))
        s_imu_cal_request.phase=allowed ? CAL_REQ_EXPIRED : CAL_REQ_REJECTED;
    else if(queued) { s_imu_cal_request.phase=CAL_REQ_SENT;s_imu_cal_request.sent_us=now;*request=s_imu_cal_request.number; }
    bool claimed=queued && allowed && s_imu_cal_request.phase==CAL_REQ_SENT;
    taskEXIT_CRITICAL(&s_imu_lock);
    return claimed;
}
void Stm32Imu_CalibrationTxFailed(void) {
    taskENTER_CRITICAL(&s_imu_lock);s_imu_cal_request.phase=CAL_REQ_TX_ERROR;taskEXIT_CRITICAL(&s_imu_lock);
}

bool Stm32Imu_PollDue(void) { return esp_timer_get_time() - s_imu_poll >= IMU_POLL_MS * 1000; }
void Stm32Imu_DeferPoll(void) { s_imu_poll = esp_timer_get_time(); }
uint32_t Stm32Imu_BeginQuery(void) {
    if (++s_imu_counter == 0U) ++s_imu_counter;
    s_imu_pending = s_imu_counter;
    s_imu_sent = s_imu_poll = esp_timer_get_time();
    return s_imu_pending;
}
void Stm32Imu_QueryFailed(void) { s_imu_pending = 0U; }

static bool ImuField(const char **cursor, int64_t *value, bool *present, bool last) {
    const char *p = *cursor;
    bool negative = *p == '-';
    if (negative) ++p;
    *present = !(negative && (*p == ',' || *p == '\0'));
    uint32_t number = 0U;
    if (*present) {
        if (*p < '0' || *p > '9') return false;
        do {
            uint32_t digit = (uint32_t)(*p - '0');
            if (number > (UINT32_MAX - digit) / 10U) return false;
            number = number * 10U + digit;
            ++p;
        } while (*p >= '0' && *p <= '9');
        if (negative && number == 0U) return false;
    }
    *value = negative ? -(int64_t)number : (int64_t)number;
    if (*p != (last ? '\0' : ',')) return false;
    *cursor = last ? p : p + 1;
    return true;
}

static void ImuAccept(void) {
    if(strncmp(s_imu_line,"IMUCAL,",7U)==0) {
        const char *ack=s_imu_line+7; int64_t v[3];bool has[3];
        for(unsigned i=0;i<3U;++i)if(!ImuField(&ack,&v[i],&has[i],i==2U)||!has[i])return;
        if(v[0]!=1 || v[1]<=0 || v[2]<0 || v[2]>1)return;
        int64_t now=esp_timer_get_time();
        taskENTER_CRITICAL(&s_imu_lock);
        if(s_imu_cal_request.phase==CAL_REQ_SENT && s_imu_cal_request.number==(uint32_t)v[1] &&
           now-s_imu_cal_request.sent_us<500000)s_imu_cal_request.phase=v[2]?CAL_REQ_ACCEPTED:CAL_REQ_REJECTED;
        taskEXIT_CRITICAL(&s_imu_lock);return;
    }
    if (strncmp(s_imu_line, "IMU,", 4U) != 0) return;
    const char *p = s_imu_line + 4;
    int64_t v[20]={0}; bool has[20]={0};
    if(!ImuField(&p,&v[0],&has[0],false)||!has[0]||(v[0]!=1 && v[0]!=2))return;
    unsigned fields=v[0]==2 ? 20U : 12U;
    for (unsigned i = 1; i < fields; ++i)
        if (!ImuField(&p, &v[i], &has[i], i == fields-1U)) return;
    for (unsigned i = 0; i < 5U; ++i) if (!has[i] || v[i] < 0) return;
    if (v[1] == 0 || v[2] > 4 || v[3] > 3 || (has[5] && v[5] < 0)) return;
    bool measurement = v[2] == 2;
    if ((v[2] <= 2 && (v[3] || v[4])) || (v[2] >= 3 && v[3] == 0)) return;
    if (measurement && !has[5]) return;
    for (unsigned i = 6; i < 12U; ++i) {
        if (has[i] != measurement) return;
        if (has[i] && (v[i] < INT16_MIN || v[i] > INT16_MAX)) return;
    }
    if(v[0]==2) {
        for(unsigned i=12;i<=15U;++i)if(!has[i]||v[i]<0)return;
        if(v[12]>3 || v[13]>5 || v[14]>250)return;
        if((v[12]!=3 && v[13]!=0)||(v[12]==3 && v[13]==0))return;
        if((v[12]==0 && (v[14]||v[15]))||(v[12]!=0 && !v[15]) ||
           (v[12]==1 && v[14]>=250))return;
        bool calibrated=v[12]==2;
        if(calibrated && (v[14]!=250 || !measurement))return;
        for(unsigned i=16;i<20U;++i) {
            if(has[i]!=calibrated)return;
            if(has[i] && (i==16 ? v[i]<0 : v[i]<-8192000LL || v[i]>8191750LL))return;
        }
    }
    int64_t now = esp_timer_get_time();
    if (s_imu_pending != (uint32_t)v[1] || now < s_imu_sent ||
        now - s_imu_sent >= IMU_REPLY_MS * 1000) return;
    uint32_t rtt = (uint32_t)((now - s_imu_sent + 999) / 1000);
    ImuSnapshot next = { .received = true, .has_sample = has[5], .state = (uint32_t)v[2],
        .hal_status = (uint32_t)v[3], .error = (uint32_t)v[4], .sample_age = (uint32_t)v[5], .received_us = now };
    if (next.sample_age > UINT32_MAX - rtt) next.sample_age = UINT32_MAX;
    else next.sample_age += rtt; /* Conservative transport age; never mix board clocks. */
    for (unsigned i = 0; i < 6U; ++i) if (measurement) next.raw[i] = (int16_t)v[i+6];
    if(v[0]==2) {
        next.cal_known=true; next.cal_state=(uint32_t)v[12];next.cal_reason=(uint32_t)v[13];next.cal_samples=(uint32_t)v[14];next.cal_request=(uint32_t)v[15];
        next.cal_age=(uint32_t)v[16];
        if(next.cal_age>UINT32_MAX-rtt)next.cal_age=UINT32_MAX;else next.cal_age+=rtt;
        for(unsigned i=0;i<3U;++i)next.cal_sum[i]=(int32_t)v[17+i];
    }
    s_imu_pending = 0U;
    taskENTER_CRITICAL(&s_imu_lock);
    s_imu = next;
    /* A lost ACK must not contradict a later coherent observation of the
     * same request actually running/completed/failed on STM32. Not success. */
    if(next.cal_known && next.cal_state!=0U && next.cal_request==s_imu_cal_request.number &&
       (s_imu_cal_request.phase==CAL_REQ_SENT || s_imu_cal_request.phase==CAL_REQ_EXPIRED))
        s_imu_cal_request.phase=CAL_REQ_ACCEPTED;
    taskEXIT_CRITICAL(&s_imu_lock);
}

void Stm32Imu_Feed(const uint8_t *bytes, size_t length) {
    if (!bytes) return;
    for (size_t i = 0; i < length; ++i) {
        uint8_t c = bytes[i];
        if (c == '\n') {
            if (!s_imu_discard) {
                if (s_imu_length && s_imu_line[s_imu_length-1] == '\r') --s_imu_length;
                s_imu_line[s_imu_length] = '\0'; ImuAccept();
            }
            s_imu_length = 0U; s_imu_discard = false;
        } else if (!s_imu_discard) {
            if (c == 0U || s_imu_length >= sizeof(s_imu_line)-1U) s_imu_discard = true;
            else s_imu_line[s_imu_length++] = (char)c;
        }
    }
}

int Stm32Imu_Json(char *out, size_t capacity, bool stm32_connected, bool paused) {
    static const char *states[] = { "initializing", "ready", "valid", "init_error", "read_error" };
    ImuSnapshot s;unsigned request_phase; int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_imu_lock);
    s = s_imu;
    if(s_imu_cal_request.phase==CAL_REQ_SENT && now-s_imu_cal_request.sent_us>=500000)s_imu_cal_request.phase=CAL_REQ_EXPIRED;
    request_phase=s_imu_cal_request.phase;
    taskEXIT_CRITICAL(&s_imu_lock);
    uint64_t elapsed = s.received && now >= s.received_us ? (uint64_t)(now-s.received_us)/1000U : UINT32_MAX;
    uint32_t response_age = elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
    uint32_t age = s.sample_age > UINT32_MAX-response_age ? UINT32_MAX : s.sample_age+response_age;
    bool connected = s.received && response_age < IMU_LINK_MS && stm32_connected;
    bool valid = connected && s.state == 2U && s.has_sample && age <= IMU_SAMPLE_MAX_AGE_MS;
    char axes[160] = "null,null,null],\"gyro_dps\":[null,null,null";
    if (valid) {
        int n = snprintf(axes, sizeof(axes), "%.6f,%.6f,%.6f],\"gyro_dps\":[%.6f,%.6f,%.6f",
            s.raw[0]/16384.0, s.raw[1]/16384.0, s.raw[2]/16384.0,
            s.raw[3]/131.0, s.raw[4]/131.0, s.raw[5]/131.0);
        if (n < 0 || (size_t)n >= sizeof(axes)) return -1;
    }
    char response[12] = "null", sample[12] = "null", hal[12] = "null", error[12] = "null";
    if (s.received) {
        (void)snprintf(response, sizeof(response), "%" PRIu32, response_age);
        (void)snprintf(hal, sizeof(hal), "%" PRIu32, s.hal_status);
        (void)snprintf(error, sizeof(error), "%" PRIu32, s.error);
    }
    if (s.has_sample) (void)snprintf(sample, sizeof(sample), "%" PRIu32, age);
    int n = snprintf(out, capacity,
        "{\"received\":%s,\"connected\":%s,\"stm32_connected\":%s,\"paused\":%s,\"valid\":%s,\"stale\":%s,"
        "\"state\":\"%s\",\"hal_status\":%s,\"i2c_error\":%s,\"response_age_ms\":%s,\"sample_age_ms\":%s,"
        "\"max_age_ms\":%u,\"accel_g\":[%s]}",
        s.received ? "true" : "false", connected ? "true" : "false", stm32_connected ? "true" : "false",
        paused ? "true" : "false", valid ? "true" : "false", valid ? "false" : "true",
        s.received ? states[s.state] : "no_data", hal, error, response, sample, IMU_SAMPLE_MAX_AGE_MS, axes);
    if(n<0 || (size_t)n>=capacity)return -1;
    bool calibrated=valid && s.cal_known && s.cal_state==2U;
    char offset[128]="null,null,null",corrected[128]="null,null,null",raw[32]="null,null,null",cal_age[12]="null";
    if(valid)(void)snprintf(raw,sizeof(raw),"%d,%d,%d",(int)s.raw[3],(int)s.raw[4],(int)s.raw[5]);
    if(calibrated) {
        double bias[3];for(unsigned i=0;i<3U;++i)bias[i]=s.cal_sum[i]/250.0;
        (void)snprintf(offset,sizeof(offset),"%.6f,%.6f,%.6f",bias[0]/131.0,bias[1]/131.0,bias[2]/131.0);
        (void)snprintf(corrected,sizeof(corrected),"%.6f,%.6f,%.6f",(s.raw[3]-bias[0])/131.0,(s.raw[4]-bias[1])/131.0,(s.raw[5]-bias[2])/131.0);
    }
    if(s.cal_known && s.cal_state==2U) {
        uint32_t age_cal=s.cal_age>UINT32_MAX-response_age?UINT32_MAX:s.cal_age+response_age;
        (void)snprintf(cal_age,sizeof(cal_age),"%" PRIu32,age_cal);
    }
    static const char *cal_states[]={"pending","running","valid","failed"};
    static const char *reasons[]={"none","motion","i2c","timing","mode","insufficient"};
    static const char *requests[]={"idle","queued","sent","accepted","rejected","tx_error","expired"};
    int extra=snprintf(out+n-1,capacity-(size_t)n+1U,
        ",\"gyro_raw\":[%s],\"gyro_corrected_dps\":[%s],\"calibration\":{\"available\":%s,\"state\":\"%s\",\"valid\":%s,"
        "\"reason\":\"%s\",\"samples\":%" PRIu32 ",\"target_samples\":250,\"request\":%" PRIu32 ",\"age_ms\":%s,\"offset_dps\":[%s],\"request_phase\":\"%s\"}}",
        raw,corrected,s.cal_known?"true":"false",s.cal_known?cal_states[s.cal_state]:"unavailable",calibrated?"true":"false",
        reasons[s.cal_reason],s.cal_samples,s.cal_request,cal_age,offset,requests[request_phase]);
    return extra>=0 && (size_t)extra<capacity-(size_t)n+1U ? n-1+extra : -1;
}
