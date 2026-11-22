#ifndef STM32_IMU_H
#define STM32_IMU_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define IMU_POLL_MS 500U
#define IMU_REPLY_MS 200U
#define IMU_LINK_MS 1500U
#define IMU_SAMPLE_MAX_AGE_MS 1500U /* Monitoring only; no control authority. */

/* Only app_main owns parser/query state; HTTP obtains a consistent copy. */
bool Stm32Imu_PollDue(void);
void Stm32Imu_DeferPoll(void);
uint32_t Stm32Imu_BeginQuery(void);
void Stm32Imu_QueryFailed(void);
bool Stm32Imu_FusionPollDue(void);
void Stm32Imu_DeferFusionPoll(void);
uint32_t Stm32Imu_BeginFusionQuery(void);
void Stm32Imu_FusionQueryFailed(void);
void Stm32Imu_Feed(const uint8_t *bytes, size_t length);
int Stm32Imu_Json(char *out, size_t capacity, bool stm32_connected, bool paused);
/* Request only: HTTP never writes UART. 0=queued, 1=replay, 2=busy, 3=unsafe. */
int Stm32Imu_RequestCalibration(const char *id);
bool Stm32Imu_CalibrationPending(void);
bool Stm32Imu_ClaimCalibration(uint32_t *request);
void Stm32Imu_CalibrationTxFailed(void);
#endif
