#ifndef STM32_NAVIGATION_H
#define STM32_NAVIGATION_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define NAVIGATION_JSON_CAPACITY 4096U
#define NAVIGATION_POLL_MS 1000U
#define NAVIGATION_REPLY_MS 200U
#define NAVIGATION_LINK_MS 3000U
typedef struct {
    bool received, reference_available, reverse_entered;
    uint32_t state, elapsed_ms, observations, window_observations, window_ms;
    uint32_t left_span, right_span, reference_tick, entry_tick, source_age;
    int64_t reference_left, reference_right, received_us;
} Stm32NavigationWaitSnapshot;
typedef struct {
    bool received, deltas_available;
    uint32_t profile, session_id, started_tick, state, last_phase, reason, flags;
    uint32_t elapsed_ms, phase_elapsed_ms, advance_um, reverse_um;
    uint32_t front_mm, rear_mm, sensor_flags, snapshot_age, front_age, rear_age;
    uint32_t odometry_error, decision_age, event_age;
    int64_t left_delta, right_delta;
    Stm32NavigationWaitSnapshot stabilization, turn_stabilization;
    bool turn_received;
    uint32_t turn_flags, turn_mdeg, turn_elapsed_ms, turn_target_mdeg, turn_left;
} Stm32NavigationSnapshot;
/* Unico propietario del parser/planificador: app_main. HTTP solo copia. */
void Stm32Navigation_Feed(const uint8_t *bytes, size_t length);
bool Stm32Navigation_PollDue(void);
void Stm32Navigation_DeferPoll(void);
uint32_t Stm32Navigation_BeginQuery(void);
bool Stm32Navigation_WaitQueryDue(void);
bool Stm32Navigation_TurnQueryDue(void);
uint32_t Stm32Navigation_BeginTurnQuery(void);
uint32_t Stm32Navigation_BeginWaitQuery(void);
void Stm32Navigation_QueryFailed(void);
int Stm32Navigation_Json(char *out, size_t capacity, bool stm32_connected, bool paused);
#endif
