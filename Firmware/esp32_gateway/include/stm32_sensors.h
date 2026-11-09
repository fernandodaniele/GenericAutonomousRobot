#ifndef STM32_SENSORS_H
#define STM32_SENSORS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SENSORS_POLL_MS 500U
#define SENSORS_REPLY_MS 200U
#define SENSORS_LINK_MS 1500U
/* Vigencia visual compatible con consultas de 500 ms. Independiente de los
 * limites de Safety STM32: no autoriza movimiento ni cambia el enlace. */
#define SENSORS_MONITOR_MAX_AGE_MS 1500U
#define SENSORS_SNAPSHOT_AGE_MS SENSORS_MONITOR_MAX_AGE_MS
#define SENSORS_ULTRASOUND_AGE_MS SENSORS_MONITOR_MAX_AGE_MS
#define SENSORS_IR_AGE_MS SENSORS_MONITOR_MAX_AGE_MS

typedef struct {
    bool valid, connected, available;
    uint32_t query, snapshot_age, front_state, front_mm, front_age;
    uint32_t rear_state, rear_mm, rear_age, ir_mask, ir_age, cliff;
    uint64_t age_ms; /* Tiempo desde recepcion completa; solo reloj ESP32. */
} Stm32SensorsSnapshot;

/* Unico propietario de parser/consultas: app_main, nunca HTTP o ISR. */
void Stm32Sensors_Feed(const uint8_t *bytes, size_t length);
bool Stm32Sensors_PollDue(void);
void Stm32Sensors_DeferPoll(void);
uint32_t Stm32Sensors_BeginQuery(void);
void Stm32Sensors_QueryFailed(void);
void Stm32Sensors_GetSnapshot(Stm32SensorsSnapshot *out);
int Stm32Sensors_Json(char *out, size_t capacity, bool stm32_connected, bool paused);
#endif
