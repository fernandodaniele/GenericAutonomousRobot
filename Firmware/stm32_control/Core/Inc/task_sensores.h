#ifndef TASK_SENSORES_H
#define TASK_SENSORES_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "drv_TCRT5000.h"

typedef struct
{
    TCRT_Lecturas_t ir;           // Estado de los cuatro TCRT5000
    uint8_t hay_caida;           // 1 si alguno no detecta piso
    float distancia_m;           // Ultima distancia ultrasonica valida, en metros
    uint8_t ultrasonido_valido;  // 1 si la ultima medicion fue valida
    TickType_t timestamp;        // Tick de la ultima publicacion

    // Campos nuevos: pendientes de actualizacion por TaskSensores.
    // Disparo de la ultima captura frontal valida, en ticks FreeRTOS.
    TickType_t timestamp_ultrasonido_frontal;
    float distancia_trasera_m;   // Distancia trasera en metros; usar solo si es valida
    uint8_t ultrasonido_trasero_valido; // 1 si la ultima medicion trasera fue valida
    // Disparo de la ultima captura trasera valida, en ticks FreeRTOS.
    TickType_t timestamp_ultrasonido_trasero;
} SensoresDatos_t;

void TaskSensores_Create(void);

// Devuelve pdTRUE si pudo copiar el ultimo estado; uso solo desde tareas.
BaseType_t TaskSensores_GetData(SensoresDatos_t *out);

#endif /* TASK_SENSORES_H */
