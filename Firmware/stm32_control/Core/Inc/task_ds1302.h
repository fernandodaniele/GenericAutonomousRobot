#ifndef TASK_DS1302_H
#define TASK_DS1302_H

#include "ds1302.h"

/**
 * @brief  Crea la tarea de prueba del RTC DS1302.
 *
 * La tarea lee el DS1302 una vez por segundo y vuelca la fecha/hora por USB CDC
 * con el formato "[Www AAAA-Mmm-DD HH:MM:SS]". Sirve para validar el pinout
 * (PD8/PD9/PD10) y la comunicación bit-bang antes de integrarlo al logger.
 *
 * @param ds1302  Objeto del driver ya inicializado con DS1302_Init().
 */
void TaskDs1302_Create(Ds1302_t *ds1302);

#endif /* TASK_DS1302_H */
