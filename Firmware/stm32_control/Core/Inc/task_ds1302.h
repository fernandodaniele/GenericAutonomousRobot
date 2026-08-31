#ifndef TASK_DS1302_H
#define TASK_DS1302_H

/**
 * @brief  Crea la tarea de humo del logger + RTC.
 *
 * Emite un `LOG_INFO` de heartbeat una vez por segundo. Como `mid_log` antepone
 * el timestamp del DS1302 a cada línea, sirve para ver de un vistazo que el RTC
 * avanza y que la cadena RTC → mid_log → USB CDC funciona.
 */
void TaskDs1302_Create(void);

#endif /* TASK_DS1302_H */
