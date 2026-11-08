# Manual de códigos del datalogger

Proyecto: GenericAutonomousRobot. Etapa D1.

Referencia auditada: rama `G1-add/DataloggerEventos`, HEAD `0e2c4b7` (base B6).
Este documento define cómo interpretar los futuros registros de `EVENTOS.CSV`.
No implica que su generación, cola de eventos o persistencia estén implementadas
ni validadas físicamente.

## 1. Formato del registro

Cada fila representa un evento. La cabecera es:

```csv
FECHA_HORA,TICK,CODIGO,DATO
```

| Columna | Significado |
|---|---|
| `FECHA_HORA` | Fecha y hora entregada por el RTC DS1302 cuando el consumidor procesa el evento, con formato `AAAA-MM-DD HH:MM:SS`. |
| `TICK` | Valor de `TickType_t` capturado cuando ocurre el evento, expresado como entero decimal sin signo. |
| `CODIGO` | Identificador numérico decimal de la tabla de eventos de este manual. |
| `DATO` | Información complementaria mediante pares `clave=valor` separados por `\|`, sin significado posicional. Puede estar vacío si no corresponde un dato. |

En las filas CSV se utiliza el carácter literal `|`, por ejemplo:
`anterior=2|nuevo=3|evento=3`. Los valores de estados, máscaras y eventos del
firmware se escriben en decimal. Evitar comas, saltos de línea y los separadores
`|` y `=` dentro de los valores; si un campo CSV contiene comas o comillas,
debe aplicarse el escape CSV con comillas dobles.

### Interpretación temporal

- `FECHA_HORA` corresponde al procesamiento, no necesariamente al instante
  exacto de ocurrencia. Una espera en la cola o en el almacenamiento puede
  separar ambos instantes.
- No se garantiza la exactitud del RTC hasta implementar y validar su futura
  configuración manual. Una fecha aparentemente válida no acredita una hora
  correcta. Los ejemplos de este documento son ilustrativos.
- `TICK` depende de la frecuencia configurada en FreeRTOS. En esta referencia,
  `configTICK_RATE_HZ=1000` y `configUSE_16_BIT_TICKS=0`: se utiliza un contador
  de 32 bits y cada tick nominal equivale a 1 ms. Esta equivalencia debe
  revisarse si cambia la configuración.
- El contador puede desbordarse y vuelve a comenzar después de un reinicio.
  No comparar ingenuamente ticks como si fueran un tiempo absoluto creciente.
  Los intervalos deben interpretarse con resta modular y dentro del rango del
  contador. Dos eventos con el mismo tick pueden ser distintos; ese valor no
  demuestra por sí solo su orden dentro del tick.

## 2. Códigos de eventos

Los códigos de registro son distintos de las máscaras de Safety y de los
valores de los eventos de la FSM.

| Código | Evento | Activación y dato complementario propuesto |
|---|---|---|
| **100** | Arranque del sistema | Una vez por arranque. `estado`, `fallas` y `bloqueo` describen la condición inicial; no certifican que todos los periféricos funcionen. |
| **110** | Cambio efectivo de estado de la FSM | Solo cuando cambia `robot_state`. `anterior`, `nuevo` y `evento` identifican la transición realizada y su causa. |
| **201** | Nueva `SAFETY_FAULT_CLIFF` | Aparece el bit `1` (`1U << 0`) en la máscara enclavada. `bit=1|mascara=N`. Corresponde a caída detectada mediante IR. |
| **202** | Nueva `SAFETY_FAULT_OBSTACLE` | Aparece el bit `2` (`1U << 1`). `bit=2|mascara=N`. No distingue por sí solo entre obstáculo frontal y trasero. |
| **203** | Nueva `SAFETY_FAULT_SENSOR` | Aparece el bit `4` (`1U << 2`). `bit=4|mascara=N`. Agrupa ausencia de datos, invalidez o antigüedad excesiva; no identifica por sí solo un timeout ECHO o error DMA concreto. |
| **204** | Nueva `SAFETY_FAULT_EMERGENCY` | Aparece el bit `8` (`1U << 3`). `bit=8|mascara=N`. Está definida, pero actualmente no tiene productor operativo en el firmware. No se genera por recibir STOP. |
| **210** | Rearme de Safety completado exitosamente | Solo al completar el rearme. `fallas_anteriores=N|fallas=0|bloqueo=0`. Solicitar o encolar RESET no basta. |
| **300** | Solicitud STOP recibida | Una vez por solicitud recibida para su procesamiento. `estado=N` permite conocer el estado al recibirla. No acredita una transición a IDLE ni un rearme de Safety. |
| **900** | Registros descartados por saturación de la cola | `descartados=N` indica la cantidad de registros perdidos pendiente de informar, no el tamaño de la cola. Su persistencia es de mejor esfuerzo. |

### Máscaras reales de Safety

La enumeración `SafetyFault_t` define:

| Constante | Valor decimal |
|---|---:|
| `SAFETY_FAULT_NONE` | 0 |
| `SAFETY_FAULT_CLIFF` | 1 |
| `SAFETY_FAULT_OBSTACLE` | 2 |
| `SAFETY_FAULT_SENSOR` | 4 |
| `SAFETY_FAULT_EMERGENCY` | 8 |

Las fallas se acumulan mediante OR. Por ejemplo, `mascara=3` significa CLIFF y
OBSTACLE; no es un código de evento adicional. `fallas=0` no garantiza movimiento
habilitado: el sistema comienza con Safety bloqueado y máscara cero.

## 3. Estados y eventos de la FSM

La enumeración `RobotControlState_t` contiene:

| Valor | Estado |
|---|---|
| **0** | `ROBOT_STATE_IDLE` |
| **1** | `ROBOT_STATE_AUTONOMOUS` |
| **2** | `ROBOT_STATE_MANUAL` |
| **3** | `ROBOT_STATE_SAFETY_STOP` |

La base B6 ya prevé AUTONOMOUS en la FSM, pero todavía no incorpora la navegación
autónoma desarrollada en B7. La existencia del estado 1 no acredita navegación
activa ni incorpora sus subestados.

Para interpretar `evento` en los registros 110, `RobotControlEvent_t` define:

| Valor | Evento del firmware |
|---|---|
| 0 | `ROBOT_EVENT_START_AUTO` |
| 1 | `ROBOT_EVENT_START_MANUAL` |
| 2 | `ROBOT_EVENT_STOP` |
| 3 | `ROBOT_EVENT_SAFETY_STOP` |
| 4 | `ROBOT_EVENT_SAFETY_RESET` |

Estos valores no deben confundirse con `CODIGO`: por ejemplo, `evento=3` es una
solicitud de transición Safety; `nuevo=3` es el estado SAFETY_STOP.

## 4. Reglas del registro

1. Registrar eventos, no datos periódicos de sensores, PI o PWM.
2. Registrar una falla solo cuando aparece un bit nuevo en la máscara
   enclavada: conceptualmente, `bits_nuevos = mascara_nueva & ~mascara_anterior`.
   Si aparecen varios bits a la vez, corresponde un registro por cada bit
   nuevo, con su código 201–204. Repetir `ReportFault` para un bit ya enclavado
   no genera otra entrada de esa falla.
3. La recuperación de las lecturas no limpia automáticamente el enclavamiento.
   Después de un rearme exitoso, una nueva aparición del mismo bit constituye
   otro evento y debe registrarse nuevamente.
4. Registrar 110 solamente si cambia efectivamente el estado, no por cada
   evento solicitado, encolado, rechazado o reiterado sin transición.
5. Registrar 210 solo si el rearme se completó correctamente. `RESET_QUEUED`
   confirma encolado, no recuperación ni autorización de movimiento.
6. Distinguir el registro 300 de una transición 110 a IDLE. STOP anula
   consignas, pero no elimina fallas enclavadas. Puede recibirse STOP estando
   en SAFETY_STOP y permanecer en ese estado, sin transición a IDLE.
7. No interpretar un timeout de consigna de 500 ms como pérdida de comunicación.
   Es un vencimiento de movimiento, sin código propio en esta versión del manual
   y sin generar por sí mismo una falla de Safety.
8. Un fallo, demora o indisponibilidad de SD no puede bloquear Safety ni el
   control de motores. La futura implementación debe separar la captura del
   evento de las operaciones de RTC, formato y almacenamiento.
9. El código 900 será de mejor esfuerzo. Si el almacenamiento permanece
   indisponible, no se garantiza que pueda persistirse el aviso de pérdida.
   La ausencia de un registro 900 no demuestra que no se hayan perdido eventos.

## 5. Ejemplos CSV

Arranque bloqueado, rearme inicial y entrada efectiva en MANUAL:

```csv
FECHA_HORA,TICK,CODIGO,DATO
2026-11-08 14:25:00,0,100,estado=0|fallas=0|bloqueo=1
2026-11-08 14:25:00,20,110,anterior=0|nuevo=3|evento=3
2026-11-08 14:25:02,2100,210,fallas_anteriores=0|fallas=0|bloqueo=0
2026-11-08 14:25:02,2100,110,anterior=3|nuevo=0|evento=4
2026-11-08 14:25:03,3000,110,anterior=0|nuevo=2|evento=1
```

Obstáculo enclavado y STOP recibido sin salir de SAFETY_STOP:

```csv
FECHA_HORA,TICK,CODIGO,DATO
2026-11-08 14:25:04,4000,202,bit=2|mascara=2
2026-11-08 14:25:04,4020,110,anterior=2|nuevo=3|evento=3
2026-11-08 14:25:05,5000,300,estado=3
```

STOP desde MANUAL sin falla enclavada, seguido de aviso de registros perdidos:

```csv
FECHA_HORA,TICK,CODIGO,DATO
2026-11-08 14:26:00,60000,300,estado=2
2026-11-08 14:26:00,60020,110,anterior=2|nuevo=0|evento=2
2026-11-08 14:26:01,61000,900,descartados=3
```

Los bloques representan escenarios ilustrativos independientes. Los tiempos
y su orden de presentación no constituyen garantías de latencia del firmware.

## 6. Referencias del firmware

Enumeraciones y máscaras confirmadas en la referencia indicada al inicio:

- [task_safety.h](../Firmware/stm32_control/Core/Inc/task_safety.h): `SafetyFault_t`.
- [task_safety.c](../Firmware/stm32_control/Core/Src/task_safety.c):
  `TaskSafety_ReportFault()`, `TaskSafety_RequestReset()` y condiciones de falla.
- [task_control_robot.h](../Firmware/stm32_control/Core/Inc/task_control_robot.h):
  `RobotControlState_t` y `RobotControlEvent_t`.
- [task_control_robot.c](../Firmware/stm32_control/Core/Src/task_control_robot.c):
  transiciones efectivas y recepción de STOP.
- [task_motor_control.c](../Firmware/stm32_control/Core/Src/task_motor_control.c):
  vencimiento de consignas y anulación de objetivos.
- [FreeRTOSConfig.h](../Firmware/stm32_control/Core/Inc/FreeRTOSConfig.h):
  frecuencia y ancho del contador de ticks.

La etapa D1 crea únicamente este manual. No implementa códigos, productores,
cola, configuración del RTC ni escritura de `EVENTOS.CSV`.
