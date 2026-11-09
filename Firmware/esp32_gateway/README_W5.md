# W5 — Telemetría de sensores (solo lectura)

La STM32 sigue adquiriendo y supervisando sensores. La ESP32 consulta una copia,
conserva la última respuesta y sirve el panel offline. HTTP nunca transmite UART.
No se incluyen MPU6050, encoders, AUTO, RTC ni funciones nuevas de movimiento.

## Adquisición y metadatos

Se conservan base de 10 ms, IR cada 20 ms, separación ultrasónica mínima de
70 ms, alternancia nominal de 140 ms por sensor y timeout de 50 ms.
`TaskSensores_GetData()` copia la cola con `xQueuePeek(..., 0)`.

Los campos anteriores y sus significados permanecen intactos, incluidos los
timestamps del disparo de la **última captura válida**, que utiliza Safety.
Se agregan al final de `SensoresDatos_t`:

- `resultado_frontal_disponible` / `resultado_trasero_disponible`: existe un
  intento resuelto, válido o fallido. Inicialmente cero; no confundir con validez.
- `timestamp_resultado_frontal` / `timestamp_resultado_trasero`: disparo del
  último intento resuelto, también si falló el armado o terminó en timeout.
- `timestamp_ir`: instante de lectura de los cuatro GPIO. La primera publicación
  siempre ocurre después de leer IR; una instantánea disponible incluye IR.

Una captura pendiente conserva el resultado anterior y su antigüedad. Los fallos
no se presentan como distancias de cero. El fallo es agregado: el protocolo no
afirma distinguir timeout, error DMA o distancia rechazada. Los IR no tienen
diagnóstico eléctrico: 1 es `TCRT_DETECTA` (piso), 0 es `TCRT_NO_DETECTA` (posible
caída). No se invierte esta lógica ni se afirma que detecte un cable desconectado.

## Protocolo UART v1

Consultas ASCII, terminadas en LF (el receptor admite CRLF):

```text
SENSORS
SENSORS Q=42
```

La forma simple responde con Q=0, útil desde un emisor de pruebas. La ESP32 usa
identificadores de consulta de 1 a 4294967295. Se rechazan argumentos vacíos,
signos, overflow, Q=0 explícito o contenido adicional con `ERR_UNKNOWN`.
`STATUS` y las respuestas operativas anteriores no cambian.

Respuesta CSV de orden fijo, sin espacios, con 14 campos y LF:

```text
SENSORS,1,42,1,3,1,345,87,2,-,50,15,9,0
```

| Índice | Campo | Contrato |
|---|---|---|
| 0 | Identificador | `SENSORS` |
| 1 | Versión | `1` |
| 2 | Q | Identificador devuelto sin alteración |
| 3 | Disponible | 0: cola sin publicación; 1: copia disponible |
| 4 | Edad snapshot | ms desde publicación; `-` si no hay snapshot |
| 5 | Estado frontal | 0: sin resultado; 1: válido; 2: intento fallido |
| 6 | Distancia frontal | entero en mm, 20–4000 si válido; `-` en otro caso |
| 7 | Edad frontal | ms desde disparo del intento resuelto; `-` si estado 0 |
| 8–10 | Trasero | mismo contrato, independiente del frontal |
| 11 | IR | máscara 0–15; bits 0,1,2,3 = delantero izquierdo, delantero derecho, trasero izquierdo, trasero derecho; bit 1 significa piso |
| 12 | Edad IR | ms desde lectura GPIO; `-` sin snapshot |
| 13 | Caída | 0/1; coincide con algún IR sin piso; `-` sin snapshot |

Sin publicación:

```text
SENSORS,1,42,0,-,0,-,-,0,-,-,-,-,-
```

Las edades STM32 se calculan por resta unsigned de `TickType_t`, a 1000 Hz en
este proyecto. Cero es un timestamp legítimo. Conversiones saturan a UINT32_MAX.
La respuesta máxima con todas las edades e identificador máximos cabe en 96
bytes, incluido NUL; no se amplían los buffers UART existentes. No se usa printf
de punto flotante ni se accede a un sensor desde el handler de consulta.

## Transporte ESP32 y prioridades

- SENSORS se intenta como máximo una vez cada 500 ms, sin recuperación de turnos.
- Una respuesta correlacionada debe completarse antes de 200 ms desde el envío.
  Duplicados, respuestas sin consulta, Q diferente o respuesta tardía no renuevan
  la caché. Una falla espera el siguiente período; no hay reintento inmediato.
- Se omite SENSORS durante una sesión manual, movimiento/parada pendiente o
  control operativo. Se vuelve a comprobar después de adquirir el mutex TX.
- `app_main` procesa controles y movimiento primero. Si corresponde STATUS,
  no intenta SENSORS en esa iteración. STATUS conserva su período de 1000 ms.
- El buzón STM32 conserva una sola orden, sin acumular movimientos. Para cerrar
  la carrera entre la consulta y un comando concurrente, sus prioridades son:
  STOP > DETENER > movimientos/RESET/MANUAL > STATUS/PING/otros > SENSORS.
  La sustitución conserva el timestamp del primer byte de la orden ganadora.
- Una trama ya transmitida/procesándose no se puede deshacer. La respuesta UART
  STM32 es síncrona y acotada por el timeout de 50 ms existente; puede retrasar
  TaskComms, pero no deshabilita interrupciones ni el scheduler. Safety y
  MotorControl tienen prioridad superior. La latencia física queda pendiente.
- Un único `uart_read_bytes()` alimenta los parsers STATUS y SENSORS.

El parser de telemetría tiene un buffer de 96 bytes, rechaza NUL, overflow,
campos/estados contradictorios y líneas incompletas. Se recupera al recibir LF.
La copia HTTP se protege con un portMUX breve; no se mantiene durante E/S.

## HTTP y antigüedad

`GET /api/sensors`, sin efectos operativos, JSON y `Cache-Control: no-store`.

```json
{
  "valid": true, "connected": true, "stm32_connected": true, "paused": false,
  "age_ms": 10, "snapshot_available": true, "snapshot_age_ms": 13,
  "limits_ms": {"snapshot": 1500, "ultrasound": 1500, "ir": 1500},
  "front": {"state": "valid", "valid": true, "failed": false, "stale": false, "distance_mm": 345, "age_ms": 97},
  "rear": {"state": "invalid", "valid": false, "failed": true, "stale": false, "distance_mm": null, "age_ms": 60},
  "ir": {"available": true, "stale": false, "age_ms": 19, "floor": [true,true,true,true], "cliff": false}
}
```

- `valid`: alguna respuesta SENSORS completa, coherente y correlacionada recibida;
  no significa que todos los sensores sean válidos.
- `connected`: esa telemetría tiene menos de 1500 ms desde recepción.
- `stm32_connected`: disponibilidad independiente observada mediante STATUS W2.
- `paused`: hay control operativo/manual que impide nuevas consultas opcionales.
- `snapshot_available`: STM32 tenía datos en su cola al responder.
- Sin respuesta o muestra, edades/distancias y estados IR ausentes son `null`;
  `state` ultrasónico es `no_sample`, `valid=false`, `failed=false`.
- Un intento fallido tiene `state=invalid`, `failed=true`, distancia `null` y edad
  del intento. Un resultado válido retenido conserva distancia aunque esté viejo.
- Las edades suman conservadoramente el RTT completo de consulta y el tiempo de
  caché ESP32. No se restan relojes de placas diferentes. El navegador sigue
  aumentando la edad local. Son edades aproximadas de monitoreo.
- W5.8: `stale` considera enlace, snapshot y edad individual. El límite visual
  centralizado `SENSORS_MONITOR_MAX_AGE_MS` es 1500 ms para snapshot, ultrasónicos
  e IR. Lecturas habituales de 300–900 ms no generan advertencias permanentes.
  Una edad de fuente igual a 1500 ms todavía se considera reciente si el enlace
  sigue vigente; por encima se muestra histórica. Esto no modifica los límites
  100/200 ms de Safety STM32 ni autoriza movimiento.
- El timeout de enlace se conserva: a los 1500 ms sin telemetría nueva se pierde
  `connected`, incluso si alguna edad de fuente coincide exactamente con el
  límite visual. STATUS mantiene su propio timeout de W2, sin modificaciones.
- El panel usa los límites publicados en JSON y sigue aumentando las edades
  localmente, incluidos los intervalos pausados durante el control manual.
  Una pausa se identifica como tal; no fuerza `stale=false` ni detiene el reloj.
- El último STATUS de W2 tiene prioridad sobre la copia incluida en el último
  JSON de sensores. Una pérdida detectada por STATUS se refleja en el siguiente
  render periódico del panel (100 ms nominales), sin esperar otro GET de sensores.
  Ese error no queda oculto por el indicador de consulta pausada. Sin W2 disponible,
  se usa el campo `stm32_connected` de la API como respaldo.
- El archivo offline `/sensors.js` consulta HTTP cada 500 ms después de terminar
  la consulta anterior, timeout HTTP de 1500 ms. Un fallo de este panel no cancela
  ni renueva autorizaciones manuales; W4 y Safety siguen gobernándolas.

LRU continúa activado. Se añaden dos rutas, sin aumentar sockets, tareas ni stack.

## Validación reproducible (sin hardware)

```powershell
cmd /c Firmware\stm32_control\Tests\safety_b11\run.bat
powershell -NoProfile -ExecutionPolicy Bypass -File Firmware/stm32_control/Tests/sensors_w5/run.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Firmware/esp32_gateway/Tests/w2/run.ps1
```

Con Node disponible: `node Firmware/esp32_gateway/Tests/w2/run_js.js` ejecuta los
cuatro arneses y repite control manual con el panel real y su endpoint en falla.
El mismo runner admite fuentes cargadas en V8 cuando Node no está disponible.
Las pruebas sustituyen SDK/RTOS/
GPIO/UART/reloj; no certifican tiempos físicos, salud eléctrica ni frenado.

Después de autorización: ambos firmwares compatibles, motores sin potencia,
comparar muestras con Debug y objetos conocidos, provocar fallo de ECHO,
verificar los cuatro IR, observar pausa durante pulsación, STOP/DETENER,
desconexión STM32 y recuperación HTTP. No hay autorización de movimiento físico.
