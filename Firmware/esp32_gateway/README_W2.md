# Monitoreo STM32 W2

W2 consulta STATUS y muestra exclusivamente informacion recibida de la STM32.
No agrega comandos operativos HTTP ni modifica el firmware STM32. La consola
mantiene `p/s/r/t/m/w/x/a/d/e` y sus tramas originales.

## Flujo y tiempos

- `app_main` consulta STATUS al comenzar su bucle y luego una vez por segundo,
  con la demora normal de la lectura UART (hasta 100 ms) y del scheduler.
- `Stm32Status_PollDue()` agenda desde la ejecucion real: no acumula consultas
  ni compensa retrasos con rafagas. No espera una respuesta para continuar.
- Hay un solo lector UART2: `Uart_ReadResponse()`. Imprime los bytes como antes
  y los entrega al ensamblador de lineas de `stm32_status.c`.
- Cada orden usa una sola llamada `uart_write_bytes()` con su trama completa.
  El driver de ESP-IDF 5.5.0 serializa esas llamadas mediante su `tx_mux`, tambien
  con TX buffer de tamano cero. No se agrega otro mutex ni otro lector/tarea.
- Las constantes `STM32_STATUS_POLL_MS=1000` y `STM32_STATUS_STALE_MS=3000` estan
  centralizadas en `include/stm32_status.h`.

El parser admite exactamente `STATE=N FAULTS=N STOP=N` terminado en LF o CRLF,
incluso fragmentado en lecturas o mezclado con otras lineas. STATE debe estar
entre 0 y 3, STOP entre 0 y 1; los numeros son decimales sin signo de 32 bits.
Rechaza texto adicional, campos ausentes, overflow numerico y NUL embebido.
Una linea demasiado larga se descarta completa hasta LF (buffer de 96 bytes).
Una linea parcial no se publica. Solo un STATUS completo y valido actualiza la
copia y su instante de recepcion; otras respuestas y errores no la rejuvenecen.
Los bits de fallas desconocidos se conservan y se identifican como no definidos.

## API de solo lectura

`GET /api/status` devuelve `application/json` y `Cache-Control: no-store`.

Sin una respuesta valida desde este arranque:

```json
{"valid":false,"connected":false,"state":null,"faults":null,"stop":null,"age_ms":null}
```

Ejemplo ilustrativo del formato, no informacion simulada por el firmware:

```json
{"valid":true,"connected":true,"state":3,"faults":4,"stop":1,"age_ms":125}
```

- `valid`: existe un STATUS valido recibido desde el arranque.
- `connected`: ese STATUS tiene menos de 3000 ms. A los 3000 ms ya es falso.
- `state`: 0 IDLE, 1 AUTONOMOUS, 2 MANUAL, 3 SAFETY_STOP.
- `faults`: mascara enclavada; 1 CLIFF, 2 OBSTACLE, 4 SENSOR, 8 EMERGENCY.
- `stop`: 0/1 devuelto por `TaskSafety_IsStopActive()`, no el acuse de un comando
  STOP ni una medida independiente del PWM.
- `age_ms`: milisegundos desde la recepcion completa, con el reloj monotono de
  64 bits `esp_timer_get_time()`. No es la antiguedad de los sensores STM32.

Cuando los datos vencen, `valid` sigue siendo verdadero y se conserva el ultimo
estado, incluida cualquier falla. La ausencia de respuesta no borra fallas.
La copia compartida y el instante de recepcion se protegen con un `portMUX_TYPE`;
el formateo JSON y la transmision HTTP se realizan fuera de la seccion critica.

Se conservan `/`, `/style.css`, `/app.js` y el significado de `/health`:
`{"status":"ok","service":"esp32-web"}` indica solo disponibilidad HTTP ESP32.

## Pagina

La pagina consulta `/api/status` al cargar y espera 1 segundo despues de cada
consulta finalizada para la siguiente. No solapa peticiones. Cada consulta tiene
timeout de 2500 ms. Un error HTTP/JSON o de red marca el monitoreo no disponible
y los valores anteriores como historicos; no los presenta como actuales.
La edad retenida se incrementa aproximadamente con el reloj monotono del navegador.
La respuesta de `/health` se muestra por separado y conserva su boton de W1.
No hay controles de movimiento, cambios de modo ni recursos externos.

## Compilacion y pruebas locales

Desde la raiz del repositorio:

```powershell
pio run -d Firmware/esp32_gateway -e esp32
powershell -NoProfile -ExecutionPolicy Bypass -File Firmware/esp32_gateway/Tests/w2/run.ps1
node Firmware/esp32_gateway/Tests/w2/test_web.js
```

Las pruebas C usan GCC nativo MinGW, los modulos reales y mocks del SDK. Incluyen
dos hilos Windows para ejercitar productor y lector HTTP simultaneos. No hay
interfaces de prueba en produccion. Los resultados de compilacion nativa se
guardan en `.pio/build/w2-native/`, ignorado por Git. El arnes JavaScript no usa
paquetes: puede ejecutarse con Node o con un runtime V8 que invoque
`runWebTests(source)` sobre el contenido real de `web/app.js`.

Estas pruebas no demuestran entrega fisica UART, respuesta bajo carga Wi-Fi ni
tiempos de ejecucion en la ESP32. El uso de RAM estatico no incluye el heap de
Wi-Fi/HTTP ni sustituye una medicion de heap/stacks en ejecucion.

## Validacion fisica posterior, previa autorizacion

1. Alimentar ESP32 y STM32 por sus conexiones de laboratorio, con potencia de
   motores desconectada. Acceder al AP desde PC y celular sin Internet.
2. Comparar el STATUS de consola con FSM, mascara y STOP mostrados en la web.
3. Usar los comandos existentes de consola bajo supervision; comprobar que las
   respuestas siguen visibles y que las consultas automaticas no corrompen tramas.
4. Interrumpir el enlace UART: comprobar datos historicos y `connected=false`
   tras 3 segundos; restaurarlo y comprobar recuperacion por STATUS valido.
5. Reiniciar ESP32 sin STM32: deben mostrarse campos desconocidos, nunca cero
   como indicacion de seguridad. Comprobar fallos HTTP/desconexion Wi-Fi y acceso
   simultaneo desde dos clientes, observando heap, stacks y errores UART.

STATUS no tiene secuencia, checksum ni identificador de solicitud. La edad mide
recepcion, no el instante de generacion interno: una respuesta demorada puede
parecer reciente. `connected=true` indica trafico STATUS valido reciente; no
garantiza salud integral ni autorizacion de movimiento. Los tres campos STM32 se
consultan secuencialmente en su implementacion actual y no constituyen una toma
atomica de Safety/FSM. W2 no cambia ese contrato ni permite decisiones de control.

No se realizaron pruebas fisicas ni programacion de hardware durante W2.
