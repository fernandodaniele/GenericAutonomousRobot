# Controles operativos W3

Solo ESP32. Se reutiliza el protocolo presente en
`Firmware/stm32_control/Core/Src/task_comms.c`: STOP, RESET y MANUAL. AUTO no
esta implementado como comando UART en esta rama, aunque existe ese estado en
la FSM. Se informa como pendiente, sin boton operativo ni cambios STM32.
No se incorporan ordenes direccionales, velocidades ni renovaciones de movimiento.

## Contrato HTTP

`POST /api/command` recibe un cuerpo de texto exacto: `STOP`, `RESET` o `MANUAL`.
No acepta JSON, saltos de linea, parametros de velocidad ni tramas arbitrarias.
No existe una ruta GET que ejecute comandos.

Cabeceras obligatorias:

- `Content-Type: text/plain` (valor exacto).
- `X-Request-ID`: 32 caracteres hexadecimales minusculos generados por el cliente.
- `Origin`: el mismo origen HTTP que `Host` (por ejemplo,
  `Origin: http://192.168.4.1` y `Host: 192.168.4.1`). El navegador las envia.

El servidor no habilita CORS ni registra OPTIONS. La cabecera personalizada
requiere preflight en solicitudes de otros origenes, ademas de la comprobacion
explicita de Origin. El identificador sirve para deduplicar, no es una credencial
de autenticacion. El acceso sigue limitado a la red de laboratorio WPA2 de W1;
no hay usuarios, roles ni exposicion a Internet.

Ejemplo ilustrativo de respuesta HTTP **202 Accepted**:

```json
{"accepted":true,"id":"0123456789abcdef0123456789abcdef","command":"STOP","phase":"queued","duplicate":false,"confirmed":false}
```

Esto significa aceptacion por ESP32, no recepcion ni ejecucion por STM32.
Los errores usan JSON con `accepted:false` y `error`: 400 para formato/comando/ID
invalido, 403 para origen ausente o distinto, 409 para otro comando pendiente,
413 para cuerpo mayor de seis bytes, 415 para tipo incorrecto y 408 para cuerpo
incompleto o agotamiento del plazo de lectura. No se reintentan comandos.
Los cuerpos y cabeceras se leen en buffers acotados. Los errores que dejan datos
sin consumir cierran la conexion, sin drenar un cuerpo de tamano arbitrario.
Las lecturas de socket tienen timeout de un segundo y el bucle de cuerpo tiene
presupuesto temporal de un segundo (una lectura ya iniciada puede completar
despues). Esto no constituye una garantia de latencia para STOP.

## Despacho y duplicados

`web_commands.c` conserva un unico pendiente y ocho comprobantes recientes,
protegidos mediante un `portMUX_TYPE`. No se agregan tareas, colas FreeRTOS,
asignaciones dinamicas ni esperas por espacio. El handler solo acepta la solicitud.
El bucle existente de `app_main` toma el pendiente antes de consultar STATUS.
La llamada al driver y los accesos HTTP se realizan fuera de secciones criticas.

El mismo ID y comando devuelve su comprobante sin reenviar; el mismo ID con otro
comando se rechaza. El historial es volatil y acotado: no garantiza idempotencia
tras un reinicio ni despues de que el comprobante haya sido desalojado.
La pagina no reintenta, inhibe pulsaciones mientras observa el resultado y filtra
repeticiones del mismo boton durante un segundo, incluso con respuesta rapida.

STOP puede reemplazar un pendiente web aun no tomado para transmitir, que queda
`cancelled`. Si otro comando ya esta transmitiendose, STOP queda para el siguiente
despacho; no puede deshacer esa trama ni una orden de consola. RESET/MANUAL se
rechazan si ya existe un pendiente o envio en curso. Los comandos de consola
siguen disponibles, sin cambios de tramas ni prioridades STM32.
El navegador intenta abortar el POST anterior al solicitar STOP; es de mejor
esfuerzo y no deshace una solicitud ya aceptada. Tampoco garantiza el orden de
solicitudes retrasadas en la red ni de acciones de otros clientes.

`Uart_SendControl()` valida una enumeracion cerrada y comprueba la cantidad de
bytes entregados a `uart_write_bytes()`. Es la misma via para los tres comandos
de consola. El `tx_mux` del driver ESP-IDF 5.5.0 serializa tramas completas entre
consola y `app_main`, incluyendo STATUS y los demas comandos existentes. No se
crea un segundo receptor ni se espera una respuesta STM32 para despachar.
El envio conserva las dependencias de funcionamiento del driver UART del SDK.

## Tres niveles de informacion

1. API aceptada: respuesta 202, `confirmed:false`.
2. Entrega al driver UART: fase `transmitted`; un retorno corto/error es `failed`,
   sin reintento. El fallo no garantiza que ningun byte haya salido.
3. Estado del robot: siempre proviene del STATUS periodico de W2.

`GET /api/status` conserva los campos W2 y agrega `command` solo cuando existe
una solicitud web. Ejemplo del campo adicional:

```json
{"id":"0123456789abcdef0123456789abcdef","name":"STOP","phase":"transmitted","status_after_send":true}
```

Las fases son `queued`, `sending`, `transmitted`, `failed` y `cancelled`.
Se publica el comprobante mas reciente, compartido por todos los clientes.
Una solicitud posterior de otro cliente puede impedir seguir la anterior desde
esa pagina; no se inventa un resultado ni se reenvia para recuperarlo.
`status_after_send` exige STATUS valido y reciente, recibido estrictamente
despues de terminar la entrega al driver; igual microsegundo se rechaza.

No se procesa `RESET_QUEUED` como rearme exitoso ni se confunden los otros acuses
de encolado con estados efectivos. El protocolo carece de ID de correlacion:
un STATUS posterior puede provenir de una consulta anterior ya demorada.
Por eso la interfaz dice **estado posterior observado**, no ejecucion confirmada
del comando. No garantiza causalidad ni una lectura atomica global STM32.

El navegador espera como maximo 3,5 segundos por el POST y observa durante seis
segundos desde la pulsacion. Si no ve un estado reciente compatible, informa
que no pudo confirmar el resultado. RESET requiere observar En espera, fallas 0
y bloqueo 0; MANUAL requiere Modo manual, fallas 0 y bloqueo 0; STOP admite
En espera o Detenido por seguridad. Son criterios de presentacion, sin alterar
las decisiones de la FSM/Safety. No se ordena movimiento al observarlos.
Un timeout HTTP o de observacion **no cancela una solicitud ya aceptada** y nunca
provoca un reintento. Verificar el panel antes de solicitar otra accion.

## Pruebas y compilacion

Desde la raiz:

```powershell
pio run -d Firmware/esp32_gateway -e esp32
powershell -NoProfile -ExecutionPolicy Bypass -File Firmware/esp32_gateway/Tests/w2/run.ps1
node Firmware/esp32_gateway/Tests/w2/test_web.js
node Firmware/esp32_gateway/Tests/w2/test_commands.js
```

El arnes nativo existente conserva las regresiones W2 y agrega HTTP, deduplicado,
cancelacion, despacho real, errores y concurrencia con mocks del SDK. Los dos
arneses JavaScript no tienen dependencias; tambien pueden invocarse en V8 como
`runWebTests(source)` y `runCommandTests(source)` sobre el `app.js` real.
Esto no demuestra latencias ni entrega fisica de comandos sobre hardware.

## Validacion fisica posterior, previa autorizacion

Con potencia de motores desconectada, comparar consola, panel y respuestas UART
para STOP, RESET y MANUAL. Verificar rearme rechazado y aceptado, STOP con falla
enclavada, doble pulsacion, dos clientes, desconexion UART/Wi-Fi y recuperacion
sin reenvios automaticos. Probar que STATUS sigue actualizandose y AUTO permanece
indisponible. No repetir ensayos PI ni habilitar movimiento para validar W3.

STOP por web depende de Wi-Fi, ESP32, UART y STM32. No sustituye una parada de
emergencia fisica certificada ni garantiza detener el robot ante fallo de CPU,
scheduler, alimentacion o electronica. No se programo hardware ni se realizaron
pruebas fisicas durante W3.
