# Control manual web W4

Solo ESP32. No modifica STM32, velocidades, PWM, Safety, FSM ni el protocolo
UART. Los mandos direccionales requieren mantener una pulsacion primaria activa.
No hay movimiento por click, apertura de pagina, recuperacion de red ni cambios
de modo. STOP global W3 permanece separado de DETENER movimiento.

## Autorizacion y emision

La sesion privada reside en `web_commands.c`, bajo el mismo portMUX que las
solicitudes W3. Tiene un unico token propietario de 128 bits, generado mediante
`esp_fill_random()` con Wi-Fi activo. Se guarda solo en RAM y no se imprime ni
se publica por STATUS. El navegador lo conserva solo durante la pulsacion.

- Abrir no emite ninguna trama de movimiento.
- Cada renovacion tiene una secuencia uint32 no nula, estrictamente creciente.
  Duplicados y secuencias anteriores no prolongan la vigencia ni autorizan otra
  emision. Al agotar uint32 se necesita una nueva pulsacion/sesion.
- La autorizacion vence a los 300 ms exactos desde la apertura/ultima renovacion
  aceptada, usando `esp_timer_get_time()`. El navegador renueva aproximadamente
  cada 200 ms, sin peticiones solapadas ni rafagas de compensacion.
- Cada renovacion permite como maximo una trama. Existe un solo pendiente y una
  renovacion posterior lo reemplaza. Nunca se transmite periodicamente una
  direccion guardada si no hubo una renovacion nueva.
- Liberar, vencer o detectar condiciones inseguras invalida el token y la
  generacion, elimina pendientes y solicita DETENER una sola vez. El mismo token
  no puede abrirse o renovarse despues; otra sesion recibe un token nuevo.
- Antes de admitir y antes de emitir se exige STATUS valido, MANUAL, fallas 0,
  bloqueo 0 y edad estrictamente menor de 2000 ms. No cambia el umbral visual W2
  de 3000 ms. La autorizacion nunca sustituye las comprobaciones de STM32.

STOP/RESET/MANUAL de consola o web revocan la sesion. Un cambio de control exige
ademas STATUS recibido estrictamente despues de su barrera temporal antes de
admitir otra sesion. No demuestra causalidad: STATUS no tiene correlacion de
peticiones y podria venir demorado, como se documento en W3.

## Orden y concurrencia

El bucle existente procesa primero controles W3, luego vencimientos/DETENER o
una direccion manual pendiente, y conserva STATUS periodico y el unico lector
UART. No se crean tareas ni colas FreeRTOS adicionales.

Todos los productores UART comparten un mutex TX estatico, con espera maxima
de 20 ms para adquirirlo y 20 ms para esperar FIFO vacio. Solo despues de esas
esperas se comprueban nuevamente token/generacion/secuencia/direccion, vigencia,
STATUS y ausencia de controles W3 pendientes. Se consume el permiso una vez,
inmediatamente antes de llamar al driver con una trama completa. No hay E/S ni
esperas dentro de secciones portMUX. El tx_mux del SDK tambien conserva la
serializacion de tramas.

Una direccion ya comprometida/en curso no puede deshacerse. STOP o liberacion
concurrentes invalidan las siguientes y la parada la sigue en el despacho. STOP
revoca y reemplaza pendientes web segun W3; DETENER descarta las direcciones
pendientes y no espera el timeout de consigna para solicitar parada.
RESET/MANUAL envian primero DETENER cuando hay una parada manual pendiente;
si esa entrega falla no continuan con el cambio de control.

La consola conserva sus comandos fuera de una sesion web. Sus direcciones se
rechazan durante una sesion o parada pendiente, en vez de mezclarse con ella.
STOP y DETENER de consola siguen permitidos. Ante un error de envio manual se
cierra la sesion; DETENER se intenta una vez, sin reintentos indefinidos. Si no
se logra transmitir, queda la proteccion existente de STM32, no una garantia de
que la parada llego fisicamente.

## API

Todos los endpoints son POST con `Content-Type: text/plain`, `X-Request-ID` de
32 hexadecimales minusculos y las validaciones Origin/Host de W3. No hay CORS
abierto ni comandos UART arbitrarios. Los identificadores recientes de apertura
y parada explicita se recuerdan en ocho entradas; no son autenticacion ni una
garantia durable de idempotencia. La seguridad de la sesion no depende de esa
cache: un token cerrado deja de ser valido aunque se desalojen identificadores.

| Ruta | Cuerpo exacto | Cabeceras adicionales | Respuesta aceptada |
| --- | --- | --- | --- |
| `/api/manual/start` | `OPEN` | Ninguna | 201, `accepted:true`, `token`, `lease_ms:300` |
| `/api/manual/renew` | `AVANZAR`, `RETROCEDER`, `GIRAR_IZQUIERDA` o `GIRAR_DERECHA` | `X-Session-Token`, `X-Sequence` decimal | 202, `accepted:true`, `lease_ms:300` |
| `/api/manual/end` | `END` | `X-Session-Token` | 202, `phase:halt_requested` |
| `/api/manual/halt` | `DETENER` | Ninguna | 202, `phase:halt_requested` |

`end` solo cierra el token propietario. `halt` es la parada explicita permitida
a cualquier cliente autorizado de la red; no cambia el modo como STOP. Un segundo
cliente no puede abrir mientras otro tiene autorizacion, ni terminarlo usando
otro token. STOP y las paradas explicitas pueden interrumpir el control de otro
cliente deliberadamente.

Errores: 400 contenido/token/secuencia invalidos; 403 origen; 409 ocupado o
repeticion; 410 token cerrado/vencido/ajeno; 412 estado no habilitado; tambien
413/415/408 segun limites, tipo y lectura del cuerpo. Los cuerpos son acotados
a 15 bytes; las cabeceras de token a 32 caracteres y secuencia a diez digitos.
Ninguna aceptacion HTTP ni `MOVE_ACCEPTED` demuestra desplazamiento fisico.

## Navegador

`manual.js` se embebe en flash junto con los recursos W1-W3, sin sistema de
archivos ni librerias. Captura un solo puntero. `pointerup`, `pointercancel`,
perdida de captura, blur, pagehide y ocultamiento de pagina cierran la pulsacion.
Una respuesta de apertura tardia se cierra sin emitir direccion. Cada peticion
manual tiene timeout de 250 ms; no se reintenta ni se abre otra sesion de forma
automatica. El plazo local de 300 ms se calcula conservadoramente desde el envio
del navegador con `performance.now()`, sin sincronizar relojes.

El indicador describe la autorizacion de esta pagina, no el movimiento real.
Las demoras pueden provocar una detencion conservadora y exigir pulsacion nueva.
Un paquete de cierre perdido no impide el vencimiento ESP32. El boton DETENER
no requiere que el estado mostrado sea habilitado; los direccionales si.

## Compilacion y pruebas

```powershell
pio run -d Firmware/esp32_gateway -e esp32
powershell -NoProfile -ExecutionPolicy Bypass -File Firmware/esp32_gateway/Tests/w2/run.ps1
node Firmware/esp32_gateway/Tests/w2/test_web.js
node Firmware/esp32_gateway/Tests/w2/test_commands.js
node Firmware/esp32_gateway/Tests/w2/test_manual.js
cmd /c Firmware\stm32_control\Tests\safety_b11\run.bat
```

Las pruebas nativas usan los modulos reales con mocks de SDK/RTOS/UART. Los
arneses JavaScript sin dependencias tambien pueden ejecutarse en V8. La bateria
STM32 preexistente verifica, sin modificar produccion, admision a 499 ms, rechazo
a 500 ms y corte de una emision unica a 500 ms. Nada de esto valida hardware.

## Limites y validacion fisica pendiente

Los 300 ms son una vigencia logica; la solicitud DETENER se procesa en el bucle
ESP32, cuya lectura UART puede esperar 100 ms, mas planificacion y transmision.
STM32 conserva su limite de 500 ms desde el primer byte de la ultima consigna,
no desde soltar o perder Wi-Fi. No se garantiza frenado en 300/500 ms ni parada
ante bloqueo de CPU/scheduler, fallo electronico o perdida de alimentacion.
El uso de RAM estatico no mide heap libre ni margen de stacks bajo Wi-Fi real.

Tras autorizacion, comenzar con potencia de motores desconectada: comparar las
tramas, objetivos y CCR2/CCR3; mantener/soltar los cuatro mandos; perder foco y
Wi-Fi; cerrar pagina; desconectar UART; provocar cambio de modo y Safety; probar
STOP, RESET y dos clientes; verificar ausencia de reanudacion al reconectar y
arbitraje de consola. No repetir calibracion PI ni afirmar frenado mecanico a
partir de los registros. No se grabo hardware ni se realizaron pruebas fisicas.
