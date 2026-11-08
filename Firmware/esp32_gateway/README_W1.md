# Interfaz local W1

La ESP32 ofrece una pagina offline y una comprobacion de su servidor HTTP.
No expone comandos del robot ni verifica la conexion STM32 o Safety.

## Acceso previsto

- SSID: `GenericRobot`.
- Clave de laboratorio: `GAR-Lab-2026` (WPA2-PSK, nunca se imprime en consola).
- Canal 1; hasta dos clientes; DHCP predeterminado de ESP-IDF.
- Abrir `http://<IP informada en consola>/` (habitualmente `192.168.4.1`).
- El dispositivo cliente puede avisar que la red no tiene Internet: conservar
  la conexion Wi-Fi para acceder al servidor local.

Las rutas GET son `/`, `/style.css`, `/app.js` y `/health`. La ultima devuelve
`{"status":"ok","service":"esp32-web"}`. La pagina consulta una vez al cargar
y permite repetir la comprobacion mediante el boton, con timeout de 4 segundos.

## Compilacion

Desde la raiz del repositorio:

```powershell
pio run -d Firmware/esp32_gateway -e esp32
```

Resultados: `.pio/build/esp32/firmware.elf` y `firmware.bin`, relativos a este
directorio. No cambiar la particion factory de 1 MiB si el ejecutable no cabe.

`src/CMakeLists.txt` registra las dependencias y los `EMBED_FILES`.
`board_build.embed_files` en `platformio.ini` genera los ensambladores de esos
mismos recursos para SCons. Los simbolos usan el nombre base de cada archivo;
el servidor transmite exactamente `end - start`, sin requerir terminador NUL.
No se utiliza una particion de sistema de archivos.

Los parametros Wi-Fi se centralizan en `src/wifi_ap.c`. El AP y el servidor se
intentan iniciar una sola vez por arranque. Un error se informa sin borrar NVS
ni abortar deliberadamente UART; para otro intento hace falta reiniciar.
No hay espera por clientes ni reintentos continuos. No se garantiza recuperacion
ante fallas internas del SDK que impidan continuar la ejecucion.

## Validacion fisica pendiente

Tras autorizacion y programacion supervisada, alimentar solo la ESP32 por USB:

1. Revisar SSID/IP y ausencia de errores en consola, sin imprimir la clave.
2. Conectar PC y celular, comprobar DHCP y abrir la pagina sin Internet.
3. Verificar las cuatro rutas, el boton, el diseno responsive y la desconexion
   de Wi-Fi durante una consulta. `/health` no implica disponibilidad STM32.
4. Conectar/desconectar clientes y verificar los mensajes de consola.
5. En una validacion posterior con STM32, comprobar los comandos UART originales
   mientras se navega. Mantener desconectada la potencia de motores. El arranque
   de red precede al bucle de recepcion existente; evaluar tambien el trafico de
   inicio y el margen del buffer UART, heap y stacks con carga real.

No se han realizado estas pruebas fisicas como parte de W1.2.
