# Pruebas permanentes del driver HC-SR04

El arnes incluye directamente Core/Src/drv_ultrasound.c y sustituye HAL,
perifericos y avance de DWT. No requiere instrumentacion en produccion,
FreeRTOS simulado, cambios de hardware ni funciones publicas adicionales.

Desde la raiz del repositorio, Windows con GCC nativo:

```powershell
& .\Firmware\stm32_control\Tests\ultrasound\run.bat 'C:\MinGW\bin\gcc.exe'
```

El argumento puede omitirse si gcc esta en PATH. Se compila con C11,
Wall/Wextra/Werror. Ejecutable y resultados quedan en build/ultrasound_tests/,
ignorado por Git. Estas fuentes no se incluyen en el firmware STM32.

Los 19 casos conservan cobertura funcional de:

- Configuracion nominal de TIM3 y transferencias halfword.
- Conversion de pulsos, incluidas distancias cortas, y desbordamiento de 16 bits.
- Adquisicion frontal/trasera con canales, buffers y errores independientes.
- Cancelacion sin ECHO o con una sola captura; posterior recuperacion y rearme.
- Rechazo de lectura ante errores DMA y fallo de armado sin emitir TRIG.
- ECHO alto, fallo de cancelacion y preservacion de la habilitacion previa de IRQ.
- Cuarenta adquisiciones consecutivas, cancelacion de resultados no consumidos,
  disparo que sustituye una adquisicion pendiente y consumo unico del resultado.

Se retiraron las aserciones exclusivas de historiales/contadores de depuracion,
su prueba de timestamp, los mocks RTOS y la compilacion en dos modos. Las
comprobaciones de cancelacion ahora verifican directamente el estado/EN/NDTR
del driver sustituido por el doble, sin recurrir a variables de observacion.

La validacion del rango sensorial y su timeout de 50 ms corresponden a
TaskSensores. Esta suite verifica el driver; no simula esa tarea ni certifica
tiempos de ECHO, IRQ, DMA o frenado fisicos. El doble avanza DWT al leerlo para
que el pulso existente pueda ejecutarse en host; no mide su duracion real.

Las pruebas de seguridad se conservan por separado en Tests/safety_b11/.
Ejecutarlas con su propio run.bat: deben seguir pasando los 99 casos.
No se autoriza movimiento fisico por el resultado de estas pruebas.
