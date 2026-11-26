# GenericAutonomousRobot (GAR)

Robot móvil diferencial desarrollado por el Grupo 1 para Técnicas Digitales III (TDIII) y Medidas Electrónicas II (MEII). La STM32 controla el movimiento, adquiere los sensores y ejecuta Safety y la navegación. La ESP32 sirve una interfaz web local para operar el robot y consultar su telemetría.

- Control manual, STOP y rearme explícito, con supervisión de proximidad, ausencia de piso y validez de sensores.
- Navegación autónoma supervisada con exploración local, odometría de encoders y maniobras acotadas.
- Panel web sin Internet, con estado, sensores, calibración manual del giroscopio y mapa 2D de ocupación posible.
- Registro de eventos en microSD. Funcionó en pruebas anteriores y requiere una nueva comprobación física por inconvenientes recientes del módulo.

La [arquitectura del sistema](Docs/ARCHITECTURE.md) describe tareas, interfaces, perfiles de navegación y límites de cada subsistema.

## Hardware

| Elemento | Configuración |
|---|---|
| Control | STM32F407G-DISC1, Cortex-M4 a 168 MHz, FreeRTOS |
| Comunicaciones | ESP32 clásico, objetivo `esp32dev` en PlatformIO con ESP-IDF |
| Tracción | Dos motores DC N20 con encoders, driver TB6612FNG |
| Sensores | Dos HC-SR04, frontal y trasero, cuatro TCRT5000 para detectar ausencia de piso y un MPU6050 |
| Registro | microSD por SPI y RTC DS1302 |
| Alimentación del prototipo | Batería LiPo 2S, 7,4 V nominales |

## Repositorio

| Ruta | Contenido |
|---|---|
| [Firmware/stm32_control](Firmware/stm32_control) | Firmware de control, Makefile y pruebas nativas |
| [Firmware/esp32_gateway](Firmware/esp32_gateway) | Firmware de comunicaciones, recursos web embebidos y pruebas |
| [Docs](Docs) | Arquitectura, diagramas del informe y convenciones |
| [Docs/Pruebas](Docs/Pruebas) | Procedimientos y resultados documentados por etapa |
| [Hardware](Hardware) | Lista de componentes y proyecto KiCad |

Abrir [PCB_Robot_Autonomo.kicad_pro](Hardware/PCB_Robot_Autonomo/PCB_Robot_Autonomo.kicad_pro) desde el gestor de KiCad 10. Mantener la estructura de la carpeta para que KiCad resuelva las tablas y modelos locales.

## Compilación y pruebas

Desde la raíz, con GNU Make y la toolchain ARM en el PATH para STM32, y PlatformIO para ESP32:

```sh
make -C Firmware/stm32_control -j4
pio run -d Firmware/esp32_gateway -e esp32
```

En Windows se puede usar `mingw32-make`. El entorno ESP32 fija la plataforma `espressif32 @ 6.12.0` y usa ESP-IDF 5.5.0. PlatformIO genera `firmware.elf` y `firmware.bin` en `Firmware/esp32_gateway/.pio/build/esp32/`. La aplicación debe caber en la partición factory de 1 MiB.

Las siguientes pruebas locales usan GCC nativo y Python 3 para Safety y arranque UART, y Node.js para JavaScript:

```powershell
.\Firmware\stm32_control\Tests\safety_b11\run.bat
.\Firmware\stm32_control\Tests\ultrasound\run.bat
powershell -ExecutionPolicy Bypass -File Firmware/esp32_gateway/Tests/w2/run.ps1
node Firmware/esp32_gateway/Tests/w2/run_js.js
```

Los runners, las fuentes de prueba y los mocks están en [Tests de STM32](Firmware/stm32_control/Tests) y [Tests de ESP32](Firmware/esp32_gateway/Tests). STM32 también dispone de runners para sensores, odometría e IMU. El [workflow de GitHub Actions](.github/workflows/main_ci.yml) compila ambos firmwares en pushes a `master` y pull requests dirigidos a `master`. Las regresiones anteriores se ejecutan por separado.

Para programar ambas placas, consultar el [procedimiento de grabación del firmware](Docs/Pruebas/AUTO3_GIRO_SUPERVISADO.md#programacion-y-validacion-posteriores-requieren-revision-previa).

## Uso y límites

Conectarse a la red local `GenericRobot` y abrir la dirección IP del punto de acceso. El panel distingue la disponibilidad del servidor ESP32 de la comunicación con STM32. Iniciar la exploración requiere una solicitud explícita y supervisión presencial.

La pose es relativa y usa parámetros geométricos provisionales. El mapa estima zonas observadas y posibles obstáculos a partir de ecos y odometría, sin SLAM ni cierre de bucle. Los HC-SR04 no cubren todo el espacio lateral durante un giro, y la falta de eco se trata como dato inválido. Si el teléfono pierde la conexión, STOP remoto puede quedar inaccesible.

## Créditos y licencia

Proyecto académico del Grupo 1. El código del proyecto se distribuye bajo la [licencia MIT](LICENSE), copyright 2026 fernandodaniele. Las bibliotecas incorporadas conservan sus avisos y licencias.
