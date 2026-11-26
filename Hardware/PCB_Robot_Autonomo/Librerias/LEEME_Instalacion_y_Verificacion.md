# Robot_Autonomo — bibliotecas KiCad 10

**Estado: PRELIMINAR, no liberado para fabricar.** Se incluyen los 9 simbolos, 9 huellas y 9 STEP simplificados. Los STEP son modelos mecanicos *conceptuales*; no son ficheros originales de los fabricantes ni permiten comprobar colisiones de todos los componentes SMD reales. **No enviar el PCB a fabricar sin verificar cada huella contra las placas fisicas.**

## Instalacion como biblioteca del proyecto

1. Abrir `PCB_Robot_Autonomo.kicad_pro` en KiCad 10, manteniendo `Librerias/` junto al archivo de proyecto.
2. Las tablas `sym-lib-table` y `fp-lib-table` de la raíz registran las bibliotecas locales. Desde Preferencias → Administrar bibliotecas de símbolos o huellas se pueden consultar en el alcance **Proyecto**.
3. Editor de esquemas → Agregar simbolo (`A`) → buscar `Robot_Autonomo` y seleccionar el componente.
4. En el simbolo ya queda asignada `Robot_Autonomo:<nombre>`. Actualizar PCB desde esquema (`F8`).
5. Ver → Visor 3D. Las huellas de `Robot_Autonomo.pretty` usan `${KIPRJMOD}/Librerias/Modelos_3D/`. La PCB también referencia modelos originales en `Librerias/Modelos 3d/` y modelos estándar de KiCad 10.

**Sobre el STM32:** Los 100 pads de la huella se numeran `1-50` para `P1.1-P1.50` y `51-100` para `P2.1-P2.50`. Ver `Documentacion/Mapeo_pines.csv`; por ejemplo pad `56` = `P2.6`. No son números de patas físicas del MCU LQFP100, sino de las cabeceras de expansión de la placa de desarrollo.

## Interruptor SW1

Las tablas activas incluyen `Robot_Interruptores`, con el símbolo
`SW_MTS102_A2_SPDT` y la huella `SW_MTS102_A2_PCB_P4.70mm`.
La PCB usa esa huella para SW1, mientras que el esquema conserva un símbolo
SPST de dos pines con una asignación de huella distinta. Revisar la correspondencia
de pines y pads antes de sincronizar la PCB desde el esquema. El filtro por
cantidad de pines puede ocultar una huella de tres pads para un símbolo de dos.

La huella se basó en la MTS-102-A2. La compatibilidad con la STM102-CI comercial
queda por verificar con el plano o la pieza adquirida antes de fabricar.

## Niveles de evidencia y dimensiones

| Huella | Estado preliminar |
| --- | --- |
| STM32F407G_DISC1 | Plano mecanico ST UM1472 para 66x97mm y headers 2x25: posicionado segun la figura 5; revisar revision exacta. |
| NodeMCU_ESP32S_38P | Nubbeo reporta 48x25.5mm / 38 pines. **Separacion de filas supuesta 22.86 mm**; confirmar con calibre. |
| TCRT5000_Modulo_AO_DO | Placa 14x32 y 4 contactos: cuerpo/orden orientativo, comprobar modulo Duaitek. |
| HC_SR04 | Modulo estandar 45x20, 4 pines 2.54, comprobar orientacion y medida real. |
| MPU6050_GY521 | Tipo GY-521 8 pines. Verificar borde, pin1 y taladros. |
| MicroSD_SPI_6P | Lector SPI 42x24.2 tipo azul; pinout y ubicacion son ORIENTATIVOS. |
| DS1302_RTC_5P | Tipo 43x23 5 pines MH; comprobar posiciones de pins y tornillos. |
| TB6612FNG_Breakout_16P | Distribucion de senales de placa tipo SparkFun, **distancia entre columnas y largo preliminares**. |
| StepDown_Mini_3A_0V8_17V | **20x11** de ficha de INFORMATICA SAN ISIDRO, no MP1584; 4 pines EN/VIN/GND/VO+, revisar distancias al borde. |

## Riesgos electricos

- HC-SR04: ECHO puede entregar 5 V; interponer adaptacion de niveles antes de una entrada MCU 3.3 V que no sea tolerante.
- ESP32 NodeMCU: GPIO6-GPIO11 son usados normalmente por la memoria flash; no son pines GPIO de uso general recomendado.
- MicroSD: los modulos con regulador de 5 a 3.3 V y adaptadores de nivel varian, verificar si VCC=3.3 V es admisible en la placa exacta.
- TB6612: VM es alimentacion de motores, VCC de logica, y STBY se debe habilitar para mover los motores.
- Step Down: 3 A es pico y depende de refrigeracion; confirmar maximo continuo y configuracion de voltaje con carga.

## De donde vienen los datos

- STM: ST UM1472 https://www.st.com/resource/en/user_manual/dm00039084.pdf (pag 15 y tabla de conectores).
- ESP32: https://www.nubbeo.com.ar/productos/placa-nodemcu-esp32s-wifi-bluetooth-esp32-wroom-arduino-nubbeo/ y pinout NodeMCU-32S comun.
- TB6612: https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide (rotulos de modulo)
- Step Down mini: https://www.mercadolibre.com.ar/step-down-salida-fija-y-ajustable-08-17v-mini-3a-pack-x-3/p/MLA2050102871 y ficha placa de 20x11.
- Otros: enlaces particulares incluidos en `Documentacion/Verificaciones.csv`.

## Criterios mecanicos

- Posiciones XY dadas en el CSV (vistas desde arriba como se pondria el modulo sobre la placa portadora).
- Los taladros de montaje del MODULO no se perforan automaticamente en la PCB madre, solo se dibujan referencias visuales si corresponde.
- Los pads de 2.54 mm son THT para insertar pines de modulos; verificar taladro y diametro segun tiras de pines reales (tipicamente 0.95mm y 1.90mm de pad). El Step Down usa taladro 0.8 y pad 1.45mm para agujeros finos.
- Los modelos STEP conservan posiciones relativas XY de los pads pero se representan con hardware simplificado. Su altura relativa sobre la PCB portadora es aproximada.

## Archivos

- `Robot_Autonomo.kicad_sym`: 9 simbolos.
- `Robot_Autonomo.pretty/`: 9 huellas (1 archivo `.kicad_mod` cada una).
- `Modelos_3D/`: 9 modelos `.step` SIMPLIFICADOS.
- `../sym-lib-table` y `../fp-lib-table`: tablas activas del proyecto.
- `Documentacion/Mapeo_pines.csv`: numeros y nombres de las conexiones.
- `Documentacion/Verificaciones.csv`: nivel de verificacion y dudas fisicas.
- `generar_bibliotecas.py`: script editable para corregir las posiciones tras medir los modulos.

Este paquete no sustituye la comprobacion del hardware que se comprara/ya se compro. Revisar con regla/calibre antes de fabricar.
