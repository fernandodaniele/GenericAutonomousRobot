# Mapa de pines — STM32F407VG (Firmware/stm32_control)

Estado de la rama `G1-int/sd-storage` (Fases 1–5 del `fix-plan.md`).
La placa es la **STM32F407G-DISC1**; muchos pines están "quemados" por periféricos
on-board (acelerómetro, DAC de audio, micrófono MEMS, USB OTG, LEDs, botón).

## En uso

| Pin | Función | Periférico / modo | Config en |
|-----|---------|------------------|-----------|
| PH0 / PH1 | Cristal HSE (8 MHz) | RCC | — |
| PA2 | UART hacia el ESP32 — TX | USART2 (AF7) | `usart.c` |
| PA3 | UART hacia el ESP32 — RX | USART2 (AF7) | `usart.c` |
| PA6 | **HC-SR04 ECHO** | TIM3_CH1 (AF2) + DMA1_Stream4 | `drv_ultrasound.c` |
| PA9 / PA10 / PA11 / PA12 | USB OTG FS (VBUS / ID / DM / DP) | USB_OTG_FS | `usbd_conf.c` |
| PA13 / PA14 | SWDIO / SWCLK (debug) | SYS | — |
| PB3 | SWO (trace) | SYS | — |
| PB4 | **HC-SR04 TRIG** | GPIO salida | `gpio.c` / `drv_ultrasound.c` |
| PB6 | **MPU6050 I2C — SCL** (label `Audio_SCL`) | I2C1 (AF4) | `i2c.c` |
| PB7 | **PWM motor izquierdo** | TIM4_CH2 (AF2) | `tim.c` / `drv_motor.c` |
| PB8 | **PWM motor derecho** | TIM4_CH3 (AF2) | `tim.c` / `drv_motor.c` |
| PB9 | **MPU6050 I2C — SDA** (label `Audio_SDA`) | I2C1 (AF4) | `i2c.c` |
| PB13 | **SD SPI — SCK** | SPI2 (AF5) | `spi.c` |
| PB14 | **SD SPI — MISO** | SPI2 (AF5) | `spi.c` |
| PB15 | **SD SPI — MOSI** | SPI2 (AF5) | `spi.c` |
| PC4 | **SD SPI — CS** (`SD_CS`) | GPIO salida (reposo alto) | `gpio.c` / `main.h` |
| PC14 / PC15 | Cristal LSE (OSC32) | RCC | — |
| PD0 / PD1 | **Dirección motor izquierdo** (AIN1 / AIN2) | GPIO salida | `gpio.c` / `drv_motor.c` |
| PD2 / PD3 | **Dirección motor derecho** (BIN1 / BIN2) | GPIO salida | `gpio.c` / `drv_motor.c` |
| PD5 | OTG_FS OverCurrent | GPIO entrada | `gpio.c` |
| PD8 | **DS1302 — CLK** | GPIO salida (bit-bang) | `ds1302.c` / `gpio.c` |
| PD9 | **DS1302 — DAT/IO** | GPIO bidireccional (runtime) | `ds1302.c` / `gpio.c` |
| PD10 | **DS1302 — RST** | GPIO salida | `ds1302.c` / `gpio.c` |
| PD12 / PD13 / PD14 / PD15 | LEDs LD4 / LD3 / LD5 / LD6 | GPIO salida | `gpio.c` |
| PE7 | **TCRT5000 delantero izquierdo** | GPIO entrada | `drv_TCRT5000.c` / `gpio.c` |
| PE8 | **TCRT5000 delantero derecho** | GPIO entrada | `drv_TCRT5000.c` / `gpio.c` |
| PE9 | **TCRT5000 trasero izquierdo** | GPIO entrada | `drv_TCRT5000.c` / `gpio.c` |
| PE10 | **TCRT5000 trasero derecho** | GPIO entrada | `drv_TCRT5000.c` / `gpio.c` |
| PA0 | Botón azul B1 | EXTI/EVT | `gpio.c` |

Notas:
- **PA6** era `SPI1_MISO` del acelerómetro on-board en el `.ioc` original; se
  reasignó al ECHO del HC-SR04 (SPI1 no se usa — ver abajo). El `#define
  SPI1_MISO_Pin` en `main.h` es un residuo inofensivo.
- La I2C del MPU6050 es la **misma** que la del códec CS43L22 (`Audio_SCL/SDA`);
  no hay colisión de direcciones (CS43L22 = 0x94, MPU6050 = 0xD0).
- PE7–PE10 (`Sensor_*` en `main.h`) los deja `MX_GPIO_Init` como `INPUT` sin pull.
  Revisar en banco si conviene `PULLUP` (entrada IR flotante es poco confiable).

## Libre / sin usar

| Pin | Antes (en el `.ioc`) | Estado |
|-----|----------------------|--------|
| PA5 / PA7 | SPI1 SCK / MOSI (acelerómetro LIS302DL) | **Libre.** SPI1 eliminado (nunca se llamaba `MX_SPI1_Init`). |
| PA4 / PC7 / PC10 / PC12 | I2S3 (DAC de audio CS43L22) | **Libre.** I2S3 eliminado (nunca se llamaba `MX_I2S3_Init`). |
| PB10 / PC3 | Micrófono MEMS MP45DT02 (`CLK_IN` / `PDM_OUT`) | Configurados como AF en `gpio.c` pero sin uso. NO usar para SD (por eso SPI2 va a PB13/14/15). |
| PE1 | `MEMS_INT2` (interrupción del acelerómetro) | Config EXTI en `gpio.c`, sin handler. |
| PE3 | `CS_I2C_SPI` (CS del acelerómetro) | GPIO salida en `gpio.c`, sin uso. |
| PD4 | `Audio_RST` (reset del CS43L22) | GPIO salida en `gpio.c`, sin uso. |

Los `#define SPI1_*` / `I2S3_*` que quedan en `main.h` desaparecen solos la
próxima vez que se regenere con SPI1/I2S3 deshabilitados en el `.ioc`.
