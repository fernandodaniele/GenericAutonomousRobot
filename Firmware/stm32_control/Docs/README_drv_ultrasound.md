# HC-SR04 Driver — STM32F407VG

Driver para el sensor ultrasónico HC-SR04 sobre **STM32F407VG @ 168 MHz**, escrito con STM32 CubeHAL. Forma parte del firmware `stm32_control` del robot autónomo.

---

## Tabla de contenidos

- [Descripción general](#descripción-general)
- [Hardware requerido](#hardware-requerido)
- [Configuración de clocks](#configuración-de-clocks)
- [API](#api)
- [Uso rápido](#uso-rápido)
- [Integración en otro proyecto](#integración-en-otro-proyecto)
- [Funcionamiento interno](#funcionamiento-interno)
- [Limitaciones conocidas](#limitaciones-conocidas)

---

## Descripción general

El HC-SR04 mide distancias por tiempo de vuelo ultrasónico:

1. Se envía un pulso de 10 µs al pin **TRIG**.
2. El sensor emite un burst de 8 ciclos a 40 kHz.
3. El pin **ECHO** permanece en alto durante el tiempo de ida y vuelta del sonido.
4. La distancia se calcula como `d = t_echo * 340 / 2`.

Este driver genera el pulso TRIG con **GPIO + retardo DWT** (contador de ciclos del Cortex-M4) y mide la duración del ECHO con **TIM3 en modo Input Capture + DMA**, capturando los timestamps de ambos flancos sin intervención de la CPU.

---

## Hardware requerido

| Señal HC-SR04 | Pin STM32F407VG | Periférico |
|---|---|---|
| TRIG | **PB4** | GPIO output push-pull |
| ECHO | **PA6** | TIM3 CH1 (IC, AF2) |

> El driver configura los pines internamente. No es necesario inicializarlos en el proyecto.

> **Nota:** PA9 no debe usarse para TRIG en esta placa porque es el pin de detección de VBUS del USB OTG FS. Con el cable USB conectado (necesario para el puerto serie CDC), hay 5 V fijos en PA9 que impiden cualquier control desde el MCU.

### Conexión física

```
HC-SR04          STM32F407VG
-------          -----------
VCC  ──────────  5V
GND  ──────────  GND
TRIG ──────────  PB4
ECHO ──────────  PA6
```

---

## Configuración de clocks

| Parámetro | Valor |
|---|---|
| Fuente de clock | HSE 8 MHz |
| PLL (M/N/P) | 8 / 336 / 2 |
| SYSCLK | 168 MHz |
| APB1 (divisor 4) | 42 MHz → TIM3 clock **84 MHz** |
| Prescaler TIM3 | 83 → **1 MHz (1 µs/tick)** |
| Retardo TRIG | DWT cycle counter a 168 MHz |

TIM3 está en APB1 (divisor 4 → 42 MHz de bus, ×2 = 84 MHz de timer). Con `PSC = 83`, cada tick equivale exactamente a 1 µs.

---

## API

```c
#include "drv_ultrasound.h"
```

### `HCSR04_Init`

```c
void HCSR04_Init(void);
```

Inicializa el GPIO de TRIG (PB4), el contador DWT, TIM3 y el DMA. Llama una sola vez antes del loop principal, después de `HAL_Init()` y `SystemClock_Config()`.

---

### `HCSR04_Trigger`

```c
void HCSR04_Trigger(void);
```

Inicia una medición: genera el pulso TRIG de 10 µs por GPIO y arma la captura DMA en TIM3. Aborta cualquier captura pendiente antes de iniciar. Es **no bloqueante** para la parte del ECHO — retorna después del pulso de 10 µs. Llama a `HCSR04_IsReady()` para saber cuándo llegó el eco.

---

### `HCSR04_IsReady`

```c
uint8_t HCSR04_IsReady(void);
```

Retorna `1` cuando la captura DMA completó los dos flancos del ECHO. Retorna `0` si la medición sigue en curso o no se ha iniciado.

---

### `HCSR04_GetDistance`

```c
float HCSR04_GetDistance(void);
```

Calcula y retorna la distancia en **metros** a partir de los timestamps capturados. Resetea el flag interno. Debe llamarse solo después de que `HCSR04_IsReady()` retorne `1`.

---

## Uso rápido

```c
#include "drv_ultrasound.h"

// En main():
HCSR04_Init();

while (1) {
    HCSR04_Trigger();
    HAL_Delay(200);                  // esperar el eco (max ~25 ms)

    if (HCSR04_IsReady()) {
        float dist = HCSR04_GetDistance();
        // usar dist (en metros)
    }
}
```

> Separar mediciones al menos **60 ms** para evitar ecos de la medición anterior. El ejemplo usa 200 ms.

> **Requisito de build:** para que `sprintf("%.3f", dist)` funcione con nano-newlib, agregar `-u _printf_float` a los `LDFLAGS` del Makefile.

---

## Integración en otro proyecto

### Archivos a copiar

| Archivo | Descripción |
|---|---|
| `Core/Inc/drv_ultrasound.h` | Header del driver |
| `Core/Src/drv_ultrasound.c` | Implementación |

### Conflictos de callbacks HAL

`drv_ultrasound.c` define los siguientes símbolos globales que sobreescriben las implementaciones `__weak` de HAL:

| Símbolo | Motivo |
|---|---|
| `HAL_TIM_IC_MspInit` | Configura GPIO PA6, clock TIM3, DMA1 Stream4 |
| `HAL_TIM_IC_CaptureCallback` | Setea el flag interno cuando llegan los dos flancos |

**Si tu proyecto ya define alguno de estos callbacks** para otros periféricos, fusionálos manualmente: copia el bloque `if (htim->Instance == TIM3)` del driver dentro de tu callback existente.

---

## Funcionamiento interno

### Diagrama de secuencia

```
HCSR04_Trigger()
    ├── HAL_TIM_IC_Stop_DMA(...)                         ← aborta captura anterior si la hay
    ├── GPIO PB4 = HIGH
    ├── delay_us(10)  [DWT, ~1680 ciclos a 168 MHz]
    ├── GPIO PB4 = LOW                                   ← pulso TRIG de 10 µs completado ✓
    └── HAL_TIM_IC_Start_DMA(&htim3, CH1, captures, 2)  ← arma captura de 2 flancos

HC-SR04:
    ├── Recibe pulso TRIG de 10 µs
    ├── Emite burst de 8 ciclos a 40 kHz
    └── ECHO sube al enviar, baja al recibir el eco

TIM3 IC — DMA, ambos flancos, PA6:
    captures[0] = timestamp flanco de subida  de ECHO
    captures[1] = timestamp flanco de bajada  de ECHO
    DMA completa 2 capturas → HAL_TIM_IC_CaptureCallback → flag interno = 1

HCSR04_GetDistance():
    diff = captures[1] - captures[0]   (con manejo de overflow de 16 bits)
    distancia = (diff / 1_000_000) * 340 / 2   [metros]
```

### DMA

| Parámetro | Valor |
|---|---|
| Stream / Canal | DMA1 Stream4 Channel5 |
| Dirección | Periférico → Memoria (`TIM3->CCR1` → `captures[]`) |
| Modo | Normal (no circular), exactamente 2 transferencias |
| Ancho de dato | 16 bits (half-word) |

### Cálculo de distancia

```
diff_us   = captures[1] - captures[0]     [µs, con corrección de overflow 16-bit]
t_vuelo   = diff_us / 1_000_000           [segundos]
distancia = t_vuelo × 340 / 2            [metros]
```

Corrección de overflow (contador TIM3 de 16 bits, ARR = 0xFFFF):

```c
diff = (captures[1] >= captures[0])
     ? captures[1] - captures[0]
     : 0xFFFF - captures[0] + captures[1] + 1;
```

---

## Limitaciones conocidas

| Limitación | Detalle |
|---|---|
| Rango útil | 2 cm – 400 cm (limitación del sensor HC-SR04) |
| Separación mínima entre mediciones | 60 ms (recomendado 200 ms) |
| Timeout silencioso | Si el eco no llega, `HCSR04_IsReady()` nunca retorna `1`. El siguiente `HCSR04_Trigger()` aborta la captura pendiente y reinicia limpio. |
| TIM3 de uso exclusivo | El driver usa TIM3 de forma exclusiva. No puede usarse para otra cosa en el mismo proyecto. |
| Plataforma | Escrito y probado en STM32F407VG a 168 MHz. Para otros MCU o frecuencias, ajustar `PSC` de TIM3 para mantener 1 MHz de tick. |
