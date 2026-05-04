● Qué hace el test

  El loop dispara una medición cada ~200 ms y envía el resultado por USB serial (CDC virtual):

  HCSR04_Trigger()   →  pulso TRIG de 10 µs en PA9 (TIM1 OC, single-shot)
  HAL_Delay(200)     →  espera el eco (el HC-SR04 tarda máx ~25 ms para 4 m)
  HCSR04_IsReady()   →  pregunta si DMA capturó los dos flancos del ECHO en PA6 (TIM3 IC)
  HCSR04_GetDistance() → calcula (captures[1] - captures[0]) / 1MHz * 340 / 2  → metros
  CDC_Transmit_FS()  →  envía "Dist: 0.234 m\r\n" por USB

  Si el eco no llega (sin sensor, fuera de rango, cable suelto) envía "Dist: timeout\r\n".

  ---
  Cómo verificar que funciona

  1. Conectar el hardware

  ┌─────────┬─────────────┐
  │ HC-SR04 │ STM32F407VG │
  ├─────────┼─────────────┤
  │ VCC     │ 5V          │
  ├─────────┼─────────────┤
  │ GND     │ GND         │
  ├─────────┼─────────────┤
  │ TRIG    │ PA9         │
  ├─────────┼─────────────┤
  │ ECHO    │ PA6         │
  └─────────┴─────────────┘

  2. Flashear y abrir el puerto serie

  # Linux — conectar la placa por micro USB, luego:
  screen /dev/ttyACM0 115200
  # o con minicom:
  minicom -D /dev/ttyACM0 -b 115200

  3. Qué deberías ver

  Con el sensor apuntando a una pared a ~30 cm:
  Dist: 0.298 m
  Dist: 0.301 m
  Dist: 0.299 m

  Acercando la mano:
  Dist: 0.102 m
  Dist: 0.098 m

  Sin sensor o con los cables sueltos:
  Dist: timeout
  Dist: timeout

  ---
  Señales de que algo falla

  ┌───────────────────────────────────┬────────────────────────────────────────────────────────────────────────────────────┐
  │              Síntoma              │                                   Causa probable                                   │
  ├───────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────┤
  │ Solo timeout con sensor conectado │ ECHO en pin equivocado, o prescaler de TIM3 desajustado para 168 MHz               │
  ├───────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────┤
  │ Valores erráticos / muy grandes   │ ECHO llega pero el DMA captura el flanco en overflow (diff negativo corregido mal) │
  ├───────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────┤
  │ Nada en el puerto serie           │ USB CDC no inicializado, o usbd_cdc_if.h no linkea bien                            │
  ├───────────────────────────────────┼────────────────────────────────────────────────────────────────────────────────────┤
  │ LED no parpadea                   │ El programa colgó en Error_Handler al init                                         │
  └───────────────────────────────────┴────────────────────────────────────────────────────────────────────────────────────┘

  Nota sobre el prescaler: el driver fue escrito para STM32F401RE a 84 MHz, donde TIM3 (APB1×2) corre a 84 MHz y PSC=83 da exactamente 1 MHz. En el F407VG a 168 MHz, APB1 está dividido por 4
  (42 MHz → TIM3 a 84 MHz), así que el prescaler de TIM3 sigue siendo correcto. Pero TIM1 (APB2 a 168 MHz con /2 → 84 MHz → TIM1 a 168 MHz) con PSC=83 daría 2 MHz en vez de 1 MHz, acortando
  el pulso TRIG a ~5 µs. Si el sensor no responde, ese es el primer lugar donde buscar.




 Hace falta abrir cubeMX y configurar entradas, salidas o timers?

● No. El driver configura todo internamente desde HCSR04_Init() — no hay nada que tocar en CubeMX.

  Lo que hace el driver sin intervención tuya:

  ┌────────────────────────────┬────────────────────────────────────────────────────────┐
  │          Recurso           │                   Quién lo configura                   │
  ├────────────────────────────┼────────────────────────────────────────────────────────┤
  │ TIM1 CH2 — PA9 (TRIG, AF1) │ HAL_TIM_OC_MspInit en drv_ultrasound.c                 │
  ├────────────────────────────┼────────────────────────────────────────────────────────┤
  │ TIM3 CH1 — PA6 (ECHO, AF2) │ HAL_TIM_IC_MspInit en drv_ultrasound.c                 │
  ├────────────────────────────┼────────────────────────────────────────────────────────┤
  │ DMA1 Stream4 Channel5      │ HAL_TIM_IC_MspInit en drv_ultrasound.c                 │
  ├────────────────────────────┼────────────────────────────────────────────────────────┤
  │ Clocks TIM1, TIM3, DMA1    │ __HAL_RCC_TIM1_CLK_ENABLE() etc. dentro de los MspInit │
  ├────────────────────────────┼────────────────────────────────────────────────────────┤
  │ NVIC (interrupciones)      │ Habilitadas dentro de los MspInit                      │
  └────────────────────────────┴────────────────────────────────────────────────────────┘

  Los MspInit son callbacks __weak del HAL — el driver los sobreescribe con su propia implementación, así que el HAL los llama automáticamente cuando HCSR04_Init() invoca HAL_TIM_OC_Init() y
  HAL_TIM_IC_Init().

  ---
  Lo único que sí necesitás verificar en CubeMX es que PA9 y PA6 no estén asignados a otro periférico en tu .ioc — si CubeMX ya generó código que inicializa esos pines para otra función,
  habría conflicto. Podés abrir el .ioc y confirmar que aparecen como n/a o sin asignar.
