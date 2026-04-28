# Plan de Diseño: Data Logger con STM32F4 Discovery

Este documento detalla el plan técnico para implementar un sistema de registro de datos (Data Logger) utilizando la interfaz **SDIO** y el sistema de archivos **FATFS**.

## 1. Hardware Requerido
* **Microcontrolador:** STM32F407G-DISC1 (Discovery Board).
* **Almacenamiento:** Tarjeta MicroSD (Recomendado: 8GB a 32GB, Clase 10, Formato FAT32).
* **Interfaz:** Adaptador de tarjeta MicroSD (sin reguladores de nivel si se conecta a 3.3V).
* **Componentes pasivos:** 5 resistencias de pull-up (10kΩ) para las líneas SDIO (opcional pero recomendado para estabilidad).

## 2. Arquitectura de Conexión (SDIO 4-bit)
Se utilizarán los pines nativos del periférico SDIO para maximizar el rendimiento:

| Señal SD | Pin STM32F4 | Descripción |
| :--- | :--- | :--- |
| **CLK** | PC12 | Reloj de la interfaz |
| **CMD** | PD2 | Línea de comandos |
| **D0** | PC8 | Dato bit 0 |
| **D1** | PC9 | Dato bit 1 |
| **D2** | PC10 | Dato bit 2 |
| **D3** | PC11 | Dato bit 3 (también gestiona CD/DAT3) |
| **VCC** | 3V | Alimentación de 3.3V constante |
| **GND** | GND | Referencia de tierra común |

## 3. Configuración de Software (STM32CubeIDE)

### A. Periférico SDIO
* **Mode:** SD 4-bit Wide Bus.
* **Clock Divide Factor:** Configurar para obtener una frecuencia cercana a 24MHz (máximo 48MHz).
* **DMA:** Activar `SDIO_RX` y `SDIO_TX` para liberar ciclos de CPU durante la escritura.

### B. Middleware FATFS
* **Mode:** SD Card.
* **Configuration:** * `USE_LFN` (Long File Name): Habilitado si se requieren nombres de archivo largos.
    * `MAX_SS` (Sector Size): 512 bytes (estándar para tarjetas SD).

## 4. Estructura de la "Librería de Formateo"
Para mantener el código limpio, se separará la lógica de bajo nivel (FATFS) de la lógica de aplicación.

### Funciones Principales:
1.  `SD_Init()`: Monta la tarjeta y verifica si está lista.
2.  `SD_LogData(char* data)`: Abre el archivo, añade el string de datos y lo cierra de forma segura.
3.  `SD_FormatCSV(...)`: Función auxiliar que toma variables (float, int, timestamp) y las convierte en una cadena separada por comas.

## 5. Estrategia de Robustez
* **Manejo de Errores:** Verificar el valor de retorno `FRESULT` en cada operación de FATFS.
* **Flush de Datos:** Implementar un sistema de guardado periódico (cada X muestras o cada X segundos) para evitar pérdida de datos si se desconecta la alimentación inesperadamente.
* **LEDs de Estado:** Usar los LEDs integrados en la Discovery para indicar:
    * Verde: Sistema funcionando / Escribiendo.
    * Rojo: Error en tarjeta SD o Error de montaje.

## 6. Pasos de Implementación
1.  **Fase 1:** Conexión física y prueba de montaje (`f_mount`).
2.  **Fase 2:** Creación de un archivo de texto simple y escritura de una cadena "Hola Mundo".
3.  **Fase 3:** Integración de la lógica de sensores y formateo CSV.
4.  **Fase 4:** Pruebas de estrés (velocidad de escritura y duración de batería).
