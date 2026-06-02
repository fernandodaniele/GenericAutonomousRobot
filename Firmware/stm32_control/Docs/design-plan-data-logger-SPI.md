# Plan de Diseño Alternativo: Data Logger con STM32F4 (Interfaz SPI)

Este plan detalla la implementación de un sistema de registro de datos utilizando el bus **SPI** en lugar de SDIO. Es ideal si los pines de SDIO están ocupados o si se busca una implementación más genérica.

## 1. Hardware Requerido
* **Microcontrolador:** STM32F407G-DISC1 (Discovery Board).
* **Almacenamiento:** Tarjeta MicroSD (FAT32, max 32GB).
* **Módulo:** Adaptador Micro SD con pines SPI (debe ser compatible con 3.3V).

## 2. Conexión SPI (Pines Sugeridos)
Utilizando el periférico **SPI1** del STM32F4:

| Pin Módulo SD | Pin STM32F4 | Función SPI |
| :--- | :--- | :--- |
| **VCC** | 3.3V | Alimentación |
| **GND** | GND | Tierra |
| **CS / SS** | PA4 | Chip Select (GPIO) |
| **SCK** | PA5 | Serial Clock |
| **MISO** | PA6 | Master In Slave Out |
| **MOSI** | PA7 | Master Out Slave In |

## 3. Configuración de Software (STM32CubeIDE)

### A. Periférico SPI1
* **Mode:** Full-Duplex Master.
* **Baud Rate:** Configurar inicialmente entre 400kHz y 1MHz para la fase de inicialización. Luego se puede subir hasta 10-20MHz.
* **Clock Polarity (CPOL):** Low (0).
* **Clock Phase (CPHA):** 1 Edge (0).

### B. Middleware FATFS
* **Mode:** User-defined (esto requiere proveer las funciones de enlace manuales).
* **Configuración:**
    * `USE_LFN`: Habilitado para nombres largos.
    * `FS_TINY`: Habilitado si se quiere ahorrar RAM (usa solo 1 sector de buffer).

## 4. Implementación de la Capa de Enlace (diskio.c)
A diferencia de SDIO, para SPI se deben implementar manualmente las siguientes funciones en `user_diskio.c`:
1.  `USER_initialize`: Inicia el SPI y pone la tarjeta en modo SPI enviando comandos CMD0 y CMD8.
2.  `USER_status`: Verifica si la tarjeta está presente.
3.  `USER_read`: Lee bloques de 512 bytes vía SPI.
4.  `USER_write`: Escribe bloques de 512 bytes vía SPI.

## 5. Estrategia de Escritura (Optimización para SPI)
Debido a que SPI es más lento que SDIO:
* **Buffered Logging:** Acumular datos en un array de 512 bytes en RAM antes de llamar a `f_write`.
* **Sync Control:** Usar `f_sync` en lugar de `f_close` después de cada escritura importante para asegurar la integridad de los datos sin la penalización de tiempo de re-abrir el archivo.

## 6. Pasos a seguir
1.  **Conexión física:** Verificar voltajes de 3.3V.
2.  **Prueba de Comunicación:** Enviar comando de reset (CMD0) y esperar respuesta 0x01.
3.  **Montaje de Sistema de Archivos:** Ejecutar `f_mount`.
4.  **Desarrollo de Librería de Formateo:** Crear funciones que conviertan datos de sensores a cadenas CSV y gestionen el buffer de escritura.
