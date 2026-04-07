# Hardware
/* A continuación se describe una selección de componentes enfocados en la construcción de un robot recoge pelotas de una cancha de tenis, el mismo debe ser capaz de cumplir los requisitos de autonomía impuestos por la cátedra.

===== MICROCONTROLADORES ====
Dedicado al funcionamiento del robot: STM32F407G-DISC1 (ARM Cortex-M4 @ 168MHz)
Dedicado a las comunicaciones: ESP32 Dev Kit

 ===== ACTUADORES =====
Motores de rodaje: pololu 100 rpm x 2
Motores de sistema de capción: pololu 50 rpm x 2

==== DRIVER DE MOTORES =====
Para ambos pares de motores: TB6612FNG 

===== SENSORES DE POSICIÓN ====
Encargados de esquivar elementos no deseados: HC-SR04, Ultrasónicos x 3
Encargados de evitar caer en caso de canaletas abiertas: TCRT-5000, infrarojo x 2
Encargado de cuantificar el movimiento realizado: MPU6050, Giróscopio x1

==== SENSOR DE DETECCIÓN ====
Encargado de la lectura de pelotas: Cámara integrada a la ESP 32.

==== ALIMENTACIÓN ====
Baterias 18650, al menos 3 elementos.

*/