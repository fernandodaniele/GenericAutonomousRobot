#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H

/*
 * Control del robot desde el celular.
 *
 * El ESP32 levanta su propia red WiFi (modo punto de acceso). Al conectarse
 * con el celular y abrir http://192.168.4.1 aparece una pagina con los
 * botones START y STOP, que se reenvian a la STM32 por UART como lineas
 * "START\n" y "STOP\n".
 */

#define WIFI_CONTROL_SSID      "GAR-Robot"
#define WIFI_CONTROL_PASSWORD  "robot1234"   // minimo 8 caracteres (WPA2)
#define WIFI_CONTROL_CHANNEL   1
#define WIFI_CONTROL_MAX_CONN  2

/**
 * @brief Inicia la red WiFi del ESP32 y el servidor web de control.
 *        Llamar despues de Uart_Init().
 */
void WifiControl_Start(void);

#endif // WIFI_CONTROL_H
