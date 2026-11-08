#ifndef WIFI_AP_H
#define WIFI_AP_H

#include "esp_err.h"

/* Una inicializacion por arranque, desde app_main. No espera clientes.
 * Los errores se devuelven al llamador; no borra NVS ni reinicia el equipo. */
esp_err_t WifiAp_Start(void);

#endif
