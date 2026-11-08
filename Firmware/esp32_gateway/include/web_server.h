#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"

/* Un intento de inicio por arranque, despues de WifiAp_Start().
 * Solo sirve recursos locales y salud HTTP; no accede a UART. */
esp_err_t WebServer_Start(void);

#endif
