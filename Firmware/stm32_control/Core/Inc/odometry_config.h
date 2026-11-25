#ifndef ODOMETRY_CONFIG_H
#define ODOMETRY_CONFIG_H
/* +X hacia adelante, +Y a la izquierda y giro antihorario positivo.
 * Los offsets nominales suponen el eje de ruedas centrado en el chasis. */
/* Largo/ancho [m], 170 x 170 mm nominales. Medir chasis y centro del eje.
 * Informativos: no son una envolvente de colision ni intervienen en Safety. */
#define ODOM_CHASSIS_LENGTH_M 0.170f
#define ODOM_CHASSIS_WIDTH_M 0.170f
/* Diámetros efectivos provisionales en metros. Calibrar por rodadura bajo
 * carga. Los radios se derivan de estos valores. */
#define ODOM_LEFT_DIAMETER_M 0.0254f
#define ODOM_RIGHT_DIAMETER_M 0.0254f
#define ODOM_LEFT_RADIUS_M (ODOM_LEFT_DIAMETER_M * 0.5f)
#define ODOM_RIGHT_RADIUS_M (ODOM_RIGHT_DIAMETER_M * 0.5f)
/* CPR efectivo [cuentas/vuelta DE RUEDA], incluye cuadratura y reductora.
 * Un CPR cero impide estimar pose métrica u ocupación. Medir por separado:
 * abs(conteo_final-conteo_inicial)/vueltas, usando /api/map (strings int64).
 * No multiplicar por 4 nuevamente un valor medido con TIM1/TIM2. */
#define ODOM_LEFT_WHEEL_CPR 1435.2f
#define ODOM_RIGHT_WHEEL_CPR 1421.6f
/* Distancia efectiva provisional entre centros de ruedas, en metros.
 * Puede ajustarse con ensayos de giro. */
#define ODOM_TRACK_M 0.1492f
/* 1 solo tras validar CPR, radios y base. 0 permite metrica PROVISIONAL si
 * todos los valores indispensables son completos/coherentes; nunca con CPR=0. */
#define ODOM_CALIBRATION_CONFIRMED 0
/* 1 solo despues de medir montaje y validar incertidumbre angular. Una
 * configuracion completa solo se etiqueta calibrada si AMBOS flags son 1. */
#define ODOM_SENSOR_GEOMETRY_CONFIRMED 0
/* Posiciones [m], +/-85 mm nominales, Y=0. Medir centro acustico respecto al
 * punto medio de ruedas; corregir si el eje no coincide con centro de chasis. */
#define ODOM_FRONT_X_M 0.085f
#define ODOM_FRONT_Y_M 0.0f
/* Yaw [rad], nominales 0/180 grados. Medir alineacion real, no invertir encoders. */
#define ODOM_FRONT_YAW_RAD 0.0f
#define ODOM_REAR_X_M (-0.085f)
#define ODOM_REAR_Y_M 0.0f
#define ODOM_REAR_YAW_RAD 3.141592654f
/* Incertidumbre angular provisional del eco, en radianes. Verificar con
 * ensayos antes de confirmar el montaje. No garantiza cobertura lateral. */
#define ODOM_BEAM_HALF_ANGLE_RAD 0.785398163f
#endif
