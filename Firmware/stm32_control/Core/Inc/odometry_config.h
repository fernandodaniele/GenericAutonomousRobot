#ifndef ODOMETRY_CONFIG_H
#define ODOMETRY_CONFIG_H
/* W7.4: configuracion unica. +X adelante, +Y izquierda, yaw CCW.
 * Nominal != calibrado. El origen supone eje de ruedas centrado en el chasis.
 * Editar diametros/CPR/base/extrinsecos aqui, reconstruir y reiniciar; no hay
 * ajuste remoto ni modificacion de Safety, PI o PWM. */
/* Largo/ancho [m], 170 x 170 mm nominales. Medir chasis y centro del eje.
 * Informativos: no son una envolvente de colision ni intervienen en Safety. */
#define ODOM_CHASSIS_LENGTH_M 0.170f
#define ODOM_CHASSIS_WIDTH_M 0.170f
/* Diametros [m], 26.5 mm nominales. Calibrar por rodadura bajo carga o medir
 * cada rueda; radios derivados, NO editar por separado. */
#define ODOM_LEFT_DIAMETER_M 0.0265f
#define ODOM_RIGHT_DIAMETER_M 0.0265f
#define ODOM_LEFT_RADIUS_M (ODOM_LEFT_DIAMETER_M * 0.5f)
#define ODOM_RIGHT_RADIUS_M (ODOM_RIGHT_DIAMETER_M * 0.5f)
/* CPR efectivo [cuentas/vuelta DE RUEDA], incluye cuadratura y reductora.
 * DESCONOCIDO: 0 bloquea toda metrica/ocupacion. Medir independientemente:
 * abs(conteo_final-conteo_inicial)/vueltas, usando /api/map (strings int64).
 * No multiplicar por 4 nuevamente un valor medido con TIM1/TIM2. */
#define ODOM_LEFT_WHEEL_CPR 0.0f
#define ODOM_RIGHT_WHEEL_CPR 0.0f
/* Base [m], 150 mm nominales entre centros de ruedas. Medir fisicamente;
 * ajustar base efectiva en un ensayo posterior autorizado. No usar el
 * ROBOT_WHEEL_BASE=200 de mid_kinematics como medida real. */
#define ODOM_TRACK_M 0.150f
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
/* Incertidumbre del MODELO [rad]: +/-45 grados provisionales (extremo amplio
 * ya admitido por W7). NO es una medicion del haz ni afirma cobertura lateral.
 * Revisar mediante especificacion/ensayo de ecos antes de confirmar sensores.
 * Se conserva el algoritmo y el margen de alcance W7. */
#define ODOM_BEAM_HALF_ANGLE_RAD 0.785398163f
#endif
