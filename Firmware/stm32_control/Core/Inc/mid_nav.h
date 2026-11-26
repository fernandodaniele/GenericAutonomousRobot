/*
 * mid_nav.h
 *
 *  Created on: Nov 2, 2026
 *      Author: Grupo 2
 *
 *  Modulo de decision de la navegacion autonoma. Maquina de estados SIN
 *  dependencias de HAL ni de FreeRTOS: se puede compilar y probar en la PC
 *  (ver Test/test_mid_nav.c y Test/test_mid_nav_scan.c).
 *
 *  Recorrido por tramos: avanza recto manteniendo el rumbo con el giroscopio.
 *  Con el frente a menos de NAV_OBSTACLE_MM (o al cumplir NAV_STEP_MS) frena,
 *  mide quieto y decide una sola vez: seguir, girar 90 grados en pivote
 *  (derecha si tiene NAV_SIDE_FREE_MM, si no izquierda) o retroceder y volver
 *  a medir. Los giros terminan por angulo, leido del MPU6050. No hay giros
 *  "buscando" espacio libre.
 *
 *  El precipicio tiene prioridad: retrocede lo minimo, gira 90 grados en el
 *  lugar alejandose del sensor que vio el borde y vuelve a medir. Ante un giro
 *  brusco se emite una alerta.
 */

#ifndef CORE_INC_MID_NAV_H_
#define CORE_INC_MID_NAV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "mid_kinematics.h"   /* solo macros y tipos (ROBOT_MAX_WHEEL_SPEED_MMS, ROBOT_WHEEL_BASE), sin HAL */

/* ===== Velocidades =====
 * Avance, retroceso y giros van a NAV_SPEED_PCT % del PWM maximo. Con
 * ROBOT_MAX_WHEEL_SPEED_MMS = 209 (Pololu 100 RPM, rueda 40 mm), 50 % =
 * ~105 mm/s = PWM 4200.
 * Para cambiar la velocidad alcanza con tocar NAV_SPEED_PCT. */
#ifndef NAV_SPEED_PCT
#define NAV_SPEED_PCT          50
#endif
/* Distancia entre ruedas (mm): la misma que usa la cinematica */
#ifndef NAV_WHEEL_BASE_MM
#define NAV_WHEEL_BASE_MM      ROBOT_WHEEL_BASE
#endif
#ifndef NAV_FWD_SPEED_MMS
#define NAV_FWD_SPEED_MMS      (ROBOT_MAX_WHEEL_SPEED_MMS * NAV_SPEED_PCT / 100.0f)
#endif
#ifndef NAV_BACK_SPEED_MMS
#define NAV_BACK_SPEED_MMS     (-NAV_FWD_SPEED_MMS)
#endif
/* Todos los giros (escaneo y precipicio) son en el lugar: cada rueda a
 * NAV_FWD_SPEED_MMS en sentidos opuestos (w * B / 2 = v). Ya no se usa el
 * pivote sobre una rueda (barria un circulo de radio B y chocaba de costado). */
#ifndef NAV_TURN_SPOT_RAD_S
#define NAV_TURN_SPOT_RAD_S    (2.0f * NAV_FWD_SPEED_MMS / NAV_WHEEL_BASE_MM)
#endif

/* ===== Distancias ===== */
/* De frente: frena con menos de esto. Se pone 30 cm para absorber la latencia
 * de lectura y parar cerca de 25 cm. */
#ifndef NAV_OBSTACLE_MM
#define NAV_OBSTACLE_MM        200    // antes 300
#endif
/* Frente libre para arrancar un tramo: 5 cm de colchon sobre NAV_OBSTACLE_MM
 * para no volver a frenar apenas arranca. */
#ifndef NAV_CLEAR_MM
#define NAV_CLEAR_MM           250    // antes 350 (sigue 5 cm sobre NAV_OBSTACLE_MM)
#endif
/* Un lado cuenta como libre para doblar con al menos esto.
 * Se prefiere la derecha; si no esta libre, la izquierda. */
#ifndef NAV_SIDE_FREE_MM
#define NAV_SIDE_FREE_MM       400
#endif
/* Limite lateral (un solo valor): un lateral a esto o menos frena el avance y
 * pasa directo a SCAN, y corta un giro hacia ese lado. Tambien corta el giro
 * si el frente esta a esto o menos. */
#ifndef NAV_SIDE_STOP_MM
#define NAV_SIDE_STOP_MM       40     // antes 50
#endif
/* Lecturas propias seguidas de un lateral sin eco o demasiado cerca (TOO_CLOSE)
 * con las que ese lado cuenta como bloqueado. */
#ifndef NAV_SIDE_BAD_MAX
#define NAV_SIDE_BAD_MAX       10     // antes 3: frenaba por lecturas sueltas
#endif

/* Filtro: lecturas propias seguidas que tienen que confirmar un obstaculo
 * (frente < NAV_OBSTACLE_MM o TOO_CLOSE, lateral o frente a NAV_SIDE_STOP_MM o
 * menos) antes de frenar o cortar un giro. Una lectura suelta (eco fantasma)
 * ya no frena. El frente se lee cada ~120 ms: 2 lecturas = ~1 cm mas de avance. */
#ifndef NAV_OBSTACLE_CONFIRM
#define NAV_OBSTACLE_CONFIRM   2
#endif
/* Frente tan cerca que no hay lugar para girar en el lugar: antes del giro
 * retrocede NAV_SCAN_CLOSE_BACK_MS. */
#ifndef NAV_FRONT_BACKUP_MM
#define NAV_FRONT_BACKUP_MM    100
#endif

/* ===== Tiempos ===== */
#ifndef NAV_BRAKE_MS
#define NAV_BRAKE_MS           400
#endif
/* Pausa con freno entre frenar y retroceder, entre retroceder y girar y
 * entre girar y avanzar: el robot queda quieto y los ultrasonidos leen
 * estable antes de decidir. */
#ifndef NAV_SETTLE_MS
#define NAV_SETTLE_MS          300
#endif
/* Tope del tramo recto (ms, fijo, no depende de la velocidad): al cumplirlo
 * frena y vuelve a medir aunque el frente siga libre.
 * 0 = sin tope (corre hasta la pared). */
#ifndef NAV_STEP_MS
#define NAV_STEP_MS            0      // avance continuo: solo frena por obstaculo (antes 3000)
#endif
/* Retroceso del escaneo con todo bloqueado (atras no hay sensores).
 * PROVISORIO: verificar en el piso. */
#ifndef NAV_SCAN_BACK_MS
#define NAV_SCAN_BACK_MS       1000
#endif
/* Retroceso corto antes de girar cuando el frente esta pegado */
#ifndef NAV_SCAN_CLOSE_BACK_MS
#define NAV_SCAN_CLOSE_BACK_MS 150    // antes 300; una sola vez por maniobra
#endif
/* Retroceso del precipicio: lo minimo para que ambos TCRT queden sobre el
 * piso, mas ~1 cm (atras no hay sensores). PROVISORIO: verificar en el piso. */
#ifndef NAV_CLIFF_BACKUP_MS
#define NAV_CLIFF_BACKUP_MS    600
#endif

/* ===== Giros por angulo (giroscopio) ===== */
#ifndef NAV_TURN_ANGLE_MDEG
/* PRUEBA: en el panel, un giro fisico de ~90 grados se lee como ~10 grados.
 * Se usa 10 grados leidos como referencia del giro "de 90" para ver como
 * responde; si el factor se confirma, corregir la escala del giroscopio y
 * volver a 90000. */
#define NAV_TURN_ANGLE_MDEG    10000
#endif
/* Al frenar el robot sigue girando un poco: el giro se corta en
 * NAV_TURN_ANGLE_MDEG - NAV_TURN_COAST_MDEG (84 grados). Ajustar con lo que
 * se pasa de 90 en el campo "rumbo" de la telemetria. */
#ifndef NAV_TURN_COAST_MDEG
#define NAV_TURN_COAST_MDEG    0       // sin descuento mientras se prueba con 10 grados
#endif
/* Tope de seguridad de un giro. A ~60 grados/s (en el lugar al 50 %) 90
 * grados reales tardan ~1,5 s nominal; se limita a 1 s para que un giro que no
 * corta por angulo no quede dando vueltas. */
#ifndef NAV_TURN_TIMEOUT_MS
#define NAV_TURN_TIMEOUT_MS    1000
#endif

/* ===== Precipicio ===== */
/* Precipicio confirmado: lecturas de sensores seguidas con borde antes de
 * entrar en CLIFF_RECOVER (evita falsos positivos del granito). */
#ifndef NAV_CLIFF_CONFIRM
#define NAV_CLIFF_CONFIRM      3
#endif
/* Bucle de retroceso: tras este numero de CLIFF_RECOVER seguidos sin haber
 * avanzado NAV_CLIFF_LOOP_RESET_MS en FORWARD, pasa a FAULT (frenado, se
 * sale con STOP y START). */
#ifndef NAV_CLIFF_LOOP_MAX
#define NAV_CLIFF_LOOP_MAX     3
#endif
/* Tiene que ser menor que NAV_STEP_MS: si no, un tramo completo nunca
 * reiniciaria los contadores de maniobras. */
#ifndef NAV_CLIFF_LOOP_RESET_MS
#define NAV_CLIFF_LOOP_RESET_MS 1500
#endif
#if (NAV_STEP_MS > 0) && (NAV_CLIFF_LOOP_RESET_MS >= NAV_STEP_MS)
#error "NAV_CLIFF_LOOP_RESET_MS tiene que ser menor que NAV_STEP_MS"
#endif

/* ===== Sensores y fallas ===== */
/* Lado sin eco al elegir hacia donde girar:
 * 0 = cuenta como espacio libre 0 (no dobla hacia un lado sin lectura).
 * 1 = cuenta como "desconocido" (NAV_CLEAR_MM). */
#ifndef NAV_NO_ECHO_AS_UNKNOWN
#define NAV_NO_ECHO_AS_UNKNOWN 0
#endif
/* 0 = sin eco del frontal sigue andando (a ciegas de frente) en vez de frenar.
 * 1 = se detiene hasta que el sensor vuelva a responder. */
#ifndef NAV_STOP_ON_NO_ECHO
#define NAV_STOP_ON_NO_ECHO    0
#endif
#ifndef NAV_FRONT_NO_ECHO_MAX
#define NAV_FRONT_NO_ECHO_MAX  3
#endif
/* 1 = sin MPU6050 el robot entra en FAULT (1 destello) y retoma solo cuando
 *     el IMU vuelve a responder: los giros terminan por angulo.
 * 0 = solo para pruebas: sin IMU los giros terminan por NAV_TURN_TIMEOUT_MS. */
#ifndef NAV_REQUIRE_IMU
#define NAV_REQUIRE_IMU        1
#endif
#ifndef NAV_GYRO_ALERT_DPS
#define NAV_GYRO_ALERT_DPS     200
#endif
#ifndef NAV_ALERT_HOLDOFF_MS
#define NAV_ALERT_HOLDOFF_MS   500
#endif

/* ===== Escaneo (robot quieto) ===== */
/* Lecturas validas por sensor; se usa la mediana */
#ifndef NAV_SCAN_SAMPLES
#define NAV_SCAN_SAMPLES       3
#endif
/* Tiempo maximo midiendo: un sensor sin eco no traba la decision.
 * Cada lateral se lee cada 240 ms: 3 lecturas entran con margen. */
#ifndef NAV_SCAN_MAX_MS
#define NAV_SCAN_MAX_MS        1200
#endif
/* Giros o retrocesos seguidos sin avanzar antes de FAULT (salir con STOP y START) */
#ifndef NAV_SCAN_STUCK_MAX
#define NAV_SCAN_STUCK_MAX     6
#endif
/* Retrocesos seguidos sin ningun lado libre antes de FAULT */
#ifndef NAV_SCAN_BACKUP_MAX
#define NAV_SCAN_BACKUP_MAX    5
#endif

/* ===== Rumbo recto con el MPU6050 =====
 * El rumbo de referencia se toma al recibir START, al salir de FAULT y al
 * terminar cada giro. Cortar y retomar un tramo no lo cambia. */
#ifndef NAV_HEADING_HOLD
#define NAV_HEADING_HOLD       1
#endif
#ifndef NAV_HEADING_KP
#define NAV_HEADING_KP         1.0f     // rad/s de correccion por radian de desvio
#endif
#ifndef NAV_HEADING_MAX_RAD_S
#define NAV_HEADING_MAX_RAD_S  0.15f    // Correccion maxima
#endif
#ifndef NAV_HEADING_DEADBAND_MDEG
#define NAV_HEADING_DEADBAND_MDEG 2000  // Desvios menores a 2 grados no se corrigen
#endif

typedef enum {
    NAV_STATE_IDLE = 0,
    NAV_STATE_FORWARD,
    NAV_STATE_CLIFF_RECOVER,
    NAV_STATE_FAULT,
    NAV_STATE_SCAN
} NavState_t;

typedef enum {
    NAV_CMD_NONE = 0,
    NAV_CMD_START,
    NAV_CMD_STOP
} NavCommand_t;

/* Lectura del frontal, independiente del driver (sin HAL) */
typedef enum {
    NAV_RANGE_OK = 0,
    NAV_RANGE_NO_ECHO,
    NAV_RANGE_FAR,
    NAV_RANGE_TOO_CLOSE
} NavRange_t;

/* Eventos emitidos (mascara de bits) */
#define NAV_EVT_ALERT   (1u << 0)   // Giro brusco
#define NAV_EVT_CLIFF   (1u << 1)   // Entro en CLIFF_RECOVER
#define NAV_EVT_FAULT   (1u << 2)   // Entro en FAULT

typedef struct {
    uint32_t now_ms;
    NavCommand_t command;
    bool snapshot_stale;           // Foto de sensores vieja (> 300 ms)
    NavRange_t front;
    uint32_t front_mm;
    NavRange_t left;
    uint32_t left_mm;
    NavRange_t right;
    uint32_t right_mm;
    uint8_t front_no_echo_count;
    /* Ultima lectura propia de cada lateral (se conserva entre fotos: los
     * laterales se miden 1 de cada 4 fotos) y lecturas malas seguidas */
    NavRange_t left_last;
    uint32_t left_last_mm;
    uint8_t left_bad_count;
    NavRange_t right_last;
    uint32_t right_last_mm;
    uint8_t right_bad_count;
    uint32_t sensor_ms;            // Marca de tiempo de la foto de sensores (una lectura nueva la cambia)
    bool cliff_left;
    bool cliff_right;
    int16_t gyro_z_dps;
    int32_t heading_mdeg;          // Rumbo del MPU6050, milesimas de grado (+ = izquierda)
    bool imu_ok;
} NavInput_t;

typedef struct {
    NavState_t state;
    float linear_mms;              // Velocidad lineal pedida (mm/s)
    float angular_rads;            // Velocidad angular pedida (rad/s, + = izquierda)
    bool brake;                    // true: freno activo, ignorar velocidades
    uint32_t events;               // NAV_EVT_* ocurridos en este paso
    bool fault_latched;            // FAULT por bucle de retroceso (requiere STOP)
} NavOutput_t;

typedef struct {
    NavState_t state;
    uint32_t state_enter_ms;
    uint8_t phase;                 // Sub-fase dentro de SCAN (SPHASE_*) o CLIFF_RECOVER (MPHASE_*)
    uint32_t phase_enter_ms;
    int8_t turn_dir;               // +1 izquierda, -1 derecha
    int32_t turn_start_mdeg;       // Rumbo al empezar el giro en curso
    uint32_t last_alert_ms;
    bool alert_armed;
    uint32_t last_sensor_ms;       // Ultima foto contada para el borde
    uint8_t cliff_hits;            // Lecturas seguidas con borde
    uint8_t cliff_streak;          // CLIFF_RECOVER seguidos sin avanzar
    bool fault_latched;            // FAULT que no se recupera solo
    uint16_t scan_samples[3][NAV_SCAN_SAMPLES]; // Espacio libre (mm) ordenado: frente, izquierda, derecha
    uint8_t scan_n[3];             // Lecturas validas por sensor en este escaneo
    uint8_t scan_streak;           // Giros o retrocesos seguidos sin avanzar
    uint8_t backup_streak;         // Retrocesos seguidos sin ningun lado libre
    int32_t heading_target_mdeg;   // Rumbo a mantener en el tramo recto
    bool close_backup_done;        // Ya hizo el retroceso corto en esta maniobra
    uint8_t front_block_hits;      // Lecturas de frente seguidas con obstaculo (< NAV_OBSTACLE_MM)
    uint8_t front_close_hits;      // Lecturas de frente seguidas a NAV_SIDE_STOP_MM o menos
    uint8_t left_close_hits;       // Lecturas izquierdas seguidas a NAV_SIDE_STOP_MM o menos
    uint8_t right_close_hits;
} NavContext_t;

void Nav_Init(NavContext_t* ctx);

/**
 * @brief Ejecuta un paso de la maquina de estados.
 */
NavOutput_t Nav_Step(NavContext_t* ctx, const NavInput_t* in);

/**
 * @brief Nombre corto del estado (para telemetria).
 */
const char* Nav_StateName(NavState_t s);

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_MID_NAV_H_ */
