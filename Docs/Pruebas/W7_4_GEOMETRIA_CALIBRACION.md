# W7.4 — Geometría provisional y calibración sencilla

## Configuración única

La fuente de los parámetros operativos es [odometry_config.h](../../Firmware/stm32_control/Core/Inc/odometry_config.h).
[Arquitectura](../ARCHITECTURE.md#sensores-y-estimación) resume la geometría vigente.
La tabla siguiente describe los parámetros y cómo comprobarlos en el prototipo.

| Parámetro | Configuración consolidada | Unidad / significado | Cómo comprobar o ajustar |
| --- | --- | --- | --- |
| `ODOM_CHASSIS_LENGTH_M` / `WIDTH_M` | 0,170 / 0,170 | m, cuerpo nominal 170×170 mm; informativo | Medir chasis; no es una envolvente de colisión validada. |
| `ODOM_LEFT_DIAMETER_M` / `RIGHT_DIAMETER_M` | 0,0254 / 0,0254 | m, diámetros efectivos provisionales de cada rueda | Medir cada diámetro efectivo o distancia de rodadura bajo carga. |
| `ODOM_LEFT_RADIUS_M` / `RIGHT_RADIUS_M` | 0,0127 / 0,0127, derivados | m, radio = diámetro / 2 | Editar el diámetro, no duplicar el ajuste en el radio. |
| `ODOM_TRACK_M` | 0,1492 | m, separación efectiva provisional entre centros de ruedas | Medir separación y, en un ensayo autorizado, base efectiva. |
| `ODOM_FRONT_X_M` / `Y_M` | +0,085 / 0 | m, centro acústico frontal respecto al punto medio de ruedas | Medir montaje real. |
| `ODOM_REAR_X_M` / `Y_M` | −0,085 / 0 | m, centro acústico trasero | Medir montaje real. |
| `ODOM_FRONT_YAW_RAD` / `REAR_YAW_RAD` | 0 / π | rad, 0° / 180° nominales | Medir alineación física. |
| `ODOM_LEFT_WHEEL_CPR` / `RIGHT_WHEEL_CPR` | **1435,2 / 1421,6** | cuentas efectivas por vuelta de rueda, izquierda/derecha | Medir por separado con el procedimiento de abajo. |
| `ODOM_BEAM_HALF_ANGLE_RAD` | π/4 ≈ 0,785398 | rad, semiamplitud **provisional de incertidumbre del modelo** | Revisar con especificación y ensayo de ecos antes de confirmarla. |
| `ODOM_CALIBRATION_CONFIRMED` | 0 | Confirmación física de ruedas, CPR y base | Cambiar a 1 solo cuando esas medidas hayan sido validadas. |
| `ODOM_SENSOR_GEOMETRY_CONFIRMED` | 0 | Confirmación física de montaje e incertidumbre | Cambiar a 1 solo después de validar esos parámetros. |

Convención: +X hacia adelante, +Y hacia la izquierda, ángulo positivo
antihorario. Los offsets ±85 mm suponen que el centro geométrico del chasis
coincide aproximadamente con el punto medio entre ruedas. Corregir ambos
offsets si el eje está desplazado.

El margen angular ±45° se eligió como envolvente amplia provisional dentro
del rango que el modelo W7 ya admitía. **No es una medición ni una especificación
atribuida al HC-SR04, ni prueba cobertura lateral o seguridad de un giro.**
No cambia el algoritmo de ocupación ni el margen de alcance W7 de 100 mm.
Su valor también está sujeto a revisión física; no permite etiquetar como
calibrado un montaje meramente nominal.

El `ROBOT_WHEEL_BASE=200` preexistente en `mid_kinematics.h` no se usa para W7;
no se modifica esa capa histórica ni se cambia el control validado. Los valores
que realmente usa el mapa proceden únicamente de `odometry_config.h`.

## CPR efectivo y estado de calibración

`Hardware/README.md` identifica dos motores **N20**, sin modelo exacto,
PPR/CPR ni relación de reducción. Ese nombre no determina una resolución.
`Core/Src/tim.c` configura TIM1/TIM2 con `TIM_ENCODERMODE_TI12`, ambas entradas
directas y sin prescaler de captura; se cuenta cuadratura de los dos canales.
TIM1 es de 16 bits y TIM2 de 32 bits. Estos anchos son capacidad del contador,
no cuentas por revolución.

`drv_encoder.c` obtiene deltas modulares firmados. W7 acumula esos mismos
deltas cada 10 ms, sin otra lectura de los timers. Si se conocieran la resolución
física y la reducción, habría que comprobar la convención PPR/CPR del fabricante
antes de aplicar el factor de cuadratura. Medir cuentas por vuelta de rueda
con TIM1/TIM2 evita tener que deducir el producto de esos factores.

La entrega W7.4 original tenía ambos CPR en cero y bloqueaba la estimación
métrica. La configuración consolidada tiene CPR positivos y permite estimación
métrica aproximada. Ambos indicadores de confirmación siguen en cero, por lo
que la clase vigente es **provisional**.

## Tres estados independientes de la antigüedad de datos

| Clase | Condición | Métricas y mapa | Etiqueta web |
| --- | --- | --- | --- |
| 0: `incomplete` | Falta o es inválido algún parámetro indispensable, incluido cualquiera de los CPR, radios, base o montaje/modelo angular | No genera pose métrica ni ecos localizados; conteos disponibles | Parámetros incompletos |
| 1: `provisional` | Configuración completa y coherente, pero falta al menos una confirmación física | Permite estimación métrica y ocupación **aproximadas** | Geometría provisional: estimación aproximada, no calibrada físicamente |
| 2: `calibrated` | Configuración completa y ambos flags de confirmación física en 1 | Permite métrica con parámetros declarados validados | Parámetros físicamente calibrados |

La clase 2 no se obtiene por compilar, pasar tests, medir solo CPR o recibir
coordenadas. Ambos flags son afirmaciones explícitas del responsable de la
calibración. Incluso con los dos en 1, un CPR cero, negativo, NaN o infinito,
o una geometría incoherente, mantiene clase 0 y el bloqueo métrico.

No confundir calidad de configuración con calidad de una muestra: una
discontinuidad de encoder/tiempo conserva la clase de los parámetros, pero
invalida la pose; un dato antiguo conserva sus coordenadas históricas y las
advertencias de vigencia. No se relajan intervalos, overflow, límites, asociación
temporal de ecos ni comprobaciones de validez existentes.

## Telemetría y compatibilidad

Se conserva la consulta `MAPPING Q=N S=P`. La respuesta de pose pasa a:

```
POSE,2,Q,E,T,ESTADO,CLASE,L,R,X,Y,A,DS,DA,EDAD
```

`CLASE` es 0/1/2. Se usa **versión 2** porque en POSE v1 ese campo era booleano:
un receptor antiguo no debe interpretar provisional=1 como confirmación física.
El resto de campos, unidades, buffers y temporización se conservan; `RAY,1,...`,
STATUS y SENSORS no cambian. Los estados de adquisición POSE existentes se
mantienen: 0 sin muestra, 1 sin metadatos métricos completos, 2 métrica válida,
3 discontinuidad. La clase de calibración es un campo separado.

La ESP32 nueva admite POSE v1 de manera conservadora: el antiguo CAL=1 permite
métrica como **provisional**, nunca acredita calibración física completa porque
v1 no demostraba la confirmación del montaje. Los valores o versiones
desconocidos y las combinaciones incoherentes se rechazan. Un cambio de clase
descarta grilla/trayectoria anteriores para no mezclar distintas calidades de
geometría; no provoca ninguna actuación.

`GET /api/map` añade `calibration`, con los strings de la tabla. El booleano
`calibrated` se conserva y solo es true para clase 2. Una pose provisional puede
ser válida y reciente con `calibrated:false`: no hay contradicción. La página
valida esa combinación, muestra siempre la etiqueta y conserva la edad y las
advertencias históricas. No cambia la cruceta ni AUTO/MANUAL/STOP.

En una futura actualización autorizada conviene actualizar primero ESP32,
que entiende ambas versiones, y después STM32; recargar la página para obtener
el JavaScript nuevo. Una ESP32 antigua rechazará POSE v2 y mantendrá el mapa
sin actualizar; los comandos de operación y STATUS/SENSORS conservan su contrato.
No se programaron placas durante esta implementación.

## Procedimiento sencillo para medir CPR sin estimarlo

1. **Potencia de motores desconectada.** Identificar rueda izquierda/derecha y
   marcar una referencia angular. No forzar una reductora que no permita giro
   manual; si no puede girarse así, acordar otro ensayo supervisado.
2. Consultar `/api/map` o el panel. Anotar `epoch`, `pose.tick` y los strings
   `pose.left_counts` / `pose.right_counts`. No hace falta pose métrica para
   leerlos. Esperar una respuesta reciente, sin discontinuidad de odometría.
3. Girar lentamente una rueda **N vueltas completas conocidas**, por ejemplo
   5 o 10, hasta la misma referencia. Mantener la otra inmóvil si es posible.
   Anotar nuevamente el conteo de esa rueda. El `epoch` debe ser el mismo y el
   tick debe haber avanzado: un reinicio entre lecturas invalida el ensayo.
4. Calcular por separado:

   `CPR_izquierdo = abs(L_final - L_inicial) / N_izquierdo`

   `CPR_derecho = abs(R_final - R_inicial) / N_derecho`

   Los conteos ya incluyen la cuadratura del timer y toda la transmisión
   mecánica hasta la rueda: **no multiplicar por 4 ni por la reducción otra vez**.
   Con conteos muy grandes, restar como enteros de 64 bits/BigInt, no convertir
   primero a Number de JavaScript. Se pueden copiar del panel a una calculadora
   que preserve enteros; no hay que escribir registros CNT ni reiniciarlos.
5. Repetir en ambos sentidos y comparar. Conservar los signos validados:
   avance debe dar positivos, retroceso negativos; el CPR guardado es magnitud
   positiva. Una discrepancia grande requiere investigar la medición/cableado,
   no introducir una inversión de signo para ocultarla.
6. Editar **solo los dos CPR** en `odometry_config.h`, con los valores obtenidos
   para cada motor. Si radios, base o montaje siguen nominales, dejar los dos
   flags de confirmación en 0. Reconstruir; la configuración completa pasará
   a **provisional**, no a calibrada. Programar requerirá autorización posterior.
7. Más adelante, medir radios/base/montaje e incertidumbre, comparar distancia
   y giro contra referencias reales y recién entonces revisar los flags.
   No se modifica PI/PWM ni se autoriza planificación sobre el mapa.

## Cambios de software

- `odometry_config.h`: medidas nominales, CPR pendiente, unidades y métodos.
- `mid_odometry.h/.c`: clasificación completa/provisional/calibrada; los cálculos
  de pose y límites W7 se conservan, permitiendo metadatos provisionales.
- `mid_mapping.c`: clase explícita por POSE v2, sin cambios de consultas/cadencia.
- ESP32 `stm32_mapping.c`: parseo v2, compatibilidad conservadora v1 y JSON.
- Web `map.js`, `index.html`: etiqueta explícita, sin tocar controles operativos.
- Pruebas reales de odometría, integración UART, API y JavaScript ampliadas;
  se conservan las regresiones anteriores. Valores de CPR de pruebas son
  sintéticos y no forman parte de la configuración de producción.

Las compilaciones se mantienen en `build/w74` (STM32) y `.pio/w74-build`
(ESP32), separadas de los ELF W7 previos. No se modifican prioridades, stacks,
Safety, FSM, control, drivers, configuración de periféricos ni particiones.

## Verificaciones de software W7.4

- STM32: compilación completa y verificación final correctas, sin errores ni
  advertencias. `text=90460`, `data=924`, `bss=34608`: Flash 91384 B y RAM
  estática 35532 B. ELF `Firmware/stm32_control/build/w74/stm32_control.elf`.
- ESP32: compilación completa y final correctas, sin errores ni advertencias.
  Aplicación 899561/1048576 B (85,8 %); RAM estática 69712/327680 B (21,3 %).
  ELF `Firmware/esp32_gateway/.pio/w74-build/esp32/firmware.elf`.
- 28 grupos de odometría/protocolo (22 anteriores y 6 W7.4), 104 grupos nativos
  ESP32 (98 anteriores y 6 W7.4), 117 grupos JavaScript (114 anteriores y 3 W7.4).
  JavaScript se ejecutó con el runner real en V8; no se instalaron dependencias.
- Regresiones STM32: 411 normales, 55 con giro experimental izquierdo y 55
  derecho; 8 grupos de adquisición y 19 del driver ultrasónico. No se habilita
  el giro experimental en el firmware normal ni se repiten ensayos físicos.
- Se conserva por comparación con la copia previa todo el resto de W7 pendiente.
  Safety, FSM, MotorControl, drivers, modelo de grilla, controles y arbitraje
  UART no tienen modificaciones adicionales W7.4. `git diff --check` sin errores.

Estos resultados verifican software con mocks y parámetros sintéticos de test,
no una calibración o un mapa físico. La RAM indicada no es heap libre medido
en ejecución. En esa entrega la configuración seguía incompleta por CPR
desconocido. No se grabaron placas, operaron motores ni hicieron commits.
