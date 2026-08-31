# ============================================================================
#  GNUmakefile — wrapper regen-safe del build STM32
# ============================================================================
#  GNU Make usa este archivo ANTES que 'Makefile'. Incluye el Makefile que
#  genera STM32CubeMX y le suma el código que CubeMX no conoce: drivers de la
#  aplicación, tareas de FreeRTOS, middleware propio y FatFs vendorizado.
#
#  Un "Generate Code" de CubeMX reescribe 'Makefile' por completo pero NUNCA
#  toca este archivo, así que el build sigue funcionando sin retoques.
#  ('Makefile' debe quedar como salida pura de CubeMX: no agregar fuentes ahí.)
#
#  El CI y `make` a secas usan este wrapper automáticamente.
#  Si editás las listas de abajo:  make clean && make
#  (la regla de compilación depende de 'Makefile', no de este archivo).
# ============================================================================

include Makefile

# --- Fuentes propias (no las conoce CubeMX) --------------------------------
APP_C_SOURCES := \
  Core/Src/drv_motor.c \
  Core/Src/drv_ultrasound.c \
  Core/Src/drv_TCRT5000.c \
  Core/Src/drv_uart.c \
  Core/Src/mpu6050.c \
  Core/Src/ds1302.c \
  Core/Src/drv_sd_spi.c \
  Core/Src/mid_kinematics.c \
  Core/Src/mid_log.c \
  Core/Src/task_example.c \
  Core/Src/task_ds1302.c \
  Core/Src/task_sensors.c

# --- FatFs R0.12c vendorizado a mano (ver FATFS/README.md) ----------------
APP_C_SOURCES += \
  FATFS/App/fatfs.c \
  FATFS/Target/user_diskio.c \
  Middlewares/Third_Party/FatFs/src/ff.c \
  Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
  Middlewares/Third_Party/FatFs/src/diskio.c

APP_C_INCLUDES := \
  -IFATFS/App \
  -IFATFS/Target \
  -IMiddlewares/Third_Party/FatFs/src

APP_OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(APP_C_SOURCES:.c=.o)))

# --- Inyección en el build de CubeMX -------------------------------------
# C_SOURCES / C_INCLUDES son variables recursivas: OBJECTS y CFLAGS las
# expanden al usarse (en las recetas), así que basta con appendear acá.
C_SOURCES  += $(APP_C_SOURCES)
C_INCLUDES += $(APP_C_INCLUDES)

# La lista de prerequisites de la regla del .elf de 'Makefile' ya quedó fijada
# (con la base) cuando se leyó el include. Se le suman los objetos propios como
# prerequisites extra del mismo target (make funde prereqs de varias reglas).
$(BUILD_DIR)/$(TARGET).elf: $(APP_OBJECTS)

# vpath es acumulativo: agrega los directorios de las fuentes propias.
vpath %.c $(sort $(dir $(APP_C_SOURCES)))
