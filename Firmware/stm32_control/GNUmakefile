# ============================================================================
#  GNUmakefile — wrapper regen-safe del build STM32
# ============================================================================
#  GNU Make usa este archivo ANTES que 'Makefile'. Incluye el Makefile que
#  genera STM32CubeMX y le agrega TODO lo que CubeMX 4.7.0-B52 omite o rompe al
#  regenerar sobre este proyecto:
#
#    - main() -> vive en Core/Src/app_main.c (CubeMX no genera main() acá,
#      probado 3 veces; su Core/Src/main.c sale sin main()).
#    - Fuentes del framework (HAL / FreeRTOS / lib USB / system) -> CubeMX a
#      veces las borra del C_SOURCES del Makefile. Se listan acá también.
#    - Drivers/tareas/middleware propios + FatFs vendorizado.
#
#  $(sort) de-duplica: da igual si el Makefile de CubeMX trae o no cada fuente.
#  CubeMX NUNCA toca este archivo -> el build siempre linkea.
#
#  El CI y `make` a secas usan este wrapper automáticamente.
#  Editaste una lista? -> make clean && make
#  Ver fix-plan.md §5.
# ============================================================================

include Makefile

# --- Framework generado/copiado por CubeMX (HAL, FreeRTOS, lib USB, system) --
FW_C_SOURCES := \
  Core/Src/system_stm32f4xx.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
  Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
  Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
  Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
  Middlewares/Third_Party/FreeRTOS/Source/list.c \
  Middlewares/Third_Party/FreeRTOS/Source/queue.c \
  Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
  Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
  Middlewares/Third_Party/FreeRTOS/Source/timers.c \
  Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c

# --- app_main.c (main() — CubeMX no lo genera) + código propio --------------
APP_C_SOURCES := \
  Core/Src/app_main.c \
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

# --- FatFs R0.12c vendorizado a mano (ver FATFS/README.md) -----------------
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

# --- Fusión con lo que trajo el Makefile de CubeMX -------------------------
# $(sort) ordena y de-duplica: si el Makefile ya lista una fuente, no se repite.
C_SOURCES   := $(sort $(C_SOURCES) $(FW_C_SOURCES) $(APP_C_SOURCES))
ASM_SOURCES := $(sort $(ASM_SOURCES) startup_stm32f407xx.s)
C_INCLUDES  += $(APP_C_INCLUDES)

# La lista de prerequisites de la regla del .elf ya quedó fijada (con lo que
# tenía el Makefile) al leer el include. Se suman los objetos que agregamos.
EXTRA_OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir \
                   $(FW_C_SOURCES:.c=.o) $(APP_C_SOURCES:.c=.o) startup_stm32f407xx.o))
$(BUILD_DIR)/$(TARGET).elf: $(EXTRA_OBJECTS)

# vpath acumulativo para los directorios de las fuentes agregadas.
vpath %.c $(sort $(dir $(FW_C_SOURCES) $(APP_C_SOURCES)))
