HOST_OPENOCD := LPC54114
NAME := lpc54114impl

$(NAME)_TYPE := kernel

$(NAME)_COMPONENTS += platform/arch/arm/armv7m
$(NAME)_COMPONENTS += rhino libc cli vfs

GLOBAL_CFLAGS += -DA_LITTLE_ENDIAN
GLOBAL_CFLAGS += -DCPU_LPC54114J256BD64_cm4
GLOBAL_DEFINES += CONFIG_AOS_CLI_STACK_SIZE=8192

ifeq ($(COMPILER),iar)
GLOBAL_INCLUDES += ../../arch/arm/armv7m/iccarm/m4/
GLOBAL_CFLAGS += --cpu=Cortex-M4 \
                 --cpu_mode=thumb \
                 --endian=little
GLOBAL_LDFLAGS += --silent --cpu=Cortex-M4
GLOBAL_LDFLAGS += --config platform/mcu/lpc54114/iar/LPC54114J256_cm4.icf
else
GLOBAL_CFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_CFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -MMD -MP -mfpu=fpv4-sp-d16
GLOBAL_CFLAGS += -Wno-format -Wno-incompatible-pointer-types

GLOBAL_ASMFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_ASMFLAGS += -D__STARTUP_CLEAR_BSS
GLOBAL_ASMFLAGS += -D__STARTUP_INITIALIZE_NONCACHEDATA
GLOBAL_ASMFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

GLOBAL_INCLUDES += ../../arch/arm/armv7m/gcc/m4/
GLOBAL_LDFLAGS += -Lplatform/mcu/lpc54114/gcc/
GLOBAL_LDFLAGS += -lpower_cm4_hardabi
GLOBAL_LDFLAGS += --specs=nano.specs --specs=nosys.specs
GLOBAL_LDFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs
GLOBAL_LDFLAGS += -Xlinker --gc-sections -Xlinker -static -Xlinker -z -Xlinker muldefs
GLOBAL_LDFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
GLOBAL_LDFLAGS += -T platform/mcu/lpc54114/gcc/LPC54114J256_cm4_flash.ld

$(NAME)_CFLAGS  += -Wall -Werror -Wno-unused-variable -Wno-unused-parameter -Wno-implicit-function-declaration
$(NAME)_CFLAGS  += -Wno-type-limits -Wno-sign-compare -Wno-pointer-sign -Wno-uninitialized
$(NAME)_CFLAGS  += -Wno-return-type -Wno-unused-function -Wno-unused-but-set-variable
$(NAME)_CFLAGS  += -Wno-unused-value -Wno-strict-aliasing
endif

$(NAME)_SOURCES     :=

#$(NAME)_SOURCES     += ../../arch/arm/armv7m/gcc/m4/port_c.c
#$(NAME)_SOURCES     += ../../arch/arm/armv7m/gcc/m4/port_s.S

$(NAME)_SOURCES     += ./drivers/fsl_clock.c
$(NAME)_SOURCES     += ./drivers/fsl_common.c
$(NAME)_SOURCES     += ./drivers/fsl_dma.c
$(NAME)_SOURCES     += ./drivers/fsl_flashiap.c
$(NAME)_SOURCES     += ./drivers/fsl_flexcomm.c
$(NAME)_SOURCES     += ./drivers/fsl_inputmux.c
$(NAME)_SOURCES     += ./drivers/fsl_spi.c
$(NAME)_SOURCES     += ./drivers/fsl_spi_dma.c
$(NAME)_SOURCES     += ./drivers/fsl_gpio.c
$(NAME)_SOURCES     += ./drivers/fsl_reset.c
$(NAME)_SOURCES     += ./drivers/fsl_power.c
$(NAME)_SOURCES     += ./drivers/fsl_usart.c
$(NAME)_SOURCES     += ./system_LPC54114_cm4.c

ifeq ($(COMPILER),iar)
$(NAME)_SOURCES     += ./iar/startup_LPC54114_cm4.s
else
$(NAME)_SOURCES     += ./gcc/startup_LPC54114_cm4.S
endif

$(NAME)_SOURCES     += ./hal/hal_uart.c
$(NAME)_SOURCES     += ./hal/hal_flash.c

$(NAME)_SOURCES     += ./aos/aos.c
$(NAME)_SOURCES     += ./aos/soc_impl.c

