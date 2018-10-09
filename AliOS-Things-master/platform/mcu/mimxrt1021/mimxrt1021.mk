HOST_OPENOCD := MIMXRT1021
NAME := mimxrt1021impl

$(NAME)_TYPE := kernel

$(NAME)_COMPONENTS += platform/arch/arm/armv7m
$(NAME)_COMPONENTS += rhino libc modules.fs.kv framework.common cli vfs digest_algorithm

GLOBAL_DEFINES += CONFIG_AOS_CLI_STACK_SIZE=8192

GLOBAL_CFLAGS += -DA_LITTLE_ENDIAN
GLOBAL_CFLAGS += -DCPU_MIMXRT1021DAG5A

ifeq ($(COMPILER),iar)
GLOBAL_INCLUDES += ../../arch/arm/armv7m/iccarm/m7/
GLOBAL_CFLAGS += --cpu=Cortex-M7.fp.dp \
                 --cpu_mode=thumb \
                 --endian=little
GLOBAL_LDFLAGS += --silent --cpu=Cortex-M7.fp.dp
GLOBAL_LDFLAGS += --config platform/mcu/mimxrt1021/iar/MIMXRT1021xxxxx_sdram_txt.icf
else

GLOBAL_CFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_CFLAGS += -mcpu=cortex-m7 -mfloat-abi=hard -MMD -MP -mfpu=fpv5-d16
GLOBAL_CFLAGS += -Wno-format -Wno-incompatible-pointer-types

GLOBAL_ASMFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_ASMFLAGS += -D__STARTUP_CLEAR_BSS
GLOBAL_ASMFLAGS += -D__STARTUP_INITIALIZE_NONCACHEDATA
GLOBAL_ASMFLAGS += -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16

GLOBAL_LDFLAGS += --specs=nano.specs --specs=nosys.specs
GLOBAL_LDFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_LDFLAGS += -Xlinker --gc-sections -Xlinker -static -Xlinker -z -Xlinker muldefs
GLOBAL_LDFLAGS += -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16
GLOBAL_LDFLAGS += -T platform/mcu/mimxrt1021/gcc/MIMXRT1021xxxxx_sdram_txt.ld

$(NAME)_CFLAGS  += -Wall -Werror -Wno-unused-variable -Wno-unused-parameter -Wno-implicit-function-declaration
$(NAME)_CFLAGS  += -Wno-type-limits -Wno-sign-compare -Wno-pointer-sign -Wno-uninitialized
$(NAME)_CFLAGS  += -Wno-return-type -Wno-unused-function -Wno-unused-but-set-variable
$(NAME)_CFLAGS  += -Wno-unused-value -Wno-strict-aliasing
endif

# Common source
$(NAME)_SOURCES := drivers/fsl_clock.c \
                   drivers/fsl_common.c \
                   drivers/fsl_gpio.c \
                   drivers/fsl_lpuart.c \
                   system_MIMXRT1021.c \
                   hal/hal_uart.c \
                   hal/hal_flash.c \
                   aos/aos.c \
                   aos/soc_impl.c

GLOBAL_INCLUDES += ./ \
                   drivers \
                   CMSIS/Include

# Toolchain related source
ifeq ($(COMPILER),iar)
$(NAME)_SOURCES += iar/startup_MIMXRT1021.s
else
$(NAME)_SOURCES += gcc/startup_MIMXRT1021.S
endif

# Component related source
$(NAME)_SOURCES += component/mini_bl/fsl_mini_bl.c
GLOBAL_INCLUDES += component/mini_bl

$(NAME)_SOURCES += boot/fsl_flexspi_nor_boot.c
GLOBAL_INCLUDES += boot
