HOST_OPENOCD := MIMXRT1052
NAME := mimxrt1052impl

$(NAME)_TYPE := kernel

$(NAME)_COMPONENTS += platform/arch/arm/armv7m
$(NAME)_COMPONENTS += rhino libc modules.fs.kv framework.common cli vfs digest_algorithm

GLOBAL_CFLAGS += -DA_LITTLE_ENDIAN
GLOBAL_CFLAGS += -DCPU_MIMXRT1052DVL6B
GLOBAL_CFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_CFLAGS += -mcpu=cortex-m7 -mfloat-abi=hard -MMD -MP -mfpu=fpv5-d16
GLOBAL_CFLAGS += -Wno-format -Wno-incompatible-pointer-types

GLOBAL_DEFINES += CONFIG_AOS_CLI_STACK_SIZE=8192

GLOBAL_ASMFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_ASMFLAGS += -D__STARTUP_CLEAR_BSS
GLOBAL_ASMFLAGS += -D__STARTUP_INITIALIZE_NONCACHEDATA
GLOBAL_ASMFLAGS += -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16

GLOBAL_LDFLAGS += --specs=nano.specs --specs=nosys.specs
GLOBAL_LDFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_LDFLAGS += -Xlinker --gc-sections -Xlinker -static -Xlinker -z -Xlinker muldefs
GLOBAL_LDFLAGS += -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16
GLOBAL_LDFLAGS += -T platform/mcu/mimxrt1052/gcc/MIMXRT1052xxxxx_sdram_txt.ld

$(NAME)_CFLAGS  += -Wall -Werror -Wno-unused-variable -Wno-unused-parameter -Wno-implicit-function-declaration
$(NAME)_CFLAGS  += -Wno-type-limits -Wno-sign-compare -Wno-pointer-sign -Wno-uninitialized
$(NAME)_CFLAGS  += -Wno-return-type -Wno-unused-function -Wno-unused-but-set-variable
$(NAME)_CFLAGS  += -Wno-unused-value -Wno-strict-aliasing

# Common source
$(NAME)_SOURCES := drivers/fsl_clock.c \
                   drivers/fsl_common.c \
                   drivers/fsl_gpio.c \
                   drivers/fsl_lpuart.c \
                   system_MIMXRT1052.c \
                   hal/hal_uart.c \
                   hal/hal_flash.c \
                   aos/aos.c \
                   aos/soc_impl.c

GLOBAL_INCLUDES += ./ \
                   drivers \
                   CMSIS/Include

# Toolchain related source
$(NAME)_SOURCES += gcc/startup_MIMXRT1052.S

# Component related source
$(NAME)_SOURCES += component/mini_bl/fsl_mini_bl.c
GLOBAL_INCLUDES += component/mini_bl

$(NAME)_SOURCES += boot/fsl_flexspi_nor_boot.c
GLOBAL_INCLUDES += boot
