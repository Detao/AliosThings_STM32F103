HOST_OPENOCD := MKL43Z4
NAME := mkl43z4impl

$(NAME)_TYPE := kernel

$(NAME)_COMPONENTS += platform/arch/arm/armv6m
$(NAME)_COMPONENTS += rhino libc modules.fs.kv framework.common cli vfs digest_algorithm

GLOBAL_CFLAGS += -DCPU_MKL43Z256VLH4
GLOBAL_CFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_CFLAGS += -mcpu=cortex-m0plus -mfloat-abi=soft -MMD -MP
GLOBAL_CFLAGS += -Wno-format -Wno-incompatible-pointer-types

GLOBAL_ASMFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_ASMFLAGS += -D__STARTUP_CLEAR_BSS
GLOBAL_ASMFLAGS += -mcpu=cortex-m0plus -mfloat-abi=soft
GLOBAL_INCLUDES += ../../arch/arm/armv6m/gcc/m0plus/

GLOBAL_LDFLAGS += --specs=nano.specs --specs=nosys.specs
GLOBAL_LDFLAGS += -Wall -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -mthumb -mapcs -std=gnu99
GLOBAL_LDFLAGS += -Xlinker --gc-sections -Xlinker -static -Xlinker -z -Xlinker muldefs
GLOBAL_LDFLAGS += -mcpu=cortex-m0plus -mfloat-abi=soft
GLOBAL_LDFLAGS += -T platform/mcu/mkl43z4/gcc/MKL43Z256xxx4_flash.ld

$(NAME)_SOURCES     += ./drivers/fsl_clock.c
$(NAME)_SOURCES     += ./drivers/fsl_common.c
$(NAME)_SOURCES     += ./drivers/fsl_flash.c
$(NAME)_SOURCES     += ./drivers/fsl_gpio.c
$(NAME)_SOURCES     += ./drivers/fsl_lpuart.c
$(NAME)_SOURCES     += ./drivers/fsl_uart.c
$(NAME)_SOURCES     += ./drivers/fsl_smc.c
$(NAME)_SOURCES     += ./system_MKL43Z4.c
$(NAME)_SOURCES     += ./gcc/startup_MKL43Z4.S
$(NAME)_SOURCES     += ./hal/hal_uart.c
$(NAME)_SOURCES     += ./hal/hal_flash.c
$(NAME)_SOURCES     += ./aos/aos.c
$(NAME)_SOURCES     += ./aos/soc_impl.c

