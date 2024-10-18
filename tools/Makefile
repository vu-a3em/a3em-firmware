PROJECT := A3EM
CONFIG := bin
SHELL := /bin/bash

ifdef BOARD_REV
REVISION := $(BOARD_REV)
else
REVISION := A
endif

BSP = apollo4_pro
PART = apollo4p
PART_DEF = AM_PART_APOLLO4P

$(info Building test for Revision $(REVISION):)
$(info BSP = $(BSP))
$(info PART = $(PART))
$(info PART_DEF = $(PART_DEF))

TOOLCHAIN ?= arm-none-eabi
CPU = cortex-m4
FPU = fpv4-sp-d16
FABI = hard
FLASH_START = 0x00018000
ID_FLASH_LOCATION = 0x001FFFF8

DEFINES  = -D_HW_REVISION=$(REVISION)
DEFINES += -D_DATETIME="\"$(shell date -u)\""
DEFINES += -DPART_$(PART)
DEFINES += -D$(PART_DEF)
DEFINES += -D'SEGGER_RTT_SECTION="SHARED_RW"'
DEFINES += -DAM_PACKAGE_BGA
DEFINES += -DAM_DEBUG_PRINTF
DEFINES += -Dgcc

LINKER_FILE := ../AmbiqSDK/bsp/$(BSP)/linker/a3em.ld
STARTUP_FILE := ../AmbiqSDK/bsp/$(BSP)/linker/startup_gcc.c

#### Required Executables ####
CC = $(TOOLCHAIN)-gcc
GCC = $(TOOLCHAIN)-gcc
CPP = $(TOOLCHAIN)-cpp
LD = $(TOOLCHAIN)-ld
CP = $(TOOLCHAIN)-objcopy
OD = $(TOOLCHAIN)-objdump
RD = $(TOOLCHAIN)-readelf
AR = $(TOOLCHAIN)-ar
SIZE = $(TOOLCHAIN)-size
RM = $(shell which rm 2>/dev/null)
EXECUTABLES = CC LD CP OD AR RD SIZE GCC
K := $(foreach exec,$(EXECUTABLES),\
        $(if $(shell which $($(exec)) 2>/dev/null),,\
        $(info $(exec) not found on PATH ($($(exec))).)$(exec)))
$(if $(strip $(value K)),$(info Required Program(s) $(strip $(value K)) not found))

ifneq ($(strip $(value K)),)
all clean:
	$(info Tools $(TOOLCHAIN)-gcc not installed.)
	$(RM) -rf bin
else

INCLUDES  = -I../AmbiqSDK/bsp/$(BSP)
INCLUDES += -I../AmbiqSDK/mcu/$(PART)
INCLUDES += -I../AmbiqSDK/mcu/$(PART)/hal
INCLUDES += -I../AmbiqSDK/mcu/$(PART)/hal/mcu
INCLUDES += -I../AmbiqSDK/CMSIS/AmbiqMicro/Include
INCLUDES += -I../AmbiqSDK/CMSIS/ARM/Include
INCLUDES += -I../AmbiqSDK/devices
INCLUDES += -I../AmbiqSDK/utils
INCLUDES += -I../src/app
INCLUDES += -I../src/boards
INCLUDES += -I../src/boards/rev$(REVISION)
INCLUDES += -I../src/external/fatfs
INCLUDES += -I../src/external/segger
INCLUDES += -I../src/peripherals/include
INCLUDES += -I.

VPATH  = ../AmbiqSDK/bsp/$(BSP)/linker
VPATH += ../AmbiqSDK/mcu/$(PART)/hal
VPATH += ../AmbiqSDK/mcu/$(PART)/hal/mcu
VPATH += ../AmbiqSDK/devices
VPATH += ../AmbiqSDK/utils
VPATH += ../src/app
VPATH += ../src/boards
VPATH += ../src/boards/rev$(REVISION)
VPATH += ../src/external/fatfs
VPATH += ../src/external/segger
VPATH += ../src/peripherals/src
VPATH += .

SRC =
SRC += am_devices_led.c
SRC += am_util_delay.c
SRC += am_util_stdio.c
SRC += am_util_string.c
SRC += ff.c
SRC += ffunicode.c
SRC += startup_gcc.c
SRC += SEGGER_RTT.c

SRC += audio.c
SRC += battery.c
SRC += comparator.c
SRC += digipot.c
SRC += henrik.c
SRC += imu.c
SRC += led.c
SRC += logging.c
SRC += magnet.c
SRC += mram.c
SRC += rtc.c
SRC += runtime_config.c
SRC += storage.c
SRC += system.c
SRC += vhf.c

.PHONY: all dc_offset
.PRECIOUS: $(CONFIG)/%.axf
.SECONDEXPANSION:

all:
	$(error Make targets include: dc_offset)

program:
ifneq ($(BUILT_BY), eclipse)
	printf "r\n" > $(CONFIG)/flash.jlink
	printf "loadfile $(CONFIG)/$(TARGET).bin $(FLASH_START)\nr\ng\nexit\n" >> $(CONFIG)/flash.jlink
	$(JLINK) $(JLINK_FLAGS) $(CONFIG)/flash.jlink
endif

dc_offset: TARGET = ToolDcOffset
dc_offset: SRC += audadc_dc_offset_calibrator.c
dc_offset: $(CONFIG) $(CONFIG)/audadc_dc_offset_calibrator.o $(CONFIG)/$$(TARGET).bin program

OBJS = $(SRC:%.c=$(CONFIG)/%.o)
DEPS = $(SRC:%.c=$(CONFIG)/%.d)

LIBS  = ../AmbiqSDK/bsp/$(BSP)/gcc/bin/libam_bsp.a
LIBS += ../AmbiqSDK/mcu/$(PART)/hal/mcu/gcc/bin/libam_hal.a

CFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
CFLAGS+= -ffunction-sections -fdata-sections -fno-omit-frame-pointer
CFLAGS+= -MMD -MP -std=c99 -Wall -g -O0
CFLAGS+= $(DEFINES)
CFLAGS+= $(INCLUDES)

LFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
LFLAGS+= -nostartfiles -static
LFLAGS+= -Wl,--gc-sections,--entry,Reset_Handler,-Map,$(CONFIG)/$(TARGET).map
LFLAGS+= -Wl,--start-group -lm -lc -lgcc -lnosys $(LIBS) -Wl,--end-group

CPFLAGS = -Obinary
ODFLAGS = -S

$(CONFIG):
	@mkdir -p $@

$(CONFIG)/%.o: %.c $(CONFIG)/%.d
	@echo " Compiling $<" ;\
	$(CC) -c $(CFLAGS) $< -o $@

$(CONFIG)/%.o: %.s $(CONFIG)/%.d
	@echo " Assembling $<" ;\
	$(CC) -c $(CFLAGS) $< -o $@

$(CONFIG)/%.axf: $(OBJS) $(LIBS)
	@echo " Linking $@" ;\
	$(CC) -Wl,-T,$(LINKER_FILE) -o $@ $(OBJS) $(LFLAGS)

$(CONFIG)/%.bin: $(CONFIG)/%.axf
	@echo " Copying $@..." ;\
	$(CP) $(CPFLAGS) $< $@ ;\
	$(OD) $(ODFLAGS) $< > $(CONFIG)/$(TARGET).lst
	@$(SIZE) $(OBJS) $(LIBS) $(CONFIG)/$(TARGET).axf >$(CONFIG)/$(TARGET).size

clean:
	@echo "Cleaning..." ;\
	$(RM) -rf $(CONFIG)
$(CONFIG)/%.d: ;

# Include JTag flashing Makefile
include ../Jtag.mk

# Automatically include any generated dependencies
-include $(DEPS)

endif
