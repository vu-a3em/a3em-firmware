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
DEFINES += -DARM_MATH_NEON
DEFINES += -Dgcc

LINKER_FILE := ../AmbiqSDK/bsp/$(BSP)/linker/a3em.ld
STARTUP_FILE := ../AmbiqSDK/bsp/$(BSP)/linker/startup_gcc.c

#### Required Executables ####
CC = $(TOOLCHAIN)-gcc
GCC = $(TOOLCHAIN)-gcc
CPP = $(TOOLCHAIN)-g++
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
INCLUDES += -I../src/ai
INCLUDES += -I../src/app
INCLUDES += -I../src/boards
INCLUDES += -I../src/boards/rev$(REVISION)
INCLUDES += -I../src/external
INCLUDES += -I../src/external/fatfs
INCLUDES += -I../src/external/segger
INCLUDES += -I../src/external/tensorflow
INCLUDES += -I../src/external/tensorflow/third_party
INCLUDES += -I../src/external/tensorflow/third_party/flatbuffers/include
INCLUDES += -I../src/external/tensorflow/third_party/gemmlowp
INCLUDES += -I../src/peripherals/include

VPATH  = ../AmbiqSDK/bsp/$(BSP)/linker
VPATH += ../AmbiqSDK/mcu/$(PART)/hal
VPATH += ../AmbiqSDK/mcu/$(PART)/hal/mcu
VPATH += ../AmbiqSDK/devices
VPATH += ../AmbiqSDK/utils
VPATH += ../src/ai
VPATH += ../src/app
VPATH += ../src/boards
VPATH += ../src/boards/rev$(REVISION)
VPATH += ../src/external/fatfs
VPATH += ../src/external/segger
VPATH += ../src/peripherals/src
VPATH += ./peripherals
VPATH += ./app
VPATH += ./ai

SRC =
SRC += am_devices_led.c
SRC += am_util_delay.c
SRC += am_util_stdio.c
SRC += am_util_string.c
SRC += ff.c
SRC += ffunicode.c
SRC += startup_gcc.c
SRC += SEGGER_RTT.c

SRC += ai.cc
SRC += fft.c
SRC += mfcc.c

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

# Uncomment these lines to debug AmbiqSDK files
#SRC += $(foreach SOURCE,$(wildcard ../AmbiqSDK/mcu/$(PART)/hal/*.c),$(patsubst %.c,%.c,$(notdir $(SOURCE))))
#SRC += $(foreach SOURCE,$(wildcard ../AmbiqSDK/mcu/$(PART)/hal/mcu/*.c),$(patsubst %.c,%.c,$(notdir $(SOURCE))))

.PHONY: all program clean ai app_magnet_no_gps audio audio_continuous battery comparator configuration digipot fft full imu led logging magnet mfcc power_off rtc_set rtc storage system vhf
.PRECIOUS: $(CONFIG)/%.axf
.SECONDEXPANSION:

all:
	$(error Make targets include: ai app_magnet_no_gps audio audio_continuous battery comparator configuration digipot fft full imu led logging magnet mfcc power_off rtc_set rtc storage system vhf)

program:
ifneq ($(BUILT_BY), eclipse)
	printf "r\n" > $(CONFIG)/flash.jlink
	printf "loadfile $(CONFIG)/$(TARGET).bin $(FLASH_START)\nr\ng\nexit\n" >> $(CONFIG)/flash.jlink
	$(JLINK) $(JLINK_FLAGS) $(CONFIG)/flash.jlink
endif

ai: TARGET = TestAI
ai: SRC += test_ai.c
ai: $(CONFIG) $(CONFIG)/test_ai.o $(CONFIG)/$$(TARGET).bin program

app_magnet_no_gps: TARGET = TestAppMagnetNoGps
app_magnet_no_gps: SRC += test_app_magnet_no_gps.c
app_magnet_no_gps: $(CONFIG) $(CONFIG)/test_app_magnet_no_gps.o $(CONFIG)/$$(TARGET).bin program

audio: TARGET = TestAudio
audio: DEFINES += -DENABLE_AUDIO_DL=1
audio: SRC += test_audio.c
audio: $(CONFIG) $(CONFIG)/test_audio.o $(CONFIG)/$$(TARGET).bin program

audio_continuous: TARGET = TestAudioContinuous
audio_continuous: DEFINES += -DENABLE_AUDIO_DL=1
audio_continuous: SRC += test_audio_continuous.c
audio_continuous: $(CONFIG) $(CONFIG)/test_audio_continuous.o $(CONFIG)/$$(TARGET).bin program

battery: TARGET = TestBattery
battery: SRC += test_battery.c
battery: $(CONFIG) $(CONFIG)/test_battery.o $(CONFIG)/$$(TARGET).bin program

comparator: TARGET = TestComparator
comparator: SRC += test_comparator.c
comparator: $(CONFIG) $(CONFIG)/test_comparator.o $(CONFIG)/$$(TARGET).bin program

configuration: TARGET = TestConfiguration
configuration: SRC += test_configuration.c
configuration: $(CONFIG) $(CONFIG)/test_configuration.o $(CONFIG)/$$(TARGET).bin program

digipot: TARGET = TestDigipot
digipot: SRC += test_digipot.c
digipot: $(CONFIG) $(CONFIG)/test_digipot.o $(CONFIG)/$$(TARGET).bin program

fft: TARGET = TestFFT
fft: SRC += test_fft.c
fft: $(CONFIG) $(CONFIG)/test_fft.o $(CONFIG)/$$(TARGET).bin program

full: TARGET = TestFull
full: SRC += active_main.c main.c
full: $(CONFIG) $(CONFIG)/active_main.o $(CONFIG)/main.o $(CONFIG)/$$(TARGET).bin program

imu: TARGET = TestIMU
imu: SRC += test_imu.c
imu: $(CONFIG) $(CONFIG)/test_imu.o $(CONFIG)/$$(TARGET).bin program

led: TARGET = TestLED
led: SRC += test_led.c
led: $(CONFIG) $(CONFIG)/test_led.o $(CONFIG)/$$(TARGET).bin program

logging: TARGET = TestLogging
logging: SRC += test_logging.c
logging: $(CONFIG) $(CONFIG)/test_logging.o $(CONFIG)/$$(TARGET).bin program

magnet: TARGET = TestMagnet
magnet: SRC += test_magnet.c
magnet: $(CONFIG) $(CONFIG)/test_magnet.o $(CONFIG)/$$(TARGET).bin program

mfcc: TARGET = TestMFCC
mfcc: SRC += test_mfcc.c
mfcc: $(CONFIG) $(CONFIG)/test_mfcc.o $(CONFIG)/$$(TARGET).bin program

power_off: TARGET = TestPowerOff
power_off: SRC += test_power_off.c
power_off: $(CONFIG) $(CONFIG)/test_power_off.o $(CONFIG)/$$(TARGET).bin program

rtc_set: TARGET = TestRTCSet
rtc_set: SRC += test_rtc.c
rtc_set: CFLAGS += -DSET_RTC
rtc_set: $(CONFIG) $(CONFIG)/test_rtc.o $(CONFIG)/$$(TARGET).bin program

rtc: TARGET = TestRTC
rtc: SRC += test_rtc.c
rtc: $(CONFIG) $(CONFIG)/test_rtc.o $(CONFIG)/$$(TARGET).bin program

storage: TARGET = TestStorage
storage: SRC += test_storage.c
storage: $(CONFIG) $(CONFIG)/test_storage.o $(CONFIG)/$$(TARGET).bin program

system: TARGET = TestSystem
system: SRC += test_system.c
system: $(CONFIG) $(CONFIG)/test_system.o $(CONFIG)/$$(TARGET).bin program

vhf: TARGET = TestVHF
vhf: SRC += test_vhf.c
vhf: $(CONFIG) $(CONFIG)/test_vhf.o $(CONFIG)/$$(TARGET).bin program

CSRC   = $(filter %.c,$(SRC))
CPPSRC = $(filter %.cc,$(SRC))
ASRC   = $(filter %.s,$(SRC))

OBJS = $(CSRC:%.c=$(CONFIG)/%.o)
OBJS+= $(CPPSRC:%.cc=$(CONFIG)/%.o)
OBJS+= $(ASRC:%.s=$(CONFIG)/%.o)

DEPS = $(CSRC:%.c=$(CONFIG)/%.d)
DEPS+= $(CPPSRC:%.cc=$(CONFIG)/%.d)
DEPS+= $(ASRC:%.s=$(CONFIG)/%.d)

LIBS  = ../AmbiqSDK/bsp/$(BSP)/gcc/bin/libam_bsp.a
LIBS += ../AmbiqSDK/mcu/$(PART)/hal/mcu/gcc/bin/libam_hal.a
LIBS += ../AmbiqSDK/CMSIS/ARM/Lib/ARM/libarm_cortexM4lf_math.a
LIBS += ../src/external/tensorflow/lib/libtensorflow-microlite-cm4-gcc-release.a

CFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
CFLAGS+= -ffunction-sections -fdata-sections -fno-omit-frame-pointer -fno-exceptions
CFLAGS+= -MMD -MP -Wall -g -O0
CFLAGS+= $(DEFINES)
CFLAGS+= $(INCLUDES)

LFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
LFLAGS+= -nostartfiles -static -fno-exceptions -z execstack
LFLAGS+= -Wl,--gc-sections,--entry,Reset_Handler,-Map,$(CONFIG)/$(TARGET).map
LFLAGS+= -Wl,--start-group -lm -lc -lgcc -lnosys -Wl,--no-whole-archive,--no-wchar-size-warning $(LIBS) -Wl,--end-group

CPFLAGS = -Obinary
ODFLAGS = -S

$(CONFIG):
	@mkdir -p $@

$(CONFIG)/%.o: %.cc $(CONFIG)/%.d
	@echo " Compiling $<" ;\
	$(CPP) -c $(CFLAGS) -fno-rtti -fno-threadsafe-statics -fno-unwind-tables $< -o $@

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
