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
DEFINES += -DDISABLEFLOAT16
DEFINES += -DTF_LITE_STATIC_MEMORY
DEFINES += -DTF_LITE_DISABLE_X86_NEON
DEFINES += -DCMSIS_NN
DEFINES += -DKERNELS_OPTIMIZED_FOR_SPEED
DEFINES += -DTF_LITE_STRIP_ERROR_STRINGS
DEFINES += -DCPU_M4=1
DEFINES += -DCMSIS_DEVICE_ARM_CORTEX_M_XX_HEADER_FILE=\""ARMCM4".h\"
DEFINES += -D"ARMCM4"
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
INCLUDES += -I../AmbiqSDK/CMSIS/ARM/PrivateInclude
INCLUDES += -I../AmbiqSDK/CMSIS/ARM/Include
INCLUDES += -I../AmbiqSDK/CMSIS/ARM
INCLUDES += -I../AmbiqSDK/devices
INCLUDES += -I../AmbiqSDK/utils
INCLUDES += -I../src/ai
INCLUDES += -I../src/app
INCLUDES += -I../src/boards
INCLUDES += -I../src/boards/rev$(REVISION)
INCLUDES += -I../src/external
INCLUDES += -I../src/external/fatfs
INCLUDES += -I../src/external/flatbuffers/include
INCLUDES += -I../src/external/gemmlowp
INCLUDES += -I../src/external/segger
INCLUDES += -I../src/external/ruy
INCLUDES += -I../src/external/tflite-micro
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/c
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/core
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/core/api
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/core/c
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/kernels
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/kernels/internal
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/kernels/internal/optimized
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/kernels/internal/reference
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/micro
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/micro/arena_allocator
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/micro/cortex_m_generic
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/micro/kernels
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/micro/memory_planner
INCLUDES += -I../src/external/tflite-micro/tensorflow/lite/micro/tflite_bridge
INCLUDES += -I../src/peripherals/include

VPATH  = ../AmbiqSDK/bsp/$(BSP)/linker
VPATH += ../AmbiqSDK/mcu/$(PART)/hal
VPATH += ../AmbiqSDK/mcu/$(PART)/hal/mcu
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/ActivationFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/BasicMathFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/BayesFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/CommonTables
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/ComplexMathFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/ConcatenationFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/ControllerFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/ConvolutionFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/DistanceFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/FastMathFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/FilteringFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/FullyConnectedFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/InterpolationFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/LSTMFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/MatrixFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/NNSupportFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/PadFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/PoolingFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/QuaternionMathFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/ReshapeFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/SoftmaxFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/StatisticsFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/SupportFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/SVDFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/SVMFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/TransformFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/TransposeFunctions
VPATH += ../AmbiqSDK/CMSIS/ARM/Source/WindowFunctions
VPATH += ../AmbiqSDK/devices
VPATH += ../AmbiqSDK/utils
VPATH += ../src/ai
VPATH += ../src/app
VPATH += ../src/boards
VPATH += ../src/boards/rev$(REVISION)
VPATH += ../src/external/fatfs
VPATH += ../src/external/segger
VPATH += ../src/external/tflite-micro/tensorflow/lite/micro/tflite_bridge
VPATH += ../src/external/tflite-micro/tensorflow/lite/micro/memory_planner
VPATH += ../src/external/tflite-micro/tensorflow/lite/micro/kernels/cmsis_nn
VPATH += ../src/external/tflite-micro/tensorflow/lite/micro/kernels
VPATH += ../src/external/tflite-micro/tensorflow/lite/micro/cortex_m_generic
VPATH += ../src/external/tflite-micro/tensorflow/lite/micro/arena_allocator
VPATH += ../src/external/tflite-micro/tensorflow/lite/micro
VPATH += ../src/external/tflite-micro/tensorflow/lite/kernels/internal
VPATH += ../src/external/tflite-micro/tensorflow/lite/core/c
VPATH += ../src/external/tflite-micro/tensorflow/lite/core/api
VPATH += ../src/external/tflite-micro/tensorflow/lite/core
VPATH += ../src/external/tflite-micro/tensorflow/lite/c
VPATH += ../src/external/tflite-micro/tensorflow/lite/
VPATH += ../src/external/tflite-micro/tensorflow/compiler/mlir/lite/schema
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

SRC += CommonTables.c
SRC += TransformFunctions.c
SRC += $(foreach SOURCE,$(wildcard ../AmbiqSDK/CMSIS/ARM/Source/ConvolutionFunctions/*.c),$(patsubst %.c,%.c,$(notdir $(SOURCE))))
SRC += $(foreach SOURCE,$(wildcard ../AmbiqSDK/CMSIS/ARM/Source/FullyConnectedFunctions/*.c),$(patsubst %.c,%.c,$(notdir $(SOURCE))))
SRC += $(foreach SOURCE,$(wildcard ../AmbiqSDK/CMSIS/ARM/Source/NNSupportFunctions/*.c),$(patsubst %.c,%.c,$(notdir $(SOURCE))))
SRC += $(foreach SOURCE,$(wildcard ../AmbiqSDK/CMSIS/ARM/Source/TransposeFunctions/*.c),$(patsubst %.c,%.c,$(notdir $(SOURCE))))

SRC += flatbuffer_utils.cc
SRC += memory_helpers.cc
SRC += micro_allocation_info.cc
SRC += micro_allocator.cc
SRC += micro_context.cc
SRC += micro_interpreter_context.cc
SRC += micro_interpreter_graph.cc
SRC += micro_interpreter.cc
SRC += micro_op_resolver.cc
SRC += micro_resource_variable.cc
SRC += micro_utils.cc
SRC += system_setup.cc
SRC += non_persistent_arena_buffer_allocator.cc
SRC += persistent_arena_buffer_allocator.cc
SRC += single_arena_buffer_allocator.cc
SRC += greedy_memory_planner.cc
SRC += linear_memory_planner.cc
SRC += non_persistent_buffer_planner_shim.cc
SRC += flatbuffer_conversions_bridge.cc
SRC += micro_error_reporter.cc
SRC += kernel_util.cc
SRC += kernel_util2.cc
SRC += tensor_utils.cc
SRC += common.cc
SRC += common2.cc
SRC += portable_tensor_utils.cc
SRC += portable_tensor_utils2.cc
SRC += tensor_ctypes.cc
SRC += runtime_shape.cc
SRC += quantization_util.cc
SRC += flatbuffer_conversions.cc
SRC += schema_utils.cc
SRC += conv.cc
SRC += conv_common.cc
SRC += depthwise_conv.cc
SRC += depthwise_conv_common.cc
SRC += dequantize.cc
SRC += dequantize_common.cc
SRC += fully_connected.cc
SRC += fully_connected_common.cc
SRC += leaky_relu.cc
SRC += leaky_relu_common.cc
SRC += pad.cc
SRC += pad_common.cc
SRC += pooling.cc
SRC += pooling_common.cc
SRC += prelu.cc
SRC += prelu_common.cc
SRC += quantize.cc
SRC += quantize_common.cc
SRC += reduce.cc
SRC += reduce_common.cc
SRC += reshape.cc
SRC += reshape_common.cc
SRC += transpose.cc
SRC += transpose_common.cc

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

CFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
CFLAGS+= -ffunction-sections -fdata-sections -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-exceptions -fmessage-length=0 -funsigned-char
CFLAGS+= -fno-omit-frame-pointer -MMD -MP -Wall -g -O0
CFLAGS+= $(DEFINES)
CFLAGS+= $(INCLUDES)

LFLAGS = -mthumb -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FABI)
LFLAGS+= -nostartfiles -static -fno-exceptions
LFLAGS+= -Wl,--gc-sections,--entry,Reset_Handler,-Map,$(CONFIG)/$(TARGET).map
LFLAGS+= -Wl,--start-group -lm -lc -lgcc -lnosys $(LIBS) -Wl,--end-group
LFLAGS+= -Wl,--print-memory-usage

CPFLAGS = -Obinary
ODFLAGS = -S

$(CONFIG):
	@mkdir -p $@

# TFLite defines a number of source files with the same name - workaround here
$(CONFIG)/common.o: ../src/external/tflite-micro/tensorflow/lite/kernels/internal/common.cc $(CONFIG)/common.d
	@echo " Compiling $<" ;\
	$(CPP) -c -std=c++17 $(CFLAGS) -fno-rtti -fno-threadsafe-statics $< -o $@
$(CONFIG)/common2.o: ../src/external/tflite-micro/tensorflow/lite/core/c/common.cc $(CONFIG)/common2.d
	@echo " Compiling $<" ;\
	$(CPP) -c -std=c++17 $(CFLAGS) -fno-rtti -fno-threadsafe-statics $< -o $@
$(CONFIG)/kernel_util.o: ../src/external/tflite-micro/tensorflow/lite/kernels/kernel_util.cc $(CONFIG)/kernel_util.d
	@echo " Compiling $<" ;\
	$(CPP) -c -std=c++17 $(CFLAGS) -fno-rtti -fno-threadsafe-statics $< -o $@
$(CONFIG)/kernel_util2.o: ../src/external/tflite-micro/tensorflow/lite/micro/kernels/kernel_util.cc $(CONFIG)/kernel_util2.d
	@echo " Compiling $<" ;\
	$(CPP) -c -std=c++17 $(CFLAGS) -fno-rtti -fno-threadsafe-statics $< -o $@
$(CONFIG)/portable_tensor_utils.o: ../src/external/tflite-micro/tensorflow/lite/kernels/internal/portable_tensor_utils.cc $(CONFIG)/portable_tensor_utils.d
	@echo " Compiling $<" ;\
	$(CPP) -c -std=c++17 $(CFLAGS) -fno-rtti -fno-threadsafe-statics $< -o $@
$(CONFIG)/portable_tensor_utils2.o: ../src/external/tflite-micro/tensorflow/lite/kernels/internal/reference/portable_tensor_utils.cc $(CONFIG)/portable_tensor_utils2.d
	@echo " Compiling $<" ;\
	$(CPP) -c -std=c++17 $(CFLAGS) -fno-rtti -fno-threadsafe-statics $< -o $@

$(CONFIG)/%.o: %.cc $(CONFIG)/%.d
	@echo " Compiling $<" ;\
	$(CPP) -c -std=c++17 $(CFLAGS) -fno-rtti -fno-threadsafe-statics $< -o $@

$(CONFIG)/%.o: %.c $(CONFIG)/%.d
	@echo " Compiling $<" ;\
	$(CC) -c -std=c17 $(CFLAGS) $< -o $@

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
