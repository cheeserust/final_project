TOOLCHAIN ?= /opt/st/stm32cubeide_2.1.1/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin
CC := $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY := $(TOOLCHAIN)/arm-none-eabi-objcopy
SIZE := $(TOOLCHAIN)/arm-none-eabi-size

TARGET := pb3_high_test
BUILD_DIR := build
ROOT := ..

CFLAGS := \
	-mcpu=cortex-m4 \
	-std=gnu11 \
	-g3 \
	-O0 \
	-DSTM32F411xE \
	-DHSE_VALUE=25000000 \
	-DHSE_STARTUP_TIMEOUT=100 \
	-DLSE_STARTUP_TIMEOUT=5000 \
	-DLSE_VALUE=32768 \
	-DEXTERNAL_CLOCK_VALUE=12288000 \
	-DHSI_VALUE=16000000 \
	-DLSI_VALUE=32000 \
	-DVDD_VALUE=3300 \
	-DPREFETCH_ENABLE=1 \
	-DINSTRUCTION_CACHE_ENABLE=1 \
	-DDATA_CACHE_ENABLE=1 \
	-I$(ROOT)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
	-I$(ROOT)/Drivers/CMSIS/Include \
	-ffunction-sections \
	-fdata-sections \
	-Wall \
	-Wextra \
	--specs=nano.specs \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mthumb

LDFLAGS := \
	-mcpu=cortex-m4 \
	-T$(ROOT)/STM32F411CEUX_FLASH.ld \
	--specs=nosys.specs \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
	-Wl,--gc-sections \
	-static \
	--specs=nano.specs \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mthumb \
	-Wl,--start-group -lc -lm -Wl,--end-group

SRCS := \
	main.c \
	$(ROOT)/Core/Src/system_stm32f4xx.c \
	$(ROOT)/Core/Startup/startup_stm32f411ceux.s

OBJS := \
	$(BUILD_DIR)/main.o \
	$(BUILD_DIR)/system_stm32f4xx.o \
	$(BUILD_DIR)/startup_stm32f411ceux.o

.PHONY: all clean flash

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/main.o: main.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/system_stm32f4xx.o: $(ROOT)/Core/Src/system_stm32f4xx.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/startup_stm32f411ceux.o: $(ROOT)/Core/Startup/startup_stm32f411ceux.s | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c -x assembler-with-cpp $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)
	$(SIZE) $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

flash: all
	st-flash --reset write $(BUILD_DIR)/$(TARGET).bin 0x08000000

clean:
	rm -rf $(BUILD_DIR)
