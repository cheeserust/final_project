# Ubuntu Makefile for STM32F411CEU6 + MCP2515 Board1 CAN-only 4-axis trajectory test firmware
TARGET      = arm-none-eabi
CC          = $(TARGET)-gcc
OBJCOPY     = $(TARGET)-objcopy
OBJDUMP     = $(TARGET)-objdump

OUT_FILE_NAME = rom_0x08000000
LDS_FILE_NAME = rom_0x08000000.lds

# Board1-only firmware. BOARD_ROLE is kept for compatibility with old build scripts.
BOARD_ROLE ?= 1

CSRC = $(wildcard *.c)
ASRC = $(wildcard *.s)
OBJS = $(CSRC:.c=.o) $(ASRC:.s=.o)

CFLAGS  = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DSTM32F411xE
CFLAGS += -I. -std=gnu99 -O0 -Wall -g3 -c
CFLAGS += -DBOARD_ROLE=$(BOARD_ROLE)
CFLAGS += -fno-builtin -funsigned-char -fno-strict-aliasing -fno-common
CFLAGS += -fno-unwind-tables -fno-asynchronous-unwind-tables

ASFLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -g3 -c

LDFLAGS  = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LDFLAGS += --specs=nano.specs --specs=nosys.specs -u _printf_float
LDFLAGS += -nostartfiles -ffreestanding
LDFLAGS += -Wl,-Map=$(OUT_FILE_NAME).map -Wl,--cref -Wl,-EL -T $(LDS_FILE_NAME)

LIBS = -lc -lgcc

all: $(OUT_FILE_NAME).bin

$(OUT_FILE_NAME).bin: $(OUT_FILE_NAME).elf
	$(OBJCOPY) $< $@ -O binary
	$(OBJDUMP) -x -D $< > __dump.txt
	$(OBJDUMP) -x -D -S $< > __dump_all.txt

$(OUT_FILE_NAME).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

%.o: %.c $(wildcard *.h)
	$(CC) $(CFLAGS) $< -o $@

%.o: %.s
	$(CC) $(ASFLAGS) $< -o $@

clean:
	rm -f *.o $(OUT_FILE_NAME).bin $(OUT_FILE_NAME).elf $(OUT_FILE_NAME).map __dump.txt __dump_all.txt

flash-stlink: $(OUT_FILE_NAME).bin
	st-flash write ./$(OUT_FILE_NAME).bin 0x08000000

reset:
	st-flash reset
