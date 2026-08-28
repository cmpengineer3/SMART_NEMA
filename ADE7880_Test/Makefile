# ═══════════════════════════════════════════════════════════════════════════
#  Makefile — ADE7880 Read Test (firmware diagnostik)
#  Target : STM32F103RGTx, HSE + PLLx9 = 72 MHz
#
#  Pemakaian:
#     make            → build, hasilnya di build/ADE7880_Test.hex
#     make clean      → hapus hasil build
#     make size       → tampilkan pemakaian flash/RAM
#
#  Butuh arm-none-eabi-gcc di PATH.
#  (Kalau memakai STM32CubeIDE, Makefile ini TIDAK diperlukan — cukup
#   File > Import > Existing Projects, lalu Build.)
# ═══════════════════════════════════════════════════════════════════════════

TARGET  = ADE7880_Test
BUILD   = build

PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
CP      = $(PREFIX)objcopy
SZ      = $(PREFIX)size

CPU     = -mcpu=cortex-m3
MCU     = $(CPU) -mthumb

C_DEFS  = -DUSE_HAL_DRIVER -DSTM32F103xG

C_INCLUDES = \
  -ICore/Inc \
  -IDrivers/STM32F1xx_HAL_Driver/Inc \
  -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
  -IDrivers/CMSIS/Include

OPT      = -O2 -g
CFLAGS   = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall \
           -ffunction-sections -fdata-sections -fno-common -std=gnu11

LDSCRIPT = STM32F103RGTX_FLASH.ld
# -u _printf_float : aktifkan dukungan %f pada printf newlib-nano
LDFLAGS  = $(MCU) -specs=nano.specs -specs=nosys.specs -u _printf_float \
           -T$(LDSCRIPT) -Wl,-Map=$(BUILD)/$(TARGET).map,--cref \
           -Wl,--gc-sections

C_SOURCES  = $(wildcard Core/Src/*.c)
C_SOURCES += $(wildcard Drivers/STM32F1xx_HAL_Driver/Src/*.c)
ASM_SOURCES = Core/Startup/startup_stm32f103rgtx.s

OBJECTS  = $(addprefix $(BUILD)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin size

$(BUILD)/%.o: %.c Makefile | $(BUILD)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD)/%.o: %.s Makefile | $(BUILD)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(CP) -O ihex $< $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(CP) -O binary -S $< $@

$(BUILD):
	mkdir -p $@

size: $(BUILD)/$(TARGET).elf
	@echo ""
	@$(SZ) $<

clean:
	rm -rf $(BUILD)

.PHONY: all clean size
