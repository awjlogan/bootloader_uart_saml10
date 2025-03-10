##############################################################################
BUILD = build
BIN = saml10BL
##############################################################################
.PHONY: all directory clean size

# Path to toolchain, e.g. /path/to/bin/ Leave empty if already on path.
TC_PATH =
CC = $(TC_PATH)arm-none-eabi-gcc
OBJCOPY = $(TC_PATH)arm-none-eabi-objcopy
SIZE = $(TC_PATH)arm-none-eabi-size

ifeq ($(OS), Windows_NT)
  MKDIR = gmkdir
else
  MKDIR = mkdir
endif

CFLAGS += -W -Wall -Wextra -Wpedantic --std=gnu99 -Os -g3
CFLAGS += -nostdlib -flto -ffreestanding
CFLAGS += -fno-diagnostics-show-caret -fno-common
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -Wuninitialized
CFLAGS += -Wshadow -Wdouble-promotion -Wundef
CFLAGS += -mcpu=cortex-m23 -mthumb
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m23 -mthumb -nostartfiles
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -Wl,--script=./linker/saml10_bl.ld

INCLUDES += \
  -I./include

SRCS += $(wildcard ./src/*.c)

DEFINES += \
  -D__SAML10E15A__ \
  -DDONT_USE_CMSIS_INIT \
  -DF_CPU=16000000 \
  -D__ARM_FEATURE_DSP=0

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@$(MKDIR) -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)