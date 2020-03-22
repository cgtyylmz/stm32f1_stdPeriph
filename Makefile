PROJECT = main

# MCU
MCU = cortex-m3

# C definations
DEFS = -DSTM32F1 -DSTM32F10X_MD -DSTM32F103xB -DUSE_STDPERIPH_DRIVER
# Debug level
DBG = -g3

# Optimisation level
OPT = #-Os

#####################################################################
#                              TOOLS                                #
#####################################################################

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
GDB = $(PREFIX)gdb
CP = $(PREFIX)objcopy
AS = $(PREFIX)gcc -x assembler-with-cpp
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#####################################################################
#                      DEFAULT DIRECTORIES                          #
#####################################################################
# Source Files
SRC = ./src

# Object Files
OBJ_DIR = ./obj

# Binary Files
BUILD_DIR = ./build

# Include directories
INCDIR = ./inc ./driver/inc

# Library directories
LIBDIR = ./lib

# Linker script file
LINKER = ./linker/STM32F103XB_FLASH.ld

# Startup assembly source file
STARTUP = ./startup/startup_stm32f103xb.s

# Add all source files from src/ directory
SRC_FILES = $(wildcard $(SRC)/*.c)


# Header file
INC = $(patsubst %,-I%, $(INCDIR))

# Library files
LIB = $(patsubst %,-L%, $(LIBDIR))

#####################################################################
#                  User/Aplication Source Files                     #
#####################################################################

# Driver Source Files
SRC_FILES += driver/src/stm32f10x_gpio.c
SRC_FILES += driver/src/stm32f10x_rcc.c
#SRC_FILES += driver/src/stm32f10x_adc.c


OBJ_FILES = $(addprefix $(OBJ_DIR)/,$(notdir $(SRC_FILES:.c=.o)))
vpath %.c $(sort $(dir $(SRC_FILES)))
# list of ASM program objects
OBJ_FILES += $(addprefix $(OBJ_DIR)/,$(notdir $(STARTUP:.s=.o)))
vpath %.s $(sort $(dir $(STARTUP)))

#####################################################################
#                          Flags                                    #
#####################################################################

COMFLAGS = -mcpu=$(MCU) -mthumb -mfloat-abi=soft
ASFLAGS = $(COMFLAGS) $(DBG)
CPFLAGS = $(COMFLAGS) $(OPT) $(DEFS) $(DBG)   -Wall -fmessage-length=0 -ffunction-sections
LDFLAGS = $(COMFLAGS) -T$(LINKER) -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map -Wl,--gc-sections $(LIB)

#####################################################################
#                        Makefile Rules                             #
#####################################################################

all: $(OBJ_FILES) $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).hex $(BUILD_DIR)/$(PROJECT).bin
	$(PREFIX)size $(BUILD_DIR)/$(PROJECT).elf

$(OBJ_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CPFLAGS) -I . $(INC) $< -o $@

$(OBJ_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(CC) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(PROJECT).elf: $(OBJ_FILES) Makefile | $(BUILD_DIR)
	$(CC) $(OBJ_FILES) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)
	mkdir -p $(OBJ_DIR)

flash: $(BUILD_DIR)/$(PROJECT).bin
	st-flash write $(BUILD_DIR)/$(PROJECT).bin 0x8000000

erase:
	st-flash erase

clean:
	rm -fR $(BUILD_DIR) $(OBJ_DIR)
