##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [2.24.1] date: [Tue Sep 19 16:02:48 MSK 2017]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = EO81_ControllerUOV


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# source path
SOURCES_DIR =  
SOURCES_DIR += src
SOURCES_DIR += src/usrlib
SOURCES_DIR += mcu_files/spl
SOURCES_DIR += mcu_files/spl/src
SOURCES_DIR += mcu_files/CMSIS
 
# firmware library path
PERIFLIB_PATH = 

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  
C_SOURCES += ./src/main.c
C_SOURCES += ./src/menu.c
C_SOURCES += ./src/keyboard.c
C_SOURCES += ./src/func.c
C_SOURCES += ./src/usrlib/display.c
C_SOURCES += ./src/usrlib/MBMaster.c
C_SOURCES += ./src/usrlib/MBSlave.c
C_SOURCES += ./src/usrlib/crc.c
C_SOURCES += ./src/usrlib/eeprom.c
C_SOURCES += ./src/usrlib/stm32f1_deb.c
C_SOURCES += ./mcu_files/CMSIS/system_stm32f10x.c
C_SOURCES += ./mcu_files/spl/src/misc.c
C_SOURCES += ./mcu_files/spl/src/stm32f10x_usart.c
C_SOURCES += ./mcu_files/spl/src/stm32f10x_gpio.c
C_SOURCES += ./mcu_files/spl/src/stm32f10x_rcc.c
C_SOURCES += ./mcu_files/spl/src/stm32f10x_rtc.c
C_SOURCES += ./mcu_files/spl/src/stm32f10x_pwr.c
C_SOURCES += ./mcu_files/spl/src/stm32f10x_flash.c




# C++ sourses
CPP_SOURCES =


# ASM sources
ASM_SOURCES = mcu_files/startup_stm32f103xb.s


######################################
# firmware library
######################################
PERIFLIB_SOURCES = 


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
#PREFIX = /home/ap/Code/gcc-arm-none-eabi-6-2017-q2-update/bin/arm-none-eabi-
CPP = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  
# discovery
# C_DEFS += -DSTM32F051x8
# EO78
#C_DEFS += -DSTM32F030x6


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES += -Isrc
C_INCLUDES += -Isrc/usrlib
C_INCLUDES += -Imcu_files/spl
C_INCLUDES += -Imcu_files/spl/inc
C_INCLUDES += -Imcu_files/CMSIS


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -std=c99

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = 
# discovery
#LDSCRIPT += linker/STM32F051R8Tx_FLASH.ld
# EO78
LDSCRIPT += mcu_files/STM32F103RBTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CPP) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@	

flash_stlink:  $(BUILD_DIR)/$(TARGET).bin
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000

erase_stlink:
	st-flash erase

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***