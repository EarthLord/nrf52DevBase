#### Toolchain commands ####
GCC_INSTALL_ROOT	:= /usr/local/gcc-arm-none-eabi-4_9-2015q2
GCC_VERSION		:= 4.9.3
GCC_PREFIX		:= arm-none-eabi

CC      := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-gcc
AS      := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-as
AR      := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-ar -r
LD      := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-ld
NM      := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-nm
OBJDUMP := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-objdump
OBJCOPY := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-objcopy
GDB     := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-gdb
SIZE    := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-size

MK	:= mkdir -p
RM	:= rm -rf

### General Variables ####
OUTPUT_NAME		= main
OUTPUT_DIR		= build
OBJ_DIR			= obj
SRC_DIR			= src
LINK_DIR		= link

### Device related stuff ###
BOARD			:= BOARD_PCA10036
CPU				:= cortex-m4
DEVICE 			:= NRF52
DEVICESERIES 	:= nrf52

### Programmer ###
JLINK_DIR 			= /opt/SEGGER/JLink/
JLINK 				= $(JLINK_DIR)JLinkExe
JLINKGDBSERVER		= $(JLINK_DIR)JLinkGDBServer
JLINKDEVICE 		= nrf52
NVMC_CONFIG_ADRS	= 4001E504

FLASH_START_ADDRESS	= 0
#SOFTDEVICE 			= {path to the softdevice}.hex

# Include directories
INCLUDEDIRS	 = $(shell find include -type d)
INCLUDEDIRS	+= $(GCC_INSTALL_ROOT)/lib/gcc/$(GCC_PREFIX)/$(GCC_VERSION)/include/
INCLUDEDIRS	+= $(GCC_INSTALL_ROOT)/lib/gcc/$(GCC_PREFIX)/$(GCC_VERSION)/include-fixed/
INCLUDEDIRS	+= $(GCC_INSTALL_ROOT)/$(GCC_PREFIX)/include/ 
### Source files ###
# Project Source
C_SRC  = $(shell find src -name *.c | awk -F/ '{print $$NF}')

### Assembly source files
ASSEMBLY_SRC = gcc_startup_$(DEVICESERIES).S

### Compiler related stuff ###
#Small size
CFLAGS	= -O2
#info for the debugger
CFLAGS  += -ggdb
CFLAGS	+= -mcpu=$(CPU)
CFLAGS	+= -mthumb
CFLAGS	+= -mabi=aapcs
CFLAGS  += -mfloat-abi=hard		#Which floating point ABI to use
CFLAGS  += -mfpu=fpv4-sp-d16	#The type of FPU we are using
CFLAGS	+= --std=gnu99
CFLAGS	+= -Wall					#Enable All Warnings
#CFLAGS  += -ffunction-sections      #Create a separate function section
#CFLAGS  += -fdata-sections          #Create a separate data section
CFLAGS	+= -D$(DEVICE)
CFLAGS	+= -D$(BOARD)
CFLAGS  += -DCONFIG_GPIO_AS_PINRESET
CFLAGS	+= $(patsubst %,-I%, $(INCLUDEDIRS))

### Linker related stuff ###
#LDDIRS 	 = $(GCC_INSTALL_ROOT)/$(GCC_PREFIX)/lib/armv6-m
#LDDIRS 	+= $(GCC_INSTALL_ROOT)/lib/gcc/$(GCC_PREFIX)/$(GCC_VERSION)/armv6-m
LDDIRS		+= $(LINK_DIR)

LD_SCRIPT = $(LINK_DIR)/nrf52_xxaa.ld

LDFLAGS = -Xlinker
LDFLAGS += -Map=$(OUTPUT_DIR)/$(OUTPUT_NAME).map
#LDFLAGS += --specs=nano.specs
#LDFLAGS += -u _printf_float
LDFLAGS += -mcpu=$(CPU) 
LDFLAGS += -mthumb 
LDFLAGS += -mabi=aapcs
LDFLAGS  += -mfloat-abi=hard		#Which floating point ABI to use
LDFLAGS  += -mfpu=fpv4-sp-d16	#The type of FPU we are using 
#LDFLAGS += -Wl,--gc-sections		#Remove unused sections (needs ffunction-section and fdata-section CFLAGS)
LDFLAGS += -T$(LD_SCRIPT)
LDFLAGS	+= -D$(DEVICE)
LDFLAGS	+= -D$(BOARD)
LDFLAGS	+= $(patsubst %,-L%, $(LDDIRS))

# Sorting removes duplicates
BUILD_DIRS := $(sort $(OBJ_DIR) $(OUTPUT_DIR) )

# Make a list of source paths
C_SRC_DIRS = $(shell find src -type d) 
ASSEMBLY_SRC_DIRS = $(shell find src -type d)

# Object files
C_OBJ 			= $(addprefix $(OBJ_DIR)/, $(C_SRC:.c=.o))
ASSEMBLY_OBJ 	= $(addprefix $(OBJ_DIR)/, $(ASSEMBLY_SRC:.S=.o))

# Set source lookup paths
vpath %.c $(C_SRC_DIRS)
vpath %.S $(ASSEMBLY_SRC_DIRS)

# Include automatically previously generated dependencies
-include $(addprefix $(OBJ_DIR)/, $(C_OBJ:.o=.d))

### Rules ###
# Default build target
.PHONY : all size clean

size :
	$(SIZE) $(OUTPUT_DIR)/$(OUTPUT_NAME).elf

all : release

clean : 
	$(RM) $(OUTPUT_DIR)/*
	$(RM) $(OBJ_DIR)/*
	- $(RM) JLink.log
	- $(RM) .gdbinit

.PHONY: release
release :  $(OUTPUT_DIR)/$(OUTPUT_NAME).bin $(OUTPUT_DIR)/$(OUTPUT_NAME).hex

$(BUILD_DIRS) : 
	@echo 
	@echo "Creating directories"
	- $(MK) $@

# Create objects from C source files
$(OBJ_DIR)/%.o : %.c
	@echo
	@echo "Build header dependencies for file: " $<
	$(CC) $(CFLAGS) -M $< -MF "$(@:.o=.d)" -MT $@
	@echo
	@echo "Compiling: " $<
	$(CC) $(CFLAGS) -c -o $@ $<

## Assemble .S files
$(OBJ_DIR)/%.o : %.S
	@echo
	@echo "Compiling: " $<
	$(CC) $(CFLAGS) -c -o $@ $<


## Link C and assembler objects to an .elf file
$(OUTPUT_DIR)/$(OUTPUT_NAME).elf : $(BUILD_DIRS) $(C_OBJ) $(ASSEMBLY_OBJ)
	@echo
	@echo "Linking object files: " 
	$(CC) $(LDFLAGS) $(C_OBJ) $(ASSEMBLY_OBJ) -o $(OUTPUT_DIR)/$(OUTPUT_NAME).elf

## Create binary .bin file from the .elf file
$(OUTPUT_DIR)/$(OUTPUT_NAME).bin : $(OUTPUT_DIR)/$(OUTPUT_NAME).elf
	@echo
	@echo "Create binary(.bin) file from: " $<
	$(OBJCOPY) -O binary $(OUTPUT_DIR)/$(OUTPUT_NAME).elf $(OUTPUT_DIR)/$(OUTPUT_NAME).bin

## Create binary .hex file from the .elf file
$(OUTPUT_DIR)/$(OUTPUT_NAME).hex : $(OUTPUT_DIR)/$(OUTPUT_NAME).elf
	@echo
	@echo "Create binary(.hex) file from: " $<
	$(OBJCOPY) -O ihex $(OUTPUT_DIR)/$(OUTPUT_NAME).elf $(OUTPUT_DIR)/$(OUTPUT_NAME).hex

## Program device
upload: rm.jlink upload.jlink #stopdebug
	@echo "Starting uploading"
	$(JLINK) $(OUTPUT_DIR)/upload.jlink
	@echo "Done uploading"

rm.jlink:
	-rm -rf $(OUTPUT_DIR)/upload.jlink
		
upload.jlink:
	echo "device $(JLINKDEVICE)\nspeed 1000\nw4 $(NVMC_CONFIG_ADRS) 1\nloadbin $(PWD)/$(OUTPUT_DIR)/$(OUTPUT_NAME).bin $(FLASH_START_ADDRESS)\nr\ng\nqc\n" > $(OUTPUT_DIR)/upload.jlink
		  
upload-softdevice: upload-softdevice.jlink #stopdebug
	@echo
	@echo "Convert from hex to binary. Split original hex in two to avoid huge (>250 MB) binary file with just 0s. "
	$(OBJCOPY) -Iihex -Obinary --remove-section .sec3 $(SOFTDEVICE) $(OUTPUT_DIR)/_mainpart.bin
	$(OBJCOPY) -Iihex -Obinary --remove-section .sec1 --remove-section .sec2 $(SOFTDEVICE) $(OUTPUT_DIR)/_uicr.bin
	$(JLINK) $(OUTPUT_DIR)/upload-softdevice.jlink

upload-softdevice.jlink:
	@echo
	@echo "Do magic. Write to NVMC to enable erase, do erase all and erase UICR, reset, enable writing, load mainpart bin, load uicr bin. Reset."
	@echo " Resetting in between is needed to disable the protections. "
	echo "w4 $(NVMC_CONFIG_ADRS) 1\nloadbin \"$(OUTPUT_DIR)/_mainpart.bin\" 0\nloadbin \"$(OUTPUT_DIR)/_uicr.bin\" 0x10001000\nr\ng\nexit\n" > $(OUTPUT_DIR)/upload-softdevice.jlink

recover: recover.jlink erase-all.jlink pin-reset.jlink
	$(JLINK) $(OUTPUT_DIR)/recover.jlink
	$(JLINK) $(OUTPUT_DIR)/erase-all.jlink
	$(JLINK) $(OUTPUT_DIR)/pin-reset.jlink

recover.jlink:
	echo "si 0\nt0\nsleep 1\ntck1\nsleep 1\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\nt0\nsleep 2\nt1\nsleep 2\ntck0\nsleep 100\nsi 1\nr\nexit\n" > $(OUTPUT_DIR)/recover.jlink

pin-reset.jlink:
	echo "device $(JLINKDEVICE)\nw4 $(NVMC_CONFIG_ADRS) 2\nw4 40000544 1\nr\nexit\n" > $(OUTPUT_DIR)/pin-reset.jlink

erase-all: erase-all.jlink
	$(JLINK) $(OUTPUT_DIR)/erase-all.jlink

erase-all.jlink:
	echo "device $(JLINKDEVICE)\nw4 $(NVMC_CONFIG_ADRS) 2\nw4 4001e50c 1\nw4 4001e514 1\nr\nexit\n" > $(OUTPUT_DIR)/erase-all.jlink
		  
.PHONY: upload upload-softdevice erase-all recover
