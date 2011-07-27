# This makefile is a mess.


# "INSTALL_DIR" = path to your arduino installation.  May not be needed, we only
# use avrdude from there, not any of the libraries or code.
INSTALL_DIR = /home/chris/arduino-0022
F_CPU = 16000000


# Reasonable settings for ToM Gen4
UPLOAD_RATE = 57600
AVRDUDE_PROGRAMMER = stk500v1
PORT = /dev/ttyUSB0
MCU = atmega1280
CONFIG_PATH = gen4

# Reasonable settings for RAMPS
#UPLOAD_RATE = 115200
#AVRDUDE_PROGRAMMER = stk500v2
#PORT = /dev/ttyACM0
#MCU = atmega2560
#CONFIG_PATH = ramps




############################################################################
# Below here nothing should be changed...

AVR_TOOLS_PATH = /usr/bin
SRC = 
CXXSRC = AvrPort.cpp Host.cpp Time.cpp Gcodes.cpp MGcode.cpp Axis.cpp Motion.cpp \
Globals.cpp LiquidCrystal.cpp Temperature.cpp AnalogPin.cpp ThermistorTable.cpp \
Thermistor.cpp MBIEC.cpp\
lib_sd/byteordering.cpp lib_sd/fat.cpp lib_sd/partition.cpp lib_sd/sd_raw.cpp SDCard.cpp

FORMAT = ihex
OPT = s

# Place -D or -U options here
CXXDEFS = -DF_CPU=$(F_CPU)
CXXEXTRA = -fno-threadsafe-statics -fwrapv -fno-exceptions 
CXXFLAGS = $(CXXDEFS) $(CXXINCS) -O$(OPT) $(CXXEXTRA)
LDFLAGS = -lm

# Programming support using avrdude. Settings and variables.
AVRDUDE_WRITE_FLASH = -U flash:w:main.hex:i
AVRDUDE_FLAGS = -F -V -p $(MCU) -P $(PORT) -c $(AVRDUDE_PROGRAMMER) -b $(UPLOAD_RATE) 

# Program settings
CC = $(AVR_TOOLS_PATH)/avr-gcc
CXX = $(AVR_TOOLS_PATH)/avr-g++
OBJCOPY = $(AVR_TOOLS_PATH)/avr-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)/avr-objdump
AR  = $(AVR_TOOLS_PATH)/avr-ar
SIZE = $(AVR_TOOLS_PATH)/avr-size
NM = $(AVR_TOOLS_PATH)/avr-nm
AVRDUDE = $(AVR_TOOLS_PATH)/avrdude
REMOVE = rm -f
MV = mv -f

# Define all object files.
OBJ = $(CXXSRC:.cpp=.o) 

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CXXFLAGS = -mmcu=$(MCU) -I. -I$(CONFIG_PATH) -I./lib_sd $(CXXFLAGS)

# Default target.
all: build sizeafter

build: elf hex 

elf: main.elf
hex: main.hex

# Program the device.  
upload: main.hex
	perl ./reset.pl $(PORT);$(AVRDUDE)  $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) main.hex
ELFSIZE = $(SIZE)  main.elf
sizebefore:
	@if [ -f main.elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(HEXSIZE); echo; fi

sizeafter:
	@if [ -f main.elf ]; then echo; echo $(MSG_SIZE_AFTER); $(HEXSIZE); echo; fi


.SUFFIXES: .elf .hex 

.elf.hex:
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

	# Link: create ELF output file from library.
main.elf: main.cpp core.a 
	$(CC) $(ALL_CXXFLAGS) -o $@ main.cpp -L. core.a $(LDFLAGS)

core.a: $(OBJ)
	@for i in $(OBJ); do echo $(AR) rcs core.a $$i; $(AR) rcs core.a $$i; done



# Compile: create object files from C++ source files.
.cpp.o:
	$(CXX) -c $(ALL_CXXFLAGS) $< -o $@ 

# Target: clean project.
clean:
	$(REMOVE) main.hex main.elf main.map core.a \
	$(OBJ) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d)

.PHONY:	all build elf hex program clean sizebefore sizeafter
