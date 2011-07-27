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


# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
#DEBUG = stabs
DEBUG =

OPT = s

# Place -D or -U options here
CXXDEFS = -DF_CPU=$(F_CPU)

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = 
CDEBUG = -g$(DEBUG)
CWARN = -Wall -Winline
CTUNING = -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CXXEXTRA = -fno-threadsafe-statics -fwrapv -fno-exceptions 

CXXFLAGS = $(CXXDEFS) $(CXXINCS) -O$(OPT) $(CXXEXTRA)
LDFLAGS = -lm


# Programming support using avrdude. Settings and variables.
AVRDUDE_PORT = $(PORT)
AVRDUDE_WRITE_FLASH = -U flash:w:main.hex:i
AVRDUDE_FLAGS = -F -V \
-p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) \
-b $(UPLOAD_RATE) 

# Program settings
CC = $(AVR_TOOLS_PATH)/avr-gcc
CXX = $(AVR_TOOLS_PATH)/avr-g++
OBJCOPY = $(AVR_TOOLS_PATH)/avr-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)/avr-objdump
AR  = $(AVR_TOOLS_PATH)/avr-ar
SIZE = $(AVR_TOOLS_PATH)/avr-size
NM = $(AVR_TOOLS_PATH)/avr-nm
AVRDUDE = avrdude
REMOVE = rm -f
MV = mv -f

# Define all object files.
OBJ = $(CXXSRC:.cpp=.o) 

# Define all listing files.
LST = $(CXXSRC:.cpp=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CXXFLAGS = -mmcu=$(MCU) -I. -I$(CONFIG_PATH) -I./lib_sd $(CXXFLAGS)

# Default target.
all: build sizeafter

build: elf hex 

elf: main.elf
hex: main.hex
eep: main.eep
lss: main.lss 
sym: main.sym

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


# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000 


.SUFFIXES: .elf .hex .eep .lss .sym

.elf.hex:
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.eep:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@

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
	$(REMOVE) main.hex main.eep main.cof main.elf \
	main.map main.sym main.lss core.a \
	$(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(CXXSRC:.cpp=.s) $(CXXSRC:.cpp=.d)

.PHONY:	all build elf hex eep lss sym program clean sizebefore sizeafter
