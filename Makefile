###########################
# Settings particular to your system
###########################
USE_SD = 1
USE_LCD = 1
USE_KEYPAD = 1
#USE_BT = 1
#USE_MARLIN = 1 WARNING: MARLIN NOT UPDATED AND WILL NOT WORK WITHOUT FIXES
INCLUDE_SJFW_LOOKAHEAD = 1

# EC for Gen3/4 only.  Others default to 100k Thermistors.
USE_EXTRUDERCONTROLLER = 1

# "generic" is a special config with no pins defined.
# You should be able to compile this for your chip, upload it, then configure the pins at
# runtime.
CONFIG_PATH = generic

# this needs to point to the directory where you have avrdude, avr-gcc, and such.
AVR_TOOLS_PATH = /usr/bin
AVR_GCC_PATH = $(AVR_TOOLS_PATH)

#Reasonable settings for Atmega1280 (regardless of board)
UPLOAD_RATE = 57600
AVRDUDE_PROGRAMMER = stk500v1
PORT = /dev/ttyUSB0
MCU = atmega1280

#Reasonable settings for Atmega644p
#UPLOAD_RATE = 38400
#AVRDUDE_PROGRAMMER = stk500v1
#PORT = /dev/ttyUSB0
#MCU = atmega644p

# Reasonable settings for Atmega2560 (regardless of board)
#UPLOAD_RATE = 115200
#AVRDUDE_PROGRAMMER = stk500v2
#PORT = /dev/ttyACM0
#MCU = atmega2560




########################
# Stuff you prolly don't need to change from here down.
########################
SJFW_VERSION = 1.10

ifeq ($(USE_LCD),1)
 LCD_FILES = LCDKeypad.cpp
 LCD_DEFINES = -DHAS_LCD
ifeq ($(USE_KEYPAD),1)
  KEYPAD_FILES = 
  KEYPAD_DEFINES = -DHAS_KEYPAD
endif
endif
ifeq ($(USE_SD),1)
 SD_FILES = lib_sd/byteordering.cpp lib_sd/fat.cpp lib_sd/partition.cpp lib_sd/sd_raw.cpp SDCard.cpp
 SD_DEFINES = -DHAS_SD
endif
ifeq ($(USE_EXTRUDERCONTROLLER),1)
 BOARD_FILES = temperature/MBIEC.cpp
 BOARD_DEFINES = -I./temperature/ -DUSE_MBIEC
else
 BOARD_FILES = temperature/Thermistor.cpp temperature/ThermistorTable.cpp avr/AnalogPin.cpp
 BOARD_DEFINES = -I./temperature/
endif
ifeq ($(MCU),atmega644p)
  OPT = s
else
  OPT = s
endif
ifeq ($(USE_BT),1)
  BT_FILES = 
  BT_DEFINES = -DHAS_BT
endif
ifeq ($(USE_MARLIN),1)
  MOTION_FILES = Marlin.cpp
  MOTION_DEFINES = -DUSE_MARLIN
else
  MOTION_FILES = Motion.cpp Axis.cpp
  MOTION_DEFINES =
endif
ifeq ($(INCLUDE_SJFW_LOOKAHEAD),1)
  LOOK_FILES = 
  LOOK_DEFINES = -DLOOKAHEAD
endif



  

EXTRA_FILES = $(MOTION_FILES) $(LCD_FILES) $(SD_FILES) $(BOARD_FILES) $(KEYPAD_FILES) $(BT_FILES) $(LOOK_FILES)
EXTRA_DEFINES = $(MOTION_DEFINES) $(LCD_DEFINES) $(SD_DEFINES) $(BOARD_DEFINES) $(KEYPAD_DEFINES) $(BT_DEFINES) -DSJFW_VERSION='"$(SJFW_VERSION)"' $(LOOK_DEFINES)



F_CPU = 16000000
CXXSRC = $(EXTRA_FILES) avr/AvrPort.cpp Host.cpp Time.cpp GcodeQueue.cpp GCode.cpp \
Globals.cpp Temperature.cpp avr/ArduinoMap.cpp Eeprom.cpp 


FORMAT = ihex

# Place -D or -U options here
CXXBENICE = -fno-default-inline 
CXXBEMEAN = 
CXXDEFS = -DF_CPU=$(F_CPU) $(EXTRA_DEFINES)
CXXEXTRA = -fno-threadsafe-statics -fwrapv -fno-exceptions -ffunction-sections -fdata-sections -Wall -Wextra
#work around current bug in compiler triggered by Marlin engine by disabling optimization.
ifeq ($(USE_MARLIN),1)
CXXFLAGS = $(CXXDEFS) $(CXXINCS) -O0 $(CXXEXTRA) $(CXXBENICE)
else
CXXFLAGS = $(CXXDEFS) $(CXXINCS) -O$(OPT) $(CXXEXTRA) $(CXXBENICE)
endif

LDFLAGS = -lm

# Programming support using avrdude. Settings and variables.
AVRDUDE_WRITE_FLASH = -U flash:w:main.hex:i
AVRDUDE_FLAGS = -V -p $(MCU) -P $(PORT) -c $(AVRDUDE_PROGRAMMER) -b $(UPLOAD_RATE) -D

# Program settings
CC = $(AVR_GCC_PATH)/avr-gcc
CXX = $(AVR_GCC_PATH)/avr-g++
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
ALL_CXXFLAGS = -mmcu=$(MCU) -I. -I./$(CONFIG_PATH) -I./lib_sd -I./avr -I/usr/lib/avr/include $(CXXFLAGS)

# Default target.
all: build sizeafter

build: elf hex 

elf: main.elf
hex: main.hex

# Program the device.  
upload: main.hex
	perl ./util/reset.pl $(PORT);$(AVRDUDE)  $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) main.hex
ELFSIZE = $(SIZE)  main.elf
sizebefore:
	@if [ -f main.elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
	@if [ -f main.elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi


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
