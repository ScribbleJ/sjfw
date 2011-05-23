/*
 * Copyright 2010 by Adam Mayer	 <adam@makerbot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef BOARDS_RRMBV22_CONFIGURATION_HH_
#define BOARDS_RRMBV22_CONFIGURATION_HH_

// This file details the pin assignments and features of the RepRap Motherboard
// version 1.2 for the ordinary use case.

#include "AvrPort.h"

// Interval for the stepper update in microseconds.  This interval is the minimum
// possible time between steps; in practical terms, your time between steps should
// be at least eight times this large.  Reducing the interval can cause resource
// starvation; leave this at 64uS or greater unless you know what you're doing.
#define INTERVAL_IN_MICROSECONDS 64

// --- Power Supply Unit configuration ---
// Define as 1 if a PSU is present; 0 if not.
#define HAS_PSU         0

// --- Secure Digital Card configuration ---
// NOTE: If SD support is enabled, it is implicitly assumed that the
// following pins are connected:
//  AVR    |   SD header
//---------|--------------
//  MISO   |   DATA_OUT
//  MOSI   |   DATA_IN
//  SCK    |   CLK

// Define as 1 if and SD card slot is present; 0 if not.
#define HAS_SD          1
// The pin that connects to the write protect line on the SD header.
#define SD_WRITE_PIN    Pin(PortD,0)
// The pin that connects to the card detect line on the SD header.
#define SD_DETECT_PIN   Pin(PortD,1)
// The pin that connects to the chip select line on the SD header.
#define SD_SELECT_PIN   Pin(PortB,0)

// --- Slave UART configuration ---
// The slave UART is presumed to be an RS485 connection through a sn75176 chip.
// Define as 1 if the slave UART is present; 0 if not.
#define HAS_SLAVE_UART  1
// The pin that connects to the driver enable line on the RS485 chip.
#define RS485_TX_ENABLE   Pin(PortC,5)
// The pin that connects to the active-low recieve enable line on the RS485 chip.
#define RS485_RX_ENABLE   Pin(PortC,7)

// --- Host UART configuration ---
// The host UART is presumed to always be present on the RX/TX lines.

// --- Piezo Buzzer configuration ---
// Define as 1 if the piezo buzzer is present, 0 if not.
#define HAS_BUZZER 1
// The pin that drives the buzzer
#define BUZZER_PIN Pin(PortC,6)

// --- Emergency Stop configuration ---
// Define as 1 if the estop is present, 0 if not.
#define HAS_ESTOP 1
// The pin connected to the emergency stop
#define ESTOP_PIN Pin(PortE,4)

// --- Axis configuration ---
// Define the number of stepper axes supported by the board.  The axes are
// denoted by X, Y, Z, A and B.
#define STEPPER_COUNT 5

// --- Stepper and endstop configuration ---
// Pins should be defined for each axis present on the board.  They are denoted
// X, Y, Z, A and B respectively.

// This indicates the default interpretation of the endstop values.
// If your endstops are based on the H21LOB, they are inverted;
// if they are based on the H21LOI, they are not.
#define DEFAULT_INVERTED_ENDSTOPS 1

// The X stepper step pin (active on rising edge)
#define X_STEP_PIN      Pin(PortA,6)
// The X direction pin (forward on logic high)
#define X_DIR_PIN       Pin(PortA,5)
// The X stepper enable pin (active low)
#define X_ENABLE_PIN    Pin(PortA,4)
// The X minimum endstop pin (active high)
#define X_MIN_PIN       Pin(PortB,6)
// The X maximum endstop pin (active high)
#define X_MAX_PIN       Pin(PortB,5)

// The Y stepper step pin (active on rising edge)
#define Y_STEP_PIN      Pin(PortA,3)
// The Y direction pin (forward on logic high)
#define Y_DIR_PIN       Pin(PortA,2)
// The Y stepper enable pin (active low)
#define Y_ENABLE_PIN    Pin(PortA,1)
// The Y minimum endstop pin (active high)
#define Y_MIN_PIN       Pin(PortB,4)
// The Y maximum endstop pin (active high)
#define Y_MAX_PIN       Pin(PortH,6)

// The Z stepper step pin (active on rising edge)
#define Z_STEP_PIN      Pin(PortA,0)
// The Z direction pin (forward on logic high)
#define Z_DIR_PIN       Pin(PortH,0)
// The Z stepper enable pin (active low)
#define Z_ENABLE_PIN    Pin(PortH,1)
// The Z minimum endstop pin (active high)
#define Z_MIN_PIN       Pin(PortH,5)
// The Z maximum endstop pin (active high)
#define Z_MAX_PIN       Pin(PortH,4)

// The A stepper step pin (active on rising edge)
#define A_STEP_PIN      Pin(PortJ,0)
// The A direction pin (forward on logic high)
#define A_DIR_PIN       Pin(PortJ,1)
// The A stepper enable pin (active low)
#define A_ENABLE_PIN    Pin(PortE,5)

// The B stepper step pin (active on rising edge)
#define B_STEP_PIN      Pin(PortG,5)
// The B direction pin (forward on logic high)
#define B_DIR_PIN       Pin(PortE,3)
// The B stepper enable pin (active low)
#define B_ENABLE_PIN    Pin(PortH,3)

// --- Debugging configuration ---
// The pin which controls the debug LED (active high)
#define DEBUG_PIN       Pin(PortB,7)
// By default, debugging packets should be honored; this is made
// configurable if we're short on cycles or EEPROM.
// Define as 1 if debugging packets are honored; 0 if not.
#define HONOR_DEBUG_PACKETS 1

// SCRIBBLEJHACK - LCD
#define HAS_LCD         1

#define LCD_RS_PIN      Pin(PortC,3)
#define LCD_RW_PIN      Pin(PortC,2)
#define LCD_E_PIN       Pin(PortC,0)
#define LCD_0_PIN       Pin(PortG,2)
#define LCD_1_PIN       Pin(PortG,0)
#define LCD_2_PIN       Pin(PortL,6)
#define LCD_3_PIN       Pin(PortC,1)
#define LCD_4_PIN       Pin(PortD,7)
#define LCD_5_PIN       Pin(PortG,1)
#define LCD_6_PIN       Pin(PortL,7)
#define LCD_7_PIN       Pin(PortL,5)

#define LCD_X 16
#define LCD_Y 2


// Used to report on LCD.
#define NUM_TOOLS       1
#define PLATFORM_HEATER_TOOLNUM 0
// EXTRACHARS must be length of string.
#define LCD_EXTRASTRING ""
#define LCD_EXTRACHARS 0
#define LCD_TEMP_UPDATE_INTERVAL_MICROS 2000000
// My Newhaven starts each line in memory after the previous.
// The hitachi standard starts each line offset by 40h
// GOOD FOR HITACHI/STANDARD 16CHAR
#define LCD_LINESTARTS {0x0, 0x40}
//GOOD FOR NEWHAVEN 20CHAR
//#define LINESTARTS {0x0, 0x10}
// My Newhaven can only address every other character. 
// The hitachi standard can address each char individually.
#define LCD_FULLADDRESS 1

// How many lines of logging data to keep/present
#define LOG_LINES	10
// How many chars in a filename?
#define LCD_MAX_FILE_LEN  32

#define FILE_QUEUE_LEN  10


#define HAS_KEYPAD	1
#define KP_COLS		3
#define KP_ROWS		4
#define KP_KEYMAP	{ {'1','2','3'},{'4','5','6'},{'7','8','9'},{'*','0','#'} }
#define KP_COL1_PIN	Pin(PortL,0)
#define KP_COL2_PIN	Pin(PortL,4)
#define KP_COL3_PIN	Pin(PortL,1)
#define KP_ROW1_PIN	Pin(PortL,2)
#define KP_ROW2_PIN	Pin(PortC,4)
#define KP_ROW3_PIN	Pin(PortB,5)
#define KP_ROW4_PIN	Pin(PortL,3)
#define KP_COL_PINS     { KP_COL1_PIN, KP_COL2_PIN, KP_COL3_PIN }
#define KP_ROW_PINS     { KP_ROW1_PIN, KP_ROW2_PIN, KP_ROW3_PIN, KP_ROW4_PIN }

#define KP_DEBOUNCE     50





#endif // BOARDS_RRMBV12_CONFIGURATION_HH_

