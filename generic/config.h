#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "config-common.h"

// These are only used on Gen4 and hardcoded there.
#define RS485_TX_ENABLE   Pin(PortC,5)
#define RS485_RX_ENABLE   Pin(PortC,7)



#define NUM_AXES 4
#define ENDSTOPS_INVERTING 0
#define ENDSTOPPULLUPS 0
#define SAFE_DEFAULT_FEED 1500

#define HOTEND_TEMP_PIN -1
#define PLATFORM_TEMP_PIN -1

#define HOTEND_HEAT_PIN Pin()
#define PLATFORM_HEAT_PIN Pin()

#define SD_AUTORUN 
#define SD_DETECT_PIN   Pin()
#define SD_WRITE_PIN    Pin()
#define SD_SELECT_PIN   Pin(PortB, 0)


#define X_STEP_PIN      Pin()
#define X_DIR_PIN       Pin()
#define X_ENABLE_PIN    Pin()
#define X_MIN_PIN       Pin()
#define X_MAX_PIN       Pin()
#define X_INVERT_DIR    false
#define X_STEPS_PER_UNIT 62.745
#define X_MAX_FEED      12000
#define X_AVG_FEED      6000
#define X_START_FEED    2000
#define X_ACCEL_RATE    200
#define X_DISABLE       false

#define Y_STEP_PIN      Pin()
#define Y_DIR_PIN       Pin()
#define Y_ENABLE_PIN    Pin()
#define Y_MIN_PIN       Pin()
#define Y_MAX_PIN       Pin()
#define Y_INVERT_DIR    false
#define Y_STEPS_PER_UNIT 62.745
#define Y_MAX_FEED      12000
#define Y_AVG_FEED      6000
#define Y_START_FEED    2000
#define Y_ACCEL_RATE    200
#define Y_DISABLE       false

#define Z_STEP_PIN      Pin()
#define Z_DIR_PIN       Pin()
#define Z_ENABLE_PIN    Pin()
#define Z_MIN_PIN       Pin()
#define Z_MAX_PIN       Pin()
#define Z_INVERT_DIR    true
#define Z_STEPS_PER_UNIT 2267.718
#define Z_MAX_FEED      150
#define Z_AVG_FEED      100
#define Z_START_FEED    75
#define Z_ACCEL_RATE    50
#define Z_DISABLE       true

#define A_STEP_PIN      Pin()
#define A_DIR_PIN       Pin()
#define A_ENABLE_PIN    Pin()
#define A_INVERT_DIR    false
#define A_STEPS_PER_UNIT 729.99
#define A_MAX_FEED      24000
#define A_AVG_FEED      12000
#define A_START_FEED    5000
#define A_ACCEL_RATE    2000
#define A_DISABLE       false


// The following config is for a parallel LCD connected to 
// AUX-2 in 4-bit mode.

#define USE4BITMODE
#define LCD_RS_PIN      Pin()
#define LCD_RW_PIN      Pin()
#define LCD_E_PIN       Pin()
#define LCD_0_PIN       Pin()
#define LCD_1_PIN       Pin()
#define LCD_2_PIN       Pin()
#define LCD_3_PIN       Pin()
#define LCD_4_PIN       Pin()
#define LCD_5_PIN       Pin()
#define LCD_6_PIN       Pin()
#define LCD_7_PIN       Pin()

#define LCD_X 20
#define LCD_Y 4
#define LCD_LINESTARTS {0x0, 0x40, 0x14, 0x54}


#define KP_ROWS 4
#define KP_COLS 4
#define KEYPAD_BUTTONMAP "123A", "456B", "789C", "*0#D"
#define KEYPAD_ROWPINS Pin(), Pin(), Pin(), Pin()
#define KEYPAD_COLPINS Pin(), Pin(), Pin(), Pin()
#define KEYPAD_DEBOUNCE_MICROS 50



#endif // CONFIG_H
