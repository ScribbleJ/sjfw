#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "config-common.h"

#define NUM_AXES 4
#define ENDSTOPS_INVERTING 0
#define ENDSTOPPULLUPS 1
#define SAFE_DEFAULT_FEED 1500

#define HOTEND_TEMP_PIN 13
#define PLATFORM_TEMP_PIN 10

#define HOTEND_HEAT_PIN Pin(PortB, 7)
#define PLATFORM_HEAT_PIN Pin(PortG, 5)

//#define SD_AUTORUN
#define SD_DETECT_PIN   Pin()
#define SD_WRITE_PIN    Pin()
#define SD_SELECT_PIN   Pin(PortB, 0)


#define X_STEP_PIN      Pin(PortE, 4)
#define X_DIR_PIN       Pin(PortE, 5)
#define X_ENABLE_PIN    Pin()
#define X_MIN_PIN       Pin(PortH, 1)
#define X_MAX_PIN       Pin()
#define X_INVERT_DIR    true
#define X_HOME_DIR      -1
#define X_STEPS_PER_UNIT 80.0
#define X_MAX_FEED      12000
#define X_AVG_FEED      6000
#define X_START_FEED    2000
#define X_ACCEL_RATE    200
#define X_LENGTH        190
#define X_DISABLE       false

#define Y_STEP_PIN      Pin(PortE, 3)
#define Y_DIR_PIN       Pin(PortH, 3)
#define Y_ENABLE_PIN    Pin()
#define Y_MIN_PIN       Pin(PortF, 4)
#define Y_MAX_PIN       Pin()
#define Y_INVERT_DIR    true
#define Y_HOME_DIR      -1
#define Y_STEPS_PER_UNIT 80.0
#define Y_MAX_FEED      12000
#define Y_AVG_FEED      6000
#define Y_START_FEED    2000
#define Y_ACCEL_RATE    200
#define Y_LENGTH        190
#define Y_DISABLE       false

#define Z_STEP_PIN      Pin(PortK, 0)
#define Z_DIR_PIN       Pin(PortK, 1)
#define Z_ENABLE_PIN    Pin()
#define Z_MIN_PIN       Pin()
#define Z_MAX_PIN       Pin(PortF, 7)
#define Z_INVERT_DIR    false
#define Z_HOME_DIR      1
#define Z_STEPS_PER_UNIT 400.0
#define Z_MAX_FEED      90
#define Z_AVG_FEED      90
#define Z_START_FEED    20
#define Z_ACCEL_RATE    50
#define Z_LENGTH        60
#define Z_DISABLE       false

#define A_STEP_PIN      Pin(PortK, 3)
#define A_DIR_PIN       Pin(PortK, 4)
#define A_ENABLE_PIN    Pin()
#define A_INVERT_DIR    false
#define A_HOME_DIR      0
#define A_STEPS_PER_UNIT 535.17
#define A_MAX_FEED      24000
#define A_AVG_FEED      12000
#define A_START_FEED    5000
#define A_ACCEL_RATE    2000
#define A_LENGTH        110
#define A_DISABLE       false


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

// How often to update the temperature display on the LCD.
#define LCD_TEMP_REFRESH_MILLIS 1000
#define LCD_BUFFER_SIZE 200


#define KP_ROWS 4
#define KP_COLS 4
#define KEYPAD_BUTTONMAP "123A", "456B", "789C", "*0#D"
#define KEYPAD_ROWPINS Pin(PortH,1),Pin(PortH,0),Pin(PortA,1),Pin(PortA,3)
#define KEYPAD_COLPINS Pin(PortA,5), Pin(PortA,7),Pin(PortC,6),Pin(PortC,4)
#define KEYPAD_DEBOUNCE_MICROS 50



#endif // CONFIG_H
