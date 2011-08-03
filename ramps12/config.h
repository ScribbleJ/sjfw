#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "config-common.h"

// Total number of axis in the machine
#define NUM_AXES 4
// Endstops inverting should be set for most systems.
#define ENDSTOPS_INVERTING 1
// Endstop pullups, too.
#define ENDSTOPPULLUPS 1
#define SAFE_DEFAULT_FEED 1500

// The temperature-reading pins are defined by their analog pin number.
#define HOTEND_TEMP_PIN 2
#define PLATFORM_TEMP_PIN 1

// These pins turn on the fets to supply power to heaters.
#define HOTEND_HEAT_PIN Pin(PortB, 4)
#define PLATFORM_HEAT_PIN Pin(PortH, 5)

// SD_AUTORUN looks for and runs 'sjfwauto.gcd' on boot.
//#define SD_AUTORUN 
#define SD_DETECT_PIN   Pin()
#define SD_WRITE_PIN    Pin()
#define SD_SELECT_PIN   Pin(PortB, 0)

// Steps_per_mm units below peculiar to my config, please examine them and change to match yours.
#define X_STEP_PIN      Pin(PortA, 4)
#define X_DIR_PIN       Pin(PortA, 6)
#define X_ENABLE_PIN    Pin(PortA, 2)
#define X_MIN_PIN       Pin(PortE, 5)
#define X_MAX_PIN       Pin(PortE, 4)
#define X_INVERT_DIR    false
#define X_HOME_DIR      -1
#define X_STEPS_PER_UNIT 62.745
#define X_MAX_FEED      12000
#define X_AVG_FEED      5000
#define X_START_FEED    2000
#define X_ACCEL_RATE    400
#define X_LENGTH        110
#define X_DISABLE       false

#define Y_STEP_PIN      Pin(PortD, 7)
#define Y_DIR_PIN       Pin(PortG, 1)
#define Y_ENABLE_PIN    Pin(PortC, 1)
#define Y_MIN_PIN       Pin(PortH, 1)
#define Y_MAX_PIN       Pin(PortH, 0)
#define Y_INVERT_DIR    false
#define Y_HOME_DIR      -1
#define Y_STEPS_PER_UNIT 62.745
#define Y_MAX_FEED      12000
#define Y_AVG_FEED      5000
#define Y_START_FEED    2000
#define Y_ACCEL_RATE    400
#define Y_LENGTH        110
#define Y_DISABLE       false

#define Z_STEP_PIN      Pin(PortL, 5)
#define Z_DIR_PIN       Pin(PortL, 3)
#define Z_ENABLE_PIN    Pin(PortL, 7)
#define Z_MIN_PIN       Pin(PortD, 3)
#define Z_MAX_PIN       Pin(PortD, 2)
#define Z_INVERT_DIR    true
#define Z_HOME_DIR      1
#define Z_STEPS_PER_UNIT 2267.718
#define Z_MAX_FEED      150
#define Z_AVG_FEED      100
#define Z_START_FEED    75
#define Z_ACCEL_RATE    50
#define Z_LENGTH        110
#define Z_DISABLE       true

#define A_STEP_PIN      Pin(PortC, 5)
#define A_DIR_PIN       Pin(PortC, 3)
#define A_ENABLE_PIN    Pin(PortC, 7)
#define A_INVERT_DIR    false
#define A_HOME_DIR      0
#define A_STEPS_PER_UNIT 729.99
#define A_MAX_FEED      24000
#define A_AVG_FEED      9000
#define A_START_FEED    5000
#define A_ACCEL_RATE    2000
#define A_LENGTH        110
#define A_DISABLE       false

#endif // CONFIG_H
