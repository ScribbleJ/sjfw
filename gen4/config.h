#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "config-common.h"

// MBI Extruder Controller comm settings.
#define RS485_TX_ENABLE   Pin(PortC,5)
#define RS485_RX_ENABLE   Pin(PortC,7)

#define NUM_AXES 4
#define ENDSTOPS_INVERTING 1
#define ENDSTOPPULLUPS 1
#define SAFE_DEFAULT_FEED 1500

// SD AUTORUN looks for and runs 'sjfwauto.gcd' on boot.
// #define SD_AUTORUN
#define SD_SELECT_PIN   Pin(PortB, 0)
#define SD_WRITE_PIN    Pin(PortD,0)
#define SD_DETECT_PIN   Pin(PortD,1)


#define X_STEP_PIN      Pin(PortA,6)
#define X_DIR_PIN       Pin(PortA,5)
#define X_ENABLE_PIN    Pin(PortA,4)
#define X_MIN_PIN       Pin(PortB,6)
#define X_MAX_PIN       Pin(PortB,5)
#define X_INVERT_DIR    true
#define X_STEPS_PER_UNIT 47.069852
#define X_MAX_FEED      12000
#define X_AVG_FEED      6000
#define X_START_FEED    2000
#define X_ACCEL_RATE    400
#define X_DISABLE       false

#define Y_STEP_PIN      Pin(PortA,3)
#define Y_DIR_PIN       Pin(PortA,2)
#define Y_ENABLE_PIN    Pin(PortA,1)
#define Y_MIN_PIN       Pin(PortB,4)
#define Y_MAX_PIN       Pin(PortH,6)
#define Y_INVERT_DIR    true
#define Y_STEPS_PER_UNIT 47.069852
#define Y_MAX_FEED      12000
#define Y_AVG_FEED      6000
#define Y_START_FEED    2000
#define Y_ACCEL_RATE    400
#define Y_DISABLE       false

#define Z_STEP_PIN      Pin(PortA,0)
#define Z_DIR_PIN       Pin(PortH,0)
#define Z_ENABLE_PIN    Pin(PortH,1)
#define Z_MIN_PIN       Pin(PortH,5)
#define Z_MAX_PIN       Pin(PortH,4)
#define Z_INVERT_DIR    false
#define Z_STEPS_PER_UNIT 200
#define Z_MAX_FEED      1000
#define Z_AVG_FEED      100
#define Z_START_FEED    100
#define Z_ACCEL_RATE    100
#define Z_DISABLE       false

#define A_STEP_PIN      Pin(PortJ,0)
#define A_DIR_PIN       Pin(PortJ,1)
#define A_ENABLE_PIN    Pin(PortE,5)
#define A_INVERT_DIR    true
#define A_STEPS_PER_UNIT 44.169
#define A_MAX_FEED      24000
#define A_AVG_FEED      12000
#define A_START_FEED    4000
#define A_ACCEL_RATE    1000
#define A_DISABLE       false

#endif // CONFIG_H
