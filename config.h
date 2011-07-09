#ifndef _CONFIG_H_
#define _CONFIG_H_

#define RS485_TX_ENABLE   Pin(PortC,5)
#define RS485_RX_ENABLE   Pin(PortC,7)

#define NUM_AXES 4
#define ENDSTOPS_INVERTING 1
#define SAFE_DEFAULT_FEED 1200

#define X_STEP_PIN      Pin(PortA,6)
#define X_DIR_PIN       Pin(PortA,5)
#define X_ENABLE_PIN    Pin(PortA,4)
#define X_MIN_PIN       Pin(PortB,6)
#define X_MAX_PIN       Pin(PortB,5)
#define X_INVERT_DIR    true
#define X_HOME_DIR      -1
#define X_STEPS_PER_UNIT 47.069852
#define X_MAX_FEED      12000
#define X_AVG_FEED      3000
#define X_START_FEED    1500
#define X_ACCEL_DIST    2.5
#define X_LENGTH        110

#define Y_STEP_PIN      Pin(PortA,3)
#define Y_DIR_PIN       Pin(PortA,2)
#define Y_ENABLE_PIN    Pin(PortA,1)
#define Y_MIN_PIN       Pin(PortB,4)
#define Y_MAX_PIN       Pin(PortH,6)
#define Y_INVERT_DIR    true
#define Y_HOME_DIR      -1
#define Y_STEPS_PER_UNIT 47.069852
#define Y_MAX_FEED      12000
#define Y_AVG_FEED      3000
#define Y_START_FEED    1500
#define Y_ACCEL_DIST    2.5
#define Y_LENGTH        110

#define Z_STEP_PIN      Pin(PortA,0)
#define Z_DIR_PIN       Pin(PortH,0)
#define Z_ENABLE_PIN    Pin(PortH,1)
#define Z_MIN_PIN       Pin(PortH,5)
#define Z_MAX_PIN       Pin(PortH,4)
#define Z_INVERT_DIR    false
#define Z_HOME_DIR      1
#define Z_STEPS_PER_UNIT 200
#define Z_MAX_FEED      200
#define Z_AVG_FEED      170
#define Z_START_FEED    120
#define Z_ACCEL_DIST    2
#define Z_LENGTH        110

#define A_STEP_PIN      Pin(PortJ,0)
#define A_DIR_PIN       Pin(PortJ,1)
#define A_ENABLE_PIN    Pin(PortE,5)
#define A_INVERT_DIR    true
#define A_HOME_DIR      0
#define A_STEPS_PER_UNIT 44.169
#define A_MAX_FEED      12000
#define A_AVG_FEED      3000
#define A_START_FEED    1500
#define A_ACCEL_DIST    0.5
#define A_LENGTH        110

#define B_STEP_PIN      Pin(PortG,5)
#define B_DIR_PIN       Pin(PortE,3)
#define B_ENABLE_PIN    Pin(PortH,3)
#define B_INVERT_DIR    false
#define B_HOME_DIR      0
#define B_STEPS_PER_UNIT 700
#define B_MAX_FEED      6000
#define B_AVG_FEED      3000
#define B_START_FEED    1200
#define B_ACCEL_DIST    5
#define B_LENGTH        110






#endif // CONFIG_H
