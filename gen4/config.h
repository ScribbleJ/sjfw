#ifndef _CONFIG_H_
#define _CONFIG_H_

#define USE_MBIEC
#define RS485_TX_ENABLE   Pin(PortC,5)
#define RS485_RX_ENABLE   Pin(PortC,7)

#define NUM_AXES 4
#define ENDSTOPS_INVERTING 1
#define ENDSTOPPULLUPS 1
#define SAFE_DEFAULT_FEED 1500

#define NUM_SOURCES     2
#define HOST_SOURCE     0
#define SD_SOURCE       1

#define SD_AUTORUN
#define SD_SELECT_PIN   Pin(PortB, 0)
#define SD_WRITE_PIN    Pin(PortD,0)
#define SD_DETECT_PIN   Pin(PortD,1)


#define X_STEP_PIN      Pin(PortA,6)
#define X_DIR_PIN       Pin(PortA,5)
#define X_ENABLE_PIN    Pin(PortA,4)
#define X_MIN_PIN       Pin(PortB,6)
#define X_MAX_PIN       Pin(PortB,5)
#define X_INVERT_DIR    true
#define X_HOME_DIR      -1
#define X_STEPS_PER_UNIT 47.069852
#define X_MAX_FEED      60*100
#define X_AVG_FEED      60*30
#define X_START_FEED    60*25
#define X_ACCEL_DIST    2.5
#define X_LENGTH        110
#define X_DISABLE       false

#define Y_STEP_PIN      Pin(PortA,3)
#define Y_DIR_PIN       Pin(PortA,2)
#define Y_ENABLE_PIN    Pin(PortA,1)
#define Y_MIN_PIN       Pin(PortB,4)
#define Y_MAX_PIN       Pin(PortH,6)
#define Y_INVERT_DIR    true
#define Y_HOME_DIR      -1
#define Y_STEPS_PER_UNIT 47.069852
#define Y_MAX_FEED      60*100
#define Y_AVG_FEED      60*30
#define Y_START_FEED    60*25
#define Y_ACCEL_DIST    2.5
#define Y_LENGTH        110
#define Y_DISABLE       false

#define Z_STEP_PIN      Pin(PortA,0)
#define Z_DIR_PIN       Pin(PortH,0)
#define Z_ENABLE_PIN    Pin(PortH,1)
#define Z_MIN_PIN       Pin(PortH,5)
#define Z_MAX_PIN       Pin(PortH,4)
#define Z_INVERT_DIR    false
#define Z_HOME_DIR      1
#define Z_STEPS_PER_UNIT 200
#define Z_MAX_FEED      400
#define Z_AVG_FEED      100
#define Z_START_FEED    100
#define Z_ACCEL_DIST    5
#define Z_LENGTH        110
#define Z_DISABLE       false

#define A_STEP_PIN      Pin(PortJ,0)
#define A_DIR_PIN       Pin(PortJ,1)
#define A_ENABLE_PIN    Pin(PortE,5)
#define A_INVERT_DIR    true
#define A_HOME_DIR      0
#define A_STEPS_PER_UNIT 44.169
#define A_MAX_FEED      120*60
#define A_AVG_FEED      60*60
#define A_START_FEED    60*50
#define A_ACCEL_DIST    1
#define A_LENGTH        110
#define A_DISABLE       false

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
#define B_DISABLE       true


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

#define LCD_LINESTARTS {0x0, 0x40}
#define LCD_FULLADDRESS 1






#endif // CONFIG_H
