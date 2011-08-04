#include "Host.h"
#include "Motion.h"
#include "Temperature.h"

#include "config.h"

Host& HOST = Host::Instance();
Motion& MOTION = Motion::Instance();
Temperature& TEMPERATURE = Temperature::Instance();

#ifdef HAS_LCD
#include "LiquidCrystal.h"
#include "Keypad.h"
uint8_t _lcd_linestarts[] = LCD_LINESTARTS;
LiquidCrystal LCD(LCD_RS_PIN, 
                  LCD_RW_PIN, 
                  LCD_E_PIN, 
                  LCD_0_PIN, 
                  LCD_1_PIN, 
                  LCD_2_PIN, 
                  LCD_3_PIN, 
                  LCD_4_PIN, 
                  LCD_5_PIN, 
                  LCD_6_PIN, 
                  LCD_7_PIN,
                  LCD_X,
                  LCD_Y,
                  _lcd_linestarts);
const char *buttonmap[] = { KEYPAD_BUTTONMAP };
const Pin kp_rpins[] = { KEYPAD_ROWPINS };
const Pin kp_cpins[] = { KEYPAD_COLPINS };
Keypad KEYPAD(kp_cpins, kp_rpins, buttonmap, KEYPAD_DEBOUNCE_MICROS);
#endif

