#include "LCDKeypad.h"

uint8_t _lcd_linestarts[] = LCD_LINESTARTS;
#ifdef HAS_KEYPAD
const char *_kp_buttonmap[] = { KEYPAD_BUTTONMAP };
const Pin _kp_rpins[] = { KEYPAD_ROWPINS };
const Pin _kp_cpins[] = { KEYPAD_COLPINS };
#endif
const float RATES_OF_CHANGE[] = { 5, 10, 100, 0.1, 1 };
const float* const ROC_END = RATES_OF_CHANGE+5;

