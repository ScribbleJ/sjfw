#ifndef _LCDKEYPAD_H_
#define _LCDKEYPAD_H_
#ifdef HAS_LCD

#include "config.h"
#include "Time.h"
#include "LiquidCrystal.h"
#include "Host.h"
#include "Temperature.h"

#ifdef HAS_KEYPAD
#include "Keypad.h"
extern const char *_kp_buttonmap[];
extern const Pin _kp_rpins[];
extern const Pin _kp_cpins[];
#endif

extern uint8_t _lcd_linestarts[];

class LCDKeypad
{
public:
  enum MODE { TEMP, SDSELECT };


  LCDKeypad() 
  : LCD(LCD_RS_PIN, LCD_RW_PIN, LCD_E_PIN,
        LCD_0_PIN, LCD_1_PIN, LCD_2_PIN, LCD_3_PIN,
        LCD_4_PIN, LCD_5_PIN, LCD_6_PIN, LCD_7_PIN,
        LCD_X, LCD_Y, _lcd_linestarts)
#ifdef HAS_KEYPAD        
    ,KEYPAD(_kp_cpins, _kp_rpins, _kp_buttonmap, KEYPAD_DEBOUNCE_MICROS)
#endif    
  {
    lastkey = 0;
    last_lcdrefresh = millis();
    switchmode_TEMP();
  }

  void handleUpdates()
  {
    LCD.handleUpdates();

#ifdef HAS_KEYPAD
    char pressedkey = KEYPAD.getPressedKey();
    if(pressedkey && pressedkey != lastkey)
    {
      inputswitch(pressedkey);
    }
    lastkey = pressedkey;
#endif

    unsigned long now = millis();
    if(now - last_lcdrefresh > 950)
    {
      last_lcdrefresh = now;
      updateTemp();
    }

  }

  void changeMode(MODE which)
  {
    if(currentmode == which)
      return;

    switch(which)
    {
      case TEMP:
        switchmode_TEMP();
        break;
      case SDSELECT:
        switchmode_SDSELECT();
        break;
      default:
        break;
    }
  }

private:
  unsigned long last_lcdrefresh;
  char lastkey;
  LiquidCrystal LCD;
#ifdef HAS_KEYPAD
  Keypad KEYPAD;
#endif
  MODE currentmode;
  
  void inputswitch(char key)
  {
    bool handled = false;
    switch(currentmode)
    {
      case TEMP:
        handled = keyhandler_TEMP(key);
        break;
      case SDSELECT:
        handled = keyhandler_SDSELECT(key);
        break;
    }
    if(!handled)
    {
      keyhandler_default(key);
    }
  }

  void keyhandler_default(char key) 
  {
    switch(key)
    {
      case 'A':
        changeMode(TEMP);
        break;
      case 'B':
        changeMode(SDSELECT);
        break;
      default:
        ;
    }
  }

  bool keyhandler_TEMP(char key) { return false; }
  bool keyhandler_SDSELECT(char key) { return false; }

  void switchmode_TEMP()
  {
    currentmode = TEMP;
    LCD.clear();
    LCD.write("Hotend:",7);
    LCD.write(TEMPERATURE.getHotend(),1000);
    LCD.write(':');
    LCD.write(TEMPERATURE.getHotendST(),100);
    if(LCD_Y > 1)
    {
      LCD.setCursor(0,1);
      LCD.write("Bed   :",7);
      LCD.write(TEMPERATURE.getPlatform(),1000);
      LCD.write(':');
      LCD.write(TEMPERATURE.getPlatformST(),100);
    }
    if(LCD_Y > 3)
    {
      LCD.setCursor(0,3);
      LCD.write("SJFW  ScribbleJ",15); 
    }
  }

  void updateTemp()
  {
    if(currentmode != TEMP)
      return;

    LCD.home();
    LCD.write("Hotend:",7);
    LCD.write(TEMPERATURE.getHotend(),1000);
    LCD.write(':');
    LCD.write(TEMPERATURE.getHotendST(),100);
    if(LCD_Y > 1)
    {
      LCD.setCursor(0,1);
      LCD.write("Bed   :",7);
      LCD.write(TEMPERATURE.getPlatform(),1000);
      LCD.write(':');
      LCD.write(TEMPERATURE.getPlatformST(),100);
    }
    if(LCD_Y > 3)
    {
      LCD.setCursor(0,3);
      LCD.write("SJFW  ScribbleJ",15); 
    }
  }

  void switchmode_SDSELECT()
  {
    currentmode = SDSELECT;
    LCD.clear();
    LCD.write("SD File Select", 14);
    if(LCD_Y > 3)
    {
      LCD.setCursor(0,3);
      LCD.write("SJFW  ScribbleJ",15); 
    }
      
  }

};


#endif
#endif

