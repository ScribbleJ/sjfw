#ifndef _LCDKEYPAD_H_
#define _LCDKEYPAD_H_
#ifdef HAS_LCD
/* LCD and Keypad support for sjfw -
 * (c) 2011, Christopher "ScribbleJ" Jansen
 *
 * This class provides a system of LCD menus navigated by button presses on an attached keypad.
 *
 */

#include "config.h"
#include "Time.h"
#include "LiquidCrystal.h"
#include "Host.h"
#include "Temperature.h"
#include "GCode.h"
#include "GcodeQueue.h"

#ifdef HAS_SD
#include "SDCard.h"
#endif

// These bits of global data are used by the LCD and Keypad handler classes.
#ifdef HAS_KEYPAD
#include "Keypad.h"
extern const char *_kp_buttonmap[];
extern const Pin _kp_rpins[];
extern const Pin _kp_cpins[];
#endif
extern uint8_t _lcd_linestarts[];
extern float const RATES_OF_CHANGE[];

class LCDKeypad
{
public:
  enum MODE { TEMP, SDSELECT, MOTORS, MENU };


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
    motordistance_roc = &RATES_OF_CHANGE[0];
    ROC_START = motordistance_roc;
    switchmode_TEMP();
  }

  // This needs to be called regularly e.g. from the mainloop.
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
    if(now - last_lcdrefresh > LCD_REFRESH_MILLIS)
    {
      last_lcdrefresh = now;
      update_TEMPGRAPH();
      update_TEMP();
      update_MOTORS();
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
      case MOTORS:
        switchmode_MOTORS();
        break;
      case MENU:
        switchmode_MENU();
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
  int  tempdistance;
  float motordistance;
  float const* motordistance_roc;
  float const* ROC_START;
  bool extrude;
  char tgraph[8*8];
  
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
      case MOTORS:
        handled = keyhandler_MOTORS(key);
        break;
      case MENU:
        handled = keyhandler_MENU(key);
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
      case 'C':
        changeMode(MOTORS);
        break;
      case 'D':
        changeMode(MENU);
        break;
      default:
        ;
    }
  }

  bool keyhandler_TEMP(char key)
  {
    bool handled = false;
    switch(key)
    {
      case '1':
        adjustTempHotend(5);
        handled = true;
        break;
      case '3':
        adjustTempBed(5);
        handled = true;
        break;
      case '4':
        TEMPERATURE.setHotend(0);
        handled = true;
        break;
      case '6':
        TEMPERATURE.setPlatform(0);
        handled = true;
        break;
      case '7':
        adjustTempHotend(-5);
        handled = true;
        break;
      case '9':
        adjustTempBed(-5);
        handled = true;
        break;
    }
    if(handled)
    {
      LCD.clear();
      display_TEMP();
    }
    return handled;
  }

  void adjustTempHotend(int diff)
  {
    int16_t now = TEMPERATURE.getHotendST();
    now += diff;
    if(now < 0)
      now = 0;
    TEMPERATURE.setHotend(now);
  }

  void adjustTempBed(int diff)
  {
    int16_t now = TEMPERATURE.getPlatformST();
    now += diff;
    if(now < 0)
      now = 0;
    TEMPERATURE.setPlatform(now);
  }


  void update_TEMPGRAPH()
  {
    int d=TEMPERATURE.getHotend()-TEMPERATURE.getHotendST();
    d/=2;
    if(d > 7)
      d=7;
    if(d < -8)
      d=-8;
    d+=8;
    d=15-d;

    for(int x=0;x<16;x++)
    {
      for(int y=0;y<3;y++)
      {
        tgraph[(y*16)+x] <<= 1;
        if(tgraph[16+(y*16)+x] & 0b00010000)
          tgraph[(y*16)+x] |= 1;
        tgraph[(y*16)+x] &= 0b00011111;
      }
      tgraph[x+48] <<= 1;
      if(x == d)
        tgraph[x+48] |= 1;
      tgraph[x+48] &= 0b00011111;
    }
    for(int x=0;x<8;x++)
    {
      LCD.writeCustomChar(x,tgraph+(x*8));
    }
  }

  void display_TEMP()
  {
    int t=0;
    if(LCD_X>16)
      LCD.write("Hotend: ");
    else
      LCD.write("H:");
    t = TEMPERATURE.getHotend();
    if(t == 1024)
      LCD.write("MISSING!");
    else
    {
      LCD.write(t);
      LCD.setCursor(LCD_X > 16 ? 11 : 6,0);
      LCD.write("/ ");
      LCD.write(TEMPERATURE.getHotendST());
    }
    LCD.setCursor(0,1);
    if(LCD_X>16)
      LCD.write("Bed   : ");
    else
      LCD.write("B:");
    t = TEMPERATURE.getPlatform();
    if(t == 1024)
      LCD.write("MISSING!");
    else
    {
      LCD.write(t);
      LCD.setCursor(LCD_X > 16 ? 11 : 6,1);
      LCD.write("/ ");
      LCD.write(TEMPERATURE.getPlatformST());
    }
    LCD.setCursor(LCD_X > 16 ? 16 : 12,0);
    LCD.write((char)0);
    LCD.write((char)2);
    LCD.write((char)4);
    LCD.write((char)6);
    LCD.setCursor(LCD_X > 16 ? 16 : 12,1);
    LCD.write((char)1);
    LCD.write((char)3);
    LCD.write((char)5);
    LCD.write((char)7);

    tagline();
  }

  void switchmode_TEMP()
  {
    currentmode = TEMP;
    LCD.clear();
    display_TEMP();
  }

  void update_TEMP()
  {
    if(currentmode != TEMP)
      return;

    LCD.home();
    display_TEMP();
  }

  bool keyhandler_MOTORS(char key) 
  { 
    bool handled = false;
    switch(key)
    {
      case '1':
        motordistance += *motordistance_roc;
        return true;
      case '7':
        motordistance -= *motordistance_roc;
        if(motordistance < 0)
          motordistance = 0;
        return true;
      case '*':
        extrude = !extrude;
        return true;
      case '#':
        motordistance_roc++;
        if(*motordistance_roc == 0.0f)
        {
          motordistance_roc = ROC_START;
        }
        return true;
      case '5':
      case '2':
      case '8':
      case '4':
      case '6':
      case '3':
      case '9':
        if(GCODES.isFull())
          return true;
        handled = true;
    }
    if(!handled)
      return false;

    GCode t;
    Point& p = GCode::getLastpos();
    switch(key)
    {
      case '5':
        t[M].setInt(84);
        GCODES.enqueue(t);
        return true;
      case '2':
        t[G].setInt(1);
        t[Y].setFloat(p[Y] - motordistance);
        break;
      case '8':
        t[G].setInt(1);
        t[Y].setFloat(p[Y] + motordistance);
        break;
      case '4':
        t[G].setInt(1);
        t[X].setFloat(p[X] - motordistance);
        break;
      case '6':
        t[G].setInt(1);
        t[X].setFloat(p[X] + motordistance);
        break;
      case '3':
        t[G].setInt(1);
        t[Z].setFloat(p[Z] + motordistance);
        break;
      case '9':
        t[G].setInt(1);
        t[Z].setFloat(p[Z] - motordistance);
        break;
    }
    GCODES.enqueue(t);
    return true;
  }

  void switchmode_MOTORS()
  {
    currentmode = MOTORS;
    LCD.clear();
    display_MOTORS();
  }
  void update_MOTORS()
  {
    if(currentmode != MOTORS)
      return;
    LCD.home();
    display_MOTORS();
  }

  void display_MOTORS()
  {
    Point& lastpos = GCode::getLastpos();
    LCD.label("X:", lastpos[X]);
    LCD.setCursor(8,0);
    LCD.label("Y:", lastpos[Y]);
    LCD.setCursor(0,1);
    LCD.label("Z:", lastpos[Z]);
    if(LCD_Y > 2)
    {
      LCD.setCursor(8,1);
      LCD.label("E:", lastpos[E]);
      LCD.setCursor(0,2);
      LCD.label("Move:", motordistance);
      if(LCD_X > 16)
        LCD.label(" ROC:",*motordistance_roc);
      else
        LCD.label("/",*motordistance_roc);
      LCD.setCursor(0,3);
      LCD.label("Extruder:", extrude ? "ON " : "OFF");
    }
    else
    {
      LCD.label("I:", motordistance);
      LCD.label("/",*motordistance_roc);
      LCD.label(" E:", extrude ? "-" : "*");
    }
    if(LCD_X>16)
    {
      LCD.setCursor(16,0);
      LCD.write((char)0);
      LCD.write((char)2);
      LCD.write((char)4);
      LCD.write((char)6);
      LCD.setCursor(16,1);
      LCD.write((char)1);
      LCD.write((char)3);
      LCD.write((char)5);
      LCD.write((char)7);
    }


  }

  bool keyhandler_MENU(char key) { return false; }
  void switchmode_MENU()
  {
    currentmode = MENU;
    LCD.clear();
    LCD.write("MAIN MENU");
  }


  bool keyhandler_SDSELECT(char key) 
  { 
#ifdef HAS_SD    
    if(sdcard::isReading())
      return false;

    switch(key)
    {
      case '6':
        sdcard::getNextfile();
        switchmode_SDSELECT();
        return true;
      case '#':
        sdcard::printcurrent();
        switchmode_SDSELECT();
        return true;
      default:
        return false;
    }
#endif
    return false;
  }
    
  void switchmode_SDSELECT()
  {
    currentmode = SDSELECT;
    LCD.clear();
#ifdef HAS_SD    
    if(sdcard::isReading())
    {
      LCD.write("Printing: ");
      LCD.setCursor(0,1);
      LCD.write(sdcard::getCurrentfile());
      tagline();
      return;
    }
    if(sdcard::getCurrentfile()[0] == 0)
      sdcard::getNextfile();

    LCD.write("SD File Select");
    LCD.setCursor(0,1);
    LCD.write("> ");
    LCD.write(sdcard::getCurrentfile());
    if(LCD_Y > 2)
    {
      LCD.setCursor(0,2);
      LCD.write("6=NEXT #=PRINT");
    }
#else
    LCD.write("Get SD from");
    LCD.setCursor(0,1);
    LCD.write("Kliment on IRC");
#endif    
    tagline();
  }

  void tagline()
  {
    if(LCD_Y < 4)
      return;
    LCD.setCursor(0,3);
    if(LCD_X > 16)
      LCD.write("SJFW by ScribbleJ");
    else
      LCD.write("ScribbleJ's SJFW");
  }

};


#endif
#endif

