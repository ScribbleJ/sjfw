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
#include "Temperature.h"
#include "GCode.h"
#include "GcodeQueue.h"
#include "Motion.h"

#ifdef HAS_SD
#include "SDCard.h"
#include "fat.h"
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
  enum MODE { TEMP, SDSELECT, MOTORS, MENU, MODS };


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
    MOD_AXIS=0;
    lcd_x = LCD.getCols();
    lcd_y = LCD.getRows();
    switchmode_TEMP();
  }

  void reinit()
  {
    LCD.reinit();
    LCD.clear();
    switchmode_TEMP();
    display_TEMP();
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

#ifdef HAS_SD
      update_SDSELECT();
			update_MODS();
#endif
      
#ifdef HAS_KEYPAD
      update_MOTORS();
#endif      
    }

  }

  void setRS(int pin) { LCD.setRS(pin); reinit(); }
  void setRW(int pin) { LCD.setRW(pin); reinit(); }
  void setE(int pin) { LCD.setE(pin); }
  void setD(int n, int pin) { LCD.setD(n, pin); reinit(); }
#ifdef HAS_KEYPAD  
  void setRowPin(int n, int pin) { KEYPAD.setRowPin(n, pin); }
  void setColPin(int n, int pin) { KEYPAD.setColPin(n, pin); }

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
      case MODS:
        switchmode_MODS();
        break;
      default:
        break;
    }
  }
#else
  void setRowPin(int n, int pin) { ; }
  void setColPin(int n, int pin) { ; }
#endif  

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
  int MOD_AXIS;
  bool extrude;
  char tgraph[8*8];
  int lcd_x;
  int lcd_y;


  
#ifdef HAS_KEYPAD  
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
      case MODS:
        handled = keyhandler_MODS(key);
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
        changeMode(MODS);
        break;
      default:
        ;
    }
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


  bool keyhandler_MENU(char key) { return false; }

  
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
#ifdef REVERSE_PRINTER
        t[Y].setFloat(p[Y] + motordistance);
#else
        t[Y].setFloat(p[Y] - motordistance);
#endif
        break;
      case '8':
        t[G].setInt(1);
#ifdef REVERSE_PRINTER
        t[Y].setFloat(p[Y] - motordistance);
#else
        t[Y].setFloat(p[Y] + motordistance);
#endif
        break;
      case '4':
        t[G].setInt(1);
#ifdef REVERSE_PRINTER
        t[X].setFloat(p[X] + motordistance);
#else
        t[X].setFloat(p[X] - motordistance);
#endif
        break;
      case '6':
        t[G].setInt(1);
#ifdef REVERSE_PRINTER
        t[X].setFloat(p[X] - motordistance);
#else
        t[X].setFloat(p[X] + motordistance);
#endif
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
        while(!TEMPERATURE.setHotend(0));
        handled = true;
        break;
      case '6':
        while(!TEMPERATURE.setPlatform(0));
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
      case 'A':
        reinit();
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
    while(!TEMPERATURE.setHotend(now));
  }

  void adjustTempBed(int diff)
  {
    int16_t now = TEMPERATURE.getPlatformST();
    now += diff;
    if(now < 0)
      now = 0;
    while(!TEMPERATURE.setPlatform(now));
  }
#endif  


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
    LCD.clear();
    LCD.home();
    int t=0;
    if(lcd_x>16)
    {
      LCD.write_P(PSTR("Hotend:    /   "));
      LCD.setCursor(8,0);
    }
    else
    {
      LCD.write_P(PSTR("H:   /   "));
      LCD.setCursor(2,0);
    }
    t = TEMPERATURE.getHotend();
    if(t == 1024)
      LCD.write_P(PSTR("MISSING!"));
    else
    {
      LCD.write(t);
      LCD.setCursor(lcd_x > 16 ? 11 : 6,0);
      LCD.write_P(PSTR("/ "));
      LCD.write(TEMPERATURE.getHotendST());
    }
    LCD.setCursor(0,1);
    if(lcd_x>16)
    {
      LCD.write_P(PSTR("Bed   :    /   "));
      LCD.setCursor(8,1);
    }
    else
    {
      LCD.write_P(PSTR("B:   /   "));
      LCD.setCursor(2,1);
    }

    t = TEMPERATURE.getPlatform();
    if(t == 1024)
      LCD.write_P(PSTR("MISSING!"));
    else
    {
      LCD.write(t);
      LCD.setCursor(lcd_x > 16 ? 11 : 6,1);
      LCD.write_P(PSTR("/ "));
      LCD.write(TEMPERATURE.getPlatformST());
    }
    LCD.setCursor(lcd_x > 16 ? 16 : 12,0);
    LCD.write((char)0);
    LCD.write((char)2);
    LCD.write((char)4);
    LCD.write((char)6);
    LCD.setCursor(lcd_x > 16 ? 16 : 12,1);
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

    int t=0;
    if(lcd_x>16)
    {
      LCD.setCursor(8,0);
      LCD.write_P(PSTR("        "));
      LCD.setCursor(8,0);
    }
    else
    {
      LCD.setCursor(2,0);
      LCD.write_P(PSTR("        "));
      LCD.setCursor(2,0);
    }
    t = TEMPERATURE.getHotend();
    if(t == 1024)
      LCD.write_P(PSTR("MISSING!"));
    else
    {
      LCD.write(t);
      LCD.setCursor(lcd_x > 16 ? 11 : 6,0);
      LCD.write_P(PSTR("/ "));
      LCD.write(TEMPERATURE.getHotendST());
    }
    LCD.setCursor(0,1);
    if(lcd_x>16)
    {
      LCD.setCursor(8,1);
      LCD.write_P(PSTR("         "));
      LCD.setCursor(8,1);
    }
    else
    {
      LCD.setCursor(2,1);
      LCD.write_P(PSTR("        "));
      LCD.setCursor(2,1);
    }

    t = TEMPERATURE.getPlatform();
    if(t == 1024)
      LCD.write_P(PSTR("MISSING!"));
    else
    {
      LCD.write(t);
      LCD.setCursor(lcd_x > 16 ? 11 : 6,1);
      LCD.write_P(PSTR("/ "));
      LCD.write(TEMPERATURE.getPlatformST());
    }
    LCD.setCursor(lcd_x > 16 ? 16 : 12,0);
    LCD.write((char)0);
    LCD.write((char)2);
    LCD.write((char)4);
    LCD.write((char)6);
    LCD.setCursor(lcd_x > 16 ? 16 : 12,1);
    LCD.write((char)1);
    LCD.write((char)3);
    LCD.write((char)5);
    LCD.write((char)7);

    tagline();
  }

#ifdef HAS_KEYPAD
  void switchmode_MODS()
  {
    currentmode = MODS;
    display_MODS();
  }
  void display_MODS()
  {
    LCD.clear();
    LCD.label("AXIS:",MOD_AXIS);
    LCD.setCursor(10,0);
    LCD.label("MIN:",(int32_t)MOTION.getAxis(MOD_AXIS).getStartFeed());
    LCD.setCursor(0,1);
    LCD.label("ACC:",MOTION.getAxis(MOD_AXIS).getAccel()); 
    LCD.setCursor(10,1);
    LCD.label("MAX:",(int32_t)MOTION.getAxis(MOD_AXIS).getMaxFeed());
    LCD.setCursor(0,2);
    LCD.label("FD:",MOTION.getFeedModifier()); LCD.write("%");
    LCD.setCursor(9,2);
    LCD.label("STP:",MOTION.getAxis(MOD_AXIS).getStepsPerMM());

    tagline();
  }

  void update_MODS()
  {
    if(currentmode != MODS)
      return;
    tagline();
  }

  bool keyhandler_MODS(char key) 
  { 
    bool handled = false;
    float t;
    switch(key)
    {
      case 'A': // Switch axis, or default
      case 'B':
      case 'C':
      case 'D':
        if(key - 'A' == MOD_AXIS)
          break;
        MOD_AXIS = key - 'A';
        handled = true;
        break;
      case '1': // Lower Minimum Feedrate
        t = MOTION.getAxis(MOD_AXIS).getStartFeed();
        t-=100;
        if(t>=100)
        {
          MOTION.getAxis(MOD_AXIS).setMinimumFeedrate(t);
          handled = true;
        }
        break;
      case '3': // Raise Minimum Feed
        t = MOTION.getAxis(MOD_AXIS).getStartFeed();
        t+=100;
        if(t<=MOTION.getAxis(MOD_AXIS).getMaxFeed())
        {
          MOTION.getAxis(MOD_AXIS).setMinimumFeedrate(t);
          handled = true;
        }
        break;
      case '7': // Lower Max Feed
        t = MOTION.getAxis(MOD_AXIS).getMaxFeed();
        t-=100;
        if(t>=MOTION.getAxis(MOD_AXIS).getStartFeed())
        {
          MOTION.getAxis(MOD_AXIS).setMaximumFeedrate(t);
          handled = true;
        }
        break;
      case '9':  // Raise Max Feed
        t = MOTION.getAxis(MOD_AXIS).getMaxFeed();
        t+=100;
        MOTION.getAxis(MOD_AXIS).setMaximumFeedrate(t);
        handled = true;
        break;
      case '4':  // Lower Accel
        t = MOTION.getAxis(MOD_AXIS).getAccel();
        t-=100;
        if(t>=100)
        {
          MOTION.getAxis(MOD_AXIS).setAccel(t);
          handled = true;
        }
        break;
      case '6':  // Raise Accel
        t = MOTION.getAxis(MOD_AXIS).getAccel();
        t+=100;
        MOTION.getAxis(MOD_AXIS).setAccel(t);
        handled = true;
        break;
      case '2': // jog axis positive
        switch(MOD_AXIS)
        {
          case 0:
          case 1:
            MOTION.getAxis(MOD_AXIS).setCurrentPosition(
              MOTION.getAxis(MOD_AXIS).getCurrentPosition() - 1.0f);
            break;
          case 2:
            MOTION.getAxis(MOD_AXIS).setCurrentPosition(
              MOTION.getAxis(MOD_AXIS).getCurrentPosition() - 0.1f);
            break;
          case 3:
            MOTION.getAxis(3).setStepsPerUnit(
              MOTION.getAxis(3).getStepsPerMM() + 10);
            break;
        }
        handled = true;
        break;
      case '8': // jog axis negative
        switch(MOD_AXIS)
        {
          case 0:
          case 1:
            MOTION.getAxis(MOD_AXIS).setCurrentPosition(
              MOTION.getAxis(MOD_AXIS).getCurrentPosition() + 1.0f);
            break;
          case 2:
            MOTION.getAxis(MOD_AXIS).setCurrentPosition(
              MOTION.getAxis(MOD_AXIS).getCurrentPosition() + 0.1f);
            break;
          case 3:
            MOTION.getAxis(3).setStepsPerUnit(
              MOTION.getAxis(3).getStepsPerMM() - 10);
            break;
        }
        handled = true;
        break;
      case '*': // Lower feed percentage
        t = MOTION.getFeedModifier();
        t -= 5.0f;
        if(t<LOW_FEED_MODIFIER) t = LOW_FEED_MODIFIER;
        MOTION.setFeedModifier(t);
        handled = true;
        break;
      case '#': // Raise feed percentage
        MOTION.setFeedModifier(MOTION.getFeedModifier() + 5.0f);
        handled = true;
        break;
      case '0': // Toggle fan
        GCode::togglefan();
        handled = true;
        break;
      case '5': // Toggle pause
        GCODES.togglepause();
        handled = true;
        break;
    }
    if(handled)
      display_MODS();
    return(handled);
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
    if(lcd_y > 2)
    {
      LCD.setCursor(8,1);
      LCD.label("E:", lastpos[E]);
      LCD.setCursor(0,2);
      LCD.label("Move:", motordistance);
      if(lcd_x > 16)
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
    if(lcd_x>16)
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

  void switchmode_MENU()
  {
    currentmode = MENU;
    LCD.clear();
    LCD.write_P(PSTR("MAIN MENU"));
  }

    
  void switchmode_SDSELECT()
  {
    currentmode = SDSELECT;
    LCD.clear();
    display_SDSELECT();
  }

  void update_SDSELECT()
  {
    if(currentmode != SDSELECT)
      return;
    tagline();
  }

  void display_SDSELECT()
  {
#ifdef HAS_SD    
    if(sdcard::isReading())
    {
      LCD.write_P(PSTR("Printing: "));
      LCD.setCursor(0,1);
      LCD.write(sdcard::getCurrentfile());
      tagline();
      return;
    }
    if(sdcard::getCurrentfile()[0] == 0)
      sdcard::getNextfile();

    LCD.write_P(PSTR("SD File Select"));
    LCD.setCursor(0,1);
    LCD.write_P(PSTR("> "));
    LCD.write(sdcard::getCurrentfile());
    if(lcd_y > 2)
    {
      LCD.setCursor(0,2);
      LCD.write_P(PSTR("6=NEXT #=PRINT"));
    }
#else
    LCD.write_P(PSTR("Get SD from"));
    LCD.setCursor(0,1);
    LCD.write_P(PSTR("Kliment on IRC"));
#endif    
    tagline();
  }
#endif  

  void tagline()
  {
    if(lcd_y < 4)
      return;
    LCD.setCursor(0,3);
#ifdef HAS_SD
    if(sdcard::isReading())
    {
      LCD.write_P(PSTR("SD Print: "));
      LCD.write((uint16_t)(sdcard::getFilePos() * 100 / sdcard::getFileSize()));
      LCD.write_P(PSTR("%"));
		} else {
#endif
    	LCD.write_P(PSTR(LCD_TAGLINE));
#ifdef HAS_SD
		}
#endif
  }

};


#endif
#endif

