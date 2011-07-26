// (c) 2011, Christopher "ScribbleJ" Jansen
// Licence: GPL or whatever.

#include "Gcodes.h"
#include "Host.h"
#include "ExtruderCommands.h"
#include "Temperature.h"
#include "Time.h"
#include <avr/interrupt.h>
#include "Globals.h"
#include "config.h"

#ifdef HAS_LCD
uint8_t _lcd_linestarts[] = LCD_LINESTARTS;
#endif

int main(void)
{
  sei();
  init_time();
  HOST.write("start\n");

#ifdef HAS_LCD
  LCD.begin(LCD_X,LCD_Y,_lcd_linestarts);
  LCD.noAutoscroll();
  LCD.clear();
  LCD.home();
  LCD.writestr("start",5);
  LCD.setCursor(0,1);
  LCD.writestr("ScribbleJ",9);


  unsigned long last_lcdupdate = millis();
  unsigned long last_lcdrefresh = last_lcdupdate;
  for (;;) { 
    unsigned long now = millis();
    if(now - last_lcdupdate > 50)
    {
      last_lcdupdate = now;
      LCD.handleUpdates();
    }

    if(now - last_lcdrefresh > 5000)
    {
      last_lcdrefresh = now;
      LCD.home();
      LCD.write('T');
      LCD.writeint16(TEMPERATURE.getHotend(),1000);
      LCD.write(':');
      LCD.writeint16(TEMPERATURE.getHotendST(),1000);
      LCD.setCursor(0,1);
      LCD.write('B');
      LCD.writeint16(TEMPERATURE.getPlatform(),1000);
      LCD.write(':');
      LCD.writeint16(TEMPERATURE.getPlatformST(),1000);

    }
#else
  for (;;) { 
#endif



    // Checks to see if gcodes are waiting to run and runs them if so.
    GCODES.handlenext();
      
    // Checks to see if recieve buffer has enough data to parse, and sends it to 
    // Gcodes.h for processing if so.
    HOST.scan_input();

    // We'll interleave calls to this, since it's so important.
    GCODES.handlenext();

    // Updates temperature information; scans temperature sources
    TEMPERATURE.update();
  }

  return 0;
}


