// (c) 2011, Christopher "ScribbleJ" Jansen
// Licence: GPL or whatever.

#include "Gcodes.h"
#include "pins.h"
#include "Host.h"
#include "ExtruderCommands.h"
#include "MBIEC.h"
#include "Time.h"
#include <avr/interrupt.h>
#include "Globals.h"

uint8_t _lcd_linestarts[] = LCD_LINESTARTS;
int main(void)
{
  sei();
  init_time();
  HOST.write("start\n");
  LCD.begin(LCD_X,LCD_Y,_lcd_linestarts);
  LCD.noAutoscroll();
  LCD.clear();
  LCD.writestr("start",5);


  unsigned long last_lcdupdate = millis();
  unsigned long last_lcdrefresh = last_lcdupdate;
  for (;;) { 
    unsigned long now = millis();
    if(now - last_lcdupdate > 100)
    {
      last_lcdupdate = now;
      LCD.handleUpdates();
    }

    if(now - last_lcdrefresh > 5000)
    {
      last_lcdrefresh = now;
      LCD.setCursor(0,0);
      LCD.write('T');
      LCD.writeint16(EC.getHotend(),100);
      LCD.write(':');
      LCD.writeint16(EC.getHotendST(),100);
      LCD.write('B');
      LCD.writeint16(EC.getPlatform(),100);
      LCD.write(':');
      LCD.writeint16(EC.getPlatformST(),100);

    }



    // Checks to see if gcodes are waiting to run and runs them if so.
    GCODES.handlenext();
      
    // Checks to see if recieve buffer has enough data to parse, and sends it to 
    // Gcodes.h for processing if so.
    HOST.scan_input();

    // We'll interleave calls to this, since it's so important.
    GCODES.handlenext();

    // Checks for data coming in from EC and handles it.
    EC.scan_input();

    // We'll interleave calls to this, since it's so important.
    GCODES.handlenext();

    // Runs regular temperature queries against the EC, also reports errors when
    // EC is unresponsive for too long.
    EC.update();
  }

  return 0;
}


