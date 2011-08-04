// (c) 2011, Christopher "ScribbleJ" Jansen
// Licence: GPL or whatever.

#include "GcodeQueue.h"
#include "Host.h"
#include "ExtruderCommands.h"
#include "Temperature.h"
#include "Time.h"
#include <avr/interrupt.h>
#include "Globals.h"
#include "config.h"
#include "SDCard.h"
int main(void)
{
  sei();
  init_time();
  HOST.write("start\n");

#ifdef HAS_SD
  sdcard::reset();
  // SD Card autorun only will occur if you also have an LCD.  
  // Otherwise, it seems dangerous...
#ifdef HAS_LCD
#ifdef SD_AUTORUN
  if(sdcard::autorun())
  {
    HOST.write("AUTORUN START\n");
  }
  else
  {
    HOST.write("AUTORUN FAIL\n");
  }
#endif
#endif
#endif
  
#ifdef HAS_LCD
  LCD.clear();
  LCD.write("start",5);
  LCD.setCursor(0,1);
  LCD.write("ScribbleJ",9);

  char lastkey = 0;
  unsigned long last_lcdrefresh = millis();
  for (;;) { 

#ifdef HAS_KEYPAD
    char pressedkey = KEYPAD.getPressedKey();
    if(pressedkey && pressedkey != lastkey)
    {
      HOST.write("PK:"); HOST.write(pressedkey); HOST.endl();
    }
    lastkey = pressedkey;
#endif    

    unsigned long now = millis();

    if(now - last_lcdrefresh > 1001)
    {
      last_lcdrefresh = now;
      LCD.home();
      LCD.write("Hotend:",7);
      LCD.write(TEMPERATURE.getHotend(),1000);
      LCD.write(':');
      LCD.write(TEMPERATURE.getHotendST(),100);
      LCD.setCursor(0,1);
      LCD.write("Bed   :",7);
      LCD.write(TEMPERATURE.getPlatform(),1000);
      LCD.write(':');
      LCD.write(TEMPERATURE.getPlatformST(),100);
    }

    LCD.handleUpdates();
#else
  for (;;) { 
#endif

    // Checks to see if gcodes are waiting to run and runs them if so.
    GCODES.handlenext();
      
    // Checks to see if recieve buffer has enough data to parse, and sends it to 
    // GcodeQueue.h for processing if so.
    HOST.scan_input();

    // Checks to see if gcodes are waiting to run and runs them if so.
    GCODES.handlenext();
 
    // Updates temperature information; scans temperature sources
    TEMPERATURE.update();

    // Checks to see if gcodes are waiting to run and runs them if so.
    GCODES.handlenext();

#ifdef HAS_SD
    // Manage SD card operations
    sdcard::update();
#endif
  }

  return 0;
}


