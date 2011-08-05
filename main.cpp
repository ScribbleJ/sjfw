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
  // TODO: check to see whether interleaving calls to GCODES.handlenext really gains me anything.
  for (;;) { 
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

    // Checks to see if gcodes are waiting to run and runs them if so.
    GCODES.handlenext();
#endif

#ifdef HAS_LCD
    // Update LCD, read keypresses, etc.
    LCDKEYPAD.handleUpdates();
#endif    
  }

  return 0;
}


