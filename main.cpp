// (c) 2011, Christopher "ScribbleJ" Jansen
// Licence: GPL or whatever.

#include "GcodeQueue.h"
#include "Host.h"
#include "Temperature.h"
#include "Time.h"
#include <avr/interrupt.h>
#include "Globals.h"
#include "Eeprom.h"
#ifdef USE_MARLIN
#include "Marlin.h"
#endif

#ifdef HAS_LCD
#include "LCDKeypad.h"
extern LCDKeypad LCDKEYPAD;
#endif 
#ifdef HAS_SD
#include "SDCard.h"
#endif

#include "config.h"


void mainloop()
{
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

    // Manage Eeprom operations
    eeprom::update();

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

    // Checks to see if gcodes are waiting to run and runs them if so.
    GCODES.handlenext();
#endif    

#ifdef HAS_BT
    // Look for incoming data
    BT.scan_input();
#endif

  }


}




int main(void)
{
  sei();
  init_time();

#ifdef HAS_SD
  sdcard::reset();
#ifdef SD_AUTORUN
  if(sdcard::autorun())
  {
    HOST.write("AUTORUN GOOD\n");
#ifdef HAS_BT
    BT.write("AUTORUN GOOD\n");
#endif
  }
  else
  {
    HOST.write("AUTORUN FAIL\n");
#ifdef HAS_BT
    BT.write("AUTORUN FAIL\n");
#endif    
    eeprom::beginRead();
  }
#endif
#else
  eeprom::beginRead();
#endif

  HOST.write_P(PSTR("start\n"));
#ifdef HAS_BT
  BT.write_P(PSTR("start\n"));
#endif  

#ifdef USE_MARLIN
  Marlin::init();
#endif  

  mainloop();

  return 0;
}


