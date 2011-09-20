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
    // Rather than interleaving, we'll just run it twice in succession, to catch instances where it
    // wants an immediate rerun, otherwise assume it's good 'till we get back.
    GCODES.handlenext();
    GCODES.checkaxes();
      
    // Checks to see if recieve buffer has enough data to parse, and sends it to 
    // GcodeQueue.h for processing if so.
    HOST.scan_input();

    // Updates temperature information; scans temperature sources
    TEMPERATURE.update();

    // Manage Eeprom operations
    eeprom::update();

#ifdef HAS_LCD
    // Update LCD, read keypresses, etc.
    LCDKEYPAD.handleUpdates();
#endif    

#ifdef HAS_SD
    // Manage SD card operations
    sdcard::update();
#endif

#ifdef HAS_BT
    // Look for incoming data
    BT.scan_input();
#endif

#ifdef USE_MARLIN
    Marlin::update();
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


