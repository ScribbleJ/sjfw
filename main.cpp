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
#include "SDCard.h"

#ifdef HAS_LCD
uint8_t _lcd_linestarts[] = LCD_LINESTARTS;
#endif

int main(void)
{
  sei();
  init_time();
  HOST.write("start\n");


  sdcard::reset();
  char cf[32];
  memset(cf, 0, 32);
  sdcard::SdErrorCode e;
  e = sdcard::directoryReset();
  if(e != sdcard::SD_SUCCESS)
  {
    // If I change this line to HOST.labelnum("SD Card fail ", e) then it will timeout
    // during upload of the firmware to RAMPS1.3.  No, I'm not kidding!  Somehow something
    // in the code causes the UPLOAD of the code to fail.  This makes no sense, but is 
    // absolutely true.
    HOST.write("SD Card fail.\n");
    if(e == 2)
      HOST.write("ISTOO\n");

  }


  do {
    e = sdcard::directoryNextEntry(cf,32);
  } while (e == sdcard::SD_SUCCESS && cf[0] == '.');
  if(e != sdcard::SD_SUCCESS)
  {
    HOST.write("SDNEXT FAIL\n");
  }
  else if(cf[0] == 0)
  {
    HOST.write("SD NO FILE.\n");
  }
  else
  {
    HOST.write("Read OK."); HOST.write("\n");
    HOST.write(cf); HOST.write("\n");
  }


  // SD Card autorun only will occur if you also have an LCD.  
  // Otherwise, it seems dangerous...
#ifdef HAS_SD
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
      LCD.writestr("Hotend:",7);
      LCD.writeint16(TEMPERATURE.getHotend(),1000);
      LCD.write(':');
      LCD.writeint16(TEMPERATURE.getHotendST(),100);
      LCD.setCursor(0,1);
      LCD.writestr("Bed   :",7);
      LCD.writeint16(TEMPERATURE.getPlatform(),1000);
      LCD.write(':');
      LCD.writeint16(TEMPERATURE.getPlatformST(),100);

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

#ifdef HAS_SD
    // Manage SD card operations
    sdcard::update();
#endif
  }

  return 0;
}


