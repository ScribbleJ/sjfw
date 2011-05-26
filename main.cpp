// (c) 2011, Christopher "ScribbleJ" Jansen
// Licence: GPL or whatever.

#include "Gcodes.h"
#include "pins.h"
#include "Host.h"
#include "ExtruderCommands.h"
#include "MBIEC.h"
#include "Time.h"
#include <avr/interrupt.h>

int main(void)
{
  sei();
  init_time();
  HOST.write("start\r\n");

  unsigned long last_time = millis();
  for (;;) { 
    unsigned long now = millis();
    if(now - last_time > 1000)
    {
      last_time = now;
      // Lax one second timer
    }

    // Checks to see if gcodes are waiting to run and runs them if so.
    GCODES.handlenext();
      
    // Checks to see if recieve buffer has enough data to parse, and sends it to 
    // Gcodes.h for processing if so.
    HOST.scan_input();
  
    int len = EC.rxchars();
    if(len)
    {
      uint8_t db = EC.popchar();
      HOST.labelnum("CHAR: ", db); // sends a copy of each character to the host
      uint16_t param = EC.handle_rx_char(db);
      if(param != 0) 
      {
        HOST.labelnum("PARAM: ", param);
      }
    }
  }

  return 0;
}


