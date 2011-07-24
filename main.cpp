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
  HOST.write("start\n");

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

    // Checks for data coming in from EC and handles it.
    EC.scan_input();

    // Runs regular temperature queries against the EC, also reports errors when
    // EC is unresponsive for too long.
    EC.update();
  }

  return 0;
}


