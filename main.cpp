// (c) 2011, Christopher "ScribbleJ" Jansen
// Licence: GPL or whatever.

#include <avr/interrupt.h>
#include "pins.h"
#include "Host.h"
#include "ExtruderCommands.h"
#include "MBIEC.h"
#include "Time.h"


int main(void)
{
  sei();
  init_time();

  for (;;) { 
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


    len = HOST.rxchars();
    if(len)
    {
      uint8_t host = HOST.popchar();
      unsigned long t = 0;
      switch(host)
      {
        case 'A':
          HOST.write("Doing SLAVE_CMD_GET_TEMP\r\n");
          EC.dotoolreq(SLAVE_CMD_GET_TEMP);
          break;
        case 'B':
          HOST.write("Doing SLAVE_CMD_GET_PLATFORM_TEMP\r\n");
          EC.dotoolreq(SLAVE_CMD_GET_PLATFORM_TEMP);
          break;
        case 'C':
          HOST.write("Doing SLAVE_CMD_GET_SP\r\n");
          EC.dotoolreq(SLAVE_CMD_GET_SP);
          break;
        case 'D':
          HOST.write("Doing SLAVE_CMD_GET_PLATFORM_SP\r\n");
          EC.dotoolreq(SLAVE_CMD_GET_PLATFORM_SP);
          break;
        case 'a':
          HOST.labelnum("HOTEND TEMP: ", EC.getHotend());
          break;
        case 'b':
          HOST.labelnum("PLATFORM TEMP: ", EC.getPlatform());
          break;
        case 'c':
          HOST.labelnum("HOTEND SET TEMP: ", EC.getHotendST());
          break;
        case 'd':
          HOST.labelnum("PLATFORM SET TEMP: ", EC.getPlatformST());
          break;
        case 'z':
          HOST.write("Doing EC Update.\r\n");
          EC.update();
          break;
        case 'T':
          t = millis();
          HOST.write("Millis: "); HOST.write(t,10);
          HOST.write("\r\n");
          break;
        case 't':
          t = micros();
          HOST.write("Micros: "); HOST.write(t,10);
          HOST.write("\r\n");
          break;
      }
    }
  }

  return 0;
}


