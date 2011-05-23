// Illustrative code demoing EC comms class.
// (c) 2011, Christopher "ScribbleJ" Jansen
// Licence: GPL or whatever.

#include <avr/interrupt.h>
#include "pins.h"
#include "serial.h"
#include "ExtruderCommands.h"
#include "MBIEC.h"


int main(void)
{
  sei();
  serial_init(115200);

  for (;;) { 
    int len = EC.rxchars();
    if(len)
    {
      uint8_t db = EC.popchar();
      serial_labelnum("CHAR: ", db); // sends a copy of each character to the host
      uint16_t param = EC.handle_rx_char(db);
      if(param != 0) 
      {
        serial_labelnum("PARAM: ", param);
      }
    }


    len = serial_rxchars();
    if(len)
    {
      uint8_t host = serial_popchar();
      switch(host)
      {
        case 'A':
          serial_writestr("Doing SLAVE_CMD_GET_TEMP\r\n");
          EC.dotoolreq(SLAVE_CMD_GET_TEMP);
          break;
        case 'B':
          serial_writestr("Doing SLAVE_CMD_GET_PLATFORM_TEMP\r\n");
          EC.dotoolreq(SLAVE_CMD_GET_PLATFORM_TEMP);
          break;
        case 'C':
          serial_writestr("Doing SLAVE_CMD_GET_SP\r\n");
          EC.dotoolreq(SLAVE_CMD_GET_SP);
          break;
        case 'D':
          serial_writestr("Doing SLAVE_CMD_GET_PLATFORM_SP\r\n");
          EC.dotoolreq(SLAVE_CMD_GET_PLATFORM_SP);
          break;
        case 'a':
          serial_labelnum("HOTEND TEMP: ", EC.getHotend());
          break;
        case 'b':
          serial_labelnum("PLATFORM TEMP: ", EC.getPlatform());
          break;
        case 'c':
          serial_labelnum("HOTEND SET TEMP: ", EC.getHotendST());
          break;
        case 'd':
          serial_labelnum("PLATFORM SET TEMP: ", EC.getPlatformST());
          break;
      }
    }
  }

  return 0;
}


