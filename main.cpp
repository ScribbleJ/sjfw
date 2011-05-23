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
      if(host == 'A')
      {
        serial_writestr("Doing SLAVE_CMD_GET_TEMP\r\n");
        EC.dotoolreq(SLAVE_CMD_GET_TEMP);
      }
    }
  }

  return 0;
}


