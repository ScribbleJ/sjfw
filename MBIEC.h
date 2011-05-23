#ifndef _MBIEC_H_
#define _MBIEC_H_

/* RS485 MBI Extruder Controller Comms
   Chris "ScribbleJ" Jansen
   (c)2011

   This class handles setting up the second comm port on a AVR 
   to communicate via RS485 with a MBI Extruder Controller running stock MBI
   firmware (tested with 2.8).  

   Uses different methods to set pins as it's based on code from various places and
   not what I'd call fully-integrated (yet).
*/

#include "AvrPort.h"
#include "CircularBuffer.h"

#define MBIEC_BUFSIZE 16
#define MASK(PIN) (1 << PIN)

class MBIEC
{
  public:
    // TODO: Should be made into a proper Singleton pattern, since it is.
    MBIEC(Pin tx_enable, Pin rx_enable);

    uint8_t rxchars() { return rxring.getLength(); };
    uint8_t popchar() { return rxring.pop(); };
    void write(uint8_t data) { txring.push(data); };
    uint16_t handle_rx_char(uint8_t c);
    void    dotoolreq(uint8_t command_id, uint16_t param);
    void    dotoolreq(uint8_t command_id);

    void rx_interrupt_handler()
    {
      uint8_t rc = UDR1;
      if(loopbytes == 0)
      {
        rxring.push(rc);
      }
      else
        loopbytes--;
    }

    void tx_interrupt_handler()
    {
      if(txring.getLength() == 0)
      {
        listen();
      }
      else if (TX_ENABLE_PIN.getValue()==1)
      {
        UDR1 = txring.pop();
        loopbytes++;
      }
      else  TX_ENABLE_PIN.setValue(true);
    }

  private:
    Pin TX_ENABLE_PIN;
    Pin RX_ENABLE_PIN;
    uint8_t rxbuf[MBIEC_BUFSIZE];
    CircularBufferTempl<uint8_t> rxring;
    uint8_t txbuf[MBIEC_BUFSIZE];
    CircularBufferTempl<uint8_t> txring;
    volatile uint8_t loopbytes;
    void speak()
    {
      TX_ENABLE_PIN.setValue(true);
      UDR1 = txring.pop();
      loopbytes++;
    }
    void listen() { TX_ENABLE_PIN.setValue(false); }
    uint8_t crc_update (uint8_t crc, uint8_t data);
};

extern MBIEC EC;


#endif
