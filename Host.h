#ifndef _HOST_H_
#define _HOST_H_

/* Serial comms - port1
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

#define HOST_BUFSIZE 16
#define HOST_BAUD 115200
#define MASK(PIN) (1 << PIN)

class Host
{
  public:
    // TODO: Should be made into a proper Singleton pattern, since it is.
    Host(uint16_t baudrate);

    uint8_t rxchars() { return rxring.getLength(); }
    uint8_t popchar() { return rxring.pop(); }
    void write(uint8_t data) { txring.push(data);   UCSR0B |= MASK(UDRIE0); }
    void write(const char *data) { while(char d = *data++) txring.push(d); }
    void write(uint16_t n, uint16_t rad) 
    {
      	for(uint16_t i = rad; i >= 1; i = i/10)
	      {
		      write((uint8_t)(n/i) + '0');
		      n -= (n/i) * i; // This looks like nonsense but it chops off the remainder.
	      }
    }

    void labelnum(const char *label, uint16_t num)
    {
      write(label);
      write(num, 100);
      write("\r\n");
    }

    void rx_interrupt_handler()
    {
      rxring.push(UDR0);
    }

    void udre_interrupt_handler()
    {
      if(txring.getLength() == 0)
      {
        // diable interrupt
        UCSR0B &= ~MASK(UDRIE0);
      }
      else
        UDR0 = txring.pop();
    }

  private:
    uint8_t rxbuf[HOST_BUFSIZE];
    CircularBufferTempl<uint8_t> rxring;
    uint8_t txbuf[HOST_BUFSIZE];
    CircularBufferTempl<uint8_t> txring;
};

extern Host HOST;


#endif
