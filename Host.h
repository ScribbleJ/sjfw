#ifndef _HOST_H_
#define _HOST_H_

/* Serial comms - port1
   Chris "ScribbleJ" Jansen
   (c)2011

   Loosely based on Teacup serial library
*/

#include "CircularBuffer.h"
#include <avr/interrupt.h>

#define HOST_BUFSIZE 256
#define HOST_BAUD 115200
#define MASK(PIN) (1 << PIN)

class Host
{
  public:
    // TODO: Should be made into a proper Singleton pattern, since it is.
    Host(unsigned long baudrate);

    uint8_t rxchars() { uint8_t l; cli(); l = rxring.getLength(); sei(); return l; }
    uint8_t popchar() { uint8_t c; cli(); c = rxring.pop(); sei(); return c; }
    void write(uint8_t data) { cli(); txring.push(data); sei();   UCSR0B |= MASK(UDRIE0); }
    void write(const char *data) { uint8_t i = 0, r; while ((r = data[i++])) write(r); }
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
      if(txring.getLength() > 0)
        UDR0 = txring.pop();
      else
        UCSR0B &= ~MASK(UDRIE0);
    }

  private:
    uint8_t rxbuf[HOST_BUFSIZE];
    CircularBufferTempl<uint8_t> rxring;
    uint8_t txbuf[HOST_BUFSIZE];
    CircularBufferTempl<uint8_t> txring;
};

extern Host HOST;


#endif
