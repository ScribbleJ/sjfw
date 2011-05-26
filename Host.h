#ifndef _HOST_H_
#define _HOST_H_

/* Serial comms - port1
   Chris "ScribbleJ" Jansen
   (c)2011

   Loosely based on Teacup serial library
*/

#include "CircularBuffer.h"
#include <avr/interrupt.h>
#include <stdlib.h>

#define HOST_BUFSIZE 128
#define HOST_BAUD 115200
#define MASK(PIN) (1 << PIN)

class Host
{
  public:
    // singleton
    static Host& Instance() { static Host instance(HOST_BAUD); return instance; }
  private:
    explicit Host(unsigned long baud);
    Host(Host&);
    Host& operator=(Host&);
  public:

    // We do full-duplex here and fast too, so if cli() and sei() aren't called it will cause corruption.
    uint8_t rxchars() { uint8_t l; cli(); l = rxring.getLength(); sei(); return l; }
    uint8_t popchar() { uint8_t c; cli(); c = rxring.pop(); sei(); return c; }
    uint8_t peekchar() { uint8_t c; cli(); c = rxring[0]; sei(); return c; }

    void write(uint8_t data) { cli(); txring.push(data); sei();   UCSR0B |= MASK(UDRIE0); }
    void write(const char *data) { uint8_t i = 0, r; while ((r = data[i++])) write(r); }
    void write(unsigned long n, int radix)
    {
      static const char bufsize = 8 * sizeof(unsigned long) + 1;
      char buf[bufsize];
      ultoa(n,buf,radix);
      write(buf);
    }
    void write(float n, signed char width, unsigned char prec)
    {
      static const char bufsize = 8 * sizeof(float) + 1;
      char buf[bufsize];
      dtostrf(n,width,prec,buf);
      write(buf);
    }



    void labelnum(const char *label, uint16_t num)
    {
      write(label);
      write(num,10);
      write("\r\n");
    }

    void rxerror(const char*errmsg, uint16_t linenum)
    {
      write("rs ");
      write(linenum, 10);
      write(' ');
      write(errmsg);
      write("\r\n");
      cli();
      rxring.reset();
      input_ready=0;
      sei();
    }

    void rxerror(const char* errmsg)
    {
      rxerror(errmsg, 0);
    }

    void scan_input();

    void rx_interrupt_handler()
    {
      uint8_t c = UDR0;
      rxring.push(c);
      if(c <= 32)
        input_ready++;
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
    volatile unsigned int input_ready;

};

extern Host& HOST;


#endif
