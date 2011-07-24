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

#define HOST_BUFSIZE 1024
#define HOST_BAUD 57600
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
    uint8_t rxchars() { uint8_t l; unsigned char sreg=SREG; cli(); l = rxring.getLength(); SREG=sreg; return l; }
    uint8_t popchar() { uint8_t c; unsigned char sreg=SREG; cli(); c = rxring.pop(); SREG=sreg; return c; }
    uint8_t peekchar() { uint8_t c; unsigned char sreg=SREG; c = rxring[0]; SREG=sreg; return c; }

    void write(uint8_t data) { unsigned char sreg=SREG; cli(); txring.push(data); SREG=sreg;   UCSR0B |= MASK(UDRIE0); }
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



    void labelnum(const char *label, uint16_t num, bool endl)
    {
      write(label);
      write(num,10);
      if(endl)
        write("\n");
    }
    void labelnum(const char *label, uint16_t num) { labelnum(label, num, true); };
    void labelnum(const char *label, int num) { labelnum(label, (uint16_t)num, true); };
    void labelnum(const char *label, unsigned long num) { labelnum(label, (uint16_t)num, true); };
    void labelnum(const char *label, int num, bool endl) { labelnum(label, (uint16_t)num, endl); };
    void labelnum(const char *label, unsigned long num,bool endl) { labelnum(label, (uint16_t)num, endl); };



    void labelnum(const char *label, float num, bool endl)
    {
      write(label);
      write(num,10,4);
      if(endl)
        write("\n");
    }
    void labelnum(const char *label, float num) { labelnum(label, num, true); };



    void rxerror(const char*errmsg, uint16_t linenum)
    {
      write("rs ");
      write(linenum, 10);
      write(' ');
      write(errmsg);
      write("\n");
      unsigned char sreg=SREG;
      cli();
      rxring.reset();
      input_ready=0;
      SREG=sreg;
    }

    void rxerror(const char* errmsg)
    {
      write("!! ");
      write(errmsg);
      write("\n");
      unsigned char sreg=SREG;
      cli();
      rxring.reset();
      input_ready=0;
      SREG=sreg;
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
