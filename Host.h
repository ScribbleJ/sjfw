#ifndef _HOST_H_
#define _HOST_H_

/* Serial comms - port1
   Chris "ScribbleJ" Jansen
   (c)2011

   Loosely based on Teacup serial library
*/

#include "RingBuffer.h"
#include <avr/interrupt.h>
#include <stdlib.h>
#include "config.h"

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

    uint8_t rxchars() { uint8_t l = rxring.getCount(); return l; }
    uint8_t popchar() { uint8_t c = rxring.pop(); return c; }
    uint8_t peekchar() { uint8_t c = rxring.peek(0); return c; }

    void write(uint8_t data) { txring.push(data); UCSR0B |= MASK(UDRIE0); }
    void write(const char *data) { uint8_t i = 0, r; while ((r = data[i++])) write(r); }
    void write(uint32_t n, int radix)
    {
      static const char bufsize = 8 * sizeof(uint32_t) + 1;
      char buf[bufsize];
      ultoa(n,buf,radix);
      write(buf);
    }
    void write(int32_t n, int radix)
    {
      static const char bufsize = 8 * sizeof(int32_t) + 1;
      char buf[bufsize];
      ltoa(n,buf,radix);
      write(buf);
    }
    void write(int16_t n, int radix) { write((int32_t)n, radix); }
    void write(uint16_t n, int radix) { write((int32_t)n, radix); }

    void write(float n, signed char width, unsigned char prec)
    {
      static const char bufsize = 8 * sizeof(float) + 1;
      char buf[bufsize];
      dtostrf(n,width,prec,buf);
      write(buf);
    }
    void write_P(const char* data)
    {

    }

    void endl()
    {
      write((uint8_t)'\n');
    }



    void labelnum(const char *label, uint16_t num, bool end)
    {
      write(label);
      write((uint32_t)num,10);
      if(end)
        endl();
    }
    void labelnum(const char *label, uint16_t num) { labelnum(label, num, true); };
    void labelnum(const char *label, int32_t num, bool end)
    {
      write(label);
      write(num,10);
      if(end)
        endl();
    }
    void labelnum(const char *label, int32_t num) { labelnum(label, num, true); };




    void labelnum(const char *label, uint32_t num, bool end)
    {
      write(label);
      write(num,10);
      if(end)
        endl();
    }
    void labelnum(const char *label, unsigned long num) { labelnum(label, num, true); };

    void labelnum(const char *label, int num) { labelnum(label, (uint16_t)num, true); };
    void labelnum(const char *label, int num, bool end) { labelnum(label, (uint16_t)num, end); };

    void labelnum(const char *label, float num, bool end)
    {
      write(label);
      write(num,0,4);
      if(end)
        endl();
    }
    void labelnum(const char *label, float num) { labelnum(label, num, true); };


    void rxerror(const char*errmsg, int32_t linenum)
    {
      labelnum("rs ", linenum, false);
#ifndef REPRAP_COMPAT      
      write(' ');
      write(errmsg);
#endif      
      endl();
      //rxring.reset();
      //input_ready=0;
    }

    void rxerror(const char* errmsg)
    {
      write("!! ");
      write(errmsg);
      endl();
      //rxring.reset();
      //input_ready=0;
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
      if(txring.getCount() > 0)
        UDR0 = txring.pop();
      else
        UCSR0B &= ~MASK(UDRIE0);
    }

  private:
    uint8_t rxbuf[HOST_RECV_BUFSIZE];
    RingBufferT<uint8_t> rxring;
    uint8_t txbuf[HOST_SEND_BUFSIZE];
    RingBufferT<uint8_t> txring;
    volatile uint8_t input_ready;

};

extern Host& HOST;


#endif
