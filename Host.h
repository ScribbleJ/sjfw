#ifndef _HOST_H_
#define _HOST_H_

/* Serial comms - port0 or 2
	Chris "ScribbleJ" Jansen
	(c)2011

	Loosely based on Teacup serial library
*/

#include "RingBuffer.h"
#include <avr/interrupt.h>
#include <stdlib.h>
#include "config.h"
#include <avr/pgmspace.h>

#define MASK(PIN) (1 << PIN)
#if ((defined __AVR_ATmega2560__) || (defined __AVR_ATmega1280__))
#define HIGHPORTS
#else
#undef HIGHPORTS
#endif

class Host
{
	public:
		// singleton
		static Host& Instance(int port)
		{
#ifdef HAS_BT
			if(port == 2)
				return i2();
			else
#endif
			return i0();
		}
		static Host& Instance() { return Instance(0); }
		static Host& i0() { static Host instance(HOST_BAUD,0); return instance; }
#ifdef HIGHPORTS
		static Host& i2() { static Host instance2(BT_BAUD,2); return instance2; }
#else
		static Host& i2() { return i0(); }
#endif
	private:
		explicit Host(unsigned long baud, int port);
		Host(Host&);
		Host& operator=(Host&);
		int port;
		char convbuf[32];

		void Init0(unsigned long baud);
		void Init2(unsigned long baud);
	public:

		uint8_t rxchars() { uint8_t l = rxring.getCount(); return l; }
		uint8_t popchar() { uint8_t c = rxring.pop(); return c; }
		uint8_t peekchar() { uint8_t c = rxring.peek(0); return c; }

		void write(uint8_t data)
		{
			for (;txring.isFull(););
			txring.push(data);
#ifdef HIGHPORTS
			if(port == 2)
				UCSR2B |= MASK(UDRIE2);
			else
#endif
				UCSR0B |= MASK(UDRIE0);
		}

		void write(const char *data) { uint8_t i = 0, r; while ((r = data[i++])) write(r); }
		void write(uint32_t n, int radix)
		{
			ultoa(n,convbuf,radix);
			write(convbuf);
		}
		void write(int32_t n, int radix)
		{
			ltoa(n,convbuf,radix);
			write(convbuf);
		}
		void write(int16_t n, int radix) { write((int32_t)n, radix); }
		void write(uint16_t n, int radix) { write((int32_t)n, radix); }

		void write(float n, signed char width, unsigned char prec)
		{
			dtostrf(n,width,prec,convbuf);
			write(convbuf);
		}
		void write_P(const char* data)
		{
			char c;
			while ((c = pgm_read_byte(data++)))
				write(c);
		}

		void endl()
		{
			write((uint8_t)'\n');
		}



		void labelnum(const char *label, uint16_t num, bool end=true)
		{
			write(label);
			write((uint32_t)num,10);
			if(end)
				endl();
		}
		void labelnum(const char *label, int32_t num, bool end=true)
		{
			write(label);
			write(num,10);
			if(end)
				endl();
		}
		void labelnum(const char *label, uint32_t num, bool end=true)
		{
			write(label);
			write(num,10);
			if(end)
				endl();
		}
		void labelnum(const char *label, int num, bool end=true) { labelnum(label, (uint16_t)num, end); };

		void labelnum(const char *label, float num, bool end=true)
		{
			write(label);
			write(num,0,4);
			if(end)
				endl();
		}

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

		void rx_interrupt_handler0()
		{
			uint8_t c = UDR0;
			rxring.push(c);
			if(c <= 32)
				input_ready++;
		}

#ifdef HIGHPORTS
		void rx_interrupt_handler2()
		{
			uint8_t c = UDR2;
			rxring.push(c);
			if(c <= 32)
				input_ready++;
		}
#endif



		void udre_interrupt_handler0()
		{
			if(txring.getCount() > 0)
				UDR0 = txring.pop();
			else
				UCSR0B &= ~MASK(UDRIE0);
		}

#ifdef HIGHPORTS
		void udre_interrupt_handler2()
		{
			if(txring.getCount() > 0)
				UDR2 = txring.pop();
			else
				UCSR2B &= ~MASK(UDRIE2);
		}
#endif


	private:
		uint8_t rxbuf[HOST_RECV_BUFSIZE];
		RingBufferT<uint8_t> rxring;
		uint8_t txbuf[HOST_SEND_BUFSIZE];
		RingBufferT<uint8_t> txring;
		volatile uint8_t input_ready;

};

extern Host& HOST;

#ifdef HAS_BT
extern Host& BT;
#endif


#endif
