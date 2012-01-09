/*
		(c) 2011 Christopher "ScribbleJ" Jansen
*/

#include "Host.h"
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "GcodeQueue.h"

#include "config.h"



Host::Host(unsigned long BAUD, int port_in)
	: rxring(HOST_RECV_BUFSIZE, rxbuf), txring(HOST_SEND_BUFSIZE, txbuf)
{
	input_ready = 0;
	port = port_in;
#ifdef HIGHPORTS
#ifdef HAS_BT
	if(port == 2)
		Init2(BAUD);
	else
#endif
#endif
		Init0(BAUD);
}

void Host::Init0(unsigned long BAUD)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	PRR0 &= ~(MASK(PRUSART0));
	if(BAUD > 38401)
	{
		UCSR0A = MASK(U2X0);
		UBRR0 = (((F_CPU / 8) / BAUD) - 0.5);
	}
	else
	{
		UCSR0A = 0;
		UBRR0 = (((F_CPU / 16) / BAUD) - 0.5);
	}

	UCSR0B = MASK(RXEN0) | MASK(TXEN0);
	UCSR0C = MASK(UCSZ01) | MASK(UCSZ00);

	UCSR0B |= MASK(RXCIE0) | MASK(UDRIE0);
	}
}

void Host::Init2(unsigned long BAUD)
{
#ifdef HIGHPORTS
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	PRR1 &= ~MASK(PRUSART2);
	if(BAUD > 38401)
	{
		UCSR2A = MASK(U2X2);
		UBRR2 = (((F_CPU / 8) / BAUD) - 0.5);
	}
	else
	{
		UCSR2A = 0;
		UBRR2 = (((F_CPU / 16) / BAUD) - 0.5);
	}

	UCSR2B = MASK(RXEN2) | MASK(TXEN2);
	UCSR2C = MASK(UCSZ21) | MASK(UCSZ20);

	UCSR2B |= MASK(RXCIE2) | MASK(UDRIE2);
	}
#endif
}




void Host::scan_input()
{
	if(input_ready == 0)
		return;

	if(GCODES.isFull())
		return;

	char buf[MAX_GCODE_FRAG_SIZE];
	uint8_t len;

	// HOST.labelnum("Input fragments ready:", input_ready);

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		input_ready--;
	}
	for(len=0;len<MAX_GCODE_FRAG_SIZE;len++)
	{
		buf[len] = rxring.pop();
		if(buf[len] <= 32)
			break;
	}

	// HOST.labelnum("Fragment length:", len);

	if(len == MAX_GCODE_FRAG_SIZE)
	{
		rxerror("Frag Over");
		return;
	}

#ifdef HAS_BT
#ifdef BT_DEBUG
		uint8_t temp = buf[len];
		buf[len] = 0;
		int other = 0;
		if(port == 0) other = 2;
		Host::Instance(other).write("COM: ");
		Host::Instance(other).write(buf);
		Host::Instance(other).endl();
		buf[len] = temp;
#endif
#endif
	GCODES.parsebytes(buf, len, port);

}




/*** INTERRUPT HANDLERS ***/
ISR(USART0_RX_vect)
{
	HOST.rx_interrupt_handler0();
}

ISR(USART0_UDRE_vect)
{
	HOST.udre_interrupt_handler0();
}

#ifdef HIGHPORTS
ISR(USART2_RX_vect)
{
#ifdef HAS_BT
	BT.rx_interrupt_handler2();
#endif
}

ISR(USART2_UDRE_vect)
{
#ifdef HAS_BT
	BT.udre_interrupt_handler2();
#endif
}
#endif

