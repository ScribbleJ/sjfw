#include	"serial.h"

/* Near derivative of Triffid Hunter's Teacup firmware serial code.  THANKS! */

#include	<avr/interrupt.h>
#include "CircularBuffer.h"
#include "pins.h"

#define		BUFSIZE			128

#define MASK(PIN)       (1 << PIN)

uint8_t rxbuf0[BUFSIZE];
CircularBufferTempl<uint8_t> rxring0(BUFSIZE, rxbuf0);
uint8_t txbuf0[BUFSIZE];
CircularBufferTempl<uint8_t> txring0(BUFSIZE, txbuf0);

/// initialise serial subsystem
///
/// set up baud generator and interrupts, clear buffers
void serial_init(unsigned long BAUD)
{
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

/*
	Interrupts
*/

/// receive interrupt
///
/// we have received a character, stuff it in the rx buffer if we can, or drop it if we can't
#ifdef	USART_RX_vect
ISR(USART_RX_vect)
#else
ISR(USART0_RX_vect)
#endif
{
  rxring0.push(UDR0);
}

/// transmit buffer ready interrupt
///
/// provide the next character to transmit if we can, otherwise disable this interrupt
#ifdef	USART_UDRE_vect
ISR(USART_UDRE_vect)
#else
ISR(USART0_UDRE_vect)
#endif
{
	if (txring0.getLength() > 0)
		UDR0 = txring0.pop();
	else
		UCSR0B &= ~MASK(UDRIE0);
}

/*
	Read
*/

/// check how many characters can be read
uint8_t serial_rxchars()
{
  cli();
  uint8_t foo = rxring0.getLength();
  sei();
	return foo;
}

/// read one character
uint8_t serial_popchar()
{
	uint8_t c = 0;

  cli();
  c = rxring0.pop();
  sei();

	return c;
}

/*
	Write
*/

/// send one character
void serial_writechar(uint8_t data)
{
  cli();
  txring0.push(data);
  sei();

	// enable TX interrupt so we can send this character
	UCSR0B |= MASK(UDRIE0);
}

void serial_writenum(uint16_t n, uint16_t maxradix)
{
	for(uint16_t i = maxradix; i >= 1; i = i/10)
	{
		serial_writechar((uint8_t)(n/i) + '0');
		n -= (n/i) * i; // This looks like nonsense but it chops off the remainder.
	}
}

void serial_labelnum(const char* label, uint16_t num)
{
  serial_writestr(label);
  serial_writenum(num, 100);
  serial_writestr("\r\n");
}


/// send a whole block
void serial_writeblock(void *data, int datalen)
{
	int i;

	for (i = 0; i < datalen; i++)
		serial_writechar(((uint8_t *) data)[i]);
}

/// send a string- look for null byte instead of expecting a length
void serial_writestr(const char *data)
{
	uint8_t i = 0, r;
	// yes, this is *supposed* to be assignment rather than comparison, so we break when r is assigned zero
	while ((r = data[i++]))
		serial_writechar(r);
}


