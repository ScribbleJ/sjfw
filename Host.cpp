/*  MBI RS485 Extruder Controller comms class
    To be used from the Gen3/4 host system to speak to the EC.
    (c) 2011 Christopher "ScribbleJ" Jansen
*/

#include "Host.h"
#include <avr/interrupt.h>

// We create an instance of the class here as a global - should be made into a proper Singleton pattern eventually.
Host HOST(HOST_BAUD);

Host::Host(unsigned long BAUD)
  : rxring(HOST_BUFSIZE, rxbuf), txring(HOST_BUFSIZE, txbuf)
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


/*** INTERRUPT HANDLERS ***/
ISR(USART0_RX_vect)
{
  HOST.rx_interrupt_handler();
}

ISR(USART0_UDRE_vect)
{
  HOST.udre_interrupt_handler();
}

