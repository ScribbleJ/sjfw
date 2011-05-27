/*  MBI RS485 Extruder Controller comms class
    To be used from the Gen3/4 host system to speak to the EC.
    (c) 2011 Christopher "ScribbleJ" Jansen
*/

#include "Host.h"
#include <avr/interrupt.h>
#include "Gcodes.h"

Host& HOST = Host::Instance();

Host::Host(unsigned long BAUD)
  : rxring(HOST_BUFSIZE, rxbuf), txring(HOST_BUFSIZE, txbuf)
{
  input_ready = false;
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


#define MAX_PARSEBYTES 16
void Host::scan_input()
{
  if(rxring.hasOverflow())
  {
    rxerror("Recieve Buffer Overflow.");
    return;
  }

  if(input_ready == 0)
    return;

  if(GCODES.isFull())
    return;

  char buf[MAX_PARSEBYTES];
  uint8_t len;
 
  // HOST.labelnum("Input fragments ready:", input_ready);

  cli(); 
  input_ready--;
  for(len=0;len<MAX_PARSEBYTES;len++)
  {
    buf[len] = rxring.pop();
    if(buf[len] <= 32)
      break;
  }
  sei();

  // HOST.labelnum("Fragment length:", len);

  if(len == MAX_PARSEBYTES)
  {
    rxerror("Fragment buffer overflow.");
    return;
  }


  GCODES.parsebytes(buf, len);

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

