/*  
    (c) 2011 Christopher "ScribbleJ" Jansen
*/

#include "Host.h"
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "GcodeQueue.h"



Host::Host(unsigned long BAUD)
  : rxring(HOST_RECV_BUFSIZE, rxbuf), txring(HOST_SEND_BUFSIZE, txbuf)
{
  input_ready = 0;
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
  for(len=0;len<MAX_GCODE_FRAG_SIZE;len++)
  {
    buf[len] = rxring.pop();
    if(buf[len] <= 32)
      break;
  }
  }

  // HOST.labelnum("Fragment length:", len);

  if(len == MAX_GCODE_FRAG_SIZE)
  {
    rxerror("Frag Over");
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

