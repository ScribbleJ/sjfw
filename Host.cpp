/*  
    (c) 2011 Christopher "ScribbleJ" Jansen
*/

#include "Host.h"
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "GcodeQueue.h"



Host::Host(unsigned long BAUD, int port)
  : rxring(HOST_RECV_BUFSIZE, rxbuf), txring(HOST_SEND_BUFSIZE, txbuf)
{
  input_ready = 0;
  port = port;
  if(port == 0)
    Init0(BAUD);
  if(port == 2)
    Init2(BAUD);
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
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
  PRR1 &= ~MASK(PRUSART2);
  UCSR2A = 0;
  UBRR2 = (((F_CPU / 16) / BAUD) - 0.5);

  UCSR2B = MASK(RXEN2) | MASK(TXEN2);
  UCSR2C = MASK(UCSZ21) | MASK(UCSZ20);

  UCSR2B |= MASK(RXCIE2) | MASK(UDRIE2);
  }
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


  GCODES.parsebytes(buf, len, port);

}

      


/*** INTERRUPT HANDLERS ***/
ISR(USART0_RX_vect)
{
  HOST.rx_interrupt_handler(0);
}

ISR(USART0_UDRE_vect)
{
  HOST.udre_interrupt_handler(0);
}

ISR(USART2_RX_vect)
{
  BT.rx_interrupt_handler(2);
}

ISR(USART2_UDRE_vect)
{
  BT.udre_interrupt_handler(2);
}

