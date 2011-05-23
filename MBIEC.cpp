/*  MBI RS485 Extruder Controller comms class
    To be used from the Gen3/4 host system to speak to the EC.
    (c) 2011 Christopher "ScribbleJ" Jansen
*/

#include "MBIEC.h"
#include "pins.h"
#include <avr/interrupt.h>

// We create an instance of the class here as a global - should be made into a proper Singleton pattern eventually.
MBIEC EC(RS485_TX_ENABLE, RS485_RX_ENABLE);

MBIEC::MBIEC(Pin tx_en, Pin rx_en) 
  : TX_ENABLE_PIN(tx_en), RX_ENABLE_PIN(rx_en),
    rxring(MBIEC_BUFSIZE, rxbuf), txring(MBIEC_BUFSIZE, txbuf)

{
  // Using double-speec comms; probably not necessary.
  UCSR1A = MASK(U2X1);
  UBRR1 = (((F_CPU / 8) / 38400) - 0.5);
  UCSR1B = MASK(RXEN1) | MASK(TXEN1) | MASK(RXCIE1) | MASK(TXCIE1); // | MASK(UDRIE1);
  UCSR1C = MASK(UCSZ11) | MASK(UCSZ10);
  loopbytes = 0;
  TX_ENABLE_PIN.setDirection(true);
  RX_ENABLE_PIN.setDirection(true);
  RX_ENABLE_PIN.setValue(false);  // Set to active when low.
  listen();
}


// Reading into a separate buffer here may not be necessary ... code is evolving
#define EC_MAX_COMM 16
uint16_t MBIEC::handle_rx_char(uint8_t c)
{
  static uint8_t buf[EC_MAX_COMM];
  static int len = 0;

  if(len == 0 && c != 0xD5)
    return 0;

  buf[len++] = c;
  if(len == EC_MAX_COMM)
  {
    return 0;
  }

  if(len < 2)
    return 0;

  int dlen = buf[1];

  if(dlen > EC_MAX_COMM - 4)
  {
    len = 0;
    return 0;
  }

  if(len < dlen + 3) 
    return 0;

  len = 0;
  uint16_t p = 0;

  if(dlen >= 3)
  {
    p = buf[3] | (buf[4] << 8);
  }

  return p;
}

uint8_t MBIEC::crc_update (uint8_t crc, uint8_t data) 
{
    uint8_t i;

    crc = crc ^ data;
    for (i = 0; i < 8; i++)
      if (crc & 0x01) crc = (crc >> 1) ^ 0x8C;
      else crc >>= 1;

    return crc;
}


// Yes, the way this is written now assumes the only commands
// you're going to use either take no params or 16 bits, and 
// return nothing or 16 bits.  
// Also assumes you won't be sending any params >= 255.
// This should work fine for all
// the meta-control (e.g. reset, init) and temperature
// controls, which are all we need to care about on Gen4.
// Might need some extra support for Gen3.
void MBIEC::dotoolreq(uint8_t command_id, uint16_t param)
{
  uint8_t toolnum = 0;
  uint8_t c = 2;
  uint8_t crc = 0;

  EC.write(0xD5); // start byte

  if(param < 255)
    c = 4;
  EC.write(c); // packet length

  crc = crc_update(crc, toolnum);
  EC.write(toolnum); // tool num

  c = command_id;
  crc = crc_update(crc, c);
  EC.write(c); // Command num
  if(param < 255)
  {
    c = param & 0xff;
    crc = crc_update(crc, c);
    EC.write(c); // param byte1

    c = (param>>8) & 0xff;
    crc = crc_update(crc, c);
    EC.write(c); // param byte2
  }

  EC.write(crc); // crc
  EC.speak();
}

void MBIEC::dotoolreq(uint8_t command_id)
{
  return dotoolreq(command_id, 0xffff);
}


/*** INTERRUPT HANDLERS ***/
ISR(USART1_RX_vect)
{
  EC.rx_interrupt_handler();
}

// Possibly should load UDR1 on UDRE interrupt handler - but am aping MBI's behavior for now.
ISR(USART1_TX_vect)
{
  EC.tx_interrupt_handler();
}

