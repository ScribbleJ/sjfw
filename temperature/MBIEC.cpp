/*  MBI RS485 Extruder Controller comms class
    To be used from the Gen3/4 host system to speak to the EC.
    (c) 2011 Christopher "ScribbleJ" Jansen
*/

#include "MBIEC.h"
#include "config.h"
#include "Time.h"
#include "Host.h"
#include <avr/interrupt.h>


MBIEC::MBIEC() :
    TX_ENABLE_PIN(RS485_TX_ENABLE), RX_ENABLE_PIN(RS485_RX_ENABLE),
    rxring(MBIEC_BUFSIZE, rxbuf), txring(MBIEC_BUFSIZE, txbuf)
{
  last_command=0;
  hotend_temp=0;
  platform_temp=0;
  hotend_set_temp=0;
  platform_set_temp=0;
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
    // Too much data
    len = 0;
    return 0;
  }

  if(len < 2)
    return 0;

  int dlen = buf[1];

  if(dlen > EC_MAX_COMM - 4)
  {
    // Packet too large
    len = 0;
    return 0;
  }

  if(len < dlen + 3) 
    return 0;

  len = 0;
  uint16_t p = 0;

  // Could check CRC here.

  if(dlen >= 3)
  {
    p = buf[3] | (buf[4] << 8);
    handle_command_response(p);
  }

  return p;
}

void MBIEC::handle_command_response(uint16_t p)
{
  switch(last_command)
  {
    case SLAVE_CMD_GET_TEMP:
      hotend_temp = p;
      break;
    case SLAVE_CMD_GET_SP:
      hotend_set_temp = p;
      break;
    case SLAVE_CMD_GET_PLATFORM_TEMP:
      platform_temp = p;
      break;
    case SLAVE_CMD_GET_PLATFORM_SP:
      platform_set_temp = p;
      break;
  }
  lastresponsetime = millis();
}
    
bool MBIEC::setHotend(uint16_t p)
{
  return dotoolreq(SLAVE_CMD_SET_TEMP, p);
}

bool MBIEC::setPlatform(uint16_t p)
{
  return dotoolreq(SLAVE_CMD_SET_PLATFORM_TEMP, p);
}

uint16_t MBIEC::getHotend()
{
  return hotend_temp;
}

uint16_t MBIEC::getPlatform()
{
  return platform_temp;
}

uint16_t MBIEC::getHotendST()
{
  return hotend_set_temp;
}

uint16_t MBIEC::getPlatformST()
{
  return platform_set_temp;
}


#define MAX_EC_INTERVAL_MS 500
#define MIN_EC_INTERVAL_MS 10
#define MIN_REQUEST_INTERVAL 50
void MBIEC::update()
{
  unsigned long now = millis();
  if(now - lastresponsetime > MAX_EC_INTERVAL_MS)
  {
    // TODO: Take some action.
    HOST.write("!! EXTRUDER CONTROLLER NONRESPONSIVE\n");
    // Reset alarm.
    lastresponsetime = millis();
  }
  else if(now - lastresponsetime < MIN_EC_INTERVAL_MS)
  {
    return;
  }
  if(now - lastrequesttime < MIN_REQUEST_INTERVAL)
  {
    return;
  }

  // could enfore call timing here.
  static const uint8_t commands[] = 
  {
    SLAVE_CMD_GET_TEMP,
    SLAVE_CMD_GET_PLATFORM_TEMP,
    SLAVE_CMD_GET_SP,
    SLAVE_CMD_GET_PLATFORM_SP
  };
  static const uint8_t cmdnum = 4;
  static int cmd = 0;
  dotoolreq(commands[cmd++]);
  if(cmd >= cmdnum)
    cmd = 0;
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
bool MBIEC::dotoolreq(uint8_t command_id, uint16_t param)
{
  unsigned long now = millis();
  if(now - lastrequesttime < MIN_REQUEST_INTERVAL)
  {
    return false;
  }
  lastrequesttime = now;
  uint8_t toolnum = 0;
  uint8_t c = 2;
  uint8_t crc = 0;

  last_command = command_id;

  write(0xD5); // start byte

  if(param < 255)
    c = 4;
  write(c); // packet length

  crc = crc_update(crc, toolnum);
  write(toolnum); // tool num

  c = command_id;
  crc = crc_update(crc, c);
  write(c); // Command num
  if(param < 255)
  {
    c = param & 0xff;
    crc = crc_update(crc, c);
    write(c); // param byte1

    c = (param>>8) & 0xff;
    crc = crc_update(crc, c);
    write(c); // param byte2
  }

  write(crc); // crc
  speak();

  return true;
}

bool MBIEC::dotoolreq(uint8_t command_id)
{
  return dotoolreq(command_id, 0xffff);
}


/*** INTERRUPT HANDLERS ***/
ISR(USART1_RX_vect)
{
  (MBIEC::Instance()).rx_interrupt_handler();
}

// Possibly should load UDR1 on UDRE interrupt handler - but am aping MBI's behavior for now.
ISR(USART1_TX_vect)
{
  (MBIEC::Instance()).tx_interrupt_handler();
}

