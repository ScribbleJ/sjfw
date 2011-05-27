#ifndef _MBIEC_H_
#define _MBIEC_H_

/* RS485 MBI Extruder Controller Comms
   Chris "ScribbleJ" Jansen
   (c)2011

   This class handles setting up the second comm port on a AVR 
   to communicate via RS485 with a MBI Extruder Controller running stock MBI
   firmware (tested with 2.8).  

   Uses different methods to set pins as it's based on code from various places and
   not what I'd call fully-integrated (yet).
*/

#include "AvrPort.h"
#include "CircularBuffer.h"
#include "ExtruderCommands.h"
#include <avr/interrupt.h>

#define MBIEC_BUFSIZE 16
#define MASK(PIN) (1 << PIN)

class MBIEC
{
  // THERE CAN BE ONLY ONE!
  public:
    static MBIEC& Instance() { static MBIEC instance; return instance; };
  private:
    explicit MBIEC();
    MBIEC(MBIEC&);
    MBIEC& operator=(MBIEC&);

  // Ok, on with it.
  public:
    uint8_t rxchars() { return rxring.getLength(); };
    uint8_t popchar() { return rxring.pop(); };
    void write(uint8_t data) { txring.push(data); };
    uint16_t handle_rx_char(uint8_t c);
    bool    dotoolreq(uint8_t command_id, uint16_t param);
    bool    dotoolreq(uint8_t command_id);

    void scan_input()
    {
      if(rxring.isEmpty())
        return;

      handle_rx_char(rxring.pop());
    }

    void rx_interrupt_handler()
    {
      uint8_t rc = UDR1;
      if(loopbytes == 0)
      {
        rxring.push(rc);
      }
      else
        loopbytes--;
    }

    void tx_interrupt_handler()
    {
      if(txring.getLength() == 0)
      {
        listen();
      }
      else if (TX_ENABLE_PIN.getValue()==1)
      {
        UDR1 = txring.pop();
        loopbytes++;
      }
      else  TX_ENABLE_PIN.setValue(true);
    }

    void setHotend(uint16_t p);
    void setPlatform(uint16_t p);
    uint16_t getHotend();
    uint16_t getPlatform();
    uint16_t getHotendST();
    uint16_t getPlatformST();

    void update();


  private:
    void handle_command_response(uint16_t p);
    Pin TX_ENABLE_PIN;
    Pin RX_ENABLE_PIN;
    uint8_t rxbuf[MBIEC_BUFSIZE];
    CircularBufferTempl<uint8_t> rxring;
    uint8_t txbuf[MBIEC_BUFSIZE];
    CircularBufferTempl<uint8_t> txring;
    volatile uint8_t loopbytes;
    uint8_t last_command;
    void speak()
    {
      TX_ENABLE_PIN.setValue(true);
      UDR1 = txring.pop();
      loopbytes++;
    }
    void listen() { TX_ENABLE_PIN.setValue(false); }
    uint8_t crc_update (uint8_t crc, uint8_t data);
    uint16_t hotend_temp;
    uint16_t platform_temp;
    uint16_t hotend_set_temp;
    uint16_t platform_set_temp;
    unsigned long lastresponsetime;
    unsigned long lastrequesttime;
};

extern MBIEC& EC;


#endif
