#include "Eeprom.h"
#include <avr/eeprom.h>
#include <string.h>
#include "Host.h"
#include "GcodeQueue.h"

#define EEPROM_MAX 4095

namespace eeprom
{

  bool writing = false;
  bool reading = false;
  char const* GUARDSTR = "SJ14";
  int const GUARDLEN = 4;
  int eepromptr = 0;

  void Stop()
  {
    if(writing)
    {
      writing = false;
      eeprom_busy_wait();
      eeprom_update_byte((uint8_t *)eepromptr, 0xFF); 
    }
    if(reading)
      reading = false;
  }

  bool beginRead()
  {
    if(writing || reading)
      return false;
    eepromptr = 0;
    eeprom_busy_wait();
    char vbuf[GUARDLEN+1] = { 0 };
    eeprom_read_block(vbuf, (void *)0, GUARDLEN);
    if(strcmp(GUARDSTR,vbuf) != 0)
      return false;
    eepromptr = GUARDLEN;
    reading = true;
    return true;
  }

  bool beginWrite()
  {
    if(reading || writing)
      return false;
    eepromptr = 0;
    eeprom_busy_wait();
    eeprom_update_block(GUARDSTR, (void *)0, GUARDLEN);
    eepromptr = GUARDLEN;
    writing = true;
    return true;
  }

  bool writebytes(char const* bytes, int len)
  {
    if(!writing)
      return false;
      
    if(eepromptr + len >= EEPROM_MAX)
    {
      HOST.write("EEPROM OVERFLOW");
      HOST.endl();
      writing = false;
      return false;
    }

    eeprom_busy_wait();
    eeprom_update_block(bytes, (void *)eepromptr, len);
    eepromptr += len;
    return true;
  }

  void update()
  {
    if(!reading)
      return;
    if(GCODES.isFull())
      return;

    char buf[MAX_GCODE_FRAG_SIZE];
    int x = 0;
    for(;x<MAX_GCODE_FRAG_SIZE;x++)
    {
      eeprom_busy_wait();
      buf[x] = eeprom_read_byte((uint8_t *)eepromptr);
      //HOST.write(buf[x]);
      eepromptr++;
      if(eepromptr >= EEPROM_MAX)
      {
        reading = false;
        buf[x] = '\n';
      }
      if(buf[x] <= 32)
        break;
      if(buf[x] == 0xFF)
      {
        HOST.write("READPAST\n");
        buf[x] = '\n';
        reading = false;
      }
    }
    GCODES.parsebytes(buf, x, EEPROM_SOURCE);
  }

};
