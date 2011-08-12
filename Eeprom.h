#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "config.h"

namespace eeprom 
{
  void Stop();
  bool beginRead();
  bool beginWrite();
  bool writebytes(char const*bytes, int len);
  void update();
};

#endif
