#ifndef _ARDUINOMAP_H_
#define _ARDUINOMAP_H_

#include "AvrPort.h"

namespace ArduinoMap 
{
  Pin getArduinoPin(int pinnum);
  Port getPort(int pinnum);
  uint8_t getPinnum(int pinnum);
};



#endif
