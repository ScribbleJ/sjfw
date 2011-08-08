#include "Host.h"
#include "Motion.h"
#include "Temperature.h"

#include "config.h"

Host& HOST = Host::Instance();
Motion& MOTION = Motion::Instance();
Temperature& TEMPERATURE = Temperature::Instance();

#ifdef HAS_BT
Host& BT = Host::Instance(2);
#endif

#ifdef HAS_LCD
#include "LCDKeypad.h"
LCDKeypad LCDKEYPAD;
#endif

