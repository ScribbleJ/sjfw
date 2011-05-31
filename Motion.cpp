#include "Motion.h"

Motion& MOTION = Motion::Instance();

ISR(TIMER1_COMPA_vect)
{
  MOTION.handleInterrupt();
}
