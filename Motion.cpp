#include "Motion.h"

#include "config.h"
#include "GCode.h"
#include "Axis.h"
#include "Globals.h"
#include "GcodeQueue.h"




Point& Motion::getCurrentPosition()
{
  static Point p;
  for(int ax=0;ax<NUM_AXES;ax++)
    p[ax] = AXES[ax].getCurrentPosition();
  return p;
}

void Motion::setCurrentPosition(GCode &gcode)
{
  for(int ax=0;ax<NUM_AXES;ax++)
  {
    if(!gcode[ax].isUnused())
    {
      AXES[ax].setCurrentPosition(gcode[ax].getFloat());
    }
  }
}

void Motion::setAbsolute()
{
  for(int ax=0;ax<NUM_AXES;ax++)
    AXES[ax].setAbsolute();
}

void Motion::setRelative()
{
  for(int ax=0;ax<NUM_AXES;ax++)
    AXES[ax].setRelative();
}

void Motion::setMinimumFeedrate(GCode& gcode)
{
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode[ax].isUnused()) continue;
    AXES[ax].setMinimumFeedrate(gcode[ax].getFloat());
  }
}

void Motion::setMaximumFeedrate(GCode& gcode)
{
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode[ax].isUnused()) continue;
    AXES[ax].setMaximumFeedrate(gcode[ax].getFloat());
  }
}

void Motion::setAverageFeedrate(GCode& gcode)
{
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode[ax].isUnused()) continue;
    AXES[ax].setAverageFeedrate(gcode[ax].getFloat());
  }
}

void Motion::setStepsPerUnit(GCode& gcode)
{
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode[ax].isUnused()) continue;
    AXES[ax].setStepsPerUnit(gcode[ax].getFloat());
  }
}

void Motion::setAccel(GCode& gcode)
{
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode[ax].isUnused()) continue;
    AXES[ax].setAccel(gcode[ax].getFloat());
  }
}

void Motion::disableAllMotors()
{
  for(int ax=0;ax < NUM_AXES;ax++)
    AXES[ax].disable();
}

void Motion::checkdisable(GCode& gcode)
{
  static uint8_t disable[NUM_AXES] = { 0 };
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode[ax].isUnused())
      disable[ax]++;

    if(disable[ax] > 2)
      AXES[ax].disableIfConfigured();
  }
}




void Motion::getMovesteps(GCode& gcode)
{
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    gcode.axismovesteps[ax] = AXES[ax].getMovesteps(gcode.startpos[ax], 
                                                             gcode[ax].isUnused() ? gcode.startpos[ax] : AXES[ax].isRelative() ? gcode.startpos[ax] + gcode[ax].getFloat() : gcode[ax].getFloat(), 
                                                             gcode.axisdirs[ax]);
    if(gcode.movesteps < gcode.axismovesteps[ax])
    {
      gcode.movesteps = gcode.axismovesteps[ax];
      gcode.leading_axis = ax;
    }
  }
}

unsigned long Motion::getLargestStartInterval(GCode& gcode)
{
  unsigned long mi = 0;
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode.axismovesteps[ax] == 0) continue;
    unsigned long t = AXES[ax].getStartInterval(gcode.feed);
    if(t > mi) mi = t;
  }
  return mi;
}

unsigned long Motion::getLargestEndInterval(GCode& gcode)
{
  unsigned long mi = 0;
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode.axismovesteps[ax] == 0) continue;
    unsigned long t = AXES[ax].getEndInterval(gcode.feed);
    if(t > mi) mi = t;
  }
  return mi;
}

unsigned long Motion::getLargestTimePerAccel(GCode& gcode)
{
  unsigned long mi = 0;
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode.axismovesteps[ax] == 0) continue;
    unsigned long t = AXES[ax].getTimePerAccel();
    if(t > mi) mi = t;
  }
  return mi;
}


void Motion::getActualEndpos(GCode& gcode)
{
  for(int ax=0;ax<NUM_AXES;ax++)
  {
    gcode.endpos[ax] = AXES[ax].getEndpos(gcode.startpos[ax], gcode.axismovesteps[ax], gcode.axisdirs[ax]);
  }
}


void Motion::gcode_precalc(GCode& gcode, float& feedin, Point* lastend)
{
  if(gcode.state >= GCode::PREPARED)
    return;

  
  if(gcode[G].isUnused())
    return;

  if(gcode[G].getInt() == 92)
  {
    // Just carry over new position
    for(int ax=0;ax<NUM_AXES;ax++)
    {
      if(!gcode[ax].isUnused())
      {
        (*lastend)[ax] = gcode[ax].getFloat();
      }
    }
    gcode.state = GCode::PREPARED;
    return;
  }
  if(gcode[G].getInt() != 1 and gcode[G].getInt() != 2)
  {
    // no precalc for anything but 92, 1, 2
    gcode.state = GCode::PREPARED;
    return;
  }

  // We want to carry over the previous ending position and feedrate if possible.
  gcode.startpos = *lastend;
  if(gcode[F].isUnused())
    gcode.feed = feedin;
  else
    gcode.feed = gcode[F].getFloat();

  if(gcode.feed == 0)
    gcode.feed = SAFE_DEFAULT_FEED;


  getMovesteps(gcode);
  gcode.startinterval = getLargestStartInterval(gcode);
  gcode.fullinterval = getLargestEndInterval(gcode);
  gcode.currentinterval = gcode.startinterval;
  getActualEndpos(gcode);
  gcode.decel_from  = 0;
  gcode.accel_inc   = getLargestTimePerAccel(gcode);

  long intervaldiff = gcode.startinterval - gcode.fullinterval;
  if(intervaldiff > 0)
    gcode.decel_from  = gcode.movesteps / 2;
 
  gcode.steps_acceled=0;
  gcode.accel_remainder=0;

  *lastend = gcode.endpos;
  feedin  = gcode.feed;
  gcode.state = GCode::PREPARED;
}


void Motion::gcode_execute(GCode& gcode)
{
  // Only execute codes that are prepared.
  if(gcode.state < GCode::PREPARED)
    return; // TODO: This should never happen now and is an error.
  // Don't execute codes that are ACTIVE or DONE (ACTIVE get handled by interrupt)
  if(gcode.state > GCode::PREPARED)
    return;

  // Make sure they have configured the axis!
  if(AXES[0].isInvalid())
  {
    HOST.write("!! AXIS ARE NOT CONFIGURED !!\n");
    gcode.state = GCode::DONE;
    return;
  }

  // set axis move data, invalidate all precomputes if bad data
  for(int ax=0;ax<NUM_AXES;ax++)
  {
    if(!AXES[ax].setupMove(gcode.startpos[ax], gcode.axisdirs[ax], gcode.axismovesteps[ax]))
    {
      GCODES.Invalidate();
      return;
    }
    deltas[ax] = gcode.axismovesteps[ax];
    errors[ax] = gcode.movesteps / 2;
    //AXES[ax].dump_to_host();
  }
  //gcode.dump_movedata();

  // setup pointer to current move data for interrupt
  gcode.state = GCode::ACTIVE;
  current_gcode = &gcode;

  setInterruptCycles(gcode.startinterval);
  enableInterrupt();
}

bool Motion::axesAreMoving() 
{ 
  for(int ax=0;ax<NUM_AXES;ax++) 
    if(AXES[ax].isMoving()) return true; 
  
  return false; 
}


void Motion::writePositionToHost()
{
  for(int ax=0;ax<NUM_AXES;ax++)
  {
    HOST.write(ax > Z ? 'A' - Z - 1 + ax : 'X' + ax); HOST.write(':'); HOST.write(AXES[ax].getCurrentPosition(),0,4); HOST.write(' ');
  }
}



void Motion::handleInterrupt()
{
  // interruptOverflow for step intervals > 16bit
  if(interruptOverflow)
  {
    setInterruptCycles(60000);
    interruptOverflow--;
    return;
  }

  if(current_gcode->movesteps == 0)
  {
    disableInterrupt();
    current_gcode->state = GCode::DONE;
    return;
  }

  current_gcode->movesteps--;

  // Handle acceleration and deceleration
  uint32_t lastinterval = current_gcode->currentinterval + current_gcode->accel_remainder;
  current_gcode->accel_remainder=0;
  if(current_gcode->movesteps <= current_gcode->steps_acceled)
  {
    // This is just here to log the data of the minimum steptime
    if(current_gcode->movesteps == current_gcode->steps_acceled)
    {
      current_gcode->fullinterval = lastinterval;
      //AXES[0].dump_to_host();
    }

    // Decelerate!
    while(lastinterval >= current_gcode->accel_inc)
    {
      current_gcode->currentinterval++;
      lastinterval -= current_gcode->accel_inc;
    }
    current_gcode->accel_remainder = lastinterval;
  }
  else if((current_gcode->currentinterval > current_gcode->fullinterval) && 
          (current_gcode->movesteps > current_gcode->decel_from))
  {
    // Accelerate!
    while(lastinterval >= current_gcode->accel_inc)
    {
      current_gcode->currentinterval--;
      lastinterval -= current_gcode->accel_inc;
    }
    current_gcode->accel_remainder = lastinterval;
    current_gcode->steps_acceled++;
  }
  setInterruptCycles(current_gcode->currentinterval);

  // Bresenham-style axis alignment algorithm
  for(int ax=0;ax<NUM_AXES;ax++)
  {
    if(ax == current_gcode->leading_axis)
    {
      AXES[ax].doStep();
      continue;
    }

    errors[ax] = errors[ax] - deltas[ax];
    if(errors[ax] < 0)
    {
      AXES[ax].doStep();
      errors[ax] = errors[ax] + deltas[current_gcode->leading_axis];
    }
  }

  if(!axesAreMoving())
  {
    current_gcode->movesteps = 0;
  }
      
  if(current_gcode->movesteps == 0)
  {
    disableInterrupt();
    current_gcode->state = GCode::DONE;
  }
}

void Motion::setupInterrupt() 
{
  // 16-bit registers that must be set/read with interrupts disabled:
  // TCNTn, OCRnA/B/C, ICRn
  // "Fast PWM" and no prescaler.
  //TCCR1A = _BV(WGM10) | _BV(WGM11);
  //TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS10);
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
}


void Motion::enableInterrupt() 
{
  // reset counter in case.
  // TCNT1 = 0;
   // Enable timer
  PRR0 &= ~(_BV(PRTIM1));
  // Outcompare Compare match A interrupt
  TIMSK1 |= _BV(OCIE1A);

}
void Motion::disableInterrupt() 
{
  // Outcompare Compare match A interrupt
  TIMSK1 &= ~(_BV(OCIE1A));
  // Disable timer...
  PRR0 |= _BV(PRTIM1);

}
void Motion::setInterruptCycles(unsigned long cycles) 
{
  if(cycles > 60000)
  {
    OCR1A = 60000;
    interruptOverflow = cycles / 60000;
  }
  else
    OCR1A = cycles;
}





ISR(TIMER1_COMPA_vect)
{
  MOTION.handleInterrupt();
}
