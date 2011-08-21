#include "Motion.h"

#include "config.h"
#include "GCode.h"
#include "Axis.h"
#include "Globals.h"
#include "GcodeQueue.h"
#include "ArduinoMap.h"
#include "GrblMotion.h"
#include "GrblPlanner.h"


Point Motion::getCurrentPosition()
{
  return GrblPlanner::getPosition();
}

void Motion::setCurrentPosition(GCode &gcode)
{
  Point p = getCurrentPosition();
  for(int ax=0;ax<NUM_AXES;ax++)
  {
    if(!gcode[ax].isUnused())
    {
      p[ax] = gcode[ax].getFloat();
    }
  }
  GrblPlanner::setPosition(p);
}

void Motion::setAbsolute()
{
  relative = false;
}

void Motion::setRelative()
{
  relative = true;
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

#define MOT_CHANGEPIN(FOO) \
  for(int ax=0;ax < NUM_AXES;ax++) \
  { \
    if(gcode[ax].isUnused()) \
      continue; \
    \
    AXES[ax].FOO(ArduinoMap::getPort(gcode[ax].getInt()), ArduinoMap::getPinnum(gcode[ax].getInt())); \
  } 
 

void Motion::setStepPins(GCode& gcode) { MOT_CHANGEPIN(changepinStep); }
void Motion::setDirPins(GCode& gcode) { MOT_CHANGEPIN(changepinDir); }
void Motion::setEnablePins(GCode& gcode) { MOT_CHANGEPIN(changepinEnable); }
void Motion::setMinPins(GCode& gcode) { MOT_CHANGEPIN(changepinMin); }
void Motion::setMaxPins(GCode& gcode) { MOT_CHANGEPIN(changepinMax); } 
void Motion::setAxisInvert(GCode& gcode)
{
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode[ax].isUnused()) continue;
    AXES[ax].setInvert(gcode[ax].getInt());
  }
}

void Motion::setAxisDisable(GCode& gcode)
{
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    if(gcode[ax].isUnused()) continue;
    AXES[ax].setDisable(gcode[ax].getInt());
  }
}

void Motion::setEndstopGlobals(bool inverted, bool pulledup)
{
  Axis::setPULLUPS(pulledup);
  Axis::setEND_INVERT(inverted);
}

void Motion::reportConfigStatus(Host& h)
{
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    h.labelnum("CS:", ax, false);
    h.write(' ');
    AXES[ax].reportConfigStatus(h);
    h.endl();
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



#define ACCEL_INC_TIME F_CPU/1000
void Motion::gcode_precalc(GCode& gcode, float& feedin)
{
  if(gcode.state >= GCode::PREPARED)
    return;

  if(gcode[G].isUnused())
    return;

  if(gcode[G].getInt() == 92)
  {
    // Just carry over new position
    // TODO: this
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
  if(gcode[F].isUnused())
    gcode.feed = feedin;
  else
    gcode.feed = gcode[F].getFloat();

  if(gcode.feed == 0)
    gcode.feed = SAFE_DEFAULT_FEED;



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
    Host::Instance(gcode.source).write("!! AXIS ARE NOT CONFIGURED !!\n");
    gcode.state = GCode::DONE;
    return;
  }

#ifdef DEBUG_MOVE
  gcode.dump_movedata();
#endif
}

bool Motion::axesAreMoving() 
{ 
  if(GrblPlanner::isBufferEmpty())
    return false;
  
  return true; 
}


void Motion::writePositionToHost(GCode& gc)
{
  Point p = GrblPlanner::getPosition();
  for(int ax=0;ax<NUM_AXES;ax++)
  {
    Host::Instance(gc.source).write(ax > Z ? 'A' - Z - 1 + ax : 'X' + ax); 
    Host::Instance(gc.source).write(':'); 
    Host::Instance(gc.source).write(p[ax],0,4);
    Host::Instance(gc.source).write(' ');
  }
}



