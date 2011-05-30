#ifndef _MOTION_H_
#define _MOTION_H_

#include "config.h"
#include "Movedata.h"
#include "MGcode.h"
#include "Axis.h"

class Motion
{
  // Singleton
public:
  static Motion& Instance() { static Motion instance; return instance; };
private:
  explicit Motion() :AXES((Axis[NUM_AXES])
  {
    Axis(X_STEP_PIN,X_DIR_PIN,X_ENABLE_PIN,X_MIN_PIN,X_MAX_PIN,X_STEPS_PER_UNIT,X_INVERT_DIR,X_LENGTH,X_MAX_FEED,X_AVG_FEED,X_START_FEED,X_HOME_DIR),
    Axis(Y_STEP_PIN,Y_DIR_PIN,Y_ENABLE_PIN,Y_MIN_PIN,Y_MAX_PIN,Y_STEPS_PER_UNIT,Y_INVERT_DIR,Y_LENGTH,Y_MAX_FEED,Y_AVG_FEED,Y_START_FEED,Y_HOME_DIR),
    Axis(Z_STEP_PIN,Z_DIR_PIN,Z_ENABLE_PIN,Z_MIN_PIN,Z_MAX_PIN,Z_STEPS_PER_UNIT,Z_INVERT_DIR,Z_LENGTH,Z_MAX_FEED,Z_AVG_FEED,Z_START_FEED,Z_HOME_DIR),
    Axis(A_STEP_PIN,A_DIR_PIN,A_ENABLE_PIN,Pin(),Pin(),A_STEPS_PER_UNIT,A_INVERT_DIR,A_LENGTH,A_MAX_FEED,A_AVG_FEED,A_START_FEED,A_HOME_DIR)
  })
  {};
  Motion(Motion&);
  Motion& operator=(Motion&);

private:
  Axis AXES[NUM_AXES];
  float feedrate;


public:
  float& getFeedrate() { return feedrate; }
  void setFeedrate(float f) { feedrate = f; }

  void gcode_precalc(MGcode& gcode)
  {
    Movedata* md = (Movedata*)gcode.movedata;
    if(gcode.state >= MGcode::PREPARED)
      return;

    md->nominal_movetime = getLongestMovetime();
    md->startinterval = getLowestStartInterval();

    gcode.state = MGcode::PREPARED;
  }
  bool gcode_execute(MGcode& gcode)
  {
    gcode.state = MGcode::DONE;
    for(int ax=0;ax<NUM_AXES;ax++)
      AXES[ax].dump_to_host();
  }

  void bresenham_move();

  bool axesAreMoving() 
  { 
    for(int ax=0;ax<NUM_AXES;ax++) 
      if(AXES[ax].isMoving()) return true; 
    
    return false; 
  }

  unsigned long getLongestMovetime() {};
  unsigned long getLowestStartInterval() {};

  void handleInterrupt()
  {
    
  }


};


extern Motion& MOTION;

#endif // _MOTION_H_
