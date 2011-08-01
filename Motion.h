#ifndef _MOTION_H_
#define _MOTION_H_

#include "config.h"
#include "GCode.h"
#include "Axis.h"

class Motion
{
  // Singleton

  
public:
  static Motion& Instance() { static Motion instance; return instance; };
private:
  explicit Motion() :AXES((Axis[NUM_AXES])
  {
    Axis(X_STEP_PIN,X_DIR_PIN,X_ENABLE_PIN,X_MIN_PIN,X_MAX_PIN,X_STEPS_PER_UNIT,X_INVERT_DIR,X_LENGTH,X_MAX_FEED,X_AVG_FEED,X_START_FEED,X_ACCEL_RATE,X_DISABLE,X_HOME_DIR),
    Axis(Y_STEP_PIN,Y_DIR_PIN,Y_ENABLE_PIN,Y_MIN_PIN,Y_MAX_PIN,Y_STEPS_PER_UNIT,Y_INVERT_DIR,Y_LENGTH,Y_MAX_FEED,Y_AVG_FEED,Y_START_FEED,Y_ACCEL_RATE,Y_DISABLE,Y_HOME_DIR),
    Axis(Z_STEP_PIN,Z_DIR_PIN,Z_ENABLE_PIN,Z_MIN_PIN,Z_MAX_PIN,Z_STEPS_PER_UNIT,Z_INVERT_DIR,Z_LENGTH,Z_MAX_FEED,Z_AVG_FEED,Z_START_FEED,Z_ACCEL_RATE,Z_DISABLE,Z_HOME_DIR),
    Axis(A_STEP_PIN,A_DIR_PIN,A_ENABLE_PIN,Pin(),Pin(),A_STEPS_PER_UNIT,A_INVERT_DIR,A_LENGTH,A_MAX_FEED,A_AVG_FEED,A_START_FEED,A_ACCEL_RATE,A_DISABLE,A_HOME_DIR)
  })
  {
    setupInterrupt();
    interruptOverflow=0;
  };
  Motion(Motion&);
  Motion& operator=(Motion&);

  Axis AXES[NUM_AXES];
  volatile GCode* volatile current_gcode;
  volatile unsigned long deltas[NUM_AXES];
  volatile long errors[NUM_AXES];
  volatile int interruptOverflow;

public:

  // Returns current position of all Axes
  Point& getCurrentPosition();
  // Sets current position; doesn't cause a move, just updates the current position variables.
  void setCurrentPosition(GCode &gcode);
  // Interpret movement data as absolute
  void setAbsolute();
  // Interpret movement data as relative
  void setRelative();
  // Returns true if machine is in motion
  bool axesAreMoving(); 
  // Change stored feedrates/axis data
  // WARNING: if you change steps per unit without then changing/resetting all feedrates
  // after things will not go well for you!
  void setMinimumFeedrate(GCode& gcode);
  void setMaximumFeedrate(GCode& gcode);
  void setAverageFeedrate(GCode& gcode);
  void setStepsPerUnit(GCode& gcode);
  // Motors automatically enabled when used
  void disableAllMotors();
  void wrapup(GCode& gcode) { checkdisable(gcode); }
  void checkdisable(GCode& gcode);



  // Calculate the number of steps for each axis in a move.
  void getMovesteps(GCode& gcode);
  // These three functions select the data to use during a move
  unsigned long getLargestStartInterval(GCode& gcode);
  unsigned long getLargestEndInterval(GCode& gcode);
  unsigned long getLargestTimePerAccel(GCode& gcode);
  // Uses step data to determine the actual ending position of a move
  // (seldom /precisely/ the requested position)
  void getActualEndpos(GCode& gcode);


  // Run all the math on a G0/G1 movement Gcode to deal with movement later
  void gcode_precalc(GCode& gcode, float& feedin, Point* lastend);
  // Actually execute a (precalculated) movement gcode.
  void gcode_execute(GCode& gcode);

  // Debugging and output to host...
  void writePositionToHost();

  // Functions for handling pulsing motors using a timer interrupt for timing.
  void handleInterrupt();
  void setupInterrupt(); 
  void enableInterrupt(); 
  void disableInterrupt(); 
  void setInterruptCycles(unsigned long cycles); 
};

extern Motion& MOTION;

#endif // _MOTION_H_
