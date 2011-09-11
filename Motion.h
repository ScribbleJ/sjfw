#ifndef _MOTION_H_
#define _MOTION_H_
/* Multiple-Axis motion control.  Tightly integrated with GCode class, unfortunaetly.
 * (c) 2011, Christopher "ScribbleJ" Jansen
 *
 */

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
    Axis(X_STEP_PIN,X_DIR_PIN,X_ENABLE_PIN,X_MIN_PIN,X_MAX_PIN,X_STEPS_PER_UNIT,X_INVERT_DIR,X_MAX_FEED,X_AVG_FEED,X_START_FEED,X_ACCEL_RATE,X_DISABLE),
    Axis(Y_STEP_PIN,Y_DIR_PIN,Y_ENABLE_PIN,Y_MIN_PIN,Y_MAX_PIN,Y_STEPS_PER_UNIT,Y_INVERT_DIR,Y_MAX_FEED,Y_AVG_FEED,Y_START_FEED,Y_ACCEL_RATE,Y_DISABLE),
    Axis(Z_STEP_PIN,Z_DIR_PIN,Z_ENABLE_PIN,Z_MIN_PIN,Z_MAX_PIN,Z_STEPS_PER_UNIT,Z_INVERT_DIR,Z_MAX_FEED,Z_AVG_FEED,Z_START_FEED,Z_ACCEL_RATE,Z_DISABLE),
    Axis(A_STEP_PIN,A_DIR_PIN,A_ENABLE_PIN,Pin(),Pin(),A_STEPS_PER_UNIT,A_INVERT_DIR,A_MAX_FEED,A_AVG_FEED,A_START_FEED,A_ACCEL_RATE,A_DISABLE)
  })
  {
    setupInterrupt();
    interruptOverflow=0;
    busy = false;
  };
  Motion(Motion&);
  Motion& operator=(Motion&);

  Axis AXES[NUM_AXES];
  volatile GCode* volatile current_gcode;
  volatile unsigned long deltas[NUM_AXES];
  volatile long errors[NUM_AXES];
  volatile int interruptOverflow;
  bool busy;

public:
  // Return request Axis
  Axis& getAxis(int idx) { return AXES[idx]; }

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
  void setAccel(GCode& gcode);

  void setStepPins(GCode& gcode);
  void setDirPins(GCode& gcode);
  void setEnablePins(GCode& gcode);
  void setMinPins(GCode& gcode);
  void setMaxPins(GCode& gcode);
  void setMinStopPos(GCode& gcode);
  void setMaxStopPos(GCode& gcode);
  void setAxisInvert(GCode& gcode);
  void setAxisDisable(GCode& gcode);
  void setEndstopGlobals(bool inverted, bool pulledup);
  void reportConfigStatus(Host& h);
  // Motors automatically enabled when used
  void disableAllMotors();
  void disableAxis(int axis);
  void wrapup(GCode& gcode) { checkdisable(gcode); }
  void checkdisable(GCode& gcode);

  // Uses step data to determine the actual ending position of a move
  // (seldom /precisely/ the requested position)
  void getActualEndpos(GCode& gcode);


  // Run all the math on a G0/G1 movement Gcode to deal with movement later
  void gcode_precalc(GCode& gcode, float& feedin, Point* lastend);
  // Run extra math to do G1 even better
  void gcode_optimize(GCode& gcode, GCode& nextg);
  // (re)compute acceleration curve after optimization
  void computeAccel(GCode& gcode, GCode* nextg=NULL);
  // Actually execute a (precalculated) movement gcode.
  void gcode_execute(GCode& gcode);

  // Debugging and output to host...
  void writePositionToHost(GCode& gcode);

private:
  // Calculate the number of steps for each axis in a move.
  void getMovesteps(GCode& gcode);
  float getSmallestStartFeed(GCode& gcode);
  float getSmallestEndFeed(GCode& gcode);

  // opt support
  void fix_diverge(float *ends, float* starts);
  void join_moves(float *ends, float* starts);




  // Functions for handling pulsing motors using a timer interrupt for timing.
public:  
  void handleInterrupt();
private:  
  void setupInterrupt(); 
  void enableInterrupt(); 
  void disableInterrupt(); 
  void resetTimer();
  void setInterruptCycles(unsigned long cycles); 
  int ax; // used to avoid allocing loop counter in interrupt.
  int accelsteps; 

};

extern Motion& MOTION;

#endif // _MOTION_H_
