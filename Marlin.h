#ifndef __MARLIN_H__
#define __MARLIN_H__

//#define ADVANCE
#define EXTRUDER_ADVANCE_K 0.02

#define D_FILAMENT 3.0
#define STEPS_MM_E 2000.0
#define EXTRUTION_AREA (0.25 * D_FILAMENT * D_FILAMENT * 3.14159)
#define STEPS_PER_CUBIC_MM_E (axis_steps_per_unit[E_AXIS]/ EXTRUTION_AREA)

#define NUM_AXIS 4
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3

#include "GCode.h"
#include "Point.h"

namespace Marlin
{

  void get_coordinates();
  void prepare_move();

  // This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
  // the source g-code and may never actually be reached if acceleration management is active.
  typedef struct {
    // Fields used by the bresenham algorithm for tracing the line
    long steps[NUM_AXIS];  // Step count along each axis
    long step_event_count;                    // The number of step events required to complete this block
    volatile long accelerate_until;                    // The index of the step event on which to stop acceleration
    volatile long decelerate_after;                    // The index of the step event on which to start decelerating
    volatile long acceleration_rate;                   // The acceleration rate used for acceleration calculation
    bool axisdirections[NUM_AXIS];             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)

    long advance_rate;
    volatile long initial_advance;
    volatile long final_advance;
    float advance;

    // We store the computed end position so we can compare later and recompute if we failed to hit it.
    long endposition[NUM_AXIS];
    // We stored the requested target so if we must recompute we know where we were going.
    float requestedposition[NUM_AXIS];
    float requestedfeed;

    // Fields used by the motion planner to manage acceleration
    float speed[NUM_AXIS];          // Nominal mm/minute for each axis
    float nominal_speed;                               // The nominal speed for this block in mm/min  
    float millimeters;                                 // The total travel of this block in mm
    float entry_speed;

    // Settings for the trapezoid generator
    long nominal_rate;                                 // The nominal step rate for this block in step_events/sec 
    volatile long initial_rate;                                 // The jerk-adjusted step rate at start of block  
    volatile long final_rate;                                   // The minimal rate at exit
    long acceleration;                                 // acceleration mm/sec^2
    volatile char busy;  
  } block_t;

  void check_axes_activity();
  void init();
  void st_wake_up();
  bool add_buffer_line(GCode& gcode);
  void plan_buffer_line(block_t *block);

  bool isBufferFull();
  bool isBufferEmpty();

  // Settings and external controls from sjfw

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
  void wrapup(GCode& gcode);
  void checkdisable(GCode& gcode);


};

#endif

