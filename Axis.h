#ifndef __AXIS_H__ 
#define __AXIS_H__
/* Axis control class
 * (c) 2011 Christopher "ScribbleJ" Jansen
 *
 */

#include "pins.h"
#include "Host.h"

class Axis
{
  public:
	Axis(Pin step_pin, Pin dir_pin, Pin enable_pin, Pin min_pin, Pin max_pin, 
       float steps_per_unit, bool dir_inverted, float max_length,
       float max_feedrate, float avg_feedrate, float min_feedrate, 
       float accel_distance_in_units,
       int homing_dir);

  // Interval measured in clock ticks
  // feedrate in mm/min
  // F_CPU is clock ticks/second
  unsigned long interval_from_feedrate(float feedrate)
  {
    return ((float)((unsigned long)60 * F_CPU) / (float)(feedrate * (float)steps_per_unit));
  }

  void dump_to_host();
	
	bool isMoving() { return (steps_remaining > 0); };
  // Doesn't take into account position is not updated during move.
  float getCurrentPosition() { return position; }
  void  setCurrentPosition(float pos) { position = pos; }
  void  setAbsolute() { relative = false; }
  void  setRelative() { relative = true; }
  bool  isRelative()  { return relative; }

  float getMovesteps(float start, float end, bool& dir) 
  { 
    float d = end - start; 

    if(d<0) 
    {
      d = d * -1; 
      dir=false;
    }
    else
      dir=true;
    
    return steps_per_unit * d; 
  }
  float getStartInterval(float feed) { unsigned long i = interval_from_feedrate(feed); return i < max_interval ? max_interval : i; }
  float getEndInterval(float feed) { unsigned long i = interval_from_feedrate(feed); return i < min_interval ? min_interval : i; }
  unsigned long getAccelDistance() { return accel_dist; }
  float getEndpos(float start, unsigned long steps, bool dir) 
  { 
    return start + (float)((float)steps / steps_per_unit * (dir ? 1.0 : -1.0));
  }

  void doStep()
  {
    if(steps_remaining == 0) return;
    if(direction)
    {
      if(!max_pin.isNull() && max_pin.getValue() != ENDSTOPS_INVERTING)
      {
        position += (float)(steps_to_take-steps_remaining) / steps_per_unit;
        steps_remaining = 0;
        return;
      }
    }
    else
    {
      if(!min_pin.isNull() && min_pin.getValue() != ENDSTOPS_INVERTING)
      {
        position -= (float)(steps_to_take-steps_remaining) / steps_per_unit;
        steps_remaining = 0;
        return;
      }
    }

    step_pin.setValue(true);
    step_pin.setValue(false);

    if(--steps_remaining == 0)
    {
      HOST.labelnum("FINISH MOVE, ", steps_to_take, true);
      position += (float)((float)steps_to_take / steps_per_unit * (direction ? 1.0 : -1.0));
    }
  }

  bool setupMove(float supposed_position, bool dir, unsigned long steps)
  {
    if(supposed_position != position)
      return false;
    direction = dir;
    steps_to_take = steps;
    steps_remaining = steps;
    if(direction) dir_pin.setValue(!dir_inverted);
    else dir_pin.setValue(dir_inverted);
    return true;
  }  

  unsigned long getRemainingSteps() { return steps_remaining; }

private:
  volatile float position;
	volatile bool direction;
	volatile unsigned long steps_to_take;
	volatile unsigned long steps_remaining;

	float steps_per_unit;
  float max_length;

  unsigned long min_interval, avg_interval, max_interval;
  unsigned long accel_dist;

  int homing_dir;
	
	Pin step_pin;
	Pin dir_pin;
	bool dir_inverted;
	Pin enable_pin;
	bool enable_inverted;
	Pin min_pin;
	Pin max_pin;
	bool relative;

};

#endif // __AXIS_H__
