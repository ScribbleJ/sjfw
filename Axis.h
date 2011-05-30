#ifndef __AXIS_H__ 
#define __AXIS_H__
/* Axis control class
 * (c) 2011 Christopher "ScribbleJ" Jansen
 *
 */

#include "pins.h"

class Axis
{
  public:
	Axis(Pin step_pin, Pin dir_pin, Pin enable_pin, Pin min_pin, Pin max_pin, 
       float steps_per_unit, bool dir_inverted, float max_length,
       float max_feedrate, float avg_feedrate, float min_feedrate, 
       int homing_dir);

  // Interval measured in clock ticks
  // feedrate in mm/min
  // F_CPU is clock ticks/second
  unsigned long interval_from_feedrate(float feedrate)
  {
    return ((float)((unsigned long)60 * F_CPU) / (float)(feedrate * (float)steps_per_unit));
  }

  void dump_to_host();

  unsigned long min_interval, avg_interval, max_interval;

	bool direction;

	float steps_per_unit;
  float max_length;

  int homing_dir;

	bool relative;
	
	Pin step_pin;
	Pin dir_pin;
	bool dir_inverted;
	Pin enable_pin;
	bool enable_inverted;
	Pin min_pin;
	Pin max_pin;
	
	inline bool isMoving() { if(steps_remaining > 0) return true; return false; };

	unsigned long steps_to_take;
	unsigned long steps_remaining;
};

#endif // __AXIS_H__
