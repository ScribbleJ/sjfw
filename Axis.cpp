#include "Axis.h"
#include "Motion.h"
#include "Host.h"
#include <stdlib.h>

Axis::Axis(Pin step_pin, Pin dir_pin, Pin enable_pin, Pin min_pin, Pin max_pin, 
           float steps_per_unit, bool dir_inverted, float max_length, 
           float max_feedrate, float home_feedrate, float min_feedrate, 
           float accel_rate_in_units, bool disable_after_move, int homing_dir)
           :step_pin(step_pin), dir_pin(dir_pin), enable_pin(enable_pin), min_pin(min_pin), max_pin(max_pin)
{
	// Initialize class data
	this->steps_per_unit = steps_per_unit;
  this->disable_after_move = disable_after_move;
  min_interval = interval_from_feedrate(max_feedrate);
  avg_interval = interval_from_feedrate(home_feedrate);
  max_interval = interval_from_feedrate(min_feedrate);
  accel_rate = accel_rate_in_units;
  position = 0;
  relative = false;
  this->dir_inverted = dir_inverted;
  steps_to_take = 0;
  steps_remaining = 0;

	// Initialize pins we control.
  step_pin.setDirection(true); step_pin.setValue(false);
  dir_pin.setDirection(true); dir_pin.setValue(false);
  enable_pin.setDirection(true); enable_pin.setValue(true);
  if(!min_pin.isNull()) { min_pin.setDirection(false); min_pin.setValue(ENDSTOPPULLUPS); }
  if(!max_pin.isNull()) { max_pin.setDirection(false); max_pin.setValue(ENDSTOPPULLUPS); }

}
	
void Axis::dump_to_host()
{
  HOST.labelnum("p:",position,false);
  HOST.labelnum(" mi:",min_interval,false);
  HOST.labelnum(" mi:",max_interval,false);
  HOST.labelnum(" ar:",accel_rate);
}
