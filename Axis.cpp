#include "Axis.h"
#include "Motion.h"
#include "Host.h"
#include <stdlib.h>

Axis::Axis(Pin step_pin, Pin dir_pin, Pin enable_pin, Pin min_pin, Pin max_pin, 
           float steps_per_unit, bool dir_inverted, float max_length, float max_feedrate, float home_feedrate, float min_feedrate, int homing_dir)
           :step_pin(step_pin), dir_pin(dir_pin), enable_pin(enable_pin), min_pin(min_pin), max_pin(max_pin)
{
	// Initialize class data
	this->steps_per_unit = steps_per_unit;
  this->min_interval = interval_from_feedrate(max_feedrate);
  this->avg_interval = interval_from_feedrate(home_feedrate);
  this->max_interval = interval_from_feedrate(min_feedrate);

	// Initialize pins we control.
  step_pin.setDirection(true); step_pin.setValue(false);
  dir_pin.setDirection(true); dir_pin.setValue(false);
  enable_pin.setDirection(true); enable_pin.setValue(false);
  if(!min_pin.isNull()) { min_pin.setDirection(false); min_pin.setValue(ENDSTOPPULLUPS); }
  if(!min_pin.isNull()) { max_pin.setDirection(false); max_pin.setValue(ENDSTOPPULLUPS); }

}
	
void Axis::dump_to_host()
{
  HOST.labelnum("MIN I:",min_interval,false);
  HOST.labelnum(" AVG I:",avg_interval,false);
  HOST.labelnum(" MAX I:",max_interval);
}
