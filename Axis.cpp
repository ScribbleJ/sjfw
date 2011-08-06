#include "Axis.h"
#include "Motion.h"
#include "Host.h"
#include <stdlib.h>

bool Axis::PULLUPS=ENDSTOPPULLUPS;
bool Axis::END_INVERT=ENDSTOPS_INVERTING;

Axis::Axis(Pin step_pin, Pin dir_pin, Pin enable_pin, Pin min_pin, Pin max_pin, 
           float steps_per_unit, bool dir_inverted, 
           float max_feedrate, float home_feedrate, float min_feedrate, 
           float accel_rate_in_units, bool disable_after_move)
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
  if(!step_pin.isNull()) { step_pin.setDirection(true); step_pin.setValue(false); }
  if(!dir_pin.isNull())  { dir_pin.setDirection(true); dir_pin.setValue(false); }
  if(!enable_pin.isNull()) { enable_pin.setDirection(true); enable_pin.setValue(true); }
  if(!min_pin.isNull()) { min_pin.setDirection(false); min_pin.setValue(PULLUPS); }
  if(!max_pin.isNull()) { max_pin.setDirection(false); max_pin.setValue(PULLUPS); }
}
	
void Axis::dump_to_host()
{
  HOST.labelnum("p:",position,false);
  HOST.labelnum(" mi:",min_interval,false);
  HOST.labelnum(" mi:",max_interval,false);
  HOST.labelnum(" ar:",accel_rate);
  HOST.labelnum("SP:",step_pin.getPortIndex(),false);
  HOST.labelnum(",",step_pin.getPinIndex(),false);
  HOST.labelnum("EP:",enable_pin.getPortIndex(),false);
  HOST.labelnum(",",enable_pin.getPinIndex(),false);
  HOST.labelnum("DP:",dir_pin.getPortIndex(),false);
  HOST.labelnum(",",dir_pin.getPinIndex(),false);
  HOST.labelnum("IP:",min_pin.getPortIndex(),false);
  HOST.labelnum(",",min_pin.getPinIndex(),false);
  HOST.labelnum("AP:",max_pin.getPortIndex(),false);
  HOST.labelnum(",",max_pin.getPinIndex(),true);
}
