#ifndef __AXIS_H__ 
#define __AXIS_H__
/* Axis control class
 * (c) 2011 Christopher "ScribbleJ" Jansen
 *
 */

#include "config.h"
#include "Host.h"
#include "AvrPort.h"
#include <math.h>

class Axis
{
  public:
  Axis(Pin step_pin, Pin dir_pin, Pin enable_pin, Pin min_pin, Pin max_pin, 
           float steps_per_unit, bool dir_inverted, 
           float max_feedrate, float home_feedrate, float min_feedrate, 
           float accel_rate_in_units, bool disable_after_move)
           :step_pin(step_pin), dir_pin(dir_pin), enable_pin(enable_pin), min_pin(min_pin), max_pin(max_pin)
  {
    // Initialize class data
    this->steps_per_unit = steps_per_unit;
    this->spu_int = steps_per_unit;
    this->disable_after_move = disable_after_move;
    max_feed     = max_feedrate;
    start_feed   = min_feedrate;
    accel_rate = accel_rate_in_units;
    this->dir_inverted = dir_inverted;

    // Initialize pins we control.
    if(!step_pin.isNull()) { step_pin.setDirection(true); step_pin.setValue(false); }
    if(!dir_pin.isNull())  { dir_pin.setDirection(true); dir_pin.setValue(false); }
    if(!enable_pin.isNull()) { enable_pin.setDirection(true); enable_pin.setValue(true); }
    if(!min_pin.isNull()) { min_pin.setDirection(false); min_pin.setValue(PULLUPS); }
    if(!max_pin.isNull()) { max_pin.setDirection(false); max_pin.setValue(PULLUPS); }
  }

    
  void dump_to_host()
  {
    HOST.labelnum(" sf:", start_feed, false);
    HOST.labelnum(" mf:", max_feed, false);
    HOST.labelnum(" ar:",accel_rate, false);
  }
  void  setMinimumFeedrate(float feedrate) { if(feedrate <= 0) return; start_feed = feedrate; }
  void  setMaximumFeedrate(float feedrate) { if(feedrate <= 0) return; max_feed = feedrate;  }
  void  setAverageFeedrate(float feedrate) { if(feedrate <= 0) return;  }
  // WARNING! BECAUSE OF THE WAY WE STORE ACCEL DATA< YOU MUST USE THE ABOVE THREE CALLS TO RESET THE FEEDRATES AFTER CHANGING THE STEPS
  void  setStepsPerUnit(float steps) { if(steps <= 0) return; steps_per_unit = steps; spu_int = steps; }
  void  setAccel(float rate) { if(rate <= 0) return; accel_rate = rate; }
  float getAccel() { return accel_rate; }
  void  disable() { if(!enable_pin.isNull()) enable_pin.setValue(true); }
  void  enable() { if(!enable_pin.isNull()) enable_pin.setValue(false); }

  inline void setDirection(bool direction)
  {
    if(!dir_pin.isNull())  { dir_pin.setValue(dir_inverted ? !direction : direction); }
  }


  inline bool doStep(bool direction)
  {
    if(direction)
    {
      if(!max_pin.isNull() && max_pin.getValue() != END_INVERT)
      {
        return false;
      }
    }
    else
    {
      if(!min_pin.isNull() && min_pin.getValue() != END_INVERT)
      {
        return false;
      }
    }

    step_pin.setValue(true);
    step_pin.setValue(false);
  }

  void disableIfConfigured() { if(disable_after_move) disable(); }

  void changepinStep(Port p, int bit)
  {
    step_pin = Pin(p, bit);
    if(step_pin.isNull())
      return;
    step_pin.setDirection(true); step_pin.setValue(false);
  }

  void changepinDir(Port p, int bit)
  {
    dir_pin = Pin(p, bit);
    if(dir_pin.isNull())
      return;

    dir_pin.setDirection(true); dir_pin.setValue(false);
  }

  void changepinEnable(Port p, int bit)
  {
    enable_pin = Pin(p, bit);
    if(enable_pin.isNull())
      return;

    enable_pin.setDirection(true); enable_pin.setValue(true);
  }

  void changepinMin(Port p, int bit)
  {
    min_pin = Pin(p, bit);
    if(min_pin.isNull())
      return;

    min_pin.setDirection(false); min_pin.setValue(PULLUPS);
  }

  void changepinMax(Port p, int bit)
  {
    max_pin = Pin(p, bit);
    if(max_pin.isNull())
      return;

    max_pin.setDirection(false); max_pin.setValue(PULLUPS);
  }

  void setInvert(bool v) { dir_inverted = v; }
  void setDisable(bool v) { disable_after_move = v; }
  static void setPULLUPS(bool v) { PULLUPS = v; };
  static void setEND_INVERT(bool v) { END_INVERT = v; };
  bool isInvalid() { return step_pin.isNull(); }
  void reportConfigStatus(Host& h)
  {
    if(steps_per_unit == 1)
      h.write(" no steps_per_unit ");
    if(accel_rate == 1)
      h.write(" no accel ");
    if(step_pin.isNull())
      h.write(" no step ");
    if(dir_pin.isNull())
      h.write(" no dir ");
    if(enable_pin.isNull())
      h.write(" no enable ");
    if(min_pin.isNull())
      h.write(" no min ");
    if(max_pin.isNull())
      h.write(" no max ");
    if(disable_after_move)
      h.write(" DIS ");
    if(dir_inverted)
      h.write(" INV ");
    if(PULLUPS)
      h.write(" EPULL ");
    if(END_INVERT)
      h.write(" EINV ");
  }

  float getStepsPerMM() { return steps_per_unit; }
private:
  static bool PULLUPS;
  static bool END_INVERT;

	float steps_per_unit;
  uint32_t spu_int;

  float    start_feed, max_feed;
  uint32_t accel_rate;

  int homing_dir;
	
	 Pin step_pin;
	 Pin dir_pin;
	bool dir_inverted;
	 Pin enable_pin;
	 Pin min_pin;
	 Pin max_pin;
  bool disable_after_move;

};

#endif // __AXIS_H__
