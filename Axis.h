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
    
  void dump_to_host()
  {
    HOST.labelnum("p:",position,false);
    HOST.labelnum(" sf:", start_feed, false);
    HOST.labelnum(" mf:", max_feed, false);
    HOST.labelnum(" ar:",accel_rate, false);
    HOST.labelnum(" stt:",steps_to_take);
  }
  // Interval measured in clock ticks
  // feedrate in mm/min
  // F_CPU is clock ticks/second
  uint32_t interval_from_feedrate(float feedrate)
  {
    float a = (float)(F_CPU * 60.0f) / (feedrate * steps_per_unit);
    return a;
  }

  inline uint32_t int_interval_from_feedrate(uint32_t feedrate)
  {
    // Max error - roughly 2.5%.  Only used for accel so not really a problem.
    return ((uint32_t)F_CPU * 60) / (feedrate * spu_int);
  }

	bool isMoving() { return (steps_remaining > 0); };
  // Doesn't take into account position is not updated during move.
  float getCurrentPosition() { return position; }
  void  setCurrentPosition(float pos) { position = pos; }
  void  setAbsolute() { relative = false; }
  void  setRelative() { relative = true; }
  bool  isRelative()  { return relative; }
  void  setMinimumFeedrate(float feedrate) { if(feedrate <= 0) return; start_feed = feedrate; }
  void  setMaximumFeedrate(float feedrate) { if(feedrate <= 0) return; max_feed = feedrate;  }
  void  setAverageFeedrate(float feedrate) { if(feedrate <= 0) return;  }
  // WARNING! BECAUSE OF THE WAY WE STORE ACCEL DATA< YOU MUST USE THE ABOVE THREE CALLS TO RESET THE FEEDRATES AFTER CHANGING THE STEPS
  void  setStepsPerUnit(float steps) { if(steps <= 0) return; steps_per_unit = steps; spu_int = steps; }
  void  setAccel(float rate) { if(rate <= 0) return; accel_rate = rate; }
  float getAccel() { return accel_rate; }
  void  disable() { if(!enable_pin.isNull()) enable_pin.setValue(true); }
  void  enable() { if(!enable_pin.isNull()) enable_pin.setValue(false); }

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
  float    getStartFeed(float feed) { return start_feed < feed ? start_feed : feed; }
  float    getStartFeed() { return start_feed; }
  float    getEndFeed(float feed) { return max_feed < feed ? max_feed : feed; }
  float    getMaxFeed() { return max_feed; }
  uint32_t getStartInterval(float feed) { uint32_t i = interval_from_feedrate(feed); return i; }
  uint32_t getEndInterval(float feed) { uint32_t i = interval_from_feedrate(feed); return i ; }
  uint32_t getAccelRate() { return accel_rate; }

  static float getAccelTime(float startfeed, float endfeed, uint32_t accel)
  { 
    startfeed /= 60.0f;
    endfeed   /= 60.0f;
    return (float)(endfeed - startfeed) / (float)accel;
  }
  int32_t getTimePerAccel() { return ((float)1 / (float)((accel_rate * steps_per_unit) / ((float)F_CPU))); }





  uint32_t getAccelDist(float start_feed, float end_feed, float accel) 
  { 
    end_feed /= 60.0f;
    start_feed /= 60.0f;
    float distance_in_mm = ((end_feed * end_feed) - (start_feed * start_feed)) / (2.0f * accel);
    return (float)steps_per_unit * distance_in_mm;
  }
  
  static float getFinalVelocity(float start_feed, float dist, float accel)
  {
    start_feed /= 60.0f;
    return sqrt((start_feed * start_feed) + (2.0f * accel * dist));
  }

  float getSpeedAtEnd(float start_feed, float accel, uint32_t movesteps)
  {
    start_feed /= 60.0f;
    return  sqrt((start_feed * start_feed) + (2.0f * accel * (float)((float)movesteps * steps_per_unit)));
  }

  float getEndpos(float start, uint32_t steps, bool dir) 
  { 
    return start + (float)((float)steps / steps_per_unit * (dir ? 1.0 : -1.0));
  }

  inline void doStep()
  {
    if(steps_remaining == 0) return;
    if(direction)
    {
      if(!max_pin.isNull() && max_pin.getValue() != END_INVERT)
      {
        position += (float)(steps_to_take-steps_remaining) / steps_per_unit;
        steps_remaining = 0;
        return;
      }
    }
    else
    {
      if(!min_pin.isNull() && min_pin.getValue() != END_INVERT)
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
      //HOST.labelnum("FINISH MOVE, ", steps_to_take, true);
      position += (float)((float)steps_to_take / steps_per_unit * (direction ? 1.0 : -1.0));
      //if(disable_after_move) disable();
    }
  }

  bool setupMove(float supposed_position, bool dir, uint32_t steps)
  {
    if(supposed_position != position)
      return false;
    direction = dir;
    steps_to_take = steps;
    steps_remaining = steps;
    if(direction) dir_pin.setValue(!dir_inverted);
    else dir_pin.setValue(dir_inverted);
    if(steps != 0) enable();
    return true;
  }  

  uint32_t getRemainingSteps() { return steps_remaining; }

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
      h.write_P(PSTR(" no steps_per_unit "));
    if(accel_rate == 1)
      h.write_P(PSTR(" no accel "));
    if(step_pin.isNull())
      h.write_P(PSTR(" no step "));
    if(dir_pin.isNull())
      h.write_P(PSTR(" no dir "));
    if(enable_pin.isNull())
      h.write_P(PSTR(" no enable "));
    if(min_pin.isNull())
      h.write_P(PSTR(" no min "));
    if(max_pin.isNull())
      h.write_P(PSTR(" no max "));
    if(disable_after_move)
      h.write_P(PSTR(" DIS "));
    if(dir_inverted)
      h.write_P(PSTR(" INV "));
    if(PULLUPS)
      h.write_P(PSTR(" EPULL "));
    if(END_INVERT)
      h.write_P(PSTR(" EINV "));
  }
  float getStepsPerMM() { return steps_per_unit; }

private:
  static bool PULLUPS;
  static bool END_INVERT;
  volatile float position;
	volatile bool direction;
	volatile uint32_t steps_to_take;
	volatile uint32_t steps_remaining;

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
	bool relative;
  bool disable_after_move;

};

#endif // __AXIS_H__
