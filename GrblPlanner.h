/*
  planner.h - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// This module is to be considered a sub-module of stepper.c. Please don't include 
// this file from any other module.

#ifndef planner_h
#define planner_h
                 
#include <inttypes.h>
#include "config.h"
#include "Point.h"


// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used to recreate this line from the original request
  // when we hit an endstop and have to redo it all.
  float    orig_targ[NUM_AXES];
  float    orig_feed;
  bool     orig_relative;

  // Fields used by the bresenham algorithm for tracing the line
  uint32_t steps[NUM_AXES];           // Step count along each axis
  uint8_t  directions[NUM_AXES];      // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  int32_t  step_event_count;          // The number of step events required to complete this block
  uint32_t nominal_rate;              // The nominal step rate for this block in step_events/minute
  
  // Fields used by the motion planner to manage acceleration
  double speeds[NUM_AXES];            // Nominal mm/minute for each axis
  double nominal_speed;               // The nominal speed for this block in mm/min  
  double millimeters;                 // The total travel of this block in mm
  double entry_factor;                // The factor representing the change in speed at the start of this trapezoid.
                                      // (The end of the curren speed trapezoid is defined by the entry_factor of the
                                      // next block)
  
  // Settings for the trapezoid generator
  uint32_t acceleration;              // The acceleration to use during this block
  uint32_t initial_rate;              // The jerk-adjusted step rate at start of block  
  uint32_t final_rate;                // The minimal rate at exit
  int32_t rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
  uint32_t accelerate_until;          // The index of the step event on which to stop acceleration
  uint32_t decelerate_after;          // The index of the step event on which to start decelerating
  
} block_t;
      

namespace GrblPlanner {

// Initialize the motion plan subsystem      
void plan_init();

// Very important to check the state of the buffer before adding a move:
bool isBufferEmpty();
bool isBufferFull();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion. 
void plan_buffer_line(float targ[NUM_AXES], double feed_rate, bool relative);
void plan_buffer_line(float targ[NUM_AXES], double feed_rate);

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the current block. Returns NULL if buffer empty
block_t *plan_get_current_block();

// Enables or disables acceleration-management for upcoming blocks
void plan_set_acceleration_manager_enabled(int enabled);

// Is acceleration-management currently enabled?
int plan_is_acceleration_manager_enabled();


Point getPosition();
void setPosition(Point p);


}; //namespace

#endif
