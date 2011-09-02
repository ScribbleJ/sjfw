/*
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"
#include "AvrPort.h"
#include "ArduinoMap.h"
#include "speed_lookuptable.h"
#include "Globals.h"
#include <stdlib.h>
#include <math.h>
#include <util/atomic.h>
#include <string.h>

namespace Marlin
{
  // Marlin oddities
#define HIGH true
#define LOW false
#define max(A,B) (A > B ? A : B)

  float retract_acceleration = 7000;
  float acceleration = 2000;
  float max_jerk = 20*60;


  //Stepper Movement Variables

  char axis_codes[NUM_AXIS] = {
    'X', 'Y', 'Z', 'E'};
  long feedrate = 1500, next_feedrate, saved_feedrate;
  long gcode_N, gcode_LastN;
  bool relative_mode = false;  //Determines Absolute or Relative Coordinates
  bool relative_mode_e = false;  //Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.
  unsigned long axis_steps_per_sqr_second[NUM_AXIS];



  float axis_steps_per_unit[NUM_AXIS];
  long max_feedrate[NUM_AXIS];
  long min_feedrate[NUM_AXIS];



#define WRITE(PIN,VALUE) PIN.setValue(VALUE)
#define READ(PIN) PIN.getValue()

  Pin STEP_PINS[NUM_AXIS];
  Pin DIR_PINS[NUM_AXIS];
  Pin ENABLE_PINS[NUM_AXIS];
  Pin MIN_PINS[NUM_AXIS];
  Pin MAX_PINS[NUM_AXIS];

  bool INVERT_DIRS[NUM_AXIS];
  bool DISABLE[NUM_AXIS];
  bool INVERT_ENABLE = false;

  bool ENDSTOP_INVERT = true;
  bool ENDSTOP_PULLUPS = true;

  bool RELATIVEMOVES = false;


  // Planner

  /*  
   Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
   
   s == speed, a == acceleration, t == time, d == distance
   
   Basic definitions:
   
   Speed[s_, a_, t_] := s + (a*t) 
   Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
   
   Distance to reach a specific speed with a constant acceleration:
   
   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
   d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
   
   Speed after a given distance of travel with constant acceleration:
   
   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
   m -> Sqrt[2 a d + s^2]    
   
   DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
   
   When to start braking (di) to reach a specified destionation speed (s2) after accelerating
   from initial speed s1 without ever stopping at a plateau:
   
   Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
   di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
   
   IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
   */


  // The number of linear motions that can be in the plan at any give time
  #define BLOCK_BUFFER_SIZE 16
  #define BLOCK_BUFFER_MASK 0x0f

  static block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instructions
  static volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
  static volatile unsigned char block_buffer_tail;           // Index of the block to process now

  // The current position of the tool in absolute steps
  static long position[4];   

  #define ONE_MINUTE_OF_MICROSECONDS 60000000.0

  // Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
  // given acceleration:
  inline long estimate_acceleration_distance(long initial_rate, long target_rate, long acceleration) {
    return(
    (target_rate*target_rate-initial_rate*initial_rate)/
      (2L*acceleration)
      );
  }

  // This function gives you the point at which you must start braking (at the rate of -acceleration) if 
  // you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
  // a total travel of distance. This can be used to compute the intersection point between acceleration and
  // deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

  inline long intersection_distance(long initial_rate, long final_rate, long acceleration, long distance) {
    return(
    (2*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
      (4*acceleration)
      );
  }

  // Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

  void calculate_trapezoid_for_block(block_t *block, float entry_speed, float exit_speed) {
    if(block->busy == true) return; // If block is busy then bail out.
    float entry_factor = entry_speed / block->nominal_speed;
    float exit_factor = exit_speed / block->nominal_speed;
    long initial_rate = ceil(block->nominal_rate*entry_factor);
    long final_rate = ceil(block->nominal_rate*exit_factor);
    
  #ifdef ADVANCE
    long initial_advance = block->advance*entry_factor*entry_factor;
    long final_advance = block->advance*exit_factor*exit_factor;
  #endif // ADVANCE

    // Limit minimal step rate (Otherwise the timer will overflow.)
    if(initial_rate <120) initial_rate=120;
    if(final_rate < 120) final_rate=120;
    
    // Calculate the acceleration steps
    long acceleration = block->acceleration;
    long accelerate_steps = estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration);
    long decelerate_steps = estimate_acceleration_distance(final_rate, block->nominal_rate, acceleration);

    // Calculate the size of Plateau of Nominal Rate. 
    long plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;

    // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
    // have to use intersection_distance() to calculate when to abort acceleration and start braking 
    // in order to reach the final_rate exactly at the end of this block.
    if (plateau_steps < 0) {  
      accelerate_steps = intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count);
      plateau_steps = 0;
    }  

    long decelerate_after = accelerate_steps+plateau_steps;
    long acceleration_rate = (long)((float)acceleration * 8.388608);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  // Fill variables used by the stepper in a critical section
      if(block->busy == false) { // Don't update variables if block is busy.
        block->accelerate_until = accelerate_steps;
        block->decelerate_after = decelerate_after;
        block->acceleration_rate = acceleration_rate;
        block->initial_rate = initial_rate;
        block->final_rate = final_rate;
    #ifdef ADVANCE
        block->initial_advance = initial_advance;
        block->final_advance = final_advance;
    #endif // ADVANCE
      }
    }
  }                    

  // Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
  // acceleration within the allotted distance.
  inline float max_allowable_speed(float acceleration, float target_velocity, float distance) {
    return(
    sqrt(target_velocity*target_velocity-2*acceleration*60*60*distance)
      );
  }

  // "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
  // This method will calculate the junction jerk as the euclidean distance between the nominal 
  // velocities of the respective blocks.
  inline float junction_jerk(block_t *before, block_t *after) {
    return(sqrt(
      pow((before->speed_x-after->speed_x), 2)+
      pow((before->speed_y-after->speed_y), 2)+
      pow((before->speed_z-after->speed_z)*axis_steps_per_unit[Z_AXIS]/axis_steps_per_unit[X_AXIS], 2)));
  }

  // Return the safe speed which is max_jerk/2, e.g. the 
  // speed under which you cannot exceed max_jerk no matter what you do.
  float safe_speed(block_t *block) {
    float safe_speed;
    safe_speed = max_jerk/2;  
    if (safe_speed > block->nominal_speed) safe_speed = block->nominal_speed;
    return safe_speed;  
  }

  // The kernel called by planner_recalculate() when scanning the plan from last to first entry.
  void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
    if(!current) { 
      return; 
    }

    float entry_speed = current->nominal_speed;
    float exit_factor;
    float exit_speed;
    if (next) {
      exit_speed = next->entry_speed;
    } 
    else {
      exit_speed = safe_speed(current);
    }

    // Calculate the entry_factor for the current block. 
    if (previous) {
      // Reduce speed so that junction_jerk is within the maximum allowed
      float jerk = junction_jerk(previous, current);
      if((previous->steps_x == 0) && (previous->steps_y == 0)) {
        entry_speed = safe_speed(current);
      }
      else if (jerk > max_jerk) {
        entry_speed = (max_jerk/jerk) * entry_speed;
      } 
      // If the required deceleration across the block is too rapid, reduce the entry_factor accordingly.
      if (entry_speed > exit_speed) {
        float max_entry_speed = max_allowable_speed(-acceleration,exit_speed, current->millimeters);
        if (max_entry_speed < entry_speed) {
          entry_speed = max_entry_speed;
        }
      }    
    } 
    else {
      entry_speed = safe_speed(current);
    }
    // Store result
    current->entry_speed = entry_speed;
  }

  // planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
  // implements the reverse pass.
  void planner_reverse_pass() {
    char block_index = block_buffer_head;
    block_t *block[3] = {
      NULL, NULL, NULL  };
    while(block_index != block_buffer_tail) {    
      block_index--;
      if(block_index < 0) {
        block_index = BLOCK_BUFFER_SIZE-1;
      }
      block[2]= block[1];
      block[1]= block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
    planner_reverse_pass_kernel(NULL, block[0], block[1]);
  }

  // The kernel called by planner_recalculate() when scanning the plan from first to last entry.
  void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
    if(!current) { 
      return; 
    }
    if(previous) {
      // If the previous block is an acceleration block, but it is not long enough to 
      // complete the full speed change within the block, we need to adjust out entry
      // speed accordingly. Remember current->entry_factor equals the exit factor of 
      // the previous block.
      if(previous->entry_speed < current->entry_speed) {
        float max_entry_speed = max_allowable_speed(-acceleration, previous->entry_speed, previous->millimeters);
        if (max_entry_speed < current->entry_speed) {
          current->entry_speed = max_entry_speed;
        }
      }
    }
  }

  // planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
  // implements the forward pass.
  void planner_forward_pass() {
    char block_index = block_buffer_tail;
    block_t *block[3] = {
      NULL, NULL, NULL  };

    while(block_index != block_buffer_head) {
      block[0] = block[1];
      block[1] = block[2];
      block[2] = &block_buffer[block_index];
      planner_forward_pass_kernel(block[0],block[1],block[2]);
      block_index = (block_index+1) & BLOCK_BUFFER_MASK;
    }
    planner_forward_pass_kernel(block[1], block[2], NULL);
  }

  // Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
  // entry_factor for each junction. Must be called by planner_recalculate() after 
  // updating the blocks.
  void planner_recalculate_trapezoids() {
    char block_index = block_buffer_tail;
    block_t *current;
    block_t *next = NULL;
    while(block_index != block_buffer_head) {
      current = next;
      next = &block_buffer[block_index];
      if (current) {
        calculate_trapezoid_for_block(current, current->entry_speed, next->entry_speed);      
      }
      block_index = (block_index+1) & BLOCK_BUFFER_MASK;
    }
    calculate_trapezoid_for_block(next, next->entry_speed, safe_speed(next));
  }

  // Recalculates the motion plan according to the following algorithm:
  //
  //   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor) 
  //      so that:
  //     a. The junction jerk is within the set limit
  //     b. No speed reduction within one block requires faster deceleration than the one, true constant 
  //        acceleration.
  //   2. Go over every block in chronological order and dial down junction speed reduction values if 
  //     a. The speed increase within one block would require faster accelleration than the one, true 
  //        constant acceleration.
  //
  // When these stages are complete all blocks have an entry_factor that will allow all speed changes to 
  // be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than 
  // the set limit. Finally it will:
  //
  //   3. Recalculate trapezoids for all blocks.

  void planner_recalculate() {   
    planner_reverse_pass();
    planner_forward_pass();
    planner_recalculate_trapezoids();
  }

  void plan_init() {
    block_buffer_head = 0;
    block_buffer_tail = 0;
    memset(position, 0, sizeof(position)); // clear position
  }


  inline void plan_discard_current_block() {
    if (block_buffer_head != block_buffer_tail) {
      block_buffer_tail = (block_buffer_tail + 1) & BLOCK_BUFFER_MASK;  
    }
  }

  inline block_t *plan_get_current_block() {
    if (block_buffer_head == block_buffer_tail) { 
      return(NULL); 
    }
    block_t *block = &block_buffer[block_buffer_tail];
    block->busy = true;
    return(block);
  }

  void disable(int ax)
  {
    if(!ENABLE_PINS[ax].isNull())
      ENABLE_PINS[ax].setValue(INVERT_ENABLE);
  }

  void enable(int ax)
  {
    if(!ENABLE_PINS[ax].isNull())
      ENABLE_PINS[ax].setValue(!INVERT_ENABLE);
  }


  void check_axes_activity() {
    unsigned char x_active = 0;
    unsigned char y_active = 0;  
    unsigned char z_active = 0;
    unsigned char e_active = 0;
    block_t *block;

    if(block_buffer_tail != block_buffer_head) {
      char block_index = block_buffer_tail;
      while(block_index != block_buffer_head) {
        block = &block_buffer[block_index];
        if(block->steps_x != 0) x_active++;
        if(block->steps_y != 0) y_active++;
        if(block->steps_z != 0) z_active++;
        if(block->steps_e != 0) e_active++;
        block_index = (block_index+1) & BLOCK_BUFFER_MASK;
      }
    }
    if((DISABLE[X_AXIS]) && (x_active == 0)) disable(X_AXIS);
    if((DISABLE[Y_AXIS]) && (y_active == 0)) disable(Y_AXIS);
    if((DISABLE[Z_AXIS]) && (z_active == 0)) disable(Z_AXIS);
    if((DISABLE[E_AXIS]) && (e_active == 0)) disable(E_AXIS);
  }

  // Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in 
  // mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
  // calculation the caller must also provide the physical length of the line in millimeters.
  bool add_buffer_line(float x, float y, float z, float e, float feed_rate, bool relative) 
  {

    // Calculate the buffer head after we push this byte
    int next_buffer_head = (block_buffer_head + 1) & BLOCK_BUFFER_MASK;	

    // if buffer is full, return error
    if(block_buffer_tail == next_buffer_head) { return false; }

    // Prepare to set up new block
    block_t *block = &block_buffer[block_buffer_head];
    // let's not accidentally carry anything over
    memset(block, 0, sizeof(block_t));  

    if(relative)
    {
      block->requestedposition[X_AXIS] = ((float)position[X_AXIS] / axis_steps_per_unit[X_AXIS]) + x;
      block->requestedposition[Y_AXIS] = ((float)position[Y_AXIS] / axis_steps_per_unit[Y_AXIS]) + y;
      block->requestedposition[Z_AXIS] = ((float)position[Z_AXIS] / axis_steps_per_unit[Z_AXIS]) + z;
      block->requestedposition[E_AXIS] = ((float)position[E_AXIS] / axis_steps_per_unit[E_AXIS]) + e;
    }
    else
    {
      block->requestedposition[X_AXIS] = x;
      block->requestedposition[Y_AXIS] = y;
      block->requestedposition[Z_AXIS] = z;
      block->requestedposition[E_AXIS] = e;
    }

    block->requestedfeed = feed_rate;

    plan_buffer_line(block);

    // Move buffer head
    block_buffer_head = next_buffer_head;     

    planner_recalculate();
    st_wake_up();
    return true;
  }


void plan_buffer_line(block_t* block)
{

    // The target position of the tool in absolute steps
    // Calculate target position in absolute steps
    long target[4];
    target[X_AXIS] = lround(block->requestedposition[X_AXIS]*axis_steps_per_unit[X_AXIS]);
    target[Y_AXIS] = lround(block->requestedposition[Y_AXIS]*axis_steps_per_unit[Y_AXIS]);
    target[Z_AXIS] = lround(block->requestedposition[Z_AXIS]*axis_steps_per_unit[Z_AXIS]);
    target[E_AXIS] = lround(block->requestedposition[E_AXIS]*axis_steps_per_unit[E_AXIS]);
    
   
    // Number of steps for each axis
    block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
    block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
    block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
    block->steps_e = labs(target[E_AXIS]-position[E_AXIS]);
    block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));

    // Bail if this is a zero-length block
    if (block->step_event_count == 0) { 
      return; 
    };

    float delta_x_mm = (target[X_AXIS]-position[X_AXIS])/axis_steps_per_unit[X_AXIS];
    float delta_y_mm = (target[Y_AXIS]-position[Y_AXIS])/axis_steps_per_unit[Y_AXIS];
    float delta_z_mm = (target[Z_AXIS]-position[Z_AXIS])/axis_steps_per_unit[Z_AXIS];
    float delta_e_mm = (target[E_AXIS]-position[E_AXIS])/axis_steps_per_unit[E_AXIS];
    block->millimeters = sqrt(square(delta_x_mm) + square(delta_y_mm) + square(delta_z_mm) + square(delta_e_mm));

    unsigned long microseconds;
    microseconds = lround((block->millimeters/block->requestedfeed)*1000000);
    
    // Calculate speed in mm/minute for each axis
    float multiplier = 60.0*1000000.0/microseconds;
    block->speed_z = delta_z_mm * multiplier; 
    block->speed_x = delta_x_mm * multiplier;
    block->speed_y = delta_y_mm * multiplier;
    block->speed_e = delta_e_mm * multiplier; 

    // Limit speed per axis
    float speed_factor = 1;
    float tmp_speed_factor;
    if(abs(block->speed_x) > max_feedrate[X_AXIS]) {
      speed_factor = max_feedrate[X_AXIS] / abs(block->speed_x);
    }
    if(abs(block->speed_y) > max_feedrate[Y_AXIS]){
      tmp_speed_factor = max_feedrate[Y_AXIS] / abs(block->speed_y);
      if(speed_factor > tmp_speed_factor) speed_factor = tmp_speed_factor;
    }
    if(abs(block->speed_z) > max_feedrate[Z_AXIS]){
      tmp_speed_factor = max_feedrate[Z_AXIS] / abs(block->speed_z);
      if(speed_factor > tmp_speed_factor) speed_factor = tmp_speed_factor;
    }
    if(abs(block->speed_e) > max_feedrate[E_AXIS]){
      tmp_speed_factor = max_feedrate[E_AXIS] / abs(block->speed_e);
      if(speed_factor > tmp_speed_factor) speed_factor = tmp_speed_factor;
    }
    multiplier = multiplier * speed_factor;
    block->speed_z = delta_z_mm * multiplier; 
    block->speed_x = delta_x_mm * multiplier;
    block->speed_y = delta_y_mm * multiplier;
    block->speed_e = delta_e_mm * multiplier; 
    block->nominal_speed = block->millimeters * multiplier;
    block->nominal_rate = ceil(block->step_event_count * multiplier / 60);  

    if(block->nominal_rate < 120) block->nominal_rate = 120;
    block->entry_speed = safe_speed(block);

    // Compute the acceleration rate for the trapezoid generator. 
    float travel_per_step = block->millimeters/block->step_event_count;
    if(block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0) {
      block->acceleration = ceil( (retract_acceleration)/travel_per_step); // convert to: acceleration steps/sec^2
    }
    else {
      block->acceleration = ceil( (acceleration)/travel_per_step);      // convert to: acceleration steps/sec^2
      // Limit acceleration per axis
      if((block->acceleration * block->steps_x / block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
        block->acceleration = axis_steps_per_sqr_second[X_AXIS];
      if((block->acceleration * block->steps_y / block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
        block->acceleration = axis_steps_per_sqr_second[Y_AXIS];
      if((block->acceleration * block->steps_e / block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
        block->acceleration = axis_steps_per_sqr_second[E_AXIS];
      if((block->acceleration * block->steps_z / block->step_event_count) > axis_steps_per_sqr_second[Z_AXIS])
        block->acceleration = axis_steps_per_sqr_second[Z_AXIS];
    }

  #ifdef ADVANCE
    // Calculate advance rate
    if((block->steps_e == 0) || (block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)) {
      block->advance_rate = 0;
      block->advance = 0;
    }
    else {
      long acc_dist = estimate_acceleration_distance(0, block->nominal_rate, block->acceleration);
      float advance = (STEPS_PER_CUBIC_MM_E * EXTRUDER_ADVANCE_K) * 
        (block->speed_e * block->speed_e * EXTRUTION_AREA * EXTRUTION_AREA / 3600.0)*65536;
      block->advance = advance;
      if(acc_dist == 0) {
        block->advance_rate = 0;
      } 
      else {
        block->advance_rate = advance / (float)acc_dist;
      }
    }

  #endif // ADVANCE

    // compute a preliminary conservative acceleration trapezoid
    float safespeed = safe_speed(block);
    calculate_trapezoid_for_block(block, safespeed, safespeed); 

    // Compute direction bits for this block
    block->direction_bits = 0;
    if (target[X_AXIS] < position[X_AXIS]) { 
      block->direction_bits |= (1<<X_AXIS); 
    }
    if (target[Y_AXIS] < position[Y_AXIS]) { 
      block->direction_bits |= (1<<Y_AXIS); 
    }
    if (target[Z_AXIS] < position[Z_AXIS]) { 
      block->direction_bits |= (1<<Z_AXIS); 
    }
    if (target[E_AXIS] < position[E_AXIS]) { 
      block->direction_bits |= (1<<E_AXIS); 
    }

    //enable active axes
    if(block->steps_x != 0) enable(X_AXIS);
    if(block->steps_y != 0) enable(Y_AXIS);
    if(block->steps_z != 0) enable(Z_AXIS);
    if(block->steps_e != 0) enable(E_AXIS);

    // Update position 
    memcpy(position, target, sizeof(target)); // position[] = target[]
    memcpy(block->endposition, position, sizeof(block->endposition));
  }


  void plan_set_position(float x, float y, float z, float e)
  {
    position[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
    position[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
    position[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
    position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);     
  }

  // Stepper

  // intRes = intIn1 * intIn2 >> 16
  // uses:
  // r26 to store 0
  // r27 to store the byte 1 of the 24 bit result
  #define MultiU16X8toH16(intRes, charIn1, intIn2) \
  asm volatile ( \
  "clr r26 \n\t" \
  "mul %A1, %B2 \n\t" \
  "movw %A0, r0 \n\t" \
  "mul %A1, %A2 \n\t" \
  "add %A0, r1 \n\t" \
  "adc %B0, r26 \n\t" \
  "lsr r0 \n\t" \
  "adc %A0, r26 \n\t" \
  "adc %B0, r26 \n\t" \
  "clr r1 \n\t" \
  : \
  "=&r" (intRes) \
  : \
  "d" (charIn1), \
  "d" (intIn2) \
  : \
  "r26" \
  )

  // intRes = longIn1 * longIn2 >> 24
  // uses:
  // r26 to store 0
  // r27 to store the byte 1 of the 48bit result
  #define MultiU24X24toH16(intRes, longIn1, longIn2) \
  asm volatile ( \
  "clr r26 \n\t" \
  "mul %A1, %B2 \n\t" \
  "mov r27, r1 \n\t" \
  "mul %B1, %C2 \n\t" \
  "movw %A0, r0 \n\t" \
  "mul %C1, %C2 \n\t" \
  "add %B0, r0 \n\t" \
  "mul %C1, %B2 \n\t" \
  "add %A0, r0 \n\t" \
  "adc %B0, r1 \n\t" \
  "mul %A1, %C2 \n\t" \
  "add r27, r0 \n\t" \
  "adc %A0, r1 \n\t" \
  "adc %B0, r26 \n\t" \
  "mul %B1, %B2 \n\t" \
  "add r27, r0 \n\t" \
  "adc %A0, r1 \n\t" \
  "adc %B0, r26 \n\t" \
  "mul %C1, %A2 \n\t" \
  "add r27, r0 \n\t" \
  "adc %A0, r1 \n\t" \
  "adc %B0, r26 \n\t" \
  "mul %B1, %A2 \n\t" \
  "add r27, r1 \n\t" \
  "adc %A0, r26 \n\t" \
  "adc %B0, r26 \n\t" \
  "lsr r27 \n\t" \
  "adc %A0, r26 \n\t" \
  "adc %B0, r26 \n\t" \
  "clr r1 \n\t" \
  : \
  "=&r" (intRes) \
  : \
  "d" (longIn1), \
  "d" (longIn2) \
  : \
  "r26" , "r27" \
  )

  // Some useful constants

  #define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
  #define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

  static block_t *current_block;  // A pointer to the block currently being traced

  // Variables used by The Stepper Driver Interrupt
  static unsigned char out_bits;        // The next stepping-bits to be output
  static long counter_x,       // Counter variables for the bresenham line tracer
              counter_y, 
              counter_z,       
              counter_e;
  static unsigned long step_events_completed; // The number of step events executed in the current block
  static long advance_rate, advance, final_advance = 0;
  static short old_advance = 0;
  static short e_steps;
  static unsigned char busy = false; // TRUE when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.
  static long acceleration_time, deceleration_time;
  static long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
  static unsigned short acc_step_rate; // needed for deccelaration start point



  //         __________________________
  //        /|                        |\     _________________         ^
  //       / |                        | \   /|               |\        |
  //      /  |                        |  \ / |               | \       s
  //     /   |                        |   |  |               |  \      p
  //    /    |                        |   |  |               |   \     e
  //   +-----+------------------------+---+--+---------------+----+    e
  //   |               BLOCK 1            |      BLOCK 2          |    d
  //
  //                           time ----->
  // 
  //  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates 
  //  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
  //  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
  //  The slope of acceleration is calculated with the leib ramp alghorithm.

  void st_wake_up() {
    //  TCNT1 = 0;
    ENABLE_STEPPER_DRIVER_INTERRUPT();  
  }

  inline unsigned short calc_timer(unsigned short step_rate) {
    unsigned short timer;
    if(step_rate < 32) step_rate = 32;
    step_rate -= 32; // Correct for minimal speed
    if(step_rate >= (8*256)){ // higher step rate 
      unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
      unsigned char tmp_step_rate = (step_rate & 0x00ff);
      unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
      MultiU16X8toH16(timer, tmp_step_rate, gain);
      timer = (unsigned short)pgm_read_word_near(table_address) - timer;
    }
    else { // lower step rates
      unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
      table_address += ((step_rate)>>1) & 0xfffc;
      timer = (unsigned short)pgm_read_word_near(table_address);
      timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
    }
    if(timer < 100) timer = 100;
    return timer;
  }

  // Initializes the trapezoid generator from the current block. Called whenever a new 
  // block begins.
  inline void trapezoid_generator_reset() {
    accelerate_until = current_block->accelerate_until;
    decelerate_after = current_block->decelerate_after;
    acceleration_rate = current_block->acceleration_rate;
    initial_rate = current_block->initial_rate;
    final_rate = current_block->final_rate;
    nominal_rate = current_block->nominal_rate;
    advance = current_block->initial_advance;
    final_advance = current_block->final_advance;
    deceleration_time = 0;
    advance_rate = current_block->advance_rate;
    // step_rate to timer interval
    acc_step_rate = initial_rate;
    acceleration_time = calc_timer(acc_step_rate);
    OCR1A = acceleration_time;
  }

  // "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
  // It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
  ISR(TIMER1_COMPA_vect)
  {        
    if(busy){ 
      return; 
    } // The busy-flag is used to avoid reentering this interrupt

    busy = true;
    sei(); // Re enable interrupts (normally disabled while inside an interrupt handler)

    // If there is no current block, attempt to pop one from the buffer
    if (current_block == NULL) {
      // Anything in the buffer?
      current_block = plan_get_current_block();
      if (current_block != NULL) {
        trapezoid_generator_reset();
        counter_x = -(current_block->step_event_count >> 1);
        counter_y = counter_x;
        counter_z = counter_x;
        counter_e = counter_x;
        step_events_completed = 0;
        e_steps = 0;
      } 
      else {
        DISABLE_STEPPER_DRIVER_INTERRUPT();
      }    
    } 

    if (current_block != NULL) {
      // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
      out_bits = current_block->direction_bits;

  #ifdef ADVANCE
      // Calculate E early.
      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        counter_e -= current_block->step_event_count;
        if ((out_bits & (1<<E_AXIS)) != 0) { // - direction
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            e_steps--;
          }
        }
        else {
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            e_steps++;
          }
        }
      }    
      // Do E steps + advance steps
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        e_steps += ((advance >> 16) - old_advance);
      }
      old_advance = advance >> 16;  
  #endif //ADVANCE

      // Set direction en check limit switches
      if ((out_bits & (1<<X_AXIS)) != 0) {   // -direction
        WRITE(DIR_PINS[X_AXIS], INVERT_DIRS[X_AXIS]);
        if(READ(MIN_PINS[X_AXIS]) != ENDSTOP_INVERT) {
          step_events_completed = current_block->step_event_count;
        }
      }
      else // +direction
      WRITE(DIR_PINS[X_AXIS],!INVERT_DIRS[X_AXIS]);

      if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
        WRITE(DIR_PINS[Y_AXIS],INVERT_DIRS[Y_AXIS]);
        if(READ(MIN_PINS[Y_AXIS]) != ENDSTOP_INVERT) {
          step_events_completed = current_block->step_event_count;
        }
      }
      else // +direction
      WRITE(DIR_PINS[Y_AXIS],!INVERT_DIRS[Y_AXIS]);

      if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
        WRITE(DIR_PINS[Z_AXIS],INVERT_DIRS[Z_AXIS]);
        if(READ(MIN_PINS[Z_AXIS]) != ENDSTOP_INVERT) {
          step_events_completed = current_block->step_event_count;
        }
      }
      else // +direction
      WRITE(DIR_PINS[Z_AXIS],!INVERT_DIRS[Z_AXIS]);

  #ifndef ADVANCE
      if ((out_bits & (1<<E_AXIS)) != 0)   // -direction
        WRITE(DIR_PINS[E_AXIS],INVERT_DIRS[E_AXIS]);
      else // +direction
        WRITE(DIR_PINS[E_AXIS],!INVERT_DIRS[E_AXIS]);
  #endif //!ADVANCE

      counter_x += current_block->steps_x;
      if (counter_x > 0) {
        WRITE(STEP_PINS[X_AXIS], HIGH);
        counter_x -= current_block->step_event_count;
        WRITE(STEP_PINS[X_AXIS], LOW);
      }

      counter_y += current_block->steps_y;
      if (counter_y > 0) {
        WRITE(STEP_PINS[Y_AXIS], HIGH);
        counter_y -= current_block->step_event_count;
        WRITE(STEP_PINS[Y_AXIS], LOW);
      }

      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        WRITE(STEP_PINS[Z_AXIS], HIGH);
        counter_z -= current_block->step_event_count;
        WRITE(STEP_PINS[Z_AXIS], LOW);
      }

  #ifndef ADVANCE
      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        WRITE(STEP_PINS[E_AXIS], HIGH);
        counter_e -= current_block->step_event_count;
        WRITE(STEP_PINS[E_AXIS], LOW);
      }
  #endif //!ADVANCE

      // Calculare new timer value
      unsigned short timer;
      unsigned short step_rate;
      if (step_events_completed < accelerate_until) {
        MultiU24X24toH16(acc_step_rate, acceleration_time, acceleration_rate);
        acc_step_rate += initial_rate;
        
        // upper limit
        if(acc_step_rate > nominal_rate)
          acc_step_rate = nominal_rate;

        // step_rate to timer interval
        timer = calc_timer(acc_step_rate);
        advance += advance_rate;
        acceleration_time += timer;
        OCR1A = timer;
      } 
      else if (step_events_completed >= decelerate_after) {   
        MultiU24X24toH16(step_rate, deceleration_time, acceleration_rate);
        
        if(step_rate > acc_step_rate) { // Check step_rate stays positive
          step_rate = final_rate;
        }
        else {
          step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
        }

        // lower limit
        if(step_rate < final_rate)
          step_rate = final_rate;

        // step_rate to timer interval
        timer = calc_timer(step_rate);
  #ifdef ADVANCE
        advance -= advance_rate;
        if(advance < final_advance)
          advance = final_advance;
  #endif //ADVANCE
        deceleration_time += timer;
        OCR1A = timer;
      }       
      // If current block is finished, reset pointer 
      step_events_completed += 1;  
      if (step_events_completed >= current_block->step_event_count) {
        current_block = NULL;
        plan_discard_current_block();
      }   
    } 
    busy=false;
  }

  #ifdef ADVANCE

  unsigned char old_OCR0A;
  // Timer interrupt for E. e_steps is set in the main routine;
  // Timer 0 is shared with millies
  ISR(TIMER0_COMPA_vect)
  {
    // Critical section needed because Timer 1 interrupt has higher priority. 
    // The pin set functions are placed on trategic position to comply with the stepper driver timing.
    WRITE(STEP_PINS[E_AXIS], LOW);
    // Set E direction (Depends on E direction + advance)
    if (e_steps < 0) {
      WRITE(DIR_PINS[E_AXIS],INVERT_DIRS[E_AXIS]);    
      e_steps++;
      WRITE(STEP_PINS[E_AXIS], HIGH);
    } 
    if (e_steps > 0) {
      WRITE(DIR_PINS[E_AXIS],!INVERT_DIRS[E_AXIS]);
      e_steps--;
      WRITE(STEP_PINS[E_AXIS], HIGH);
    }
    old_OCR0A += 25; // 10kHz interrupt
    OCR0A = old_OCR0A;
  }
  #endif // ADVANCE

  void st_init()
  {
    // waveform generation = 0100 = CTC
    TCCR1B &= ~(1<<WGM13);
    TCCR1B |=  (1<<WGM12);
    TCCR1A &= ~(1<<WGM11); 
    TCCR1A &= ~(1<<WGM10);

    // output mode = 00 (disconnected)
    TCCR1A &= ~(3<<COM1A0); 
    TCCR1A &= ~(3<<COM1B0); 
    TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10); // 2MHz timer

    OCR1A = 0x4000;
    DISABLE_STEPPER_DRIVER_INTERRUPT();  

  #ifdef ADVANCE
    e_steps = 0;
    TIMSK0 |= (1<<OCIE0A);
  #endif //ADVANCE
  }

  // Block until all buffered steps are executed
  void st_synchronize()
  {
    while(plan_get_current_block()) {
    }   
  }

  // Returns current position of all Axes
  Point& getCurrentPosition()
  {
    static Point p;
    return p;
  }
  // Sets current position; doesn't cause a move, just updates the current position variables.
  void setCurrentPosition(GCode &gcode)
  {
    return;
  }
  // Interpret movement data as absolute
  void setAbsolute()
  {
    RELATIVEMOVES = false;
  }
  // Interpret movement data as relative
  void setRelative()
  {
    RELATIVEMOVES = true;
  }
  // Returns true if machine is in motion
  bool axesAreMoving()
  {
    return false;
  }
  // Change stored feedrates/axis data
  // WARNING: if you change steps per unit without then changing/resetting all feedrates
  // after things will not go well for you!
#define AXESLOOP(x) for(int x=0;x<NUM_AXIS;x++)
  void setMinimumFeedrate(GCode& gcode) 
  { 
    AXESLOOP(ax) 
    {
      if(gcode[ax].isUnused())
        continue;
      min_feedrate[ax] = gcode[ax].getFloat();
    }
  }
  void setMaximumFeedrate(GCode& gcode)
  { 
    AXESLOOP(ax) 
    {
      if(gcode[ax].isUnused())
        continue;
      max_feedrate[ax] = gcode[ax].getFloat();
    }
  }
  void setAverageFeedrate(GCode& gcode) { return; }
  void setStepsPerUnit(GCode& gcode)
  { 
    AXESLOOP(ax) 
    {
      if(gcode[ax].isUnused())
        continue;
      axis_steps_per_unit[ax] = gcode[ax].getFloat();
    }
  }

  void setAccel(GCode& gcode) 
  { 
    AXESLOOP(ax) 
    {
      if(gcode[ax].isUnused())
        continue;
      if(ax == E)
        retract_acceleration = gcode[ax].getFloat();
      else
        acceleration = gcode[ax].getFloat();
    }
  }

#define PINSET(FOO) AXESLOOP(ax) { if(gcode[ax].isUnused()) continue; FOO[ax] = Pin(ArduinoMap::getPort(gcode[ax].getInt()), ArduinoMap::getPinnum(gcode[ax].getInt())); }
  void setStepPins(GCode& gcode) { PINSET(STEP_PINS); }
  void setDirPins(GCode& gcode) { PINSET(DIR_PINS); }
  void setEnablePins(GCode& gcode) { PINSET(ENABLE_PINS); }
  void setMinPins(GCode& gcode) { PINSET(MIN_PINS); }
  void setMaxPins(GCode& gcode) { PINSET(MAX_PINS); }
  void setMinStopPos(GCode& gcode) { return; }
  void setMaxStopPos(GCode& gcode) { return; }
  void setAxisInvert(GCode& gcode) 
  {  
    AXESLOOP(ax) 
    {
      if(gcode[ax].isUnused()) continue;
      INVERT_DIRS[ax] = gcode[ax].getInt();
    }
  }
  void setAxisDisable(GCode& gcode) 
  {
    AXESLOOP(ax) 
    {
      if(gcode[ax].isUnused()) continue;
      DISABLE[ax] = gcode[ax].getInt();
    }
  }
  void setEndstopGlobals(bool inverted, bool pulledup) { ENDSTOP_INVERT=inverted; ENDSTOP_PULLUPS=pulledup; }
  void reportConfigStatus(Host& h) 
  {
    
  }
  // Motors automatically enabled when used
  void disableAllMotors() { AXESLOOP(ax) { disable(ax); } }
  void wrapup(GCode& gcode) { return; }
  void checkdisable(GCode& gcode) { return; }

  void init()
  {
    plan_init();  // Initialize planner;
    st_init();    // Initialize stepper;
  }





}; // namespace Marlin
