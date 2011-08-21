/*
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

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
                                                                                                            

#include <inttypes.h>
#include <math.h>       
#include <stdlib.h>

#include "GrblPlanner.h"
#include "GrblMotion.h"
#include "config.h"
#include "Point.h"
#include "Motion.h"

namespace GrblPlanner {

#define BLOCK_BUFFER_SIZE 20

#define clear_vector(a) memset(a, 0, sizeof(a))

static block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
static volatile int block_buffer_head;           // Index of the next block to be pushed
static volatile int block_buffer_tail;           // Index of the block to process now

// The current position of the tool in absolute steps
static int32_t position[NUM_AXES];   

static uint8_t acceleration_manager_enabled;   // Acceleration management active?

#define ONE_MINUTE_OF_MICROSECONDS 60000000.0

void replan_buffer_line(block_t *b);

// Returns current position in absolute mm
Point getPosition()
{
  Point p;
  for(int ax=0;ax<NUM_AXES;ax++)
    p[ax] = position[ax] / MOTION.getAxis(ax).getStepsPerMM();

  return p;
}
void setPosition(Point p)
{
  for(int ax=0;ax<NUM_AXES;ax++)
    position[ax] = p[ax] * MOTION.getAxis(ax).getStepsPerMM();
}


// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
double estimate_acceleration_distance(double initial_rate, double target_rate, double acceleration) {
  return(
    (target_rate*target_rate-initial_rate*initial_rate)/
    (2L*acceleration)
  );
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

/*                        + <- some maximum rate we don't care about
                         /|\
                        / | \                    
                       /  |  + <- final_rate     
                      /   |  |                   
     initial_rate -> +----+--+                   
                          ^  ^                   
                          |  |                   
      intersection_distance  distance                                                                           */

double intersection_distance(double initial_rate, double final_rate, double acceleration, double distance) {
  return(
    (2*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
    (4*acceleration)
  );
}


// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// The factors represent a factor of braking and must be in the range 0.0-1.0.

/*                                                                              
                                     +--------+   <- nominal_rate
                                    /          \                                
    nominal_rate*entry_factor ->   +            \                               
                                   |             + <- nominal_rate*exit_factor  
                                   +-------------+                              
                                       time -->                                 
*/                                                                              

void calculate_trapezoid_for_block(block_t *block, double entry_factor, double exit_factor) {
  block->initial_rate = ceil(block->nominal_rate*entry_factor);
  block->final_rate = ceil(block->nominal_rate*exit_factor);
  int32_t acceleration_per_minute = block->rate_delta*ACCELERATION_TICKS_PER_SECOND*60.0;
  int32_t accelerate_steps = 
    ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration_per_minute));
  int32_t decelerate_steps = 
    floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate, -acceleration_per_minute));

  // Calculate the size of Plateau of Nominal Rate. 
  int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;
  
  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking 
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {  
    accelerate_steps = ceil(
      intersection_distance(block->initial_rate, block->final_rate, acceleration_per_minute, block->step_event_count));
    plateau_steps = 0;
  }  
  
  block->accelerate_until = accelerate_steps;
  block->decelerate_after = accelerate_steps+plateau_steps;
}                    

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
double max_allowable_speed(double acceleration, double target_velocity, double distance) {
  return(
    sqrt(target_velocity*target_velocity-2*acceleration*60*60*distance)
  );
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal 
// velocities of the respective blocks.
double junction_jerk(block_t *before, block_t *after) {
  float foo;
  for(int ax=0;ax<NUM_AXES;ax++)
  {
    foo += pow(before->speeds[ax] - after->speeds[ax], 2);
  }

  return(sqrt(foo));
}

// Calculate a braking factor to reach baseline speed which is max_jerk/2, e.g. the 
// speed under which you cannot exceed max_jerk no matter what you do.
double factor_for_safe_speed(block_t *block) {
  return(MAX_JERK/block->nominal_speed);  
}

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!current) { return; }

  double entry_factor = 1.0;
  double exit_factor;
  if (next) {
    exit_factor = next->entry_factor;
  } else {
    exit_factor = factor_for_safe_speed(current);
  }
  
  // Calculate the entry_factor for the current block. 
  if (previous) {
    // Reduce speed so that junction_jerk is within the maximum allowed
    double jerk = junction_jerk(previous, current);
    if (jerk > MAX_JERK) {
      entry_factor = (MAX_JERK/jerk);
    } 
    // If the required deceleration across the block is too rapid, reduce the entry_factor accordingly.
    if (entry_factor > exit_factor) {
      double max_entry_speed = max_allowable_speed(current->acceleration,current->nominal_speed*exit_factor, 
        current->millimeters);
      double max_entry_factor = max_entry_speed/current->nominal_speed;
      if (max_entry_factor < entry_factor) {
        entry_factor = max_entry_factor;
      }
    }    
  } else {
    entry_factor = factor_for_safe_speed(current);
  }
    
  // Store result
  current->entry_factor = entry_factor;
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass() {
  auto int8_t block_index = block_buffer_head;
  block_t *block[3] = {NULL, NULL, NULL};
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
  if(!current) { return; }
  if(previous) {
    // If the previous block is an acceleration block, but it is not long enough to 
    // complete the full speed change within the block, we need to adjust out entry
    // speed accordingly. Remember current->entry_factor equals the exit factor of 
    // the previous block.
    if(previous->entry_factor < current->entry_factor) {
      double max_entry_speed = max_allowable_speed(-current->acceleration,
        current->nominal_speed*previous->entry_factor, previous->millimeters);
      double max_entry_factor = max_entry_speed/current->nominal_speed;
      if (max_entry_factor < current->entry_factor) {
        current->entry_factor = max_entry_factor;
      }
    }
  }
}

void planner_reinit_blocks() {
  auto int8_t block_index = block_buffer_tail;
  block_t * b = NULL;
  while(block_index != block_buffer_head) 
  {
    replan_buffer_line(b);    
    block_index = (block_index+1) % BLOCK_BUFFER_SIZE;
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
void planner_forward_pass() {
  int8_t block_index = block_buffer_tail;
  block_t *block[3] = {NULL, NULL, NULL};
  
  while(block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1],block[2]);
    block_index = (block_index+1) % BLOCK_BUFFER_SIZE;
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;
  
  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      calculate_trapezoid_for_block(current, current->entry_factor, next->entry_factor);      
    }
    block_index = (block_index+1) % BLOCK_BUFFER_SIZE;
  }
  calculate_trapezoid_for_block(next, next->entry_factor, factor_for_safe_speed(next));
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
  plan_set_acceleration_manager_enabled(true);
  clear_vector(position);
}

void plan_set_acceleration_manager_enabled(int enabled) {
  if ((!!acceleration_manager_enabled) != (!!enabled)) {
    acceleration_manager_enabled = !!enabled;
  }
}

int plan_is_acceleration_manager_enabled() {
  return(acceleration_manager_enabled);
}

void plan_discard_current_block() {
  if (block_buffer_head != block_buffer_tail) {
    block_buffer_tail = (block_buffer_tail + 1) % BLOCK_BUFFER_SIZE;  
  }
}

block_t *plan_get_current_block() {
  if (block_buffer_head == block_buffer_tail) { return(NULL); }
  return(&block_buffer[block_buffer_tail]);
}

// Functions for caller to use to see how the buffer is
bool isBufferEmpty()
{
  if(block_buffer_head == block_buffer_tail) 
    return true;

  return false;
}

bool isBufferFull()
{
  if((block_buffer_head + 1) % BLOCK_BUFFER_SIZE == block_buffer_tail)
    return true;

  return false;
}

// Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in 
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void buffer_line_kernel(float targ[NUM_AXES], double feed_rate, bool relative, block_t *block) {
  // The target position of the tool in absolute steps
  
  // Calculate target position in absolute steps
  int32_t target[NUM_AXES];
  for(int ax = 0; ax<NUM_AXES; ax++)
  {
    if(relative)
      target[ax] = lround(position[ax] + (targ[ax]*(MOTION.getAxis(ax).getStepsPerMM())));
    else
      target[ax] = lround(targ[ax]*(MOTION.getAxis(ax).getStepsPerMM()));
  }

  // Number of steps for each axis
  for(int ax = 0; ax<NUM_AXES; ax++)
  {
    block->steps[ax] = labs(target[ax]-position[ax]);
    block->step_event_count = (block->step_event_count > block->steps[ax]) ? block->step_event_count : block->steps[ax];
  }

  // Bail if this is a zero-length block
  if (block->step_event_count == 0) { return; };
  
  double deltas[NUM_AXES];
  for(int ax = 0; ax<NUM_AXES; ax++)
  {
    deltas[ax] = (target[ax]-position[ax]) / (MOTION.getAxis(ax).getStepsPerMM());
    block->millimeters += square(deltas[ax]);
  }
  block->millimeters = sqrt(block->millimeters);
	
  uint32_t microseconds; 
  microseconds = lround((block->millimeters/feed_rate)*1000000);
  
  // Calculate speed in mm/minute for each axis
  double multiplier = 60.0*1000000.0/microseconds;
  for(int ax = 0; ax<NUM_AXES; ax++)
  {
    block->speeds[ax] = deltas[ax] * multiplier;
  }
  block->nominal_speed = block->millimeters * multiplier;
  block->nominal_rate = ceil(block->step_event_count * multiplier);  
  block->entry_factor = 0.0;
  
  // Compute the acceleration rate for the trapezoid generator. Depending on the slope of the line
  // average travel per step event changes. For a line along one axis the travel per step event
  // is equal to the travel/step in the particular axis. For a 45 degree line the steppers of both
  // axes might step for every step event. Travel per step event is then sqrt(travel_x^2+travel_y^2).
  // To generate trapezoids with contant acceleration between blocks the rate_delta must be computed 
  // specifically for each line to compensate for this phenomenon:
  double travel_per_step = block->millimeters/block->step_event_count;
  block->rate_delta = ceil(
    ((block->acceleration*60.0)/(ACCELERATION_TICKS_PER_SECOND))/ // acceleration mm/sec/sec per acceleration_tick
    travel_per_step);                                               // convert to: acceleration steps/min/acceleration_tick    
  if (acceleration_manager_enabled) {
    // compute a preliminary conservative acceleration trapezoid
    double safe_speed_factor = factor_for_safe_speed(block);
    calculate_trapezoid_for_block(block, safe_speed_factor, safe_speed_factor); 
  } else {
    block->initial_rate = block->nominal_rate;
    block->final_rate = block->nominal_rate;
    block->accelerate_until = 0;
    block->decelerate_after = block->step_event_count;
    block->rate_delta = 0;
  }
  
  // Compute direction bits for this block
  for(int ax=0;ax < NUM_AXES;ax++)
  {
    block->directions[ax] = target[ax] < position[ax] ? false : true;
  }
  
  // Update position 
  memcpy(position, target, sizeof(target)); // position[] = target[]
}
void plan_buffer_line(float targ[NUM_AXES], double feed_rate) 
{
  plan_buffer_line(targ, feed_rate, false);
}

void plan_buffer_line(float targ[NUM_AXES], double feed_rate, bool relative) {
  // Calculate the buffer head after we push this byte
	int next_buffer_head = (block_buffer_head + 1) % BLOCK_BUFFER_SIZE;	
  // If the buffer is full and we're here, that's a big problem.
  while(block_buffer_tail == next_buffer_head) 
  { 
    // TODO: THROW BIG ERROR
    return; 
  }
  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];

  // Save request
  memcpy(block->orig_targ, targ, sizeof(targ));
  block->orig_feed = feed_rate;
  block->orig_relative = relative;

  // Perform setup calcs on this block
  buffer_line_kernel(targ, feed_rate, relative, block);

  // Move buffer head
  block_buffer_head = next_buffer_head;     
  
  if (acceleration_manager_enabled) { planner_recalculate(); }  
  GrblMotion::st_wake_up();
}

void replan_buffer_line(block_t *b)
{
  // The target position of the tool in absolute steps
  buffer_line_kernel(b->orig_targ, b->orig_feed, b->orig_relative, b);
}



}; // namespace
