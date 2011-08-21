/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
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

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "GrblMotion.h"
#include "config.h"
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "GrblPlanner.h"
#include "Motion.h"

namespace GrblMotion {

#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CYCLES_PER_ACCELERATION_TICK ((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

#define MINIMUM_STEPS_PER_MINUTE 1200 // The stepper subsystem will never run slower than this, exept when sleeping

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

static block_t *current_block;  // A pointer to the block currently being traced

// Variables used by The Stepper Driver Interrupt
static int32_t counters[NUM_AXES];       // Counter variables for the bresenham line tracer
static uint32_t step_events_completed; // The number of step events executed in the current block
static volatile bool busy; // true when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.

// Variables used by the trapezoid generation
static uint32_t cycles_per_step_event;        // The number of machine cycles between each step event
static uint32_t trapezoid_tick_cycle_counter; // The cycles since last trapezoid_tick. Used to generate ticks at a steady
                                              // pace without allocating a separate timer
static uint32_t trapezoid_adjusted_rate;      // The current rate of step_events according to the trapezoid generator

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
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates by block->rate_delta
//  during the first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is always +/- block->rate_delta and is applied at a constant rate by trapezoid_generator_tick()
//  that is called ACCELERATION_TICKS_PER_SECOND times per second.

void set_step_events_per_minute(uint32_t steps_per_minute);

void st_wake_up() {
  ENABLE_STEPPER_DRIVER_INTERRUPT();  
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
inline void trapezoid_generator_reset() {
  trapezoid_adjusted_rate = current_block->initial_rate;  
  trapezoid_tick_cycle_counter = 0; // Always start a new trapezoid with a full acceleration tick
  set_step_events_per_minute(trapezoid_adjusted_rate);
}

// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
inline void trapezoid_generator_tick() {     
  if (current_block) {
    if (step_events_completed < current_block->accelerate_until) {
      trapezoid_adjusted_rate += current_block->rate_delta;
      if (trapezoid_adjusted_rate > current_block->nominal_rate ) {
        trapezoid_adjusted_rate = current_block->nominal_rate;
      }
      set_step_events_per_minute(trapezoid_adjusted_rate);
    } else if (step_events_completed > current_block->decelerate_after) {
      // NOTE: We will only reduce speed if the result will be > 0. This catches small
      // rounding errors that might leave steps hanging after the last trapezoid tick.
      if (trapezoid_adjusted_rate > current_block->rate_delta) {
        trapezoid_adjusted_rate -= current_block->rate_delta;
      }
      if (trapezoid_adjusted_rate < current_block->final_rate) {
        trapezoid_adjusted_rate = current_block->final_rate;
      }        
      set_step_events_per_minute(trapezoid_adjusted_rate);
    } else {
      // Make sure we cruise at exactly nominal rate
      if (trapezoid_adjusted_rate != current_block->nominal_rate) {
        trapezoid_adjusted_rate = current_block->nominal_rate;
        set_step_events_per_minute(trapezoid_adjusted_rate);
      }
    }
  }
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. It is  executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
// It is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port after each pulse.
SIGNAL(TIMER1_COMPA_vect)
{        
  // TODO: Check if the busy-flag can be eliminated by just disabeling this interrupt while we are in it
  
  if(busy){ return; } // The busy-flag is used to avoid reentering this interrupt

  busy = true;
  sei(); // Re enable interrupts (normally disabled while inside an interrupt handler)
         // ((We re-enable interrupts in order for SIG_OVERFLOW2 to be able to be triggered 
         // at exactly the right time even if we occasionally spend a lot of time inside this handler.))
    
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = GrblPlanner::plan_get_current_block();
    if (current_block != NULL) {
      trapezoid_generator_reset();
      for(int ax=0;ax < NUM_AXES;ax++)
        counters[ax] = -(current_block->step_event_count >> 1);

      step_events_completed = 0;
    } else {
      DISABLE_STEPPER_DRIVER_INTERRUPT();
    }    
  } 

  if (current_block != NULL) {
    // Set the direction pins a cuple of nanoseconds before we step the steppers
    for(int ax=0;ax < NUM_AXES;ax++)
      MOTION.getAxis(ax).setDirection(current_block->directions[ax]);

    bool keepgoing = false;
    for(int ax = 0; ax < NUM_AXES; ax++)
    {
      counters[ax] += current_block->steps[ax];
      if(counters[ax] > 0)
      {
        // DO STEP
        if(MOTION.getAxis(ax).doStep(current_block->directions[ax]) == true)
          keepgoing = true;

        counters[ax] -= current_block->step_event_count;
      }
    }
    // if keepgoing is false, it means all involved axis have finished early due to hitting
    // endstops
    if(!keepgoing)
    {
      step_events_completed = current_block->step_event_count;
    }

    // If current block is finished, reset pointer 
    step_events_completed += 1;
    if (step_events_completed >= current_block->step_event_count) 
    {
      current_block = NULL;
      GrblPlanner::plan_discard_current_block();
    }
  } else {
    // nothing to do; no current block;
  }          
  
  // In average this generates a trapezoid_generator_tick every CYCLES_PER_ACCELERATION_TICK by keeping track
  // of the number of elapsed cycles. The code assumes that step_events occur significantly more often than
  // trapezoid_generator_ticks as they well should. 
  trapezoid_tick_cycle_counter += cycles_per_step_event;
  if(trapezoid_tick_cycle_counter > CYCLES_PER_ACCELERATION_TICK) {
    trapezoid_tick_cycle_counter -= CYCLES_PER_ACCELERATION_TICK;
    trapezoid_generator_tick();
  }
  
  busy=false;
}

// Initialize and start the stepper motor subsystem
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
	
  set_step_events_per_minute(6000);
  DISABLE_STEPPER_DRIVER_INTERRUPT();  
  trapezoid_tick_cycle_counter = 0;
}

// Configures the prescaler and ceiling of timer 1 to produce the given rate as accurately as possible.
// Returns the actual number of cycles per interrupt
uint32_t config_step_timer(uint32_t cycles)
{
  uint16_t ceiling;
  uint16_t prescaler;
  uint32_t actual_cycles;
	if (cycles <= 0xffffL) {
		ceiling = cycles;
    prescaler = 0; // prescaler: 0
    actual_cycles = ceiling;
	} else if (cycles <= 0x7ffffL) {
    ceiling = cycles >> 3;
    prescaler = 1; // prescaler: 8
    actual_cycles = ceiling * 8L;
	} else if (cycles <= 0x3fffffL) {
		ceiling =  cycles >> 6;
    prescaler = 2; // prescaler: 64
    actual_cycles = ceiling * 64L;
	} else if (cycles <= 0xffffffL) {
		ceiling =  (cycles >> 8);
    prescaler = 3; // prescaler: 256
    actual_cycles = ceiling * 256L;
	} else if (cycles <= 0x3ffffffL) {
		ceiling = (cycles >> 10);
    prescaler = 4; // prescaler: 1024
    actual_cycles = ceiling * 1024L;    
	} else {
	  // Okay, that was slower than we actually go. Just set the slowest speed
		ceiling = 0xffff;
    prescaler = 4;
    actual_cycles = 0xffff * 1024;
	}
	// Set prescaler
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | ((prescaler+1)<<CS10);
  // Set ceiling
  OCR1A = ceiling;
  return(actual_cycles);
}

void set_step_events_per_minute(uint32_t steps_per_minute) {
  if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE) { steps_per_minute = MINIMUM_STEPS_PER_MINUTE; }
  cycles_per_step_event = config_step_timer((TICKS_PER_MICROSECOND*1000000*60)/steps_per_minute);
}

void st_go_home()
{
  // Todo: Perform the homing cycle
}


}; // namespace
