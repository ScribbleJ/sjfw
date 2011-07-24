#include "Time.h"
#include <avr/interrupt.h>
#include <util/atomic.h>

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_clock_cycles = 0;
volatile unsigned long timer0_millis = 0;

void init_time()
{
  // set timer 0 prescale factor to 64
  TCCR0B = _BV(CS01) | _BV(CS00);

  // enable timer 0 overflow interrupt
  TIMSK0 = _BV(TOIE0);
}

unsigned long millis()
{
  unsigned long m;

  // disable interrupts while we read timer0_millis or we might get an
  // inconsistent value (e.g. in the middle of the timer0_millis++)
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    m = timer0_millis;
  }

  return m;
}

unsigned long micros() {
  unsigned long m, t;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    t = TCNT0;

#ifdef TIFR0
    if ((TIFR0 & _BV(TOV0)) && (t == 0))
      t = 256;
#else
    if ((TIFR & _BV(TOV0)) && (t == 0))
      t = 256;
#endif

  m = timer0_overflow_count;
  }

  return ((m << 8) + t) * (64 / cyclesPerMicro());
}


ISR(TIMER0_OVF_vect)
{
  timer0_overflow_count++;
  // timer 0 prescale factor is 64 and the timer overflows at 256
  timer0_clock_cycles += 64UL * 256UL;
  while (timer0_clock_cycles > cyclesPerMicro() * 1000UL) {
    timer0_clock_cycles -= cyclesPerMicro() * 1000UL;
    timer0_millis++;
  }
}


