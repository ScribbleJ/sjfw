#include "Time.h"
#include <avr/interrupt.h>
// Code stolen from Arduino libs

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
  uint8_t oldSREG = SREG;

  // disable interrupts while we read timer0_millis or we might get an
  // inconsistent value (e.g. in the middle of the timer0_millis++)
  cli();
  m = timer0_millis;
  SREG = oldSREG;

  return m;
}

unsigned long micros() {
  unsigned long m, t;
  uint8_t oldSREG = SREG;

  cli();
  t = TCNT0;

#ifdef TIFR0
  if ((TIFR0 & _BV(TOV0)) && (t == 0))
    t = 256;
#else
  if ((TIFR & _BV(TOV0)) && (t == 0))
    t = 256;
#endif

  m = timer0_overflow_count;
  SREG = oldSREG;

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


