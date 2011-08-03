#ifndef _TIME_H_
#define _TIME_H_

/* Timekeeping and time function lib
   (c) 2011, Christopher "ScribbleJ" Jansen

*/

#define cyclesPerMicro() ( F_CPU / 1000000L )
#define cyclesToMicros(x) ( (x) / CyclesPerMicro() )
#define microsToCycles(x) ( (x) * CyclesPerMicro() )


void init_time();
unsigned long millis();
unsigned long micros();
void wait(unsigned long millis);






#endif // _TIME_H
