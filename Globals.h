#ifndef _GLOBALS_H_
#define _GLOBALS_H_

int getFreeRam () {
  // These externs are defined and used in libavr
  extern int __heap_start, *__brkval; 
  // Really I want SP, but without writing any asm, this is an easy way to get it!
  int v; 

  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval) - sizeof(int); 
}

#endif
