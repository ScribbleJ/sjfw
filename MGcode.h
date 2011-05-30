#ifndef _MGCODE_H_
#define _MGCODE_H_

// ato? functions.
#include <stdlib.h>
#include "Host.h"
#include "MBIEC.h"
#include "Movedata.h"
#include <string.h>

class CodeParam
{
public:
  CodeParam(const char name) :letter(name), state(UNUSED) { };

  void unset() { state = UNUSED; u=0; }
  void setFloat(float v) { f = v; state = FLOAT; }
  void setInt(unsigned long v) { u = v; state = INT; }
  void setFloat(const char* v) { f = atof(v); state = FLOAT; }
  void setInt(const char* v) { u = atol(v); state = INT; }
  float& getFloat() { return f; }
  unsigned long& getInt() { return u; }
  bool isUnused() { return state == UNUSED; }

  void dump_to_host() 
  { 
    HOST.write(letter); HOST.write(':');
    switch(state)
    {
      case UNUSED:
        HOST.write("UNUSED");
        break;
      case FLOAT:
        HOST.write(f, 10,5);
        break;
      case INT:
        HOST.write(u,10);
        break;
    }
    HOST.write(' ');
  }

  
  char letter;
  enum states_T { UNUSED, FLOAT, INT } state;
  union 
  {
    float f;
    unsigned long u;
  };
};


// Whether I should define such common letters as globals is a bit questionable, but...
// at any rate, it's CRITICAL the order here matches the array below.  We also
// assume multiple places that T is the last item.
enum { M=0, G, X, Y, Z, E, F, P, S, T };

// It would be a swell plan to make this class a polymorphic hierarchy and pop pointers to base
// in the queue, use dynamic alloc... 
// I might try it, dunno.
class MGcode
{
public:
  MGcode(): cps((CodeParam[10]){  CodeParam('M'), CodeParam('G'), CodeParam('X'), CodeParam('Y'),
                                  CodeParam('Z'), CodeParam('E'), CodeParam('F'), CodeParam('P'),
                                  CodeParam('S'), CodeParam('T') }) { reset(); };
  CodeParam& operator[](int idx) { return cps[idx]; }
  enum mg_states_t { NEW, PREPARED, ACTIVE, DONE };
  volatile mg_states_t state;
  

  void reset()
  {
    for(int x=0;x<T;x++)
      cps[x].unset();
    state = NEW;
    preparecalls = 0;
    executecalls = 0;
    lastms = 0;
    memset(movedata, 0, sizeof(Movedata));
  }

  // Stuff to do if it's a G move code, otherwise not I guess.
  // This function MAY get called repeatedly before the execute() function.
  // Or, it MAY not get called at all.  No guarantees.
  void prepare();
  // Do some stuff and return.  This function will be called repeatedly while 
  // the state is still ACTIVE, and you can set up an interrupt for precise timings.
  void execute();
  bool isDone() { return (state == DONE); };
  void dump_to_host();
  char* movedata[sizeof(Movedata)];

private:
  CodeParam cps[T+1];                    
  unsigned long lastms;
  unsigned int preparecalls;
  unsigned int executecalls;

  void do_m_code();
  void do_g_code();
  void write_temps_to_host();

};


#endif // _MGCODE_H_
