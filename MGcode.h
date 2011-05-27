#ifndef _MGCODE_H_
#define _MGCODE_H_

// ato? functions.
#include <stdlib.h>
#include "Host.h"

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
  

  void reset()
  {
    for(int x=0;x<T;x++)
      cps[x].unset();
    state = NEW;
    preparecalls = 0;
    executecalls = 0;
  }

  // Stuff to do if it's a G move code, otherwise not I guess.
  // This function MAY get called repeatedly before the execute() function.
  // Or, it MAY not get called at all.  No guarantees.
  void prepare()
  {
    preparecalls++;
    if(state == PREPARED)
      return;

    state = PREPARED;
  }

  // Do some stuff and return.  This function will be called repeatedly while 
  // the state is still ACTIVE, and you can set up an interrupt for precise timings.
  void execute()
  {
    executecalls++;
    if(state == DONE)
      return;

    if(cps[G].isUnused())
    {
      do_m_code();
    }
    else
    {
      do_g_code();
    }

    dump_to_host();

    state = DONE;
  }

  bool isDone() { return (state == DONE); };

  void dump_to_host()
  {
    HOST.write('P'); HOST.write(preparecalls, 10);
    HOST.write('E'); HOST.write(executecalls, 10);
    HOST.write(' ');
    for(int x=0;x<T;x++)
      cps[x].dump_to_host();
    HOST.write("\r\n");
  }

private:
  CodeParam cps[T+1];                    
  volatile mg_states_t state;
  unsigned int preparecalls;
  unsigned int executecalls;

  void do_g_code() 
  {
    switch(cps[G].getInt())
    {
      default:
        HOST.write("ok 0 GCODE "); HOST.write(cps[G].getInt(), 10); HOST.write(" NOT SUPPORTED\r\n");
        break;
    }
  }

  void do_m_code()
  {
    switch(cps[M].getInt())
    {
      case 115:
        HOST.write("ok 0 DEBUG MESSAGE\r\n");
        break;
      default:
        HOST.write("ok 0 MCODE "); HOST.write(cps[M].getInt(), 10); HOST.write(" NOT SUPPORTED\r\n");
        break;
    }
  }
};


#endif // _MGCODE_H_
