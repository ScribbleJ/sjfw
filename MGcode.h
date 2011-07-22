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

// It would be a swell plan to make this class a polymorphic hierarchy and pop pointers to base
// in the queue, use dynamic alloc... 
// I might try it, dunno.
class MGcode
{
public:
  MGcode(): cps((CodeParam[10]){  CodeParam('X'), CodeParam('Y'), CodeParam('Z'), CodeParam('E'),
                                  CodeParam('M'), CodeParam('G'), CodeParam('F'), CodeParam('P'),
                                  CodeParam('S'), CodeParam('T') }) { reset(); };
  CodeParam& operator[](int idx) { return cps[idx]; }
  enum mg_states_t { NEW, PREPARED, ACTIVE, DONE };
  volatile mg_states_t state;
  int32_t  linenum;
  

  void reset()
  {
    for(int x=0;x<T;x++)
      cps[x].unset();
    state = NEW;
    preparecalls = 0;
    executecalls = 0;
    lastms = 0;
    linenum = -1;
    memset(&movedata, 0, sizeof(Movedata));
  }

  static void resetlastpos(const Point& lp) { lastpos = lp; }


  // Stuff to do if it's a G move code, otherwise not I guess.
  // This function MAY get called repeatedly before the execute() function.
  // Or, it MAY not get called at all.  No guarantees.
  void prepare();
  // Do some stuff and return.  This function will be called repeatedly while 
  // the state is still ACTIVE, and you can set up an interrupt for precise timings.
  void execute();
  bool isDone() { return (state == DONE); };
  void dump_to_host();
  // Called when move is completed (no good place to do it. :( )
  void wrapupmove();
  // This is separated out so that eventually we could make it a union with data required
  // for other types of gcodes, or be on a path to something more drastic, like polymorphic gcode class.
  Movedata movedata;

  // Just kept in case we need it.
  void setLinenumber(int32_t num) { linenum = num; };

private:
  CodeParam cps[T+1];                    
  unsigned long lastms;
  unsigned long startmillis; // Time execution began
  unsigned int preparecalls;
  unsigned int executecalls;
  static Point lastpos;
  static float lastfeed;

  void do_m_code();
  void do_g_code();
  void write_temps_to_host();
};


#endif // _MGCODE_H_
