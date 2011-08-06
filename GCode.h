#ifndef _MGCODE_H_
#define _MGCODE_H_
/* GCode handling class.  
 * (c) 2011 Christopher "ScribbleJ" Jansen
 *
 * This class is responsible for holding and executing and preparsing a "Gcode" (G or M really)
 * If you're wondering where in the source to look for something Gcode related and it's not here,
 * check GcodeQueue, which manages a list of these.
 */

// ato? functions.
#include <stdlib.h>
#include "Host.h"
#include <string.h>
#include "Point.h"
#include "Time.h"

// CodeParameter; basically a fancy union/variable datatype.
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
  long& getInt() { return u; }
  bool isUnused() { return state == UNUSED; }

  void dump_to_host() 
  { 
    HOST.write(letter); HOST.write(':');
    switch(state)
    {
      case UNUSED:
        HOST.write("U");
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
    long u;
  };
};

// It would be a swell plan to make this class a polymorphic hierarchy and pop pointers to base
// in the queue, use dynamic alloc... 
// I might try it, dunno.
class GCode
{
public:
  GCode(): cps((CodeParam[10]){  CodeParam('X'), CodeParam('Y'), CodeParam('Z'), CodeParam('E'),
                                  CodeParam('M'), CodeParam('G'), CodeParam('F'), CodeParam('P'),
                                  CodeParam('S'), CodeParam('T') }) { reset(); };
  CodeParam& operator[](int idx) { return cps[idx]; }
  enum mg_states_t { NEW, PREPARED, ACTIVE, DONE };
  volatile mg_states_t state;
  int32_t  linenum;
  

  void reset()
  {
    for(int x=0;x<T;x++)
    {
      cps[x].unset();
      if(x<=E)
      {
        axismovesteps[x] = 0;
        axisdirs[x] = false;
      }

    }
    
    state = NEW;
    preparecalls = 0;
    executecalls = 0;
    lastms = 0;
    linenum = -1;
    feed=0;
    movesteps=0;
    leading_axis=0;

    startinterval=0;
    currentinterval=0;
    fullinterval=0;
    steps_to_accel=0;
    accel_until=0;
    decel_from=0;
    accel_inc=0;
    steps_acceled=0;
    accel_remainder=0;

  }

  static void resetlastpos(Point& lp);

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

  void setLinenumber(int32_t num) { linenum = num; };

  void dump_movedata()
  {
    HOST.labelnum("L:",linenum,false);
    HOST.labelnum(" T:",(millis() - startmillis),false);
    HOST.labelnum(" F:",feed,false);
    HOST.labelnum(" S:",movesteps,false);
    HOST.labelnum(" LA:",leading_axis,false);
    HOST.labelnum(" SI:",startinterval,false);
    HOST.labelnum(" CI:",currentinterval,false);
    HOST.labelnum(" FI:",fullinterval,false);
    HOST.labelnum(" STA:",steps_to_accel,false);
    HOST.labelnum(" ST:",steps_acceled,false);
    HOST.labelnum(" AU:",accel_until,false);
    HOST.labelnum(" DF:",decel_from,false);
    HOST.labelnum(" AI:",accel_inc,true);
  }

  static Point& getLastpos() { return lastpos; }

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

public:
  float feed;

  // This was separated out, maybe will be again...
  // preparsed data specific to G1 type moves.
  uint32_t movesteps;
  uint32_t axismovesteps[NUM_AXES];
  bool          axisdirs[NUM_AXES];
  int           leading_axis;

  uint32_t startinterval;
  uint32_t currentinterval;
  uint32_t fullinterval;
  uint32_t steps_to_accel;
  uint32_t accel_until;
  uint32_t decel_from;
  uint32_t accel_inc;
  uint32_t steps_acceled;
  uint32_t accel_remainder;
  Point         endpos;
  Point         startpos;

};


#endif // _MGCODE_H_
