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
#include "AvrPort.h"


// CodeParameter; basically a fancy union/variable datatype.
class CodeParam
{
public:
  enum states_T { UNUSED, FLOAT, INT } state;
  CodeParam(const char name) :state(UNUSED), letter(name) { };

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
        HOST.write('U');
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
#ifndef USE_MARLIN      
      if(x<=E)
      {
        axismovesteps[x] = 0;
        axisdirs[x] = false;
      }
#endif
    }
    
    state = NEW;
    preparecalls = 0;
    executecalls = 0;
    lastms = 0;
    linenum = -1;
    feed=0;

#ifndef USE_MARLIN      
    movesteps=0;
    leading_axis=0;

    currentinterval=0;
    accel_until=0;
    decel_from=0;
    accel_inc=0;
#endif // USE_MARLIN      
  }

  static void resetlastpos();

  // Stuff to do if it's a G move code, otherwise not I guess.
  // This function MAY get called repeatedly before the execute() function.
  // it WILL be called at least once.
  void prepare();
  // This function optimizes chained moves to only slow down as required.
  // It is only performed when the gcodes are already prepared.
  void optimize(GCode& next);
  // Do some stuff and return.  This function will be called repeatedly while 
  // the state is still ACTIVE, and you can set up an interrupt for precise timings.
  void execute();
  bool isDone() { return (state == DONE); };
  void dump_to_host();
  // Called when move is completed (no good place to do it. :( )
  void wrapupmove();

  void setLinenumber(int32_t num) { linenum = num; };

#ifndef USE_MARLIN
  void dump_movedata()
  {
#ifdef DEBUG_MOVE  
    HOST.labelnum("L:",linenum,false);
    HOST.labelnum(" T:",(millis() - startmillis),false);
    HOST.labelnum(" F:",feed,false);
    HOST.labelnum(" Steps:",axismovesteps[leading_axis],false);
    HOST.labelnum(" Lead:",leading_axis,false);
    HOST.labelnum(" CurI:",currentinterval,false);
    HOST.labelnum(" CurF:",currentfeed,false);
    HOST.labelnum(" StartF:",startfeed,false);
    HOST.labelnum(" MaxF:",maxfeed,false);
    HOST.labelnum(" EndF:",endfeed,false);
    HOST.labelnum(" AUnt:",accel_until,false);
    HOST.labelnum(" DFrom:",decel_from,false);
    HOST.labelnum(" AInc:",accel_inc,true);
#endif    
  }
#endif

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

  // TODO: this class is the WRONG PLACE for these functions
  void write_temps_to_host(int port);
  // set arbitrary arduino pin
  void doPinSet(int arduinopin, int on);

public:
  // TODO: this should be ELSEWHERE.
  static Pin fanpin;
  static bool DONTRUNEXTRUDER;

  // all good
  float feed;
  int source;

#ifndef USE_MARLIN
  // This was separated out, maybe will be again...
  // preparsed data specific to G1 type moves.
  uint32_t movesteps;
  uint32_t axismovesteps[NUM_AXES];
  bool     axisdirs[NUM_AXES];
  int      leading_axis;
  bool     optimized;

  float actualmm;
  float axisratio[NUM_AXES];
  
  uint32_t currentinterval;
  uint32_t startfeed;
  uint32_t maxfeed;
  uint32_t endfeed;
  uint32_t currentfeed;
  uint32_t minfeed;

  uint32_t accel_until;
  uint32_t decel_from;
  float    accel;
  uint32_t accel_inc;
  uint32_t accel_timer;

  Point    endpos;
  Point    startpos;
#endif  

};


#endif // _MGCODE_H_
