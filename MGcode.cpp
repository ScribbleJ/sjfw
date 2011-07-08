#include "MGcode.h"
#include "Gcodes.h"
#include "Time.h"
#include "Globals.h"
#include "Motion.h"

Point MGcode::lastpos;
float MGcode::lastfeed;
// Stuff to do if it's a G move code, otherwise not I guess.
// This function MAY get called repeatedly before the execute() function.
// It WILL get called at least once.
void MGcode::prepare()
{
  preparecalls++;
  if(state == PREPARED)
    return;

  if(cps[G].isUnused())
    state = PREPARED;
  else
    MOTION.gcode_precalc(*this,lastfeed,lastpos);
}

// Do some stuff and return.  This function will be called repeatedly while 
// the state is still ACTIVE, and you can set up an interrupt for precise timings.
void MGcode::execute()
{
  if(startmillis == 0)
    startmillis = millis();

  executecalls++;
  if(state < PREPARED)
  {
    prepare();
    return;
  }
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
}

void MGcode::dump_to_host()
{
  HOST.write('P'); HOST.write(preparecalls, 10);
  HOST.write('E'); HOST.write(executecalls, 10);
  HOST.write(' ');
  for(int x=0;x<T;x++)
    cps[x].dump_to_host();
  HOST.write("\r\n");
}


void MGcode::do_g_code() 
{
  switch(cps[G].getInt())
  {
    case 0:
    case 1: // Linear Move
      MOTION.gcode_execute(*this);
      break;
    case 4: // Pause for P millis
      if(millis() - cps[P].getInt() > startmillis)
      {
        state = DONE;
        HOST.write("done 0\r\n");
      }
      else
        state = ACTIVE;
      break;
    case 21: // Units are mm
      HOST.write("done 0\r\n"); // do nothing
      state = DONE;
      break;
    case 90: // Absolute positioning
      MOTION.setAbsolute();
      state = DONE;
      break;
    case 91: // Relative positioning
      MOTION.setRelative();
      state = DONE;
      break;
    case 92: // Set position
      MOTION.setCurrentPosition(*this);
      state = DONE;
      break;
    default:
      HOST.write("done 0 GCODE "); HOST.write(cps[G].getInt(), 10); HOST.write(" NOT SUPPORTED\r\n");
      state = DONE;
      break;
  }
}

void MGcode::write_temps_to_host()
{
  HOST.write("T:"); HOST.write(EC.getHotend(), 10); HOST.write('/'); HOST.write(EC.getHotendST(), 10);
  HOST.write(" B:"); HOST.write(EC.getPlatform(), 10);  HOST.write('/'); HOST.write(EC.getPlatformST(), 10);
}

void MGcode::do_m_code()
{
  switch(cps[M].getInt())
  {
    case 0: // Finish up and shut down.
      state = DONE;
      break;
    case 84: // "Stop Idle Hold" == shut down motors.
      state = DONE;
      break;

    case 104: // Set Extruder Temperature (Fast)
      if(EC.dotoolreq(SLAVE_CMD_SET_TEMP, cps[S].getInt()))
      {
        HOST.write("done 0 "); write_temps_to_host(); HOST.write("\r\n");
        state = DONE;
      }
      break;
    case 105: // Get Extruder Temperature
      HOST.write("done 0 "); write_temps_to_host(); HOST.write("\r\n");
      state = DONE;
      break;
    case 109: // Set Extruder Temperature
      if(state != ACTIVE && EC.dotoolreq(SLAVE_CMD_SET_TEMP, cps[S].getInt()))
        state = ACTIVE;

      if(EC.getHotend() >= cps[S].getInt())
        state = DONE;

      if(millis() - lastms > 1000)
      {
        lastms = millis();
        HOST.write("done 0 "); write_temps_to_host(); HOST.write("\r\n");
      }
      break;
    case 110: // Set Current Line Number
      GCODES.setLineNumber(cps[S].isUnused() ? 0 : cps[S].getInt());
      state = DONE;
      break;
    case 114: // Get Current Position
      HOST.write("done 0 ");
      MOTION.writePositionToHost();
      HOST.write("\r\n");
      state = DONE;
      break; // done C: X:0.00 Y:0.00 Z:0.00 E:0.00
    case 115: // Get Firmware Version and Capabilities
      HOST.write("done 0 PROTOCOL_VERSION:SJ FIRMWARE_NAME:sjfw MACHINE_TYPE:ThingOMatic EXTRUDER_COUNT:1 FREE_RAM:");
      HOST.write(getFreeRam(),10);
      HOST.write("\r\n");
      state = DONE;
      break;
    case 116: // Wait on temperatures
      if(EC.getHotend() >= EC.getHotendST() && EC.getPlatform() >= EC.getPlatformST())
        state = DONE;

      if(millis() - lastms > 1000)
      {
        lastms = millis();
        HOST.write("done 0 "); write_temps_to_host(); HOST.write("\r\n");
      }
      break;
    case 140: // Bed Temperature (Fast) 
      if(EC.dotoolreq(SLAVE_CMD_SET_PLATFORM_TEMP, cps[S].getInt()))
      {
        HOST.write("done 0 "); write_temps_to_host(); HOST.write("\r\n");
        state = DONE;
      }
      break;
    default:
      HOST.write("done 0 MCODE "); HOST.write(cps[M].getInt(), 10); HOST.write(" NOT SUPPORTED\r\n");
      state = DONE;
      break;
  }
}



