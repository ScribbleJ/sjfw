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
  {
    return;
  }


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
  HOST.write("\n");
}

void MGcode::wrapupmove()
{
  if(!cps[G].isUnused())
  {
    HOST.labelnum("done ", (unsigned long)linenum, false); HOST.labelnum(" G", (unsigned long)cps[G].getInt());
  }
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
      }
      else
        state = ACTIVE;
      break;
    case 21: // Units are mm
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
      HOST.labelnum("warning ",(int)linenum, false); HOST.write(" GCODE "); HOST.write(cps[G].getInt(), 10); HOST.write(" NOT SUPPORTED\n");
      state = DONE;
      break;
  }

}

void MGcode::write_temps_to_host()
{
  HOST.write(" T:"); HOST.write(EC.getHotend(), 10); HOST.write('/'); HOST.write(EC.getHotendST(), 10);
  HOST.write(" B:"); HOST.write(EC.getPlatform(), 10);  HOST.write('/'); HOST.write(EC.getPlatformST(), 10);
}

void MGcode::do_m_code()
{
  switch(cps[M].getInt())
  {
    case 0: // Finish up and shut down.
    case 84: // "Stop Idle Hold" == shut down motors.
      MOTION.disableAllMotors();
      HOST.labelnum("done ", (unsigned long)linenum, false);
      state = DONE;
      break;
    case 104: // Set Extruder Temperature (Fast)
      if(EC.dotoolreq(SLAVE_CMD_SET_TEMP, cps[S].getInt()))
      {
        HOST.labelnum("done ", (unsigned long)linenum, false); write_temps_to_host(); HOST.write("\n");
        state = DONE;
      }
      break;
    case 105: // Get Extruder Temperature
      HOST.labelnum("done ", (unsigned long)linenum, false); write_temps_to_host(); HOST.write("\n");
      state = DONE;
      break;
    case 109: // Set Extruder Temperature
      if(state != ACTIVE && EC.dotoolreq(SLAVE_CMD_SET_TEMP, cps[S].getInt()))
        state = ACTIVE;

      if(EC.getHotend() >= cps[S].getInt())
      {
        HOST.labelnum("done ", (unsigned long)linenum, false); write_temps_to_host(); HOST.write("\n");
        state = DONE;
      }

      if(millis() - lastms > 1000)
      {
        lastms = millis();
        HOST.labelnum("done ", (unsigned long)linenum, false); write_temps_to_host(); HOST.write("\n");
      }
      break;
    case 110: // Set Current Line Number
      GCODES.setLineNumber(cps[S].isUnused() ? 0 : cps[S].getInt());
      HOST.labelnum("done ", (unsigned long)linenum, false);
      HOST.write("\n");
      state = DONE;
      break;
    case 114: // Get Current Position
      HOST.labelnum("done ", (unsigned long)linenum, false);
      MOTION.writePositionToHost();
      HOST.write("\n");
      state = DONE;
      break; 
    case 115: // Get Firmware Version and Capabilities
      HOST.labelnum("done ", (unsigned long)linenum, false);
      HOST.write(" PROTOCOL_VERSION:SJ FIRMWARE_NAME:sjfw MACHINE_TYPE:ThingOMatic EXTRUDER_COUNT:1 FREE_RAM:");
      HOST.write(getFreeRam(),10);
      HOST.write("\n");
      state = DONE;
      break;
    case 116: // Wait on temperatures
      if(EC.getHotend() >= EC.getHotendST() && EC.getPlatform() >= EC.getPlatformST())
        state = DONE;

      if(millis() - lastms > 1000)
      {
        lastms = millis();
        HOST.labelnum("done ", (unsigned long)linenum, false); write_temps_to_host(); HOST.write("\n");
      }
      break;
    case 140: // Bed Temperature (Fast) 
      if(EC.dotoolreq(SLAVE_CMD_SET_PLATFORM_TEMP, cps[S].getInt()))
      {
        HOST.labelnum("done ", (unsigned long)linenum, false); write_temps_to_host(); HOST.write("\n");
        state = DONE;
      }
      break;
    case 200: // NOT STANDARD - set Steps Per Unit
      MOTION.setStepsPerUnit(*this);
      HOST.labelnum("done ", (unsigned long)linenum, false);
      state = DONE;
      break;
    case 201: // NOT STANDARD - set Feedrates
      MOTION.setMinimumFeedrate(*this);
      HOST.labelnum("done ", (unsigned long)linenum, false);
      state = DONE;
      break;
    case 202: // NOT STANDARD - set Feedrates
      MOTION.setMaximumFeedrate(*this);
      HOST.labelnum("done ", (unsigned long)linenum, false);
      state = DONE;
      break;
    case 203: // NOT STANDARD - set Feedrates
      MOTION.setAverageFeedrate(*this);
      HOST.labelnum("done ", (unsigned long)linenum, false);
      state = DONE;
      break;
    case 204: // NOT STANDARD - disable motors
      MOTION.disableAllMotors();
      HOST.labelnum("done ", (unsigned long)linenum, false);
      state = DONE;
      break;
    default:
      HOST.labelnum("done ", (unsigned long)linenum, false);
      HOST.write(" MCODE "); HOST.write(cps[M].getInt(), 10); HOST.write(" NOT SUPPORTED\n");
      state = DONE;
      break;
  }
}



