#include "MGcode.h"
#include "Gcodes.h"
#include "Time.h"
#include "Globals.h"

// Stuff to do if it's a G move code, otherwise not I guess.
// This function MAY get called repeatedly before the execute() function.
// Or, it MAY not get called at all.  No guarantees.
void MGcode::prepare()
{
  preparecalls++;
  if(state == PREPARED)
    return;

  state = PREPARED;
}

// Do some stuff and return.  This function will be called repeatedly while 
// the state is still ACTIVE, and you can set up an interrupt for precise timings.
void MGcode::execute()
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
    default:
      HOST.write("ok 0 GCODE "); HOST.write(cps[G].getInt(), 10); HOST.write(" NOT SUPPORTED\r\n");
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
    case 104: // Set Extruder Temperature (Fast)
      if(EC.dotoolreq(SLAVE_CMD_SET_TEMP, cps[S].getInt()))
      {
        HOST.write("ok 0 "); write_temps_to_host(); HOST.write("\r\n");
        state = DONE;
      }
      break;
    case 105: // Get Extruder Temperature
      HOST.write("ok 0 "); write_temps_to_host(); HOST.write("\r\n");
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
        HOST.write("ok 0 "); write_temps_to_host(); HOST.write("\r\n");
      }
      break;
    case 110: // Set Current Line Number
      GCODES.setLineNumber(cps[S].getInt());
      state = DONE;
      break;
    case 115: // Get Firmware Version and Capabilities
      HOST.write("ok 0 PROTOCOL_VERSION:SJ FIRMWARE_NAME:sjfw MACHINE_TYPE:ThingOMatic EXTRUDER_COUNT:1 FREE_RAM:");
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
        HOST.write("ok 0 "); write_temps_to_host(); HOST.write("\r\n");
      }
      break;
    case 140: // Bed Temperature (Fast) 
      if(EC.dotoolreq(SLAVE_CMD_SET_PLATFORM_TEMP, cps[S].getInt()))
      {
        HOST.write("ok 0 "); write_temps_to_host(); HOST.write("\r\n");
        state = DONE;
      }
      break;
    default:
      HOST.write("ok 0 MCODE "); HOST.write(cps[M].getInt(), 10); HOST.write(" NOT SUPPORTED\r\n");
      state = DONE;
      break;
  }
}



