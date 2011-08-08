#include "GCode.h"
#include "GcodeQueue.h"
#include "Time.h"
#include "Globals.h"
#include "Motion.h"
#include "Temperature.h"
#include "SDCard.h"

Point GCode::lastpos;
float GCode::lastfeed;
// Stuff to do if it's a G move code, otherwise not I guess.
// This function MAY get called repeatedly before the execute() function.
// It WILL get called at least once.
void GCode::prepare()
{
  preparecalls++;
  if(state == PREPARED)
    return;

  if(cps[G].isUnused())
    state = PREPARED;
  else
    MOTION.gcode_precalc(*this,lastfeed,&lastpos);
}

// Do some stuff and return.  This function will be called repeatedly while 
// the state is still ACTIVE, and you can set up an interrupt for precise timings.
void GCode::execute()
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

void GCode::dump_to_host()
{
  HOST.write('P'); HOST.write(preparecalls, 10);
  HOST.write('E'); HOST.write(executecalls, 10);
  HOST.write(' ');
  for(int x=0;x<T;x++)
    cps[x].dump_to_host();
  HOST.endl();
}

void GCode::wrapupmove()
{
  if(!cps[G].isUnused())
  {
    //dump_movedata();
#ifndef REPRAP_COMPAT    
    Host::Instance(source).labelnum("done ", linenum, false); Host::Instance(source).labelnum(" G", cps[G].getInt());
#endif    
    MOTION.wrapup(*this);
  }
}



void GCode::do_g_code() 
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
      Host::Instance(source).labelnum("warn ",linenum, false); 
      Host::Instance(source).write(" GCODE "); 
      Host::Instance(source).write(cps[G].getInt(), 10); 
      Host::Instance(source).write(" NOT SUPPORTED\n");
      state = DONE;
      break;
  }

}

void GCode::write_temps_to_host(int port)
{
  Host::Instance(port).labelnum("T:",TEMPERATURE.getHotend(),false); 
  Host::Instance(port).labelnum(" B:", TEMPERATURE.getPlatform(),false);  
#ifdef REPG_COMPAT
  Host::Instance(port).write('\n');
#endif
  Host::Instance(port).labelnum(" / SH:", TEMPERATURE.getHotendST(),false);
  Host::Instance(port).labelnum(" SP:", TEMPERATURE.getPlatformST());
}

void GCode::do_m_code()
{
  switch(cps[M].getInt())
  {
    case 0: // Finish up and shut down.
    case 18: // 18 used by RepG.
    case 84: // "Stop Idle Hold" == shut down motors.
      MOTION.disableAllMotors();
      state = DONE;
      break;
    case 104: // Set Extruder Temperature (Fast)
      if(TEMPERATURE.setHotend(cps[S].getInt()))
      {
#ifndef REPG_COMPAT        
        Host::Instance(source).labelnum("prog ", linenum, false); Host::Instance(source).write(' ');  write_temps_to_host(source);
#else
        write_temps_to_host(source);
#endif        
        state = DONE;
      }
      break;
    case 105: // Get Extruder Temperature
#ifndef REPG_COMPAT        
        Host::Instance(source).labelnum("prog ", linenum, false); Host::Instance(source).write(' ');  write_temps_to_host(source); 
#else
        write_temps_to_host(source);
#endif        
        state = DONE;

      state = DONE;
      break;
    case 109: // Set Extruder Temperature
      if(state != ACTIVE && TEMPERATURE.setHotend(cps[S].getInt()))
        state = ACTIVE;

      if(TEMPERATURE.getHotend() >= cps[S].getInt())
      {
#ifndef REPG_COMPAT        
        Host::Instance(source).labelnum("prog ", linenum, false); Host::Instance(source).write(' ');  write_temps_to_host(source);
#else
        write_temps_to_host(source);
#endif        
        state = DONE;
      }

      if(millis() - lastms > 1000)
      {
        lastms = millis();
#ifndef REPG_COMPAT        
        Host::Instance(source).labelnum("prog ", linenum, false); Host::Instance(source).write(' ');  write_temps_to_host(source); 
#else
        write_temps_to_host(source);
#endif        
      }
      break;
    case 110: // Set Current Line Number
      GCODES.setLineNumber(cps[S].isUnused() ? 1 : cps[S].getInt());
      state = DONE;
      break;
    case 114: // Get Current Position
#ifndef REPG_COMPAT    
      Host::Instance(source).labelnum("prog ", linenum, false);
      Host::Instance(source).write(' ');
#endif      
      Host::Instance(source).write("C: ");
      MOTION.writePositionToHost();
      Host::Instance(source).endl();
      state = DONE;
      break; 
    case 115: // Get Firmware Version and Capabilities
      Host::Instance(source).labelnum("prog ", linenum, false);
      Host::Instance(source).write(" PROTOCOL_VERSION:SJ FIRMWARE_NAME:sjfw FREE_RAM:");
      Host::Instance(source).write(getFreeRam(),10);
      Host::Instance(source).endl();
      state = DONE;
      break;
    case 116: // Wait on temperatures
      if(TEMPERATURE.getHotend() >= TEMPERATURE.getHotendST() && TEMPERATURE.getPlatform() >= TEMPERATURE.getPlatformST())
        state = DONE;

      if(millis() - lastms > 1000)
      {
        lastms = millis();
#ifndef REPG_COMPAT        
        Host::Instance(source).labelnum("prog ", linenum, false); Host::Instance(source).write(' ');  write_temps_to_host(source); 
#else
        write_temps_to_host(source);
#endif        
      }
      break;
    case 140: // Bed Temperature (Fast) 
      if(TEMPERATURE.setPlatform(cps[S].getInt()))
      {
#ifndef REPG_COMPAT        
        Host::Instance(source).labelnum("prog ", linenum, false); Host::Instance(source).write(' ');  write_temps_to_host(source); 
#else
        write_temps_to_host(source);
#endif        
        state = DONE;
      }
      break;
    case 200: // NOT STANDARD - set Steps Per Unit
      MOTION.setStepsPerUnit(*this);
      state = DONE;
      break;
    case 201: // NOT STANDARD - set Feedrates
      MOTION.setMinimumFeedrate(*this);
      state = DONE;
      break;
    case 202: // NOT STANDARD - set Feedrates
      MOTION.setMaximumFeedrate(*this);
      state = DONE;
      break;
    case 203: // NOT STANDARD - set Feedrates
      MOTION.setAverageFeedrate(*this);
      state = DONE;
      break;
#ifdef HAS_SD
    case 204: // NOT STANDARD - get next filename from SD
      Host::Instance(source).labelnum("prog ", linenum, false);
      Host::Instance(source).write(' ');
      Host::Instance(source).write(sdcard::getNextfile());
      Host::Instance(source).endl();
      state = DONE;
      break;
    case 205: // NOT STANDARD - print file from last 204
      Host::Instance(source).labelnum("prog ", linenum, false);
      Host::Instance(source).write(sdcard::getCurrentfile());
      if(sdcard::printcurrent())
        Host::Instance(source).write(" BEGUN");
      else
        Host::Instance(source).write(" FAIL");
     
      Host::Instance(source).endl();
      state = DONE;
      break;
#endif
    case 206: // NOT STANDARD - set accel rate
      MOTION.setAccel(*this);
      state = DONE;
      break;
    case 207: // NOT STANDARD - set hotend thermistor pin
      TEMPERATURE.changePinHotend(cps[P].getInt());
      state = DONE;
      break;
    case 208: // NOT STANDARD - set platform thermistor pin
      TEMPERATURE.changePinPlatform(cps[P].getInt());
      state = DONE;
      break;

    default:
      Host::Instance(source).labelnum("warn ", linenum, false);
      Host::Instance(source).write(" MCODE "); Host::Instance(source).write(cps[M].getInt(), 10); Host::Instance(source).write(" NOT SUPPORTED\n");
      state = DONE;
      break;
  }
#ifndef REPRAP_COMPAT  
  if(state == DONE)
  {
    if(linenum != -1)
      Host::Instance(source).labelnum("done ", linenum, false);
    else
      Host::Instance(source).write("done ");

    Host::Instance(source).labelnum(" M", cps[M].getInt(), true);
  }
#endif
}

void GCode::resetlastpos(Point& lp) 
{ 
  lastpos = lp; 
}





