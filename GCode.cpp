#include "GCode.h"
#include "GcodeQueue.h"
#include "Time.h"
#include "Globals.h"
#include "Temperature.h"
#include "SDCard.h"
#include "Eeprom.h"
#include <avr/pgmspace.h>
#include "ArduinoMap.h"
#ifndef USE_MARLIN
#include "Motion.h"
#else
#include "Marlin.h"
#endif

#ifdef HAS_LCD
#include "LCDKeypad.h"
extern LCDKeypad LCDKEYPAD;
#endif

#ifdef USE_MARLIN
#define SETOBJ(x) Marlin::x
#else
#define SETOBJ(x) MOTION.x
#endif

Point GCode::lastpos;
float GCode::lastfeed;
Pin   GCode::fanpin = Pin();
bool  GCode::DONTRUNEXTRUDER=false;
bool  GCode::ISRELATIVE=false;

// Stuff to do if it's a G move code, otherwise not I guess.
// This function MAY get called repeatedly before the execute() function.
// It WILL get called at least once.
void GCode::prepare()
{
#ifdef USE_MARLIN
	// Marlin might be able to prepare in advance of being asked to execute the codes with some help.
	state = PREPARED;
	return;
#else

	if(state == PREPARED)
		return;

	if(DONTRUNEXTRUDER)
	{
		cps[E].unset();
	}

	// Only GCodes need preparing.
	if(cps[G].isUnused())
		state = PREPARED;
	else
		MOTION.gcode_precalc(*this,lastfeed,&lastpos);
#endif
}

// Called when we have extra time to optimize a gcode.
void GCode::optimize(GCode& next)
{
#ifndef USE_MARLIN
	if(state != PREPARED)
		return;

	if(next.state != PREPARED)
		return;

	MOTION.gcode_optimize(*this, next);
#endif
}

// Called at the time of enqueue
void GCode::enqueue()
{
	if(!cps[G].isUnused())
	{
		switch(cps[G].getInt())
		{
			case 0:
			case 1:
			case 2:
				for(int ax=0;ax<NUM_AXES;ax++)
				{
					if(!cps[ax].isUnused())
					{
						if(ISRELATIVE)
						{
							float foo = lastpos[ax] + cps[ax].getFloat();
							cps[ax].setFloat(foo);
						}
					}
				}
				break;
			case 90:  // Set Absolute Positioning
				ISRELATIVE = false;
				break;
			case 91:  // Set Relative Positioning
				ISRELATIVE = true;
				break;
		}
	}
	prepare(); // Need to prepare at this point so that we have the correct lastpos
}

// Do some stuff and return.  This function will be called repeatedly while
// the state is still ACTIVE, and you can set up an interrupt for precise timings.
void GCode::execute()
{
	if(startmillis == 0)
		startmillis = millis();

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
	HOST.labelnum("L:", linenum);
	for(int x=0;x<T;x++)
		cps[x].dump_to_host();
	HOST.endl();
}

void GCode::wrapupmove()
{
#ifndef USE_MARLIN
	if(!cps[G].isUnused())
	{
#ifdef DEBUG_MOVE
		dump_movedata();
#endif
#ifndef REPRAP_COMPAT
		Host::Instance(source).labelnum("done ", linenum, false); Host::Instance(source).labelnum(" G", cps[G].getInt());
#endif
		MOTION.wrapup(*this);
	}
#endif
}


#define SKIPEXTRUDE if(DONTRUNEXTRUDER) { state = DONE; return; }
void GCode::do_g_code()
{
#ifdef USE_MARLIN
// Marlin's only method of synchronizing these sorts of commands is to wait 'till it's cleared out.
	if(cps[G].getInt() != 1 && !Marlin::isBufferEmpty())
		return;
#endif


	switch(cps[G].getInt())
	{
		case 0:
		case 1: // Linear Move
#ifdef USE_MARLIN
			if(Marlin::add_buffer_line(*this))
				state = DONE;
#else
			MOTION.gcode_execute(*this);
#endif
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
			// We shouldn't arrive here as it's handled during enqueue();
			state = DONE;
			break;
		case 91: // Relative positioning
			// We shouldn't arrive here as it's handled during enqueue();
			state = DONE;
			break;
		case 92: // Set position
			SETOBJ(setCurrentPosition(*this));
			state = DONE;
			break;
		default:
			Host::Instance(source).labelnum("warn ",linenum, false);
			Host::Instance(source).write_P(PSTR(" GCODE "));
			Host::Instance(source).write(cps[G].getInt(), 10);
			Host::Instance(source).write_P(PSTR(" NOT SUPPORTED\n"));
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
#else
	Host::Instance(port).labelnum(" / SH:", TEMPERATURE.getHotendST(),false);
	Host::Instance(port).labelnum(" SP:", TEMPERATURE.getPlatformST());
#endif
}

void GCode::do_m_code()
{
#ifdef USE_MARLIN
// Marlin's only method of synchronizing these sorts of commands is to wait 'till it's cleared out.
	if(!Marlin::isBufferEmpty())
		return;
#endif

	switch(cps[M].getInt())
	{
		case 0: // Finish up and shut down.
		case 18: // 18 used by RepG.
		case 84: // "Stop Idle Hold" == shut down motors.
			SETOBJ(disableAllMotors());
			state = DONE;
			break;
		case 104: // Set Extruder Temperature (Fast)
			SKIPEXTRUDE;
			if(cps[S].isUnused() || TEMPERATURE.setHotend(cps[S].getInt()))
			{
#ifndef REPG_COMPAT
				Host::Instance(source).labelnum("prog ", linenum, false); Host::Instance(source).write(' ');  write_temps_to_host(source);
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
#ifndef USE_MBIEC
		case 106: // Fan on
			if(!fanpin.isNull())
			{
				fanpin.setValue(true);
			}
			state = DONE;
			break;
		case 107: // Fan off
			if(!fanpin.isNull())
			{
				fanpin.setValue(false);
			}
			state = DONE;
			break;
#else
		case 106: // Fan on
			if(TEMPERATURE.setFan(true))
				state = DONE;
			break;
		case 107: // Fan off
			if(TEMPERATURE.setFan(false))
				state = DONE;
			break;
#endif
		case 109: // Set Extruder Temperature
			SKIPEXTRUDE;
			if(state != ACTIVE && (cps[S].isUnused() || TEMPERATURE.setHotend(cps[S].getInt())))
				state = ACTIVE;

			if(state == ACTIVE && TEMPERATURE.getHotend() >= TEMPERATURE.getHotendST())
			{
#ifndef REPG_COMPAT
				Host::Instance(source).labelnum("prog ", linenum, false); Host::Instance(source).write(' ');  write_temps_to_host(source);
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
			SETOBJ(writePositionToHost(*this));
			// TODO
			Host::Instance(source).endl();
			state = DONE;
			break;
		case 115: // Get Firmware Version and Capabilities
			Host::Instance(source).labelnum("prog ", linenum, false);
			Host::Instance(source).write_P(PSTR(" VERSION:" SJFW_VERSION " FREE_RAM:"));
			Host::Instance(source).write(getFreeRam(),10);
			Host::Instance(source).write_P(PSTR(" FEATURES:0/crc-v2"));
			Host::Instance(source).endl();
			state = DONE;
			break;
		case 116: // Wait on temperatures
			SKIPEXTRUDE;
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
		case 118: // Choose optional features TODO: implement properly when we have > 1 option
			if(!cps[P].isUnused())
			{
				if(cps[P].getInt() == 1)
					GCODES.enableADVANCED_CRC(source);
				else
					GCODES.disableADVANCED_CRC(source);
			}
		case 140: // Bed Temperature (Fast)
			SKIPEXTRUDE;
			if(cps[S].isUnused() || TEMPERATURE.setPlatform(cps[S].getInt()))
			{
#ifndef REPG_COMPAT
				Host::Instance(source).labelnum("prog ", linenum, false); Host::Instance(source).write(' ');  write_temps_to_host(source);
#endif
				state = DONE;
			}
			break;
		case 200: // NOT STANDARD - set Steps Per Unit
			SETOBJ(setStepsPerUnit(*this));
			state = DONE;
			break;
		case 201: // NOT STANDARD - set Feedrates
			SETOBJ(setMinimumFeedrate(*this));
			state = DONE;
			break;
		case 202: // NOT STANDARD - set Feedrates
			SETOBJ(setMaximumFeedrate(*this));
			state = DONE;
			break;
		case 203: // NOT STANDARD - set Feedrates
			SETOBJ(setAverageFeedrate(*this));
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
				Host::Instance(source).write_P(PSTR(" BEGUN"));
			else
				Host::Instance(source).write_P(PSTR(" FAIL"));

			Host::Instance(source).endl();
			state = DONE;
			break;
#endif
		case 206: // NOT STANDARD - set accel rate
			SETOBJ(setAccel(*this));
			state = DONE;
			break;
		case 207: // NOT STANDARD - set hotend thermistor pin
			if(!cps[P].isUnused())
				TEMPERATURE.changePinHotend(cps[P].getInt());
			if(!cps[S].isUnused())
				TEMPERATURE.changeOutputPinHotend(cps[S].getInt());
			state = DONE;
			break;
		case 208: // NOT STANDARD - set platform thermistor pin
			if(!cps[P].isUnused())
				TEMPERATURE.changePinPlatform(cps[P].getInt());
			if(!cps[S].isUnused())
				TEMPERATURE.changeOutputPinPlatform(cps[S].getInt());
			state = DONE;
			break;
		case 211: // NOT STANDARD - request temp reports at regular interval
			if(cps[P].isUnused())
				TEMPERATURE.changeReporting(0, Host::Instance(source));
			else
				TEMPERATURE.changeReporting(cps[P].getInt(), Host::Instance(source));
			state = DONE;
			break;
		case 215: // NOT STANDARD - set arbitrary digital pin
			if(!cps[P].isUnused() && !cps[S].isUnused())
			{
				doPinSet(cps[P].getInt(), cps[S].getInt());
			}
			state = DONE;
			break;
		case 216: // NOT STANDARD - set fan pin
			if(!cps[P].isUnused())
			{
				fanpin = Pin(ArduinoMap::getPort(cps[P].getInt()),  ArduinoMap::getPinnum(cps[P].getInt()));
				fanpin.setDirection(true);
				fanpin.setValue(false);
			}
			state = DONE;
			break;
		case 220: // NOT STANDARD - set endstop minimum positions
			SETOBJ(setMinStopPos(*this));
			state = DONE;
			break;
		case 221: // NOT STANDARD - set endstop minimum positions
			SETOBJ(setMaxStopPos(*this));
			state = DONE;
			break;
#ifdef HAS_LCD
		case 250: // NOT STANDARD - LCD RS pin
			if(!cps[P].isUnused())
				LCDKEYPAD.setRS(cps[P].getInt());
			state = DONE;
			break;
		case 251: // NOT STANDARD - LCD RW pin
			if(!cps[P].isUnused())
				LCDKEYPAD.setRW(cps[P].getInt());
			state = DONE;
			break;
		case 252: // NOT STANDARD - LCD E pin
			if(!cps[P].isUnused())
				LCDKEYPAD.setE(cps[P].getInt());
			state = DONE;
			break;
		case 253: // NOT STANDARD - LCD D pins
			if(!cps[P].isUnused() && !cps[S].isUnused())
				LCDKEYPAD.setD(cps[S].getInt(), cps[P].getInt());
			state = DONE;
			break;
		case 254: // NOT STANDARD - Keypad Row Pins
			if(!cps[P].isUnused() && !cps[S].isUnused())
				LCDKEYPAD.setRowPin(cps[S].getInt(), cps[P].getInt());
			state = DONE;
			break;
		case 255: // NOT STANDARD - Keypad Col Pins
			if(!cps[P].isUnused() && !cps[S].isUnused())
				LCDKEYPAD.setColPin(cps[S].getInt(), cps[P].getInt());
			state = DONE;
			break;
		case 256: // NOT STANDARD - reinit LCD.  This is a temporary 'fix' for the init timing errors
			LCDKEYPAD.reinit();
			state = DONE;
			break;
#endif
		case 300: // NOT STANDARD - set axis STEP pin
			SETOBJ(setStepPins(*this));
			state = DONE;
			break;
		case 301: // NOT STANDARD - set axis DIR pin
			SETOBJ(setDirPins(*this));
			state = DONE;
			break;
		case 302: // NOT STANDARD - set axis ENABLE pin
			SETOBJ(setEnablePins(*this));
			state = DONE;
			break;
		case 304: // NOT STANDARD - set axis MIN pin
			SETOBJ(setMinPins(*this));
			state = DONE;
			break;
		case 305: // NOT STANDARD - set axis MAX pin
			SETOBJ(setMaxPins(*this));
			state = DONE;
			break;
		case 307: // NOT STANDARD - set axis invert
			SETOBJ(setAxisInvert(*this));
			state = DONE;
			break;
		case 308: // NOT STANDARD - set axis disable
			SETOBJ(setAxisDisable(*this));
			state = DONE;
			break;
		case 309: // NOT STANDARD - set global endstop invert, endstop pullups.
			SETOBJ(setEndstopGlobals(cps[P].getInt() == 1 ? true: false, cps[S].getInt() == 1 ? true: false));
			state = DONE;
			break;
		case 310: // NOT STANDARD - report axis configuration status
			SETOBJ(reportConfigStatus(Host::Instance(source)));
			state = DONE;
			break;
		case 350: // NOT STANDARD - change gcode optimization
			if(!cps[P].isUnused() && cps[P].getInt() == 1)
				GCODES.enableOptimize();
			else
				GCODES.disableOptimize();
			state = DONE;
			break;
		case 351: // NOT STANDARD - disable extruder commands (test print option)
			if(!cps[P].isUnused() && cps[P].getInt() == 1)
				DONTRUNEXTRUDER = true;
			else
				DONTRUNEXTRUDER = false;
			state = DONE;
			break;
		// case 400,402 handled by GcodeQueue immediately, not queued.
		case 401: // Execute stored eeprom code.
			Host::Instance(source).write_P(PSTR("EEPROM READ: "));
			if(eeprom::beginRead())
				Host::Instance(source).write_P(PSTR("BEGUN"));
			else
				Host::Instance(source).write_P(PSTR("FAIL"));
			Host::Instance(source).endl();
			state = DONE;
			break;
		// NOT STANDARD - set thermistor table
		case 501: case 502: case 503: case 504: case 505: case 506: case 507: case 508: case 509: case 510:
		case 511: case 512: case 513: case 514: case 515: case 516: case 517: case 518: case 519: case 520:
			if(!cps[P].isUnused() && !cps[S].isUnused())
			{
				TEMPERATURE.changeTempTable(cps[P].getInt(), cps[S].getInt(), cps[M].getInt() - 501);
			}
			state = DONE;
			break;
		default:
			Host::Instance(source).labelnum("warn ", linenum, false);
			Host::Instance(source).write_P(PSTR(" MCODE ")); Host::Instance(source).write(cps[M].getInt(), 10); Host::Instance(source).write_P(PSTR(" NOT SUPPORTED\n"));
			state = DONE;
			break;
	}
#ifndef REPRAP_COMPAT
	if(state == DONE)
	{
		if(linenum != -1)
			Host::Instance(source).labelnum("done ", linenum, false);
		else
			Host::Instance(source).write_P(PSTR("done "));

		Host::Instance(source).labelnum(" M", cps[M].getInt(), true);
	}
#endif
}

void GCode::resetlastpos()
{
#ifndef USE_MARLIN
	lastpos = MOTION.getCurrentPosition();
#endif
	// TODO?
}

void GCode::doPinSet(int arduinopin, int on)
{
	Pin p = Pin(ArduinoMap::getPort(arduinopin),  ArduinoMap::getPinnum(arduinopin));
	p.setDirection(true);
	p.setValue(on == 0 ? false : true);
}

void GCode::togglefan()
{
#ifndef USE_MBIEC
	if(fanpin.isNull())
		return;
	fanpin.setValue(!fanpin.getValue());
#else
	TEMPERATURE.togglefan();
#endif
}
