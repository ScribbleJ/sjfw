#include "GcodeQueue.h"

#include "Motion.h"
#include "Host.h"
#include "Globals.h"
#include <stdlib.h>
#include <errno.h>

// REMOVEME
#include "Time.h"
#ifdef HAS_LCD
#include "LCDKeypad.h"
extern LCDKeypad LCDKEYPAD;
#endif

#include "Temperature.h"



void GcodeQueue::handlenext()
{
  static unsigned int loops = 0;
  // static unsigned long last = millis();
  if(codes.getCount() == 0)
  {
/*  unsigned long now = millis();
    if(now - last > 1000)
    {
      last = now;
      HOST.write("NOTHING IN QUEUE.\n");
    }
*/    
    return;
  }

  if(codes.peek(0).isDone())
  {
    codes.peek(0).wrapupmove();
    codes.pop();
    //HOST.labelnum("RC-QL:", codes.getCount());
    loops = 0;
    if(codes.isEmpty())
      return;
  }
  // Oops, something went wrong
  if(invalidate_codes)
  {
    for(unsigned int x=0;x<codes.getCount();x++)
      codes.peek(x).state = GCode::NEW;

    GCode::resetlastpos(MOTION.getCurrentPosition());
    HOST.write("\nINVALIDATED CODES\n");
    invalidate_codes = false;
    return;
  }

  codes.peek(0).execute();

  // If we just executed the code in front for the first time, then we just return.
  // If it's had some time to go, then we start preparing the next, and the one after, etc.
  unsigned int codesinqueue = codes.getCount();
  unsigned int less = loops < codesinqueue ? loops : codesinqueue;
  for(unsigned int x=1;x<less;x++)
    codes.peek(x).prepare();

  ++loops;
}

void GcodeQueue::setLineNumber(uint32_t l, uint8_t source) { line_number[source] = l - 1; }

void GcodeQueue::enqueue(GCode &c)
{
  if(c[M].isUnused() == c[G].isUnused()) // Both used or both unused is an error.
  {
#ifndef REPRAP_COMPAT  
    c.dump_to_host();
#endif    
    return;
  }
  codes.push(c);
  //HOST.labelnum("AC-QL:", codes.getCount());

}

bool GcodeQueue::isFull() { return codes.isFull(); }

void GcodeQueue::parsebytes(char *bytes, uint8_t numbytes, uint8_t source)
{
  uint8_t ourcrc = 0;
  bool packetdone = false;

  
  if(crc_state[source] == NOCRC && bytes[0] == 'N')
    crc_state[source] = CRC;

  if(crc_state[source] == CRC)
  {
    uint8_t crcpos=0;
    for(int x=0;x<=numbytes;x++)
    {
      if(bytes[x] == '*')
      {
        bytes[x] = 0;
        crcpos=x+1;
        crc_state[source] = CRCCOMPLETE;
#ifdef  ADV_CHECKSUMS
        crc[source] += chars_in_line[source] + x;
#endif        
        break;
      }

      crc[source] ^= bytes[x];

      if(bytes[x] < 32)
      {
        bytes[x] = 0;
        packetdone = true;
      }
    }

    // CRC finished?
    // If crcpos == 0 then no crcpos.  
    // If crcpos == 1 then it will be handled in following switch.
    if(crcpos >= 1)
    {
      ourcrc = atoi(bytes + crcpos);
      packetdone = true;
    }
  }
  else if(bytes[numbytes] < 32)
    packetdone = true;

  chars_in_line[source] += numbytes+1;
  bytes[numbytes] = 0;
  //HOST.write("did: "); HOST.write(bytes); HOST.write("\n");

  if(numbytes == 0 and !packetdone)
    return;


  GCode& c = sources[source];
  if(chars_in_line[source] == numbytes+1)
    c.reset();

  long l; 
  errno = 0;
  switch(bytes[0])
  {
    case 'N':
      l = atol(bytes+1);
      //HOST.labelnum("Starting line number:", (int) line_number + 1, true);
      // TODO: fixme; allow start with '1' to fix dumb hosts.
      if(l < 1)
      {
        line_number[source] = l-1;
      }

      line_number[source]++;
      c.setLinenumber(line_number[source]);

      if(line_number[source] != l)
      {
        //crc[source] = 0;
        //crc_state[source] = NOCRC;
        Host::Instance(source).labelnum("Invalid line:",l);
        //chars_in_line[source] = 0;
        needserror[source]=true;
        break;
      }
      break;
    case 'M':
      c[M].setInt(bytes+1);
      break;
    case 'G':
      c[G].setInt(bytes+1);
      break;
    case 'F':
      c[F].setFloat(bytes+1);
      break;
    case 'X':
      c[X].setFloat(bytes+1);
      break;
    case 'Y':
      c[Y].setFloat(bytes+1);
      break;
    case 'Z':
      c[Z].setFloat(bytes+1);
      break;
    case 'E':
      c[E].setFloat(bytes+1);
      break;
    case 'P':
      c[P].setInt(bytes+1);
      break;
    case 'S':
      c[S].setInt(bytes+1);
      break;
    case 'T':
      //c[T].setInt(bytes+1);
      break;
    // SPECIAL HANDLING FOR PINSETTING
    case '$':
      // Only activate pinsetting mode if G and M are /both/ set
      // This is both a nice way to signal something unusual is happening
      // and also causes the code to not get dropped into the queue.
      if(c[M].isUnused() || c[G].isUnused())
        break;
      doPinSetting(c, bytes+1, numbytes);
      break;
    case '%':
      if(c[M].isUnused() || c[G].isUnused())
        break;
      doTempSetting(c, bytes+1, numbytes);
      break;
#ifdef HAS_LCD      
    case '^':
      if(c[M].isUnused() || c[G].isUnused())
        break;        
      LCDKEYPAD.doLCDSettings(bytes+1, numbytes);
      break;
    case '&':
      if(c[M].isUnused() || c[G].isUnused())
        break;
      LCDKEYPAD.doKeypadSettings(bytes+1, numbytes);
      break;
#endif      
    case 0:
      ; // noise
      break;
    default:
      ; // just noise
      break;
  }

  if(packetdone)
  {
    //HOST.write("Packet parsed.\n");
    //HOST.labelnum("Ending line number:", (int) line_number, true);
    switch(crc_state[source])
    {
      case CRCCOMPLETE:
        if(crc[source] != ourcrc)
        {
          //HOST.labelnum("CRC COMPUTED: ", crc, true);
          //HOST.labelnum("CRC RECVD: ", ourcrc, true);
          //crc[source] = 0;
          //crc_state[source] = NOCRC;
          //line_number[source]--;
          Host::Instance(source).labelnum("CRC mismatch:",line_number[source]);
          needserror[source] = true;
          //chars_in_line[source] = 0;
        }
        break;
      case NOCRC:
        ;// Do nothing here.
        break;
      case CRC:
      default:
        //crc[source] = 0;
        //crc_state[source] = NOCRC;
        //line_number[source]--;
        //// Error out - no CRC before end of command or invalid state.
        Host::Instance(source).labelnum("Missing CRC:",line_number[source]);
        //chars_in_line[source] = 0;
        needserror[source] = true;
        break;
    }

    crc_state[source] = NOCRC;
    crc[source] = 0;

    if(chars_in_line[source] == 1)
    {
      c.reset();
      chars_in_line[source] = 0;
      return;
    }
    chars_in_line[source] = 0;

    if(needserror[source])
    {
      Host::Instance(source).rxerror("Unknown.", line_number[source]);
      line_number[source]--;
      c.reset();
      needserror[source] = false;
      return;
    }


    c.source = source;
    enqueue(sources[source]);

    if(source == HOST_SOURCE || source == BT_SOURCE)
    {
#ifndef REPRAP_COMPAT    
      Host::Instance(source).labelnum("ok ", codes.getCount(), true);
#else
      Host::Instance(source).write("ok \n");
#endif      
    }
  }
  else
  {
    //HOST.write("Fragment parsed.\n");
  }

  return;
  
}

// TODO: Move these.
bool GcodeQueue::doPinSetting(GCode& c, char const* str, int charsin)
{
  int axis = str[0] - 'X';
  if(str[0] < 'X')
    axis = str[0] - 'A' + 3;

  //HOST.labelnum("PCHANGE ", axis, true);
  //MOTION.getAxis(axis).dump_to_host();
  for(int x=1;x<charsin && str[x]!='*' && str[x]>32;x++)
  {
    //HOST.write("  ");
    //HOST.write(str[x]);
    //HOST.write(" is:");
    //if(str[x+1] == '-')
    //  HOST.write("NULL\n");
    //else
    //{
    //  HOST.write(str[x+1]);
    //  HOST.labelnum(",",(str[x+2]-'0'),true);
    //}

#define _DOPINSETTING(FOO)  \
    if(str[x+1] == '-') \
    {                   \
      MOTION.getAxis(axis).FOO(PortNull,0); \
      x+=2;             \
      break;         \
    }                   \
    MOTION.getAxis(axis).FOO(Port::getPortFromLetter(str[x+1]), str[x+2]-'0'); \
    x+=2; 

    switch(str[x])
    {
      case 'S':
        _DOPINSETTING(changepinStep);
        break;
      case 'D':
        _DOPINSETTING(changepinDir);
        break;
      case 'E':
        _DOPINSETTING(changepinEnable);
        break;
      case 'I':
        _DOPINSETTING(changepinMin);
        break;
      case 'A':
        _DOPINSETTING(changepinMax);
        break;
      case 'N':
        MOTION.getAxis(axis).setInvert(str[x+1]-'0');
        x++;
        break;
      case 'X':
        MOTION.getAxis(axis).setDisable(str[x+1]-'0');
        x++;
        break;
      case 'J':
        MOTION.getAxis(axis).setPULLUPS(str[x+1]-'0');
        x++;
        break;
      case 'K':
        MOTION.getAxis(axis).setEND_INVERT(str[x+1]-'0');
        x++;
        break;
    }
  }

  //MOTION.getAxis(axis).dump_to_host();
    
  return true;

}

void GcodeQueue::Invalidate() { invalidate_codes = true; }

bool GcodeQueue::doTempSetting(GCode& c, char const* str, int charsin)
{
  for(int x=0;x<charsin && str[x]!='*' && str[x]>32;x++)
  {
    switch(str[x])
    {
      case 'H':
        if(str[x+1] == '-') 
        {                   
          TEMPERATURE.changePinHotend(PortNull,0);
          x+=2; 
          continue;
        }         
        TEMPERATURE.changePinHotend(Port::getPortFromLetter(str[x+1]), str[x+2]-'0'); 
        x+=2; 
        break;
      case 'B':
        if(str[x+1] == '-') 
        {                   
          TEMPERATURE.changePinPlatform(PortNull,0);
          x+=2; 
          continue;
        }         
        TEMPERATURE.changePinPlatform(Port::getPortFromLetter(str[x+1]), str[x+2]-'0'); 
        x+=2; 
        break;
    }
  }
  return true;
}


GcodeQueue& GCODES = GcodeQueue::Instance();
