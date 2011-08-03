#include "GcodeQueue.h"

#include "Motion.h"
#include "Host.h"
#include "Globals.h"
#include <stdlib.h>
#include <errno.h>

// REMOVEME
#include "Time.h"



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
    c.dump_to_host();
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
#ifndef REPRAP_COMPAT
        crc[source] += chars_in_line[source] + x;
        HOST.labelnum("LEN: ", chars_in_line[source]);
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

    chars_in_line[source] += numbytes+1;

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
      if(line_number[source] + 1 != l)
      {
        if(l != -1)
        {
          crc[source] = 0;
          crc_state[source] = NOCRC;
          HOST.rxerror("Invalid line number.",line_number[source] + 1);
          chars_in_line[source] = 0;
          return;
        }
        break;
      }
      line_number[source]++;
      c.setLinenumber(line_number[source]);
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
      c[T].setInt(bytes+1);
      break;
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
          crc[source] = 0;
          crc_state[source] = NOCRC;
          line_number[source]--;
          HOST.rxerror("CRC mismatch.",line_number[source] + 1);
          chars_in_line[source] = 0;
          return;
        }
        break;
      case NOCRC:
        ;// Do nothing here.
        break;
      case CRC:
      default:
        crc[source] = 0;
        crc_state[source] = NOCRC;
        line_number[source]--;
        // Error out - no CRC before end of command or invalid state.
        HOST.rxerror("Missing CRC.",line_number[source] + 1);
        chars_in_line[source] = 0;
        return;
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

    enqueue(sources[source]);

    if(source == HOST_SOURCE)
    {
#ifndef REPRAP_COMPAT    
      HOST.labelnum("ok ", codes.getCount(), true);
#else
      HOST.write("ok \n");
#endif      
    }
  }
  else
  {
    //HOST.write("Fragment parsed.\n");
  }

  return;
  
}

void GcodeQueue::Invalidate() { invalidate_codes = true; }




GcodeQueue& GCODES = GcodeQueue::Instance();
