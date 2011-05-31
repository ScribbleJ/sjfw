#ifndef _GCODES_H_
#define _GCODES_H_

#include "Host.h"
#include "CircularBuffer.h"
#include "MGcode.h"
#include "Globals.h"
#include <stdlib.h>
#include <errno.h>

#define GCODE_BUFSIZE 10
class Gcodes
{
public:
  // Truly one-of-a-kind
  static Gcodes& Instance() { static Gcodes instance; return instance; }
private:
  explicit Gcodes()  :codes(GCODE_BUFSIZE, codes_buf,1), crc_state(NOCRC), crc(0), line_number(0) { };
  Gcodes(Gcodes const&);
  void operator=(const Gcodes&);
public:


  uint16_t queuelen() { return codes.getLength(); }

  void handlenext()
  {
    static unsigned int loops = 0;
    if(codes.isEmpty())
      return;


    if(codes[0].isDone())
    {
      codes.fakepop();
      loops = 0;
      if(codes.isEmpty())
        return;
    }
    codes[0].execute();

    // If we just executed the code in front for the first time, then we just return.
    // If it's had some time to go, then we start preparing the next, and the one after, etc.
    unsigned int codesinqueue = codes.getLength();
    unsigned int less = loops < codesinqueue ? loops : codesinqueue;
    for(unsigned int x=1;x<less;x++)
      codes[x].prepare(&(codes[x-1]));

    ++loops;
  }

  void setLineNumber(unsigned int l) { line_number = l; }

  void parsecompleted()
  {
    MGcode& c=codes[codes.getLength()];
    if(c[M].isUnused() == c[G].isUnused()) // Both used or both unused is an error.
    {
      HOST.rxerror("Discarding bogus Gcode.", line_number + 1);
      c.reset();
      return;
    }
    codes.fakepush();
    if(codes.hasOverflow() || codes.hasUnderflow())
    {
      codes.reset();
      HOST.rxerror("GCODE buffer FUBAR.", line_number + 1);
      // Throw some nasty-ass error
    }
    else
    {
      HOST.write("ok\r\n");
    }
    codes[codes.getLength()].reset();
    crc_state = NOCRC;
    crc = 0;
  }

  bool isFull() { return (codes.getRemainingCapacity() == 0); }

  void parsebytes(char *bytes, uint8_t numbytes)
  {
    if(numbytes == 0)
      return;

    uint8_t ourcrc = 0;
    bool packetdone = false;

    // HOST.labelnum("GCode parsing X bytes:", numbytes);
    
    if(crc_state == NOCRC && bytes[0] == 'N')
      crc_state = CRC;

    if(crc_state == CRC)
    {
      uint8_t crcpos=0;
      for(int x=0;x<=numbytes;x++)
      {
        if(bytes[x] == '*')
        {
          bytes[x] = 0;
          crcpos=x+1;
          crc_state = CRCCOMPLETE;
          break;
        }

        crc = crc ^ bytes[x];

        if(bytes[x] < 32)
        {
          bytes[x] = 0;
          packetdone = true;
        }
      }

      // CRC finished?
      // If crcpos == 0 then no crcpos.  
      // If crcpos == 1 then it will be handled in following switch.
      if(crcpos > 1)
      {
        ourcrc = atoi(bytes + crcpos);
        packetdone = true;
      }
    }
    else if(bytes[numbytes] < 32)
      packetdone = true;

    bytes[numbytes] = 0;

    int idx = codes.getLength();
    unsigned long l; 
    errno = 0;
    switch(bytes[0])
    {
      case 'N':
        l = atol(bytes+1);
        if(line_number + 1 != l)
        {
          crc = 0;
          crc_state = NOCRC;
          HOST.rxerror("Invalid line number.",line_number + 1);
          return;
        }
        line_number++;
        break;
      case 'M':
        codes[idx][M].setInt(bytes+1);
        break;
      case 'G':
        codes[idx][G].setInt(bytes+1);
        break;
      case 'F':
        codes[idx][F].setFloat(bytes+1);
        break;
      case 'X':
        codes[idx][X].setFloat(bytes+1);
        break;
      case 'Y':
        codes[idx][Y].setFloat(bytes+1);
        break;
      case 'Z':
        codes[idx][Z].setFloat(bytes+1);
        break;
      case 'E':
        codes[idx][E].setFloat(bytes+1);
        break;
      case 'P':
        codes[idx][P].setInt(bytes+1);
        break;
      case 'S':
        codes[idx][S].setInt(bytes+1);
        break;
      case 'T':
        codes[idx][T].setInt(bytes+1);
        break;
      case 0:
        ourcrc = atoi(bytes + 1);
        packetdone = true;
        break;
    }
    if(errno)
    {
      crc = 0;
      crc_state = NOCRC;
      line_number--;
      HOST.rxerror("Number Conversion Error", line_number + 1);
    }

    if(packetdone)
    {
      // HOST.write("Packet parsed.\r\n");
      switch(crc_state)
      {
        case CRCCOMPLETE:
          if(crc != ourcrc)
          {
            crc = 0;
            crc_state = NOCRC;
            line_number--;
            HOST.rxerror("CRC mismatch.",line_number + 1);
            return;
          }
          break;
        case NOCRC:
          ;// Do nothing here.
          break;
        case CRC:
        default:
          crc = 0;
          crc_state = NOCRC;
          line_number--;
          // Error out - no CRC before end of command or invalid state.
          HOST.rxerror("Missing CRC.",line_number + 1);
          return;
      }
      parsecompleted();
    }
    else
    {
      // HOST.write("Fragment parsed.\r\n");
    }

    return;
    
  }

private:
  MGcode codes_buf[GCODE_BUFSIZE];
  CircularBufferTempl<MGcode> codes;

  enum crc_state_t { NOCRC, CRC, CRCCOMPLETE } crc_state;
  uint8_t crc;
  uint16_t line_number;
};
  
extern Gcodes& GCODES;  


#endif // _GCODES_H_
