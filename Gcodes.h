#ifndef _GCODES_H_
#define _GCODES_H_

#include "Host.h"
#include "CircularBuffer.h"
#include "MGcode.h"
#include "Globals.h"
#include <stdlib.h>
#include <errno.h>

#define GCODE_BUFSIZE 15
class Gcodes
{
public:
  // Truly one-of-a-kind
  static Gcodes& Instance() { static Gcodes instance; return instance; }
private:
  explicit Gcodes()  :codes(GCODE_BUFSIZE, codes_buf,1), crc_state(NOCRC), crc(0), line_number(-1), chars_in_line(0), invalidate_codes(false) { };
  Gcodes(Gcodes const&);
  void operator=(const Gcodes&);
public:


  uint16_t queuelen() { return codes.getLength(); }

  void handlenext();

  void setLineNumber(unsigned int l) { line_number = l - 1; }

  void parsecompleted()
  {
    int currentcode = codes.getLength();
    MGcode& c=codes[currentcode];
    if(chars_in_line == 1)
    {
      // HOST.write("Noise.\r\n");
      c.reset();
      return;
    }

    if(c[M].isUnused() == c[G].isUnused()) // Both used or both unused is an error.
    {
      // For now, we will fail silently here, rather than work out a method for handling blank lines without erroring.
      HOST.labelnum("Discarding bogus Gcode.", (int)line_number + 1);
      c.dump_to_host();
      c.reset();
      return;
    }
    codes.fakepush();
    //HOST.write("Added code to queue:");
    //c.dump_to_host();
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
    chars_in_line = 0;
  }

  bool isFull() { return (codes.getRemainingCapacity() == 0); }

  void parsebytes(char *bytes, uint8_t numbytes)
  {
    uint8_t ourcrc = 0;
    bool packetdone = false;
    chars_in_line += numbytes+1;

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
        // HOST.labelnum("CRC+=", bytes[x], true);
        // HOST.labelnum("CRC=", crc, true);

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

    if(numbytes == 0 and !packetdone)
      return;

    bytes[numbytes] = 0;

    int idx = codes.getLength();
    MGcode& c = codes[idx];
    long l; 
    errno = 0;
    switch(bytes[0])
    {
      case 'N':
        l = atol(bytes+1);
        HOST.labelnum("Starting line number:", (int) line_number + 1, true);
        if(line_number + 1 != l)
        {
          if(l != -1)
          {
            crc = 0;
            crc_state = NOCRC;
            HOST.rxerror("Invalid line number.",line_number + 1);
            return;
          }
          break;
        }
        line_number++;
        c.setLinenumber(line_number);
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
      HOST.labelnum("Ending line number:", (int) line_number, true);
      switch(crc_state)
      {
        case CRCCOMPLETE:
          if(crc != ourcrc)
          {
            HOST.labelnum("CRC COMPUTED: ", crc, true);
            HOST.labelnum("CRC RECVD: ", ourcrc, true);
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
  void Invalidate() { invalidate_codes = true; }

private:
  MGcode codes_buf[GCODE_BUFSIZE];
  CircularBufferTempl<MGcode> codes;

  enum crc_state_t { NOCRC, CRC, CRCCOMPLETE } crc_state;
  uint8_t crc;
  int32_t line_number;
  uint8_t chars_in_line;
  bool invalidate_codes;
};
  
extern Gcodes& GCODES;  


#endif // _GCODES_H_
