#ifndef _GCODES_H_
#define _GCODES_H_
/* GCode physical parser and queue
 * (c) 2011 Christopher "ScribbleJ" Jansen
 *
 * This class is responsible for recieving Gcode from 'sources' in fragments.
 * A source is something on the system that wants to provide Gcode; e.g. the host, 
 * or the SD Card, or the Control Pad.
 * Fragments are a chunk of gcode up to and including a space or carriage return/newline (BUT NO MORE THAN THAT!)
 * Callers are presently expected to check isFull() before attempting to send a fragment.
 *
 */

#include "RingBuffer.h"
#include "GCode.h"
#include "config.h"
#include "AvrPort.h"

class GcodeQueue
{
public:
  // Singleton pattern... only one Gcode queue exists.
  static GcodeQueue& Instance() { static GcodeQueue instance; return instance; }
private:
  explicit GcodeQueue()  :codes(GCODE_BUFSIZE, codes_buf) 
  { 
    for(int x=0;x<GCODE_SOURCES;x++)
    {
      sources[x].reset();
      crc_state[x] = NOCRC;
      crc[x] = 0;
      line_number[x] = -1;
      chars_in_line[x] = 0;
      needserror[x] = false;
      ADVANCED_CRC[x] = false;
    }
    optimize_gcode = false;
  }
  GcodeQueue(GcodeQueue const&);
  void operator=(const GcodeQueue&);
public:

  // Should be called often from mainloop; handles gcode dispatch
  void handlenext();
  // disables inactive axes - not sure why this is here.
  void checkaxes();

  // Change current line number
  void setLineNumber(uint32_t l, uint8_t source);
  void setLineNumber(unsigned int l) { setLineNumber(l, 0); }
  // Drop a new gcode on the stack
  void enqueue(GCode& c);
  // Tells us whether queue is full.
  bool isFull();
  // Decode a (partial) gcode string
  void parsebytes(char *bytes, uint8_t numbytes) { parsebytes(bytes, numbytes, 0); }
  void parsebytes(char *bytes, uint8_t numbytes, uint8_t source);
  // Dump all precalculated data and recompute
  void Invalidate();

  void enableOptimize() { optimize_gcode = true; };
  void disableOptimize() { optimize_gcode = false; };
  bool shouldOptimize() { return optimize_gcode; };
  void enableADVANCED_CRC(int source) { ADVANCED_CRC[source] = true; }
  void disableADVANCED_CRC(int source) { ADVANCED_CRC[source] = false; }

private:
  GCode codes_buf[GCODE_BUFSIZE];
  RingBufferT<GCode> codes;

  GCode sources[GCODE_SOURCES];
  enum crc_state_t { NOCRC, CRC, CRCCOMPLETE } crc_state[GCODE_SOURCES];
  uint8_t crc[GCODE_SOURCES];
  int32_t line_number[GCODE_SOURCES];
  uint8_t chars_in_line[GCODE_SOURCES];
  bool needserror[GCODE_SOURCES];
  bool invalidate_codes;
  bool optimize_gcode; // WTF is this here?  This whole pipeline needs serious refactor.
  bool ADVANCED_CRC[GCODE_SOURCES];
};
  
extern GcodeQueue& GCODES;  


#endif // _GCODES_H_
