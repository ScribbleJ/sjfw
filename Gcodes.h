#ifndef _GCODES_H_
#define _GCODES_H_

#include "RingBuffer.h"
#include "MGcode.h"

// Number of Gcodes we can fit in the buffer.
#define GCODE_BUFSIZE 5
#define GCODE_SOURCES 2
class Gcodes
{
public:
  // Singleton pattern... only one Gcode queue exists.
  static Gcodes& Instance() { static Gcodes instance; return instance; }
private:
  explicit Gcodes()  :codes(GCODE_BUFSIZE, codes_buf) 
  { 
    for(int x=0;x<GCODE_SOURCES;x++)
    {
      sources[x].reset();
      crc_state[x] = NOCRC;
      crc[x] = 0;
      line_number[x] = -1;
      chars_in_line[x] = 0;
    }
  }
  Gcodes(Gcodes const&);
  void operator=(const Gcodes&);
public:

  // Should be called from mainloop; handles gcode dispatch
  void handlenext();
  // Change current line number
  void setLineNumber(unsigned int l, uint8_t source);
  void setLineNumber(unsigned int l) { setLineNumber(l, 0); }
  // Drop a new gcode on the stack
  void enqueue(MGcode& c);
  // Tells us whether queue is full.
  bool isFull();
  // Decode a (partial) gcode string
  void parsebytes(char *bytes, uint8_t numbytes) { parsebytes(bytes, numbytes, 0); }
  void parsebytes(char *bytes, uint8_t numbytes, uint8_t source);
  // Dump all precalculated data and recompute
  void Invalidate();

private:
  MGcode codes_buf[GCODE_BUFSIZE];
  RingBufferT<MGcode> codes;

  MGcode sources[GCODE_SOURCES];
  enum crc_state_t { NOCRC, CRC, CRCCOMPLETE } crc_state[GCODE_SOURCES];
  uint8_t crc[GCODE_SOURCES];
  int32_t line_number[GCODE_SOURCES];
  uint8_t chars_in_line[GCODE_SOURCES];
  bool invalidate_codes;
};
  
extern Gcodes& GCODES;  


#endif // _GCODES_H_
