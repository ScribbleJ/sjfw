#ifndef _GCODES_H_
#define _GCODES_H_

#include "CircularBuffer.h"
#include "MGcode.h"

// Number of Gcodes we can fit in the buffer.
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

  // Returns number of codes in queue
  uint16_t queuelen();
  // Should be called from mainloop; handles gcode dispatch
  void handlenext();
  // Change current line number
  void setLineNumber(unsigned int l);
  // Drop a new gcode on the stack
  void parsecompleted();
  // Tells us whether queue is full.
  bool isFull();
  // Decode a (partial) gcode string
  void parsebytes(char *bytes, uint8_t numbytes);
  // Dump all precalculated data and recompute
  void Invalidate();

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
