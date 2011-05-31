#include "Gcodes.h"
#include "Motion.h"

void Gcodes::handlenext()
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
  // Oops, something went wrong
  if(invalidate_codes)
  {
    for(unsigned int x=0;x<codes.getLength();x++)
      codes[x].state = MGcode::NEW;

    MGcode::resetlastpos(MOTION.getCurrentPosition());
    HOST.write("\r\nINVALIDATED CODES\r\n");
    invalidate_codes = false;
    return;
  }
  codes[0].execute();

  // If we just executed the code in front for the first time, then we just return.
  // If it's had some time to go, then we start preparing the next, and the one after, etc.
  unsigned int codesinqueue = codes.getLength();
  unsigned int less = loops < codesinqueue ? loops : codesinqueue;
  for(unsigned int x=1;x<less;x++)
    codes[x].prepare();

  ++loops;
}



Gcodes& GCODES = Gcodes::Instance();
