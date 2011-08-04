#ifndef _KEYPAD_H
#define _KEYPAD_H

#include "AvrPort.h"
#include "Time.h"
#include "config.h"

class Keypad {
public:
 
  Keypad(const Pin *cps, const Pin *rps, const char **buttonmap_in, uint8_t debounce_in)
  {                    
    buttonmap = buttonmap_in;
    debounce  = debounce_in;
    good_time = 0;
    last_char = 0;

    for (int i = 0; i < KP_COLS; i++) 
    {
      colpins[i] = cps[i];
      colpins[i].setDirection(false);
      colpins[i].setValue(true);
    }

    for (int i = 0; i < KP_ROWS; i++) 
    {
      rowpins[i] = rps[i];
      rowpins[i].setDirection(false);
      rowpins[i].setValue(true);
    }

  }
   
  uint8_t scanCol(uint8_t colnum)
  {
    colpins[colnum].setDirection(true);
    colpins[colnum].setValue(false);

    uint8_t ret = 0;
    for(int x=0;x<KP_ROWS;x++)
    {
      if(!rowpins[x].getValue())
        ret |= 1 << x;
    }

    colpins[colnum].setDirection(false);
    colpins[colnum].setValue(false);

    return ret;
  }

  char getPressedKey()
  {
    unsigned long now = micros();

    uint8_t colscan = 0;
    for(int x=0;x<KP_COLS;x++)
    {
      colscan = scanCol(x);
      if(colscan)
      {
        for(int y=0;y<KP_ROWS;y++)
        {
          if(colscan & (1 << y))
          {
            char fc = buttonmap[y][x];
            if(fc == last_char && good_time <= now)
              return fc;

            if(fc != last_char)
            {
              last_char = fc;
              good_time = now + debounce;
            }
            return 0;
          }
        }
      }
    }
    if(good_time && good_time < now)
    {
      good_time = 0;
      last_char = 0;
    }
    return 0;
  }


private:
  Pin colpins[KP_COLS];
  Pin rowpins[KP_ROWS];

  const char **buttonmap;
  uint8_t debounce;
  unsigned long good_time;
  char last_char;

};

#endif 
