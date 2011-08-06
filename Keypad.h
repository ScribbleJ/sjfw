#ifndef _KEYPAD_H
#define _KEYPAD_H
/* Button-grid-scanning-style keypad support.  IDK what the name for these are, they are common.
 * (c) 2011, Christopher "ScribbleJ" Jansen
 * 
 */

#include "AvrPort.h"
#include "Time.h"
#include "config.h"

class Keypad {
public:
 
  // TODO: Why do we copy cps and rps but sit on buttonmap directly? Consistency!
  Keypad(const Pin *cps, const Pin *rps, const char **buttonmap_in, uint8_t debounce_in)
  {                    
    buttonmap = buttonmap_in;
    debounce  = debounce_in;
    good_time = 0;
    last_char = 0;

    for (int i = 0; i < KP_COLS; i++) 
    {
      colpins[i] = cps[i];
    }

    for (int i = 0; i < KP_ROWS; i++) 
    {
      rowpins[i] = rps[i];
    }

    reinit();
  }

  void reinit()
  {
    if(colpins[0].isNull())
      return;

    for (int i = 0; i < KP_COLS; i++) 
    {
      colpins[i].setDirection(false);
      colpins[i].setValue(true);
    }

    for (int i = 0; i < KP_ROWS; i++) 
    {
      rowpins[i].setDirection(false);
      rowpins[i].setValue(true);
    }
  }

   
  // Scans one column of keys for keypresses.
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

  // returns the first pressed key that is found.
  // note - this is expected to be called in a loop like the sjfw mainloop so that it can perform debounce.
  // this function and the above are written this way to make it easy to add support for multiple buttons later.
  char getPressedKey()
  {
    // if config is invalid, maybe we get it later.
    if(colpins[0].isNull())
      return 0;

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

  // Reads configuration from specially-formatted string.
  void parseSettings(char const* str, int charsin)
  {
#define KP_PS(FOO)                                   \
    if(str[x+2] == '-')                                     \
    {                                                       \
      FOO(PortNull,0,str[x+1]-'0');                                  \
      x+=3;                                                 \
      continue;                                             \
    }                                                       \
    FOO(Port::getPortFromLetter(str[x+2]), str[x+3]-'0',str[x+1]-'0');\
    x+=3;


    for(int x=0;x<charsin && str[x]!='*' && str[x]>32;x++)
    {
      switch(str[x])
      {
        case 'C':
          KP_PS(setColPin);
          break;
        case 'R':
          KP_PS(setRowPin);
          break;
      }
    }
  }

  void setColPin(Port& p, int pin, int n) { colpins[n] = Pin(p, pin); }
  void setRowPin(Port& p, int pin, int n) { rowpins[n] = Pin(p, pin); }

private:
  Pin colpins[KP_COLS];
  Pin rowpins[KP_ROWS];

  const char **buttonmap;
  uint8_t debounce;
  unsigned long good_time;
  char last_char;

};

#endif 
