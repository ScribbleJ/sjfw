#ifndef LIQUID_CRYSTAL_HH
#define LIQUID_CRYSTAL_HH

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "AvrPort.h"
#include "RingBuffer.h"
#include "Time.h"

#define LCD_BUFFER_SIZE 100
//#define USE4BITMODE

class LiquidCrystal {
public:
 
  LiquidCrystal(Pin rs, Pin rw, Pin enable,
             Pin d0, Pin d1, Pin d2, Pin d3,
             Pin d4, Pin d5, Pin d6, Pin d7,
             uint8_t cols, 
             uint8_t lines, 
             uint8_t linestarts[] 
             ) : commandQueue(LCD_BUFFER_SIZE, command_data),
                       modeQueue(LCD_BUFFER_SIZE, mode_data) 
  {                    
    _rs_pin = rs;
    _rw_pin = rw;
    _enable_pin = enable;
    
    _data_pins[0] = d0;
    _data_pins[1] = d1;
    _data_pins[2] = d2;
    _data_pins[3] = d3; 
    _data_pins[4] = d4;
    _data_pins[5] = d5;
    _data_pins[6] = d6;
    _data_pins[7] = d7; 

    _rs_pin.setDirection(true);
    _rs_pin.setValue(false);
    _rw_pin.setDirection(true); 
    _rw_pin.setValue(false);
    _enable_pin.setDirection(true);
    _enable_pin.setValue(false);

    _numlines = lines;
    _numcols  = cols;
    _currline = 0;
    _linestarts = linestarts;

#ifdef USE4BITMODE
    wrotehalf = false;

    for (int i = 4; i < 8; i++)
#else    
    for (int i = 0; i < 8; i++) 
#endif
    {
      _data_pins[i].setDirection(true);
      _data_pins[i].setValue(false);
    }

    // Set bitmode, multiline mode - follows hitachi docs
    initialize();
#ifdef USE4BITMODE
    setFunction(false, (_numlines > 1) ? true : false, false);
#else    
    setFunction(true, (_numlines > 1) ? true : false, false);
#endif    
    // display off, cursor and blink off
    setDisplayControls(false, false, false);
    // clear
    clear();
    // No scrolling, shift right
    setEntryMode(false, false);
    // turn display on
    setDisplayControls(true, false, false);
    // Set start write address
    writeDDRAM(0);
  }
   
  // Clears the screen AND resets cursor to "home"
  void clear() { command(0x01); }

  // Sends cursor to "home"
  void home() { command(0x02); }

  // false, false means shift right don't scroll display
  void setEntryMode(bool left, bool displayscroll) 
  {
    command(0x04 |
           (left ? 0x02 : 0) |
           (displayscroll ? 0x01 : 0));
  }

  // display, cursor, and blink on
  void setDisplayControls(bool displayon, bool cursoron, bool blinkon) 
  {
    command(0x08 |
           (displayon ? 0x04 : 0) |
           (cursoron  ? 0x02 : 0) |
           (blinkon   ? 0x01 : 0) );
  }

  // shifts cursor left if "right" is false
  void shift(bool display, bool right)
  {
    command(0x10 |
           (display ? 0x08 : 0) |
           (right   ? 0x04 : 0));
  }         

  // named "setFunction" only to match Hitachi docs.
  // select bitmode, multiline, 5x10 font
  void setFunction(bool is8bit, bool is2line, bool fontselect)
  {
    command(0x20 |
           (is8bit      ? 0x10 : 0) |
           (is2line     ? 0x08 : 0) |
           (fontselect? 0x04 : 0));
  }           

  // Misleading name; sets address of next write.  does no writing.
  void writeCGRAM(uint8_t location) 
  {
    command(0x40 | location );
  }

  // Misleading name; sets address of next write.  does no writing.
  void writeDDRAM(uint8_t location)
  {
    command(0x80 | location );
  }

  // sets cursor position by col, row
  void setCursor(uint8_t col, uint8_t row)
  {
    int offset   = _linestarts[row] + col;
    writeDDRAM(offset);
  }

  // TODO: replace these awful things.
  void write(uint8_t value) { enqueue(value, true); }
  void write(char const *str, uint8_t len) 
  {
    for(int x = 0; x < len; x++) write(str[x]);
  }
  void write(uint16_t d, uint16_t maxradix)
  {
    uint16_t n = d;
    for(uint16_t i = maxradix; i >= 1; i = i/10)
    {
      write((uint8_t)(n/i) + '0');
      n -= (n/i) * i; // This looks like nonsense but it chops off the remainder.
    }
  }

  // handleUpdates MUST BE CALLED OFTEN
  // often means... at least as often as you'd like to see a new character
  // appear on the display.
  void handleUpdates()
  {
    if(isBusy())
      return;

    dequeue();
  }

  // Uses BUSY FLAG pin on LCD display to determine when LCD is ready for next write.
  bool isBusy()
  {
#ifdef USE4BITMODE
    if(wrotehalf)
      return false;
#endif      
    _rs_pin.setValue(false);
    _rw_pin.setValue(true);


#ifdef USE4BITMODE
    static bool readhalf = false;
    static bool result   = true;
    for (int i = 4; i < 8; i++)
#else    
    for (int i = 0; i < 8; i++) 
#endif    
    {
      _data_pins[i].setDirection(false);
      _data_pins[i].setValue(false);
    }

    // Pulsing enable causes LCD to read this command.
    _enable_pin.setValue(true);
    bool v = _data_pins[7].getValue();
    _enable_pin.setValue(false);

#ifdef USE4BITMODE
    if(!readhalf)
    {
      result = v;
      readhalf = !readhalf;
      return true;
    }
    else
    {
      v = result;
      readhalf = !readhalf;
      result = true;
      return v;
   }

#endif
    return v;
  }


private:
  // enqueues a "control" command.
  void command(uint8_t value) { enqueue(value, false); }

  // puts a command/write into the queue to be done later
  void enqueue(uint8_t value, bool mode) {
    if(commandQueue.isFull())
      return;  // TODO: Silent fail.

    commandQueue.push(value);
    modeQueue.push(mode); 
  }

  // pulls a command/write from the queue, sends it to the LCD.
  void dequeue()
  {
    if(commandQueue.isEmpty())
      return;

    uint8_t value;
    bool mode;
#ifdef USE4BITMODE
    if(!wrotehalf)
    {
      value = commandQueue.peek(0);
      mode = modeQueue.peek(0);
    }
    else
#endif    
    {
      // Pull command off stack
      value = commandQueue.pop();
      mode  = modeQueue.pop();
    }

    _rw_pin.setValue(false);
    _rs_pin.setValue(mode);

#ifdef USE4BITMODE
    for (int i = 0; i < 4; i++)
    {
      _data_pins[i+4].setDirection(true);
      _data_pins[i+4].setValue(((value >> (wrotehalf ? i : i+4)) & 0x01) != 0);
    }
    wrotehalf = !wrotehalf;
#else    
    for (int i = 0; i < 8; i++) {
      _data_pins[i].setDirection(true);
      _data_pins[i].setValue(((value >> i) & 0x01) != 0);
    }
#endif    

    // Pulsing enable causes LCD to read this command.
    _enable_pin.setValue(true);
    _enable_pin.setValue(false);
  }

#ifndef USE4BITMODE
  void write8init(uint8_t value)
  {
    for (int i = 0; i < 8; i++) {
      _data_pins[i].setDirection(true);
      _data_pins[i].setValue(((value >> i) & 0x01) != 0);
    }
    _enable_pin.setValue(true);
    _enable_pin.setValue(false);
  }

  // 8-bit initialization routine from hitachi doc.
  void initialize()
  {
    _rw_pin.setDirection(true);
    _rw_pin.setValue(false);
    _rs_pin.setDirection(true);
    _rs_pin.setValue(false);

    // Normally I wouldn't allow delays in the code at all, but this
    // is at startup, so no worries, and handling it in any other
    // fashion would be painful.
    wait(20);
    write8init(0b00110000); 
    wait(10);
    write8init(0b00110000); 
    wait(5);
    write8init(0b00110000); 
    wait(5);
  }
#else
  void write4init(uint8_t value)
  {
    for (int i = 0; i < 4; i++) {
      _data_pins[i+4].setDirection(true);
      _data_pins[i+4].setValue(((value >> i) & 0x01) != 0);
    }
    _enable_pin.setValue(true);
    _enable_pin.setValue(false);
  }

  // 4-bit initialization routine from hitachi doc.
  void initialize()
  {
    _rw_pin.setDirection(true);
    _rw_pin.setValue(false);
    _rs_pin.setDirection(true);
    _rs_pin.setValue(false);

    // Normally I wouldn't allow delays in the code at all, but this
    // is at startup, so no worries, and handling it in any other
    // fashion would be painful.
    wait(20);
    write4init(0b00110000); 
    wait(10);
    write4init(0b00110000); 
    wait(5);
    write4init(0b00110000); 
    wait(5);
    write4init(0b00100000); 
  }

#endif  


  Pin _rs_pin;
  Pin _rw_pin;
  Pin _enable_pin;
  Pin _data_pins[8];

  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;

  uint8_t _initialized;

  uint8_t _numlines,_currline;
  uint8_t _numcols;
  uint8_t *_linestarts;

#ifdef USE4BITMODE
  bool wrotehalf;
#endif    

  uint8_t command_data[LCD_BUFFER_SIZE];
  bool    mode_data[LCD_BUFFER_SIZE];
  RingBufferT<uint8_t> commandQueue;
  RingBufferT<bool> modeQueue;
};

#endif // LIQUID_CRYSTAL_HH
