#ifndef LIQUID_CRYSTAL_HH
#define LIQUID_CRYSTAL_HH
/* HITACHI-style LCD support, 4- and 8-bit. 
 * (c) 2011, Christopher "ScribbleJ" Jansen
 *
 * Note: 4-bit startup from poweroff is not reliable; the LCD must be powered on /long/
 * before the MCU.  You can achieve this with a reset after starting from powerup.
 *
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "AvrPort.h"
#include "ArduinoMap.h"
#include "RingBuffer.h"
#include "Time.h"
#include "config.h"
#include <stdlib.h>
#include <avr/pgmspace.h>

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
    initialized = false;

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

    _numlines = lines;
    _numcols  = cols;
    _linestarts = linestarts;

#ifdef USE4BITMODE
    wrotehalf = false;
#endif

    reinit();
  }

  void reinit()
  {
    // if the config is invalid, maybe we'll be getting it later.
    if(_data_pins[7].isNull())
      return;
      
    commandQueue.reset();
    modeQueue.reset();

    _rs_pin.setDirection(true);
    _rs_pin.setValue(false);
    _rw_pin.setDirection(true); 
    _rw_pin.setValue(false);
    _enable_pin.setDirection(true);
    _enable_pin.setValue(false);

#ifdef USE4BITMODE
    for (int i = 4; i < 8; i++)
#else    
    for (int i = 0; i < 8; i++) 
#endif
    {
      _data_pins[i].setValue(false);
      _data_pins[i].setDirection(true);
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
  // Hitachi docs say calls to this function are only honored 
  // during the specified initialization routine, not after the LCD is running.
  void setFunction(bool is8bit, bool is2line, bool fontselect)
  {
    command(0x20 |
           (is8bit      ? 0x10 : 0) |
           (is2line     ? 0x08 : 0) |
           (fontselect? 0x04 : 0));
  }           

  // Misleading name; sets address of next write.  does no writing.
  void writeCGRAM(uint8_t address) 
  {
    command(0x40 | address );
  }

  // Misleading name; sets address of next write.  does no writing.
  void writeDDRAM(uint8_t address)
  {
    command(0x80 | address );
  }

  // sets cursor position by col, row
  void setCursor(uint8_t col, uint8_t row)
  {
    int offset   = _linestarts[row] + col;
    writeDDRAM(offset);
  }

  void writeCustomChar(int idx, char const *data)
  {
    writeCGRAM(idx * 8);
    for(int x=0;x<8;x++)
    {
      write(*data);
      data++;
    }
    writeDDRAM(_linestarts[0]);
  }

  // TODO: replace these awful things.
  void write(char const value) { enqueue(value, true); }
  void write(char const *str) { for(int x=0;str[x]!=0;x++) write(str[x]); }
  void write(float n, signed char width, unsigned char prec)
  {
    static const char bufsize = 32;
    char buf[bufsize] = { 0 };
    dtostrf(n,width,prec,buf);
    write(buf);
  }
  void write(float n)
  {
    write(n, 5, 1);
  }
  void write(int32_t n)
  {
    static const char bufsize = 32;
    char buf[bufsize] = { 0 };
    ltoa(n,buf,10);
    write(buf);
  }
  void write(int16_t n) { write((int32_t)n); }
  void write(uint16_t n) {write((int32_t)n);}

  void write_P(const char* data)
  {
    char c;
    while ((c = pgm_read_byte(data++)))
      write(c);
  }


  void label(char const* str, float n) { write(str); write(n); } 
  void label(char const* str, char const* str2) { write(str); write(str2); }
  void label(char const* str, int32_t n) { write(str); write(n); }
  void label(char const* str, int n) { write(str); write((int32_t)n); }




  // handleUpdates MUST BE CALLED OFTEN
  // often means... at least as often as you'd like to see a new character
  // appear on the display.
  void handleUpdates()
  {
    // if the config is invalid, maybe we'll be getting it later.
    if(!initialized)
      return;

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
    initialized = true;
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

    // The timing on this never works right.  Maybe I need to go find different
    // source docs.
    wait(20);
    write4init(0b00100000); 
    wait(20);
    write4init(0b00100000); 
    wait(20);
    write4init(0b00100000); 
    wait(20);
    write4init(0b00100000); 
    initialized = true;
  }

#endif  


  Pin _rs_pin;
  Pin _rw_pin;
  Pin _enable_pin;
  Pin _data_pins[8];
public:
  // Set by Port, Pin
  void setRS(Port p, int bit) { _rs_pin = Pin(p, bit); }
  void setRW(Port p, int bit) { _rw_pin = Pin(p, bit); }
  void setE(Port p, int bit)  { _enable_pin = Pin(p, bit); }
  void setD(Port p, int bit, int D) { _data_pins[D] = Pin(p, bit); if(D == 7) reinit(); }
  // Set by Arduino Pin
  void setRS(int p) { setRS(ArduinoMap::getPort(p), ArduinoMap::getPinnum(p)); }
  void setRW(int p) { setRW(ArduinoMap::getPort(p), ArduinoMap::getPinnum(p)); }
  void setE(int p)  { setE(ArduinoMap::getPort(p), ArduinoMap::getPinnum(p)); }
  void setD(int D, int p) { setD(ArduinoMap::getPort(p), ArduinoMap::getPinnum(p), D); }
  //
  void setLineStart(int l, int s) { _linestarts[l] = s; }
  void setNumRows(int l) { _numlines = l; }
  void setNumCols(int l) { _numcols = l; }
  int getRows() { return _numlines; }
  int getCols() { return _numcols; }


private:
  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;

  uint8_t _initialized;

  uint8_t _numlines;
  uint8_t _numcols;
  uint8_t *_linestarts;

  bool initialized;

#ifdef USE4BITMODE
  bool wrotehalf;
#endif    

  uint8_t command_data[LCD_BUFFER_SIZE];
  bool    mode_data[LCD_BUFFER_SIZE];
  RingBufferT<uint8_t> commandQueue;
  RingBufferT<bool> modeQueue;
};

#endif // LIQUID_CRYSTAL_HH
