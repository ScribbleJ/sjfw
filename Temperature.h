#ifndef _TEMPERATURE_H_
#define _TEMPERATURE_H_
/* Class for managing temperatures
 * (c) 2011, Christopher "ScribbleJ" Jansen
 */

#include "config.h"

#ifdef USE_MBIEC
#include "MBIEC.h"
#else
#include "Thermistor.h"
#include "ArduinoMap.h"
#endif


#include <stdint.h>
#include "AvrPort.h"
#include "Host.h"


// TODO: Should support N probes/heaters instead of 2.
class Temperature
{
  public:
    static Temperature& Instance() { static Temperature instance; return instance; }
  private:
    explicit Temperature();
    Temperature(Temperature&);
    Temperature& operator=(Temperature&);

    unsigned long report_m;
    unsigned long report_l;
    Host *report_h;
  public:
    void update();
    // Set temperature of hotend and platform
    bool setHotend(uint16_t temp);
    bool setPlatform(uint16_t temp);
    // Get current temperatures
    uint16_t getHotend();
    uint16_t getPlatform();
    // Get current temperature settings
    uint16_t getHotendST();
    uint16_t getPlatformST();
    void changeReporting(unsigned long millis, Host &out)
    {
      report_h = &out;
      report_m = millis;
    }
    void doreport();

  private:
#ifdef USE_MBIEC
    MBIEC& EC;
    bool fanon;
public:
    // Changes FET pin
    void changePinHotend(Port p, int pin) { return; }
    void changePinPlatform(Port p, int pin) { return; }
    // Changes Thermistor pin
    void changePinHotend(int pin) { return; }
    void changePinPlatform(int pin) { return; }
    // Changes FET pin
    void changeOutputPinHotend(int pin) { return; }
    void changeOutputPinPlatform(int pin) { return; }
    // Changes Temperature Table
    void changeTempTable(int16_t adc_val, int16_t temp_val, int8_t which) { return; }
    // Toggles fan
    void togglefan() { fanon = !fanon; while(!EC.dofanreq(fanon)); }
    bool setFan(bool on) { fanon = on; return EC.dofanreq(fanon); }

#else
    Thermistor hotend_therm;
    Thermistor platform_therm;

    uint8_t hotend_setpoint;
    uint8_t platform_setpoint;

    Pin hotend_heat;
    Pin platform_heat;
public:
    // Changes FET pin by Port, pin
    void changePinHotend(Port p, int pin)
    {
      hotend_heat.setValue(false);
      hotend_heat = Pin(p, pin);
      hotend_heat.setDirection(true);
      hotend_heat.setValue(false);
    }
    void changePinPlatform(Port p, int pin)
    {
      platform_heat.setValue(false);
      platform_heat = Pin(p, pin);
      platform_heat.setDirection(true);
      platform_heat.setValue(false);
    }

    // changes thermistor pin
    void changePinHotend(int pin) { hotend_therm.changePin(pin); }
    void changePinPlatform(int pin) { platform_therm.changePin(pin); }

    // Changes FET pin by Arduino naming
    void changeOutputPinHotend(int pin) 
    {
      changePinHotend(ArduinoMap::getPort(pin), ArduinoMap::getPinnum(pin));
    }
    void changeOutputPinPlatform(int pin) 
    { 
      changePinPlatform(ArduinoMap::getPort(pin), ArduinoMap::getPinnum(pin));
    }
    void changeTempTable(int16_t adc_val, int16_t temp_val, int8_t which)
    {
      hotend_therm.changeTable(adc_val, temp_val, which);
    }




#endif

};

extern Temperature& TEMPERATURE;

#endif // _TEMPERATURE_H_
