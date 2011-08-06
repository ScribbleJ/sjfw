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
#endif
#include <stdint.h>
#include "AvrPort.h"


// TODO: Should support N probes/heaters instead of 2.
class Temperature
{
  public:
    static Temperature& Instance() { static Temperature instance; return instance; }
  private:
    explicit Temperature();
    Temperature(Temperature&);
    Temperature& operator=(Temperature&);

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


  private:
#ifdef USE_MBIEC
    MBIEC& EC;
public:
    void changePinHotend(Port& p, int pin) { return; }
    void changePinPlatform(Port& p, int pin) { return; }
    void changePinHotend(int pin) { return; }
    void changePinPlatform(int pin) { return; }

#else
    Thermistor hotend_therm;
    Thermistor platform_therm;

    uint8_t hotend_setpoint;
    uint8_t platform_setpoint;

    Pin hotend_heat;
    Pin platform_heat;
public:
    void changePinHotend(Port& p, int pin)
    {
      hotend_heat.setValue(false);
      hotend_heat = Pin(p, pin);
      hotend_heat.setDirection(true);
      hotend_heat.setValue(false);
    }
    void changePinPlatform(Port& p, int pin)
    {
      platform_heat.setValue(false);
      platform_heat = Pin(p, pin);
      platform_heat.setDirection(true);
      platform_heat.setValue(false);
    }

    void changePinHotend(int pin) { hotend_therm.changePin(pin); }
    void changePinPlatform(int pin) { platform_therm.changePin(pin); }
#endif

};

extern Temperature& TEMPERATURE;

#endif // _TEMPERATURE_H_
