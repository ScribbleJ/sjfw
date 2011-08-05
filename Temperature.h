#ifndef _TEMPERATURE_H_
#define _TEMPERATURE_H_

#include "config.h"
#ifdef USE_MBIEC
#include "MBIEC.h"
#else
#include "Thermistor.h"
#endif
#include <stdint.h>


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
#else
    Thermistor hotend_therm;
    Thermistor platform_therm;

    uint8_t hotend_setpoint;
    uint8_t platform_setpoint;

    Pin hotend_heat;
    Pin platform_heat;
#endif

};

extern Temperature& TEMPERATURE;

#endif // _TEMPERATURE_H_
