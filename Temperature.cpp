#include "Temperature.h"

#ifdef USE_MBIEC
Temperature::Temperature()
  : EC(MBIEC::Instance())
{
  ; // Nothing to do here.
}

void Temperature::update()
{
  EC.scan_input();
  EC.update();
}

bool Temperature::setHotend(uint16_t temp)
{
  return EC.setHotend(temp);
}

bool Temperature::setPlatform(uint16_t temp)
{
  return EC.setPlatform(temp);
}

uint16_t Temperature::getHotend()
{
  return EC.getHotend();
}

uint16_t Temperature::getPlatform()
{
  return EC.getPlatform();
}

uint16_t Temperature::getHotendST()
{
  return EC.getHotendST();
}

uint16_t Temperature::getPlatformST()
{
  return EC.getPlatformST();
}

#else
Temperature::Temperature()
  : hotend_therm(HOTEND_TEMP_PIN, 0),
    platform_therm(PLATFORM_TEMP_PIN, 1),
    hotend_heat(HOTEND_HEAT_PIN),
    platform_heat(PLATFORM_HEAT_PIN)
{
  hotend_therm.init();
  platform_therm.init();
  hotend_heat.setDirection(true); hotend_heat.setValue(false);
  platform_heat.setDirection(true); platform_heat.setValue(false);
  
}

void Temperature::update()
{
  // TODO: Which one of these actually gets to read the temperature is pretty random.
  // hopefully random enough to not worry about...
  hotend_therm.update();
  platform_therm.update();

  if(hotend_therm.getTemperature() >= hotend_setpoint)
  {
    hotend_heat.setValue(false);
  }
  else
  {
    hotend_heat.setValue(true);
  }


  if(platform_therm.getTemperature() >= platform_setpoint)
  {
    platform_heat.setValue(false);
  }
  else
  {
    platform_heat.setValue(true);
  }

}

bool Temperature::setHotend(uint16_t temp)
{
  hotend_setpoint = temp;
  return true;
}

bool Temperature::setPlatform(uint16_t temp)
{
  platform_setpoint = temp;
  return true;
}

uint16_t Temperature::getHotend()
{
  return hotend_therm.getTemperature();
}

uint16_t Temperature::getPlatform()
{
  return platform_therm.getTemperature();
}

uint16_t Temperature::getHotendST()
{
  return hotend_setpoint;
}

uint16_t Temperature::getPlatformST()
{
  return platform_setpoint;
}


#endif // USE_MBIEC
