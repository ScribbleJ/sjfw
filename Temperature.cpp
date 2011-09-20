#include "Temperature.h"

#include "Time.h"

void Temperature::doreport()
{
  if(report_m == 0)
    return;
  unsigned long now = millis();
  if(report_l + report_m < now)
  {
    report_l = now;
    (*report_h).labelnum("T:",getHotend(),false);
    (*report_h).labelnum(" B:",getPlatform());
  }
}

#ifdef USE_MBIEC
Temperature::Temperature()
  : EC(MBIEC::Instance())
{
  report_m = 0;
  report_l = 0;
  report_h = 0;
  fanon = false;
}

void Temperature::update()
{
  EC.scan_input();
  EC.update();
  doreport();
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
  report_m = 0;
  report_l = 0;
  report_h = 0;
  hotend_therm.init();
  platform_therm.init();
  hotend_heat.setDirection(true); hotend_heat.setValue(false);
  platform_heat.setDirection(true); platform_heat.setValue(false);
}

#define MIN_TEMP_INTERVAL 20
void Temperature::update()
{
  unsigned long now = millis();
  static unsigned long lasttime = now;
  if(now - lasttime > MIN_TEMP_INTERVAL)
  {
    static bool checkwhich=false;
    if(checkwhich)
      hotend_therm.update();
    else
      platform_therm.update();
    lasttime = now;
    checkwhich = !checkwhich;
  }


  if(!hotend_heat.isNull())
  {
    if(hotend_therm.getTemperature() >= hotend_setpoint)
    {
      hotend_heat.setValue(false);
    }
    else
    {
      hotend_heat.setValue(true);
    }
  }

  if(!platform_heat.isNull())
  {
    if(platform_therm.getTemperature() >= platform_setpoint)
    {
      platform_heat.setValue(false);
    }
    else
    {
      platform_heat.setValue(true);
    }
  }

  doreport();

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
