#ifndef BATTERY_MANAGER_HPP
#define BATTERY_MANAGER_HPP

#include <math.h>
#include "MediaMovil.hpp"

class BatteryManager{
private:
  const int sensorPin;
  const double v1 = 3.3;
  MediaMovil<double, 30> mm_tension;
public:
  BatteryManager(const int& _sensorPin) :
  sensorPin(_sensorPin),
  mm_tension(read())
  {
    //
  }
  double read(){
    const double sensorValue = analogRead(sensorPin);
    const double v = ((sensorValue) * (v1)) / 4095.0;
    const double v_2 = 2 * v;
    const double media_tension = mm_tension.pass(v_2);
    const double vpor = min(double(100), ((100 * media_tension) / 4.2));
    return vpor;
  }
} *battery_manager;

#endif BATTERY_MANAGER_HPP
