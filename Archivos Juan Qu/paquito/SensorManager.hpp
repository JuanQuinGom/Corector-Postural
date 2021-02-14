#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include "Vector3.hpp"
#include <vector>
#include "MediaMovil.hpp"
#include <tuple>
#include <BMI160Gen.h>

class IndividualSensorManager{
private:
  BMI160GenClass bmi;
  const int sensor;
  const int turno;
  const uint8_t busToOpen;
  MediaMovil<int, 20>
    mm_ax,
    mm_ay,
    mm_az,
    mm_gx,
    mm_gy,
    mm_gz;
  void select_bus(){
    Wire.beginTransmission(0x70); // transmit to device #8
    Wire.write(busToOpen);              // sends one byte
    Wire.endTransmission();    // stop transmitting
  }
public:
  IndividualSensorManager(const int _sensor, int _type_sensor)
  : sensor(_sensor),
  turno(_sensor%2),
  busToOpen(1<<_sensor)
  {
    select_bus();
    if(_type_sensor == 1){
      bmi.begin(BMI160GenClass::I2C_MODE);  
    }
    else{
      bmi.begin(BMI160GenClass::I2C_MODE2);    
    }
    //bmi.begin(BMI160GenClass::I2C_MODE, Wire, 0x68, 2);
    //bmi.begin(BMI160GenClass::I2C_MODE, Wire, 0x69-turno, 2);
    bmi.setGyroRange(250);
    bmi.setAccelerometerRange(2);
    // bmi.setAccelerometerRange(0X05);
  }
  //std::tie
  std::tuple<Vector3<int>, Vector3<int>> read_sensor(){
      select_bus();
      int ax, ay, az, gx, gy, gz;
      bmi.readMotionSensor(ax, ay, az, gx, gy, gz);  
      return std::forward_as_tuple(
        Vector3<int>{
          ax,
          ay,
          az
        },
        Vector3<int>{
          gx,
          gy,
          gz
        }
      );
  }
  int get_sensor(){
    return sensor;
  }
};

std::vector<IndividualSensorManager*> isms;

#endif //SENSOR_MANAGER_HPP
