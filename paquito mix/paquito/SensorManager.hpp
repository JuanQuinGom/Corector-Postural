#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include "Vector3.hpp"
#include <Vector.h>
#include "MediaMovil.hpp"
#include <tuple>
#include <BMI160Gen.h>

#define SENSE_RATE   100
#define GYRO_RANGE   250
#define ACCL_RANGE     2

class IndividualSensorManager {
  private:
    BMI160GenClass bmi;
    BMI160GenClass bmiV2;
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
    void select_bus() {
      Wire.beginTransmission(0x70); // transmit to device #8
      Wire.write(busToOpen);              // sends one byte
      Wire.endTransmission();    // stop transmitting
    }
  public:
    IndividualSensorManager(const int _sensor)
      : sensor(_sensor),
        turno(_sensor % 2),
        busToOpen(1 << _sensor)
    {
      select_bus();
      bmi.begin(BMI160GenClass::I2C_MODE);
      //bmi.begin(BMI160GenClass::I2C_MODE, Wire, 0x68, 2);
      //bmi.begin(BMI160GenClass::I2C_MODE, Wire, 0x69-turno, 2);

      bmi.setGyroRate(SENSE_RATE);
      bmi.setAccelerometerRate(SENSE_RATE);
      
      bmi.setGyroRange(250);
      bmi.setAccelerometerRange(2);
      // bmi.setAccelerometerRange(0X05);
      bmi.autoCalibrateGyroOffset();
      //Test 1 callibrate ...
      bmi.autoCalibrateAccelerometerOffset(X_AXIS, 1);//1
      bmi.autoCalibrateAccelerometerOffset(Y_AXIS, 0);//0
      bmi.autoCalibrateAccelerometerOffset(Z_AXIS, 0);//0
    }
    //std::tie
    std::tuple<Vector3<int>, Vector3<int>> read_sensor() {
      select_bus();
      int ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
      bmi.readMotionSensor(ax, ay, az, gx, gy, gz);
      return std::forward_as_tuple(
      Vector3<int> {
        ax,
        ay,
        az
      },
      Vector3<int> {
        gx,
        gy,
        gz
      }
             );
    }
    int get_sensor() {
      return sensor;
    }
};

std::vector<IndividualSensorManager*> isms;

#endif //SENSOR_MANAGER_HPP
