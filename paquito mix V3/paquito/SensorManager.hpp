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
    //BMI160GenClass BMI160;
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
      Serial.println("Testing..");
      bmi.begin(BMI160GenClass::I2C_MODE);
      Serial.println("End Testing..");
      bmi.initialize();
      //bmi.begin(BMI160GenClass::I2C_MODE, Wire, 0x68, 2);
      //bmi.begin(BMI160GenClass::I2C_MODE, Wire, 0x69-turno, 2);

      bmi.setFullScaleGyroRange(BMI160_GYRO_RANGE_500);
      bmi.setGyroRate(BMI160_GYRO_RATE_100HZ);
      bmi.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
    
    
      //IMU device must be resting in a horizontal position for the following calibration procedure to work correctly!
    
      Serial.print("Starting Gyroscope calibration...");
      bmi.autoCalibrateGyroOffset();
      Serial.println(" Done");
      Serial.print("Starting Acceleration calibration...");
      bmi.autoCalibrateXAccelOffset(0);
      bmi.autoCalibrateYAccelOffset(0);
      bmi.autoCalibrateZAccelOffset(1);
      Serial.println(" Done");
    
      //Serial.println("Enabling Gyroscope/Acceleration offset compensation");
      bmi.setGyroOffsetEnabled(true);
      bmi.setAccelOffsetEnabled(true);
    }
    //std::tie
    std::tuple<Vector3<int>, Vector3<int>> read_sensor() {
      select_bus();
      int ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
      ax = bmi.getAccelerationX();
      ay = bmi.getAccelerationY();
      az = bmi.getAccelerationZ();
      gx = bmi.getRotationX();
      gy = bmi.getRotationY();
      gz = bmi.getRotationZ();
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
