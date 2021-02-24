#ifndef CONVERT_RAW_GYRO
#define CONVERT_RAW_GYRO

#include "Vector3.hpp"

float convertRawGyro(const int& gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  //float g = (gRaw * 250.0) / 32768.0;
  //return g;

  // ex) if the range is +/-500 deg/s: +/-32768/500 = +/-65.536 LSB/(deg/s)
  float lsb_omega = float(0x7FFF) / GYRO_RANGE;
  return gRaw / lsb_omega;  // deg/sec
}

Vector3<float> convertRawGyro(const Vector3<int>& gRaw) {
  Vector3<float> r;
  r.x = convertRawGyro(gRaw.x);
  r.y = convertRawGyro(gRaw.y);
  r.z = convertRawGyro(gRaw.z);
  return r;
}

float convertRawAccel(const int& aRaw) {
  // ex) if the range is +/-2g ; +/-32768/2 = +/-16384 LSB/g
  float lsb_g = float(0x7FFF) / ACCL_RANGE;
  return aRaw / lsb_g;
}

Vector3<float> convertRawAccel(const Vector3<int>& gRaw) {
  Vector3<float> r;
  r.x = convertRawAccel(gRaw.x);
  r.y = convertRawAccel(gRaw.y);
  r.z = convertRawAccel(gRaw.z);
  return r;
}

#endif CONVERT_RAW_GYRO
