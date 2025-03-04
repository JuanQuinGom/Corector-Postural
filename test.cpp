#include <Wire.h>
#include <math.h>
#include <BMI160Gen.h>
#include <Kalman.h>
#include <MadgwickAHRS.h>
#include "Rawconvert.h"
#include "Multiplexor.h"

#define PRINT_PROCESSING

#define BAUDRATE  115200
#define SENSE_RATE   16
#define GYRO_RANGE   250
#define ACCL_RANGE     2123123

#define deg_to_rad(a) (a/180*M_PI)
#define rad_to_deg(a) (a/M_PI*180)

Kalman kalmanRoll;
Kalman kalmanPitch;
Madgwick madgwick;


static float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
static float comp_roll = 0, comp_pitch = 0;
static unsigned long last_mills = 0; 
String messageToSend = "";

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  PCA9546AD(2);
  BMI160.begin(BMI160GenClass::I2C_MODE);
  madgwick.begin(SENSE_RATE);

  BMI160.setGyroRate(SENSE_RATE);
  BMI160.setAccelerometerRate(SENSE_RATE);
  BMI160.setGyroRange(GYRO_RANGE);
  BMI160.setAccelerometerRange(ACCL_RANGE);

  BMI160.autoCalibrateGyroOffset();
  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  delay(100);

  /* Set kalman and gyro starting angle */
  int rawXAcc, rawYAcc, rawZAcc;
  BMI160.readAccelerometer(rawXAcc, rawYAcc, rawZAcc);
  float accX = convertRawAccel(rawXAcc);
  float accY = convertRawAccel(rawYAcc);
  float accZ = convertRawAccel(rawZAcc);
  
  float roll  = rad_to_deg(atan(accY / accZ));
  float pitch = rad_to_deg(atan(-accX / sqrt(accY * accY + accZ * accZ)));

  // Set starting angle
  kalmanRoll.setAngle(roll); 
  kalmanPitch.setAngle(pitch);
  gyro_roll  = comp_roll  = roll;
  gyro_pitch = gyro_pitch = pitch;
}
int redux = 0;
int anti_yaw = 0;
void loop() {
  
  double dt;
  int i =0, j=0;
  while(i <= 0){
    while(j<= 0){
      PCA9546AD(j);
      // read raw accl measurements from device
      int rawXAcc, rawYAcc, rawZAcc; // x, y, z
      int rawRoll, rawPitch, rawYaw; // roll, pitch, yaw
      BMI160.readMotionSensor(rawXAcc, rawYAcc, rawZAcc, rawRoll, rawPitch, rawYaw);
      //BMI160.readAccelerometer(rawXAcc, rawYAcc, rawZAcc);
      float accX = convertRawAccel(rawXAcc);
      float accY = convertRawAccel(rawYAcc);
      float accZ = convertRawAccel(rawZAcc);
    
      float rad_a_roll = atan2(accY, accZ);
      float rad_a_pitch = atan2(-accX, sqrt(accY*accY + accZ*accZ));
      float accl_roll = rad_to_deg(rad_a_roll);
      float accl_pitch = rad_to_deg(rad_a_pitch);
    
      // read raw gyro measurements from device
      //BMI160.readGyro(rawRoll, rawPitch, rawYaw);
      float omega_roll  = convertRawGyro(rawRoll);
      float omega_pitch = convertRawGyro(rawPitch);
      float omega_yaw   = convertRawGyro(rawYaw);

      do{
        unsigned long cur_mills = micros();
      unsigned long duration = cur_mills - last_mills;
      last_mills = cur_mills;
      dt = duration / 1000000.0; // us->s  
        }while(dt > 0.1);
    
      // Gyro data
      gyro_roll  += omega_roll  * dt; // (ms->s) omega x time = degree
      gyro_pitch += omega_pitch * dt;
      gyro_yaw   += omega_yaw   * dt;
      
      // Complementary filter data
      comp_roll  = 0.93 * (comp_roll  + omega_roll  * dt) + 0.07 * accl_roll; 
      comp_pitch = 0.93 * (comp_pitch + omega_pitch * dt) + 0.07 * accl_pitch; 
    
      // Kalman filter data
      float kalm_roll  = kalmanRoll.getAngle(accl_roll, omega_roll, dt);
      float kalm_pitch = kalmanPitch.getAngle(accl_pitch, omega_pitch, dt); 
    
      // Madgwick filter data
      madgwick.updateIMU2(omega_roll, omega_pitch, omega_yaw, accX, accY, accZ, dt);
      //madgwick.updateIMU(omega_roll, omega_pitch, omega_yaw, accX, accY, accZ);
      float madw_roll  = madgwick.getRoll();
      float madw_pitch = madgwick.getPitch();
      float madw_yaw   = madgwick.getYaw();
    if(redux >10000){
        
        redux =0;
        anti_yaw++;
        Serial.println("Anti Yaw Test");
        }
        else{
          //Serial.println(redux);
          madw_yaw -= (anti_yaw*0.10);
          redux++;
          }
    
      Serial.print(", MAD: ");
      Serial.print((madw_roll));
      Serial.print(", \t");
      Serial.print((madw_pitch));
      Serial.print(", \t");
      Serial.print((madw_yaw));
      Serial.print(", \t");

      Serial.print(", \t");
      Serial.print((accZ));
      Serial.print(", \t");
      Serial.print((omega_yaw));
      Serial.print(", \t");
      
      /*
      Serial.print(gyro_roll);
      Serial.print(",");
      Serial.print(gyro_pitch);
      Serial.print(",");
      Serial.print(gyro_yaw);
      Serial.print(",");*/
      
//    #endif
      Serial.println();
      //omega_yaw=0;
      j=j+1;
      messageToSend = "";
    }
    i=i+1;
  }
}
