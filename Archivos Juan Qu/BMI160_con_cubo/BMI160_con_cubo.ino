#include <Wire.h>
#include <math.h>
#include <BMI160Gen.h>
#include <MadgwickAHRS.h>

#define PRINT_PROCESSING
#define BAUDRATE  115200
#define SENSE_RATE   100
#define GYRO_RANGE   250
#define ACCL_RANGE     2

#define deg_to_rad(a) (a/180*M_PI)
#define rad_to_deg(a) (a/M_PI*180)

static float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
static float comp_roll = 0, comp_pitch = 0;
static unsigned long last_mills = 0;
String messageToSend = "";
Madgwick madgwick;

float convertRawGyro(int gRaw) {
  // ex) if the range is +/-500 deg/s: +/-32768/500 = +/-65.536 LSB/(deg/s)
  float lsb_omega = float(0x7FFF) / GYRO_RANGE;
  return gRaw / lsb_omega;  // deg/sec
}

float convertRawAccel(int aRaw) {
  // ex) if the range is +/-2g ; +/-32768/2 = +/-16384 LSB/g
  float lsb_g = float(0x7FFF) / ACCL_RANGE;
  return aRaw / lsb_g;
}

void PCA9546AD(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // PCA address is 0x70
  Wire.write(1 << bus);          // sEnd byte to select bus
  Wire.endTransmission();
}

BMI160GenClass BMI160V2;


void setup() {
  
  Serial.begin(115200);
  Wire.begin(); 
  PCA9546AD(2); //checar anomalia del Multi tomorrow :v9
  BMI160.begin(BMI160GenClass::I2C_MODE);
  BMI160V2.begin(BMI160GenClass::I2C_MODE2);
 
  madgwick.begin(SENSE_RATE);

  //0x68
  BMI160.setGyroRate(SENSE_RATE);
  BMI160.setAccelerometerRate(SENSE_RATE);
  BMI160.setGyroRange(GYRO_RANGE);
  BMI160.setAccelerometerRange(ACCL_RANGE);

  BMI160.autoCalibrateGyroOffset();
  //Test 1 callibrate ...
  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 1);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 0);

  //0x69
  BMI160V2.setGyroRate(SENSE_RATE);
  BMI160V2.setAccelerometerRate(SENSE_RATE);
  BMI160V2.setGyroRange(GYRO_RANGE);
  BMI160V2.setAccelerometerRange(ACCL_RANGE);

  BMI160V2.autoCalibrateGyroOffset();
  //Test 1 callibrate ...
  BMI160V2.autoCalibrateAccelerometerOffset(X_AXIS, 1);
  BMI160V2.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160V2.autoCalibrateAccelerometerOffset(Z_AXIS, 0);
  
  delay(300);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  int i=0;
  int rawXAcc[6], rawYAcc[6], rawZAcc[6]; // x, y, z
  float accX[6], accY[6], accZ[6];
  float rad_a_roll[6];
  float rad_a_pitch[6];
  float accl_roll[6];
  float accl_pitch[6];
  int rawRoll[6], rawPitch[6], rawYaw[6]; // roll, pitch, yaw
  float omega_roll[6];
  float omega_pitch[6];
  float omega_yaw[6];

  
  float gyro_roll[6];
  float gyro_pitch[6];
  float gyro_yaw[6];
  
  // Complementary filter data
  float comp_roll[6];
  float comp_pitch[6];

  float madw_roll[6] ;
  float madw_pitch[6];
  float madw_yaw[6];
  double dt;
  int j=0;
    while(j <= 0){
      int i=0;
      while(i <= 0){
      //int j=0;
      PCA9546AD(i);
      Serial.print("Tomando Sensor ");
      Serial.print(i);
      Serial.print(" direccion ");
      Serial.print(j);
      Serial.print(" : ");
      // read raw accl measurements BMI
     // int rawXAcc, rawYAcc, rawZAcc; // x, y, z
        if(j == 0){
          BMI160.readAccelerometer(rawXAcc[i], rawYAcc[i], rawZAcc[i]);    
          }else{
            BMI160V2.readAccelerometer(rawXAcc[i], rawYAcc[i], rawZAcc[i]);    
            }
        
        //BMI160V2.readAccelerometer(rawXAcc, rawYAcc, rawZAcc);  
      
      
      accX[i] = convertRawAccel(rawXAcc[i]);
      accY[i] = convertRawAccel(rawYAcc[i]);
      accZ[i] = convertRawAccel(rawZAcc[i]);
    
       rad_a_roll[i] = atan2(accY[i], accZ[i]);
       rad_a_pitch[i] = atan2(-accX[i], sqrt(accY[i]*accY[i] + accZ[i]*accZ[i]));
       accl_roll[i] = rad_to_deg(rad_a_roll[i]);
       accl_pitch[i] = rad_to_deg(rad_a_pitch[i]);
      
      // read raw gyro
      
      //BMI160.readGyro(rawRoll, rawPitch, rawYaw);
      if(j==0 ){
          BMI160.readGyro(rawRoll[i], rawPitch[i], rawYaw[i]);  
        }else{
         BMI160V2.readGyro(rawRoll[i], rawPitch[i], rawYaw[i]);  
          }
        

        //BMI160V2.readGyro(rawRoll, rawPitch, rawYaw);  
      
      omega_roll[i]  = convertRawGyro(rawRoll[i]);
      omega_pitch[i] = convertRawGyro(rawPitch[i]);
      omega_yaw[i]   = convertRawGyro(rawYaw[i]);

      do{
      unsigned long cur_mills = micros();
      unsigned long duration = cur_mills - last_mills;
      last_mills = cur_mills;
      dt = duration / 1000000.0; // us->s  
      }while(dt < 0.1);
      //if (dt > 0.1) return;
    
      // Gyro data
      gyro_roll[i]  += omega_roll[i]  * dt; // (ms->s) omega x time = degree
      gyro_pitch[i] += omega_pitch[i] * dt;
      gyro_yaw[i]   += omega_yaw[i]   * dt;
      
      // Complementary filter data
      comp_roll[i]  = 0.93 * (comp_roll[i]  + omega_roll[i]  * dt) + 0.07 * accl_roll[i]; 
      comp_pitch[i] = 0.93 * (comp_pitch[i] + omega_pitch[i] * dt) + 0.07 * accl_pitch[i]; 
    
      // Madgwick filter data
      madgwick.updateIMU2(omega_roll[i], omega_pitch[i], omega_yaw[i], accX[i], accY[i], accZ[i], dt);
      //madgwick.updateIMU(omega_roll, omega_pitch, omega_yaw, accX, accY, accZ);
       madw_roll[i]  = madgwick.getRoll();
       madw_pitch[i] = madgwick.getPitch();
       madw_yaw[i]   = madgwick.getYaw();
    
      /*Serial.print(accl_roll);
      Serial.print(",");
      Serial.print(accl_pitch);
      Serial.print(",");
      Serial.print(madw_yaw);
      Serial.print(",");
      Serial.print(gyro_roll);
      Serial.print(",");
      Serial.print(gyro_pitch);
      Serial.print(",");
      Serial.print(madw_yaw);
      Serial.print(",");*/
      
      /*Serial.print(madw_roll);
      Serial.print(",");
      Serial.print(madw_pitch);
      Serial.print(",");
      Serial.print(madw_yaw);
      Serial.print(",");*/
      /*Serial.print(madw_roll);
      Serial.print(" , ");
      Serial.print(madw_pitch);
      Serial.print(" , ");
      Serial.println(madw_yaw);
      */
      Serial.print(int(madw_roll[i]));
      Serial.print(" , ");
      Serial.print(int(madw_pitch[i]));
      Serial.print(" , ");
      Serial.println(int(madw_yaw[i]));
      
      delay(100);
      i += 1;
    }
    j += 1;
    }
}
