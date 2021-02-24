/*
* The code is released under the GNU General Public License.
* Developed by www.codekraft.it
*/

//#include <BMI160.h>
//#include <CurieImu.h>
#include <BMI160Gen.h>
#include <Wire.h>

#include <MadgwickAHRS.h>
Madgwick madG;

#define M_PI   3.14159265358979323846264338327950288


//Set up a timer Variable
uint32_t timer;

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values
float yaw, pitch, roll;
float ypr[3];

int factor = 200; // variable by which to divide gyroscope values, used to control sensitivity
// note that an increased baud rate requires an increase in value of factor

const int ledPin = 13;      // activity LED pin
boolean blinkState = false; // state of the LED

void setup() {
  Wire.begin();
  PCA9546AD(2);
  Serial.begin(9600); // initialize Serial communication
  Serial.print("Starting...");
  BMI160.begin(BMI160GenClass::I2C_MODE2);
  BMI160.initialize();

  BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_500);
  BMI160.setGyroRate(BMI160_GYRO_RATE_100HZ);
  BMI160.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);


  //IMU device must be resting in a horizontal position for the following calibration procedure to work correctly!

  Serial.print("Starting Gyroscope calibration...");
  BMI160.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration...");
  BMI160.autoCalibrateXAccelOffset(0);
  BMI160.autoCalibrateYAccelOffset(0);
  BMI160.autoCalibrateZAccelOffset(1);
  Serial.println(" Done");

  //Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  BMI160.setGyroOffsetEnabled(true);
  BMI160.setAccelOffsetEnabled(true);
}

void loop() {        // Exetutes this part of the code every 10 miliseconds -> 100Hz
    PCA9546AD(2);
    timer = micros();    // Reset the timer  
    // read raw accel/gyro measurements from device
    ax = BMI160.getAccelerationX();
    ay = BMI160.getAccelerationY();
    az = BMI160.getAccelerationZ();
    gx = BMI160.getRotationX();
    gy = BMI160.getRotationY();
    gz = BMI160.getRotationZ();
  
    // use function from MagdwickAHRS.h to return quaternions
    madG.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az);
      
    ypr[0] = madG.getYaw()* 180.0 / M_PI;
    ypr[2] = madG.getRoll()* 180.0 / M_PI;
    ypr[1] = madG.getPitch()* 180.0 / M_PI;
    
    //serialPrintFloatArr(ypr,3);
    Serial.print('\n');
    Serial.print("roll:\t");
    Serial.print(ypr[0]);
    Serial.print("\t pitch:\t");
    Serial.print(ypr[1]);
    Serial.print("\t yaw:\t");
    Serial.println(ypr[2]);

    delay(100);
}

void PCA9546AD(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // PCA address is 0x70
  Wire.write(1 << bus);          // sEnd byte to select bus
  Wire.endTransmission();
}
