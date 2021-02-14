#include <BMI160Gen.h>
#include <Wire.h>

void PCA9546AD(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // PCA address is 0x70
  Wire.write(1 << bus);          // sEnd byte to select bus
  Wire.endTransmission();
}

void setup() {
  Wire.begin();
  PCA9546AD(2);
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  BMI160GenClass BMI160V2;
  // initialize device
  Serial.println("Initializing IMU device...");
  //BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
  for(int i = 0; i<6 ; i++){
      PCA9546AD(i);    
      BMI160.begin(BMI160GenClass::I2C_MODE);    
      //BMI160V2.begin(BMI160GenClass::I2C_MODE2);
  }
  //uint8_t dev_id = BMI160.getDeviceID();
  //Serial.print("DEVICE ID: ");
  //Serial.println(dev_id, HEX);

   // Set the accelerometer range to 250 degrees/second
  BMI160.setGyroRange(250);
  BMI160V2.setGyroRange(250);
  Serial.println("Initializing IMU device...done.");
}

void loop() {
  Serial.println("Probando");
  for(int j=0; j<=1; j++){
    for (int i =0; i<=2;i++){
    PCA9546AD(i);
    int gxRaw, gyRaw, gzRaw;         // raw gyro values
    float gx, gy, gz;
  
    // read raw gyro measurements from device
    if(j==0){
      BMI160.readGyro(gxRaw, gyRaw, gzRaw);  
    }
    else{
      BMI160V2.readGyro(gxRaw, gyRaw, gzRaw);  
    }
    // convert the raw gyro data to degrees/second
    gx = convertRawGyro(gxRaw);
    gy = convertRawGyro(gyRaw);
    gz = convertRawGyro(gzRaw);
  
    // display tab-separated gyro x/y/z values
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" Direccion V");
    Serial.print(j);
    Serial.print(": \t");
    Serial.print("g:\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.print(gz);
    Serial.println();
  
    delay(500);
    } 
  }
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}
