#include <Wire.h>
#include <stdint.h>
#include <math.h>
//#include "BleManager.hpp"
#include "SensorManager.hpp"
//#include "convertRawGyro.hpp"
#include "BatteryManager.hpp"
#include "Rate.hpp"
#include <MadgwickAHRS.h>
#include <map>
#include "BluetoothSerial.h"
#include <WiFi.h>
#include <utility>

#define JSON

BluetoothSerial SerialBT;
void PCA9546AD(uint8_t);
float convertRawGyro(int);
float convertRawAccel(int);

// https://naylampmechatronics.com/blog/45_Tutorial-MPU6050-Aceler%C3%B3metro-y-Giroscopio.html

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define DBGMSG(MSG) Serial.println(\
                                   String() + __FILE__ + ": " + __FUNCTION__  + "(): " + __LINE__ + \
                                   MSG \
                                  );

// #define PRINT_LAG
#define PRINT_SENSOR 0
#define N_SENSORS 3

#define BT_BAUDRATE 38400
#define CONNECTION_PIN 4

#define IMU_ENABLE_PIN 23 //este pin es el que debe usarse para alimentar los acelerometros (bien sea desde el propio pin, o con un MOSFET)

Rate rate(100); // frecuencia de muestreo
Rate ble_rate(10); // frecuencia de envio (5 envios por segundo)

#define BT_PACKET_RATE 10

//std::map<int, Madgwick> madgwick_filters;
Madgwick filter;

/*template<class type_datatype>
class Comparer {

    private:
    type_datatype initial;
    int counter = 0;
    const int repetitions;

    public:
    Comparer (int _repetitions) : repetitions(_repetitions){

    }
    bool isAllTheSame (type_datatype actual){
        if (actual == initial){
            return ++counter >= repetitions -1;
        }
        else {
            initial = actual;
            return counter = 0;
        }
    }
};*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String bt_name = "SpineCare_" + String(mac[3]) + String(mac[4]) + String(mac[5]);
  SerialBT.begin(bt_name);
  pinMode(CONNECTION_PIN, INPUT);
  battery_manager = new BatteryManager(32);
  Wire.begin();
  PCA9546AD(2);
  BMI160.begin(BMI160GenClass::I2C_MODE);
  BMI160V2.begin(BMI160GenClass::I2C_MODE2);
  filter.begin(25);
  BMI160.setGyroRange(250);
  BMI160V2.setGyroRange(250);
  
  /*for (int i = 0; i < N_SENSORS; i++) {
    Serial.println("Sensor :" + String(i));
    isms.emplace_back(new IndividualSensorManager(i));
    madgwick_filters[i].begin(rate.get_rate());
    delay(500);
  }*/
}

auto milis_anterior = millis();
auto milis_actual = millis();

float ang_x_prev, ang_y_prev;

struct _sending_data {
  uint16_t bateria;
  struct {
    int16_t x;
    int16_t y;
    int16_t z;
  } vec[N_SENSORS];
} sending_data;


//Comparer<decltype(std::declval<IndividualSensorManager>().read_sensor())> comparer(5);

void loop() {
  static unsigned long last_send = 0;
  /*if (
    //digitalRead(CONNECTION_PIN)
    1
    ) {*/

    rate.sleep();
    // const uint8_t bateria = battery_manager->read();
    // Serial.println(String("bateria: ") + bateria);
    auto ble_send = ble_rate.has_passed();
    for(int j=0; j<=1; j++){
      for (int i =0; i<=2;i++){
      /*auto& madgwick = madgwick_filters[ism->get_sensor()];
      Vector3<int> gyro_raw, accel_raw;
      auto read_sensor = std::tie(accel_raw, gyro_raw) = ism->read_sensor();
      auto gyro_vector = convertRawGyro(gyro_raw);
      auto accel_vector = convertRawAccel(accel_raw);*/
      
     /* if (comparer.isAllTheSame(read_sensor)){
        digitalWrite(IMU_ENABLE_PIN, HIGH);
        delay(500);
        digitalWrite(IMU_ENABLE_PIN, LOW);
        ESP.restart();
      }*/
      PCA9546AD(i);
      int gxRaw, gyRaw, gzRaw, axRaw, ayRaw, azRaw;         // raw gyro values
      float gx, gy, gz,ax,ay,az;
      float roll, pitch, heading;
      // read raw gyro measurements from device
      if(j==0){
        BMI160.readMotionSensor(gxRaw, gyRaw, gzRaw, axRaw, ayRaw, azRaw);  
      }
      else{
        BMI160V2.readMotionSensor(gxRaw, gyRaw, gzRaw, axRaw, ayRaw, azRaw);  
      }
      gx = convertRawGyro(gxRaw);
      gy = convertRawGyro(gyRaw);
      gz = convertRawGyro(gzRaw);
      ax = convertRawAccel(axRaw);
      ay = convertRawAccel(ayRaw);
      az = convertRawAccel(azRaw);
      filter.updateIMU(gx, gy, gz, ax, ay, az);
      //madgwick.computeAngles();
      // Serial.println("b4 ble_send");
      //if (ble_send) {
        // Serial.println("ble_send");
        /*sending_data.vec[ism->get_sensor()].x = madgwick.getRoll();
        sending_data.vec[ism->get_sensor()].y = madgwick.getPitch();
        sending_data.vec[ism->get_sensor()].z = madgwick.getYaw();
       /* if(sending_data.vec[ism->get_sensor()].x<0) sending_data.vec[ism->get_sensor()].x += 360;
        if(sending_data.vec[ism->get_sensor()].y<0) sending_data.vec[ism->get_sensor()].y += 360;
        if(sending_data.vec[ism->get_sensor()].z<0) sending_data.vec[ism->get_sensor()].z += 360;*/
        //Serial.println("Sensor: " + String(ism->get_sensor()) + ":  \tRoll= " + String(sending_data.vec[ism->get_sensor()].x)  + " \tPitch= "  + String(sending_data.vec[ism->get_sensor()].y)+ " \Yaw= " + String(sending_data.vec[ism->get_sensor()].z));
      //}

     /* if (ism->get_sensor() == PRINT_SENSOR && (Serial))
        Serial.println(
          String("") +
          sending_data.vec[ism->get_sensor()].x + " " +
          sending_data.vec[ism->get_sensor()].y + " " +
          sending_data.vec[ism->get_sensor()].z
        );*/

         // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" Direccion V");
    Serial.print(j);
    Serial.print("x: ");
    Serial.print(heading);
    Serial.print(" , y: ");
    Serial.print(pitch);
    Serial.print(" , z: ");
    Serial.println(roll);
      } 
    }
    
    /*/if (
      1 &&
      //digitalRead(CONNECTION_PIN) && 
    (millis() - last_send > (1000 / BT_PACKET_RATE))){
      sending_data.bateria = battery_manager->read();
      //Serial.println(sending_data.bateria);
      //Serial.println("enviando mensaje");
      #ifndef JSON
      SerialBT.write((uint8_t*)&sending_data, sizeof(sending_data));
      #else
      String json = "{\n\"battery\": " + String(sending_data.bateria) + ",\n \"imu\":[\n";
      for (int i = 0; i < N_SENSORS; i++){
        json += "{\n \"x\": " + String(sending_data.vec[i].x) + ",\n \"y\": " + String(sending_data.vec[i].y) + ",\n \"z\": " + String(sending_data.vec[i].z) + "}\n";
        if (i < N_SENSORS -1 ){
          json += ",";
        }
      }
      json += "]\n}";
      SerialBT.println(json);
      Serial.println(json);
      #endif
      
      //Serial.println("Mi trama mide: " + String(sizeof(sending_data)));
      last_send = millis();
    }
#ifdef PRINT_LAG
    milis_actual = millis();
    Serial.println(String("Lag: ") + (milis_actual - milis_anterior));
    milis_anterior = milis_actual;
#endif

    //        Serial.println(value);
    //        ble_manager->send((uint8_t*)&value, sizeof(value));
    //        value++;
    //        Serial.println("Conectado");
    //        delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms

  }*/
}
//}


float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

float convertRawAccel(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

void PCA9546AD(uint8_t bus){
  Wire.beginTransmission(0x70);  // PCA address is 0x70
  Wire.write(1 << bus);          // sEnd byte to select bus
  Wire.endTransmission();
}
