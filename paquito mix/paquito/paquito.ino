#include <Wire.h>
#include <stdint.h>
#include <math.h>
//#include "BleManager.hpp"
#include "SensorManager.hpp"
#include "convertRawGyro.hpp"
#include "BatteryManager.hpp"
#include "Rate.hpp"
#include <MadgwickAHRS.h>
#include <map>
#include "BluetoothSerial.h"
#include <WiFi.h>
#include <utility>

#define JSON

BluetoothSerial SerialBT;

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
#define N_SENSORS 1

#define BT_BAUDRATE 38400
#define CONNECTION_PIN 4

#define IMU_ENABLE_PIN 23 //este pin es el que debe usarse para alimentar los acelerometros (bien sea desde el propio pin, o con un MOSFET)

Rate rate(100); // frecuencia de muestreo
Rate ble_rate(10); // frecuencia de envio (5 envios por segundo)

#define BT_PACKET_RATE 10

#define deg_to_rad(a) (a/180*M_PI)
#define rad_to_deg(a) (a/M_PI*180)

#define SENSE_RATE   100
#define GYRO_RANGE   250
#define ACCL_RANGE     2

float rad_a_roll;
float rad_a_pitch;

float accl_roll;
float accl_pitch;

float omega_roll;
float omega_pitch;
float omega_yaw;

float gyro_roll;
float gyro_pitch;
float gyro_yaw;

// Complementary filter data
float comp_roll;
float comp_pitch;

float madw_roll;
float madw_pitch;
float madw_yaw;

static unsigned long last_mills = 0;


std::map<int, Madgwick> madgwick_filters;

template<class type_datatype>
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
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String bt_name = "SpineCare_" + String(mac[3]) + String(mac[4]) + String(mac[5]);
  SerialBT.begin(bt_name);
  pinMode(CONNECTION_PIN, INPUT);
  battery_manager = new BatteryManager(32);
  Wire.begin();

  for (int i = 0; i < N_SENSORS; i++) {
    Serial.println("Sensor :" + String(i));
    isms.emplace_back(new IndividualSensorManager(i));
    madgwick_filters[i].begin(rate.get_rate());
    delay(500);
  }
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


Comparer<decltype(std::declval<IndividualSensorManager>().read_sensor())> comparer(5);

void loop() {
    double dt;
  static unsigned long last_send = 0;
  
  if (
    //digitalRead(CONNECTION_PIN)
    1
    ) {

    rate.sleep();
    // const uint8_t bateria = battery_manager->read();
    // Serial.println(String("bateria: ") + bateria);
    auto ble_send = ble_rate.has_passed();
    for (auto& ism : isms) {
      auto& madgwick = madgwick_filters[ism->get_sensor()];
      Vector3<int> gyro_raw, accel_raw;
      auto read_sensor = std::tie(accel_raw, gyro_raw) = ism->read_sensor();
      auto gyro_vector = convertRawGyro(gyro_raw);
      auto accel_vector = convertRawAccel(accel_raw);
      /*
      Serial.print("//DEBUG Sensor\n");
      Serial.print(gyro_vector.x);
      Serial.print("\n,");
      Serial.print(gyro_vector.y);
      Serial.print("\n,");
      Serial.print(gyro_vector.z);
      Serial.print("\n;");
      */
      if (comparer.isAllTheSame(read_sensor)){
        digitalWrite(IMU_ENABLE_PIN, HIGH);
        delay(500);
        digitalWrite(IMU_ENABLE_PIN, LOW);
        ESP.restart();
      }

      rad_a_roll = atan2(accel_vector.y, accel_vector.z);
      rad_a_pitch = atan2(-accel_vector.x, sqrt(accel_vector.y*accel_vector.y + accel_vector.z*accel_vector.z));
      accl_roll = rad_to_deg(rad_a_roll);
      accl_pitch = rad_to_deg(rad_a_pitch);

      omega_roll  = convertRawGyro(gyro_vector.x);
      omega_pitch = convertRawGyro(gyro_vector.y);
      omega_yaw   = convertRawGyro(gyro_vector.z);

      do{
      unsigned long cur_mills = micros();
      unsigned long duration = cur_mills - last_mills;
      last_mills = cur_mills;
      dt = duration / 1000000.0; // us->s  
      }while(dt < 0.1);

      // Gyro data
      gyro_roll  += omega_roll  * dt; // (ms->s) omega x time = degree
      gyro_pitch += omega_pitch * dt;
      gyro_yaw   += omega_yaw   * dt;
      
      // Complementary filter data --> B
      //comp_roll[i]  = 0.93 * (comp_roll[i]  + omega_roll[i]  * dt) + 0.07 * accl_roll[i]; 
      //comp_pitch[i] = 0.93 * (comp_pitch[i] + omega_pitch[i] * dt) + 0.07 * accl_pitch[i]; 
    
      // Madgwick filter data
      madgwick.updateIMU2(omega_roll, omega_pitch, omega_yaw, accel_vector.x, accel_vector.y, accel_vector.z, dt);
      //madgwick.updateIMU(omega_roll, omega_pitch, omega_yaw, accX, accY, accZ);
       madw_roll  = madgwick.getRoll();
       madw_pitch = madgwick.getPitch();
       madw_yaw   = madgwick.getYaw();

      
      /*madgwick.updateIMU(
        gyro_vector.x,
        gyro_vector.y,
        gyro_vector.z,
        accel_vector.x,
        accel_vector.y,
        accel_vector.z
      );*/
      // Serial.println("b4 ble_send");
      //if (ble_send) {
        // Serial.println("ble_send");
        sending_data.vec[ism->get_sensor()].x = madw_roll;
        sending_data.vec[ism->get_sensor()].y = madw_pitch;
        sending_data.vec[ism->get_sensor()].z = madw_yaw;
        //if(sending_data.vec[ism->get_sensor()].x<0) sending_data.vec[ism->get_sensor()].x += 360;
        //if(sending_data.vec[ism->get_sensor()].y<0) sending_data.vec[ism->get_sensor()].y += 360;
        //if(sending_data.vec[ism->get_sensor()].z<0) sending_data.vec[ism->get_sensor()].z += 360;
        //Serial.println("Sensor: " + String(ism->get_sensor()) + ":  \tRoll= " + String(sending_data.vec[ism->get_sensor()].x)  + " \tPitch= "  + String(sending_data.vec[ism->get_sensor()].y)+ " \Yaw= " + String(sending_data.vec[ism->get_sensor()].z));
      //}

      if (ism->get_sensor() == PRINT_SENSOR)
        Serial.println(
          String("") +
          sending_data.vec[ism->get_sensor()].x + " " +
          sending_data.vec[ism->get_sensor()].y + " " +
          sending_data.vec[ism->get_sensor()].z
        );
        
    }
    
    /*if (
      1 &&
      //digitalRead(CONNECTION_PIN) && 
    (millis() - last_send > (1000 / BT_PACKET_RATE))){
      sending_data.bateria = battery_manager->read();
      //Serial.println(sending_data.bateria);
      //Serial.println("enviando mensaje");
      #ifndef JSON
      SerialBT.write((uint8_t*)&sending_data, sizeof(sending_data));
      #else
      String json="";
      json = "{\n\"battery\": " + String(sending_data.bateria) + ",\n \"imu\":[\n";
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
    }*/
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

  }
}
