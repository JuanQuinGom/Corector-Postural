#ifndef BLE_MANAGER_HPP
#define BLE_MANAGER_HPP

#include <BMI160Gen.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define SERVICE_UUID        "0000FFF0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFF7-0000-1000-8000-00805F9B34FB"

class BleManager{
public:
/*
  BLEServer *pServer = NULL;
  BLEService *pService = NULL;
  BLECharacteristic *pCharacteristic = NULL;
//public:
  BleManager(const char* service_uuid, const char* characteristic_uuid){
    BLEDevice::init("Spine Care");
    pServer = BLEDevice::createServer();
    pService = pServer->createService(service_uuid);
    pCharacteristic = pService->createCharacteristic(
      characteristic_uuid,
      BLECharacteristic::PROPERTY_READ | //declaras prodiedades que tiene la comunicacion
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_INDICATE
    );
    pCharacteristic->setValue("Hello World says Neil");
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    */
  }
  void send(uint8_t* msg, size_t size){
    //SerialBT.println("mensaje: ");
    SerialBT.send(msg, size);
    //pCharacteristic->notify();
  }
} *ble_manager;

#endif //BLE_MANAGER_HPP
