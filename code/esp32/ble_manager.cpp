// ble_manager.cpp
#include "ble_manager.h"

// Global BLE objects
BLEServer* pServer = NULL;
BLECharacteristic* pStatusChar = NULL;

// State tracking
bool deviceConnected = false;
RTC_DATA_ATTR int bootCount = 0;

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    // Serial.println("BLE connected - ESP32 staying awake");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    // Serial.println("BLE disconnected");
    
    // Restart advertising for new connections
    BLEDevice::startAdvertising();
  }
};

void initBLEManager() {
  ++bootCount;
  Serial.printf("Boot count: %d\n", bootCount);
  
  BLEDevice::init("ESP32-DataLogger");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Status characteristic (read/notify)
  pStatusChar = pService->createCharacteristic(STATUS_CHAR_UUID,
                                               BLECharacteristic::PROPERTY_READ |
                                               BLECharacteristic::PROPERTY_NOTIFY);
  pStatusChar->setValue("STANDBY");

  pService->start();

  // Setup advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06); // Low power advertising
  
  BLEDevice::startAdvertising();
  // Serial.println("BLE advertising started...");
}

void enterDeepSleep() {
  // Serial.println("Entering deep sleep...");
  Serial.flush();

  // Configure wake-up timer
  esp_sleep_enable_timer_wakeup(BLE_CHECK_INTERVAL_SEC * 1000000ULL);

  // Power down BLE
  BLEDevice::deinit(true);

  esp_deep_sleep_start();
}

bool isBLEConnected() {
  return deviceConnected;
}

unsigned long getBLEAdvertisingTimeout() {
  return BLE_ADVERTISING_TIME_MS;
}

void updateBLEStatus(const char* status) {
  if (pStatusChar) {
    pStatusChar->setValue(status);
    if (deviceConnected) {
      pStatusChar->notify();
    }
  }
}