#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"

#include "storage_manager.h"
#include "ble_manager.h"
#include "sensor_manager.h"

// Global state
enum SystemState {
  STARTUP,
  BLE_ADVERTISING,
  DATA_LOGGING,
  DEEP_SLEEP
};

SystemState currentState = STARTUP;
bool isAwake = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n=== ESP32 Logger ===");
  
  // Initialize all modules
  // if (!initSensorManager()) {
  //   Serial.println("Sensor initialization failed!");
  //   while(1);
  // }
  
  if (!initStorageManager()) {
    Serial.println("Storage initialization failed!");
    while(1);
  }
  
  initBLEManager();

  // Check wakeup reason
  handleWakeupReason();
  
  // Serial.println("System ready. Commands: s=start, x=stop, d=dump, c=clear, i=info");
  // printStorageStats();
}

void loop() {
  // Handle serial commands
  handleSerialCommands();
  
  // State machine
  switch(currentState) {
    case STARTUP:
      // Transition to BLE advertising to check for connections
      currentState = BLE_ADVERTISING;
      Serial.println("Transitioning to BLE advertising mode");
      break;
      
    case BLE_ADVERTISING:
      handleBLEAdvertising();
      break;
      
    case DATA_LOGGING:
      handleDataLogging();
      break;
      
    case DEEP_SLEEP:
      // Clean shutdown of serial task before sleep
      delay(100); // Give time for task cleanup
      Serial.println("Entering deep sleep mode...");
      Serial.flush(); // Ensure all serial output is sent
      enterDeepSleep();
      break;
  }
}

void handleWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Woke up from timer - checking for BLE connections");
      break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
      Serial.println("First boot or reset");
      break;
    default:
      Serial.println("Wake up from other source");
      break;
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while(Serial.available()) Serial.read(); // Clear buffer
    
    switch(cmd) {
      case 's': case 'S': 
        startDataLogging(); 
        break;
      case 'x': case 'X': 
        stopDataLogging(); 
        break;
      case 'd': case 'D': 
        dumpStoredData(); 
        break;
      case 'c': case 'C': 
        clearStorage(); 
        break;
      case 'i': case 'I': 
        printStorageStats(); 
        break;
      case 'w': case 'W':
        Serial.println("Forcing wake mode");
        isAwake = true;
        currentState = DATA_LOGGING;
        break;
      case 'z': case 'Z':
        Serial.println("Going to sleep");
        currentState = DEEP_SLEEP;
        break;
      case 'e': case 'E':
        Serial.println("Exporting all data as CSV...");
        dumpStoredData();
      break;
    }
  }
}

void handleBLEAdvertising() {
  static unsigned long advertiseStartTime = millis();

  // If BLE connected, stay awake permanently and start logging
  if (isBLEConnected()) {
    Serial.println("=== BLE CONNECTED - SYSTEM AWAKE ===");
    startDataLogging();
    isAwake = true;
    currentState = DATA_LOGGING;
    updateBLEStatus("LOGGING");
    return;
  }

  // If no connection after timeout, sleep
  if (millis() - advertiseStartTime > getBLEAdvertisingTimeout()) {
    Serial.println("No connection made, returning to deep sleep");
    currentState = DEEP_SLEEP;
  }
}

void handleDataLogging() { 
  // Handle data logging
  updateDataLogging();
}
