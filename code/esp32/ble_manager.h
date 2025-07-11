#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

// Include ESP-IDF BLE headers first
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_sleep.h"

#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLE2902.h"

// Configuration
#define BLE_CHECK_INTERVAL_SEC  5    // Wake every 5 seconds to check for BLE
#define BLE_ADVERTISING_TIME_MS 500 // Advertise for 3 seconds, then back to sleep

// UUIDs
#define SERVICE_UUID        "eb17cfcb-ada1-483d-88c3-e58a416d8f80"
#define STATUS_CHAR_UUID    "87654321-4321-4321-4321-cba987654321"

// Function declarations
void initBLEManager();
void enterDeepSleep();
bool isBLEConnected();
unsigned long getBLEAdvertisingTimeout();
void updateBLEStatus(const char* status);

#endif