#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

struct Sample {
  uint32_t pressure;  // Pa
  uint32_t time;      // milliseconds
};

// Function declarations
bool initSensorManager();
bool readSensor(Sample& sample);
void startDataLogging();
void stopDataLogging();
void updateDataLogging();
bool isLogging();

#endif