#include "sensor_manager.h"
#include "storage_manager.h"

// Global sensor object
Adafruit_BMP3XX bmp;

// Recording variables
bool recording = false;
uint32_t startTime = 0;

bool initSensorManager() {
  Wire.begin(21, 22);
  
  if (!bmp.begin_I2C()) {
    Serial.println("BMP3 sensor initialization failed");
    return false;
  }
  
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);

  Serial.println("BMP388 sensor initialized successfully");
  return true;
}

bool readSensor(Sample& sample) {
  if (!bmp.performReading()) {
    return false;
  }

  sample.pressure = (uint32_t)bmp.pressure;
  
  // sample.time = millis() - startTime;
  unsigned long currentTime = millis();
  if (currentTime >= startTime) {
    sample.time = currentTime - startTime;
  } else {
    // Handle millis() overflow or reset case
    startTime = currentTime;
    sample.time = 0;
  }
  
  return true;
}

void startDataLogging() {
  if (!recording) {
    recording = true;
    startTime = millis();
    // Serial.println("Data logging started...");

    // Clear existing buffer
    clearSampleBuffer();
  } else {
    // Serial.println("Already logging data");
  }
}

void stopDataLogging() {
  if (recording) {
    recording = false;

    // Save any remaining samples in buffer
    flushSampleBuffer();

    Serial.printf("Data logging stopped\n");
    Serial.printf("Total samples recored: %lu samples\n", getTotalSampleCount());
  } else {
    Serial.println("Not currently logging");
  }
}

void updateDataLogging() {
  if (!recording) {
    return;
  }
  
  // Read sensor at hardware maximum rate
  Sample sample;
  if (readSensor(sample)) {
    addSampleToBuffer(sample);
  } else {
    Serial.println("Sensor read failed");
  }
}

bool isLogging() {
  return recording;
}