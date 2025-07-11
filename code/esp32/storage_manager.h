#ifndef STORAGE_MANAGER_H
#define STORAGE_MANAGER_H

#include <esp_partition.h>
#include "sensor_manager.h"

#define CHUNK_SIZE 500         // Buffer size for efficiency
#define METADATA_SIZE 4096     // First sector reserved for metadata

// Function declarations
bool initStorageManager();
void addSampleToBuffer(const Sample& sample);
void clearSampleBuffer();
void flushSampleBuffer();
void saveChunk(uint16_t numSamples);
void dumpStoredData();
void clearStorage();
void printStorageStats();
uint32_t getTotalSampleCount();
void dumpStoredDataCSVBatch(uint32_t startSample, uint32_t numSamples);
void exportDataInfo();

#endif