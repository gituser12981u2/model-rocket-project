#include "storage_manager.h"

// Direct storage variables
const esp_partition_t* storage_partition = NULL;
uint32_t write_offset = 0;
uint32_t total_partition_size = 0;
uint32_t sampleCount = 0;

// Buffer management
Sample buffer[CHUNK_SIZE];
uint16_t chunkIndex = 0;

bool initStorageManager() {
  // Find custom storage partition
  storage_partition = esp_partition_find_first(
    ESP_PARTITION_TYPE_DATA,
    ESP_PARTITION_SUBTYPE_ANY,
    "storage"
  );

  if (!storage_partition) {
    // Serial.println("No storage partition found");
    return false;
  }
    
  total_partition_size = storage_partition->size;
  Serial.printf("Storage partition found: %s, size: %d bytes (%.1f MB)\n", 
                storage_partition->label, 
                total_partition_size,
                total_partition_size / 1024.0 / 1024.0);
      
  // Check if this is uninitialized flash (all 0xFF)
  uint32_t stored_offset = 0;
  esp_err_t read_err = esp_partition_read(storage_partition, 0, &stored_offset, sizeof(stored_offset));

  if (read_err != ESP_OK || stored_offset == 0xFFFFFFFF || stored_offset > total_partition_size - METADATA_SIZE) {
    Serial.println("Uninitialized or corrupted metadata, clearing...");
    clearStorage();
    return true;
  }
  
  write_offset = stored_offset;  
  sampleCount = write_offset / sizeof(Sample);

  return true;
}

void addSampleToBuffer(const Sample& sample) {
  buffer[chunkIndex] = sample;
  chunkIndex++;
    
  if (chunkIndex >= CHUNK_SIZE) {
    saveChunk(CHUNK_SIZE);
    chunkIndex = 0;
  }
}

void clearSampleBuffer() {
  chunkIndex = 0;
}

void flushSampleBuffer() {
  if (chunkIndex > 0) {
    saveChunk(chunkIndex);
    chunkIndex = 0;
  }
}

void saveChunk(uint16_t numSamples) {
  if (!storage_partition) return;

  size_t data_size = numSamples * sizeof(Sample);

  // Check if there is space
  if (write_offset + data_size > total_partition_size - METADATA_SIZE) {
    // Serial.println("Storage partition full!");
    return;
  }

  // Calculate actual write position (skip metadata sector)
  uint32_t actual_write_pos = write_offset + METADATA_SIZE;

  // Erase sector if needed (4KB sectors)
  uint32_t sector_start = (actual_write_pos / 4096) * 4096;
  if (actual_write_pos % 4096 == 0) {
    esp_err_t err = esp_partition_erase_range(storage_partition, sector_start, 4096);
    if (err != ESP_OK) {
      Serial.printf("Erase failed: %s\n", esp_err_to_name(err));
      return;
    }
  }

  // Write data
  esp_err_t err = esp_partition_write(storage_partition, actual_write_pos,
                                      buffer, data_size);

  if (err == ESP_OK) {
    write_offset += data_size;
    sampleCount += numSamples;

    // Update the offset marker in metadata sector
    esp_partition_erase_range(storage_partition, 0, 4096);  // Erase metadata sector
    esp_partition_write(storage_partition, 0, &write_offset, sizeof(write_offset));

    // Serial.printf("Saved %d samples (%lu bytes) at offset %lu (actual: %lu)\n", 
    //               numSamples, data_size, write_offset, actual_write_pos);  
  } else {
    Serial.printf("Write failed: %s\n", esp_err_to_name(err));
  }
}

void dumpStoredData() {
  if (!storage_partition || write_offset == 0) {
    // Serial.println("No data");
    return;
  }
  
  Serial.println("\nSample,Time_ms,Pressure_Pa");

  Sample readBuffer[CHUNK_SIZE];
  uint32_t read_pos = METADATA_SIZE; // Start after metadata
  uint32_t sample_num = 0;

  while (read_pos < write_offset + METADATA_SIZE) {
    // Read up to CHUNK_SIZE samples
    size_t to_read = min((size_t)(CHUNK_SIZE * sizeof(Sample)),
                         (size_t)(write_offset + METADATA_SIZE - read_pos));
    
    esp_partition_read(storage_partition, read_pos, readBuffer, to_read);
    
    size_t samples_read = to_read / sizeof(Sample);
    for (size_t i = 0; i < samples_read; i++) {
      Serial.printf("%lu,%lu,%lu\n", sample_num++, 
                    readBuffer[i].time, readBuffer[i].pressure);
      
    }

    read_pos += to_read;
  }
  
  Serial.printf("\nDumped %lu samples\n", sample_num);
}

void clearStorage() {
  Serial.println("Clearing all data...");

  if (storage_partition) {
    esp_err_t err = esp_partition_erase_range(storage_partition, 0, total_partition_size);
    if (err != ESP_OK) {
      Serial.printf("Failed to erase partition: %s\n", esp_err_to_name(err));
      return;
    }
    
    // Reset offset and write it to flash
    write_offset = 0;
    esp_partition_write(storage_partition, 0, &write_offset, sizeof(write_offset));
  }

  // Reset variables
  sampleCount = 0;
  chunkIndex = 0;
  
  Serial.println("Storage cleared");
}

void printStorageStats() {
  if (!storage_partition) {
    Serial.println("No storage partition!");
    return;
  }
  
  Serial.println("\n=== Storage Statistics ===");
  Serial.printf("Partition size: %.2f MB\n", total_partition_size / 1024.0 / 1024.0);
  Serial.printf("Used: %.2f KB (%.1f%%)\n", 
                (write_offset + METADATA_SIZE) / 1024.0,
                ((write_offset + METADATA_SIZE) * 100.0) / total_partition_size);
  Serial.printf("Free: %.2f MB\n", 
                (total_partition_size - write_offset - METADATA_SIZE) / 1024.0 / 1024.0);
  Serial.printf("Samples stored: %lu\n", sampleCount);
  Serial.printf("Max samples: ~%lu\n", 
                (total_partition_size - METADATA_SIZE) / sizeof(Sample));
  
  if (sampleCount > 0) {
    Serial.printf("Recording time at 100Hz: ~%.1f minutes\n", 
                  sampleCount / 100.0 / 60.0);
  }
  Serial.println("========================\n");
}

uint32_t getTotalSampleCount() {
  return sampleCount;
}

void dumpStoredDataCSVBatch(uint32_t startSample, uint32_t numSamples) {
  if (!storage_partition || write_offset == 0) {
    Serial.println("No data");  // Fixed: was serial.println
    return;
  }

  uint32_t totalSamples = write_offset / sizeof(Sample);

  if (startSample >= totalSamples) {
    Serial.println("Start sample beyond available data");
    return;
  }

  uint32_t endSample = min(startSample + numSamples, totalSamples);

  // Calculate read position
  uint32_t read_pos = METADATA_SIZE + (startSample * sizeof(Sample));  // Fixed: was META_SIZE
  uint32_t samples_to_read = endSample - startSample;

  Serial.printf("# Exporting samples %lu to %lu\n", startSample, endSample - 1);
  Serial.println("Sample,Time_ms,Pressure_Pa");

  Sample readBuffer[CHUNK_SIZE];
  uint32_t sample_num = startSample;
  uint32_t remaining_samples = samples_to_read;

  while (remaining_samples > 0 && read_pos < write_offset + METADATA_SIZE) {
    // Read up to CHUNK_SIZE samples or remaining samples
    size_t to_read = min((size_t)(CHUNK_SIZE * sizeof(Sample)),
                         (size_t)(remaining_samples * sizeof(Sample)));
    
    esp_partition_read(storage_partition, read_pos, readBuffer, to_read);
    
    size_t samples_read = to_read / sizeof(Sample);
    for (size_t i = 0; i < samples_read; i++) {
      Serial.printf("%lu,%lu,%lu\n", sample_num++, 
                    readBuffer[i].time, readBuffer[i].pressure);
    }

    read_pos += to_read;
    remaining_samples -= samples_read;
    
    // Small delay to prevent buffer overflow
    delay(1);
  }
  
  Serial.printf("# Exported %lu samples\n", sample_num - startSample);
}

void exportDataInfo() {
  if (!storage_partition) {
    Serial.println("No storage partition!");
    return;
  }
  
  uint32_t totalSamples = write_offset / sizeof(Sample);
  
  Serial.println("\n=== Export Information ===");
  Serial.printf("Total samples available: %lu\n", totalSamples);
  Serial.printf("Recording duration: %.2f minutes\n", totalSamples / 100.0 / 60.0);
  Serial.printf("Data size: %.2f KB\n", write_offset / 1024.0);
  
  if (totalSamples > 0) {
    Serial.println("\nExport commands:");
    Serial.println("  'e' - Export all data as CSV");
    Serial.println("  'b' - Export batch (usage: b <start> <count>)");
    Serial.println("  Example: 'b 1000 500' exports 500 samples starting from sample 1000");
  }
  Serial.println("========================\n");
}