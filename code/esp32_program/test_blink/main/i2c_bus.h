#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c.h"
#include "esp_err.h"

// I2C Bus configuration
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 1000

// I2C Bus Management Functions
esp_err_t i2c_bus_init(void);
esp_err_t i2c_bus_deinit(void);
esp_err_t i2c_bus_write_reg(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
esp_err_t i2c_bus_read_reg(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_bus_scan_bus(uint8_t *found_devices, size_t max_devices, size_t *num_found);

#endif // I2C_BUS_H
