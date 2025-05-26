#ifndef BMP388_H
#define BMP388_H

#include "driver/i2c.h"
#include "esp_err.h"

// I2C Configuration
#define I2C_MASTER_SCL_IO 22 // GPIO pin for SCL
#define I2C_MASTER_SDA_IO 21 // GPIO pin for SDA
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// BMP388 Configuration
#define BMP388_ADDR 0x77 // BMP388 I2C address
#define BMP388_CHIP_ID_REG 0x00
#define BMP388_STATUS_REG 0x03
#define BMP388_DATA_REG 0x04 // Start of pressure and temperature data
#define BMP388_PWR_CTRL_REG 0x1B
#define BMP388_OSR_REG 0x1C
#define BMP388_ODR_REG 0x1D

// Calibration coefficient registers
#define BMP388_CALIB_DATA_REG 0x31

// LED Configuration
#define LED_PIN 2

// Data structure for sensor readings
typedef struct
{
    float temperature;
    float pressure;
    uint32_t timestamp;
} sensor_data_t;

// BMP388 calibration coefficients
typedef struct
{
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
} bmp388_calib_t;

static bmp388_calib_t calib_data;

// Initialize I2C master
esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK)
    {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0)
}