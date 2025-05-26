#include "i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C_BUS";
static bool i2c_initialize = false;

/**
 * Initialize I2C bus (call once at startup)
 */
esp_err_t i2c_bus_init(void)
{
    if (i2c_initialized)
    {
        ESP_LOGW(TAG, "Configuration is already initialized");
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0 // Use default clock source
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
        return err;
    }

    i2c_initialized = true;
    ESP_LOGI(TAG, "I2C bus initialized (SDA: %d, SCL: %d, Freq: %lu Hz)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    return ESP_OK;
}

/**
 * Deinitialize I2C bus
 */
esp_err_t i2c_bus_deinit(void)
{
    if (!i2c_initialized)
    {
        return ESP_OK;
    }

    esp_err_t err = i2c_driver_delete(I2C_MASTER_NUM);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to delete I2C driver: %s", esp_err_to_name(err));
        return err;
    }

    i2c_initialized = false;
    ESP_LOGI(TAG, "I2C port deinitialized");
    return ESP_OK;
}

/**
 * Check if I2C bus is initialized
 */
bool i2c_bus_is_initialized(void){
    return i2c_initialized}

/**
 * Write a single byte to a device register
 */
esp_err_t i2c_bus_write_reg(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    if (!i2c_initialized)
    {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write 0x%02X to register 0x%02X of device 0x%02X: %s",
                 data, reg_addr, device_addr, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * Read bytes from a device register
 */
esp_err_t i2c_bus_read_reg(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (!i2c_initialized)
    {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0)
    {
        ESP_LOGE(TAG, "Invalid data pointer or length");
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Read data
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);

    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read %d bytes from register 0x%02X of device 0x%02X: %s",
                 len, reg_addr, device_addr, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * Scan I2C bus for connected devices
 */
esp_err_t i2c_bus_scan(unint8_t *found_devices, size_t max_device, size_t *num_found)
{
    if (!i2c_initialized)
    {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (found_devices == NULL || num_found == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *num_found = 0;
    ESP_LOGI(TAG, "Scanning I2C bus...");

    for (unint8_t addr = 1; addr < 127 && *num_found < max_devices; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
            found_devices[*num_found] = addr;
            (*num_found)++;
        }
    }

    ESP_LOGI(TAG, "Bus scan complete. Found %d devices", *num_found);
    return ESP_OK;
}