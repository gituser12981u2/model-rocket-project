#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "BLINK_EXMAPLE";

// GPIO pin for the LED
#define LED_PIN 2 // GPIO 2 for onboard LED

void app_main(void)
{
  // Configure the GPIO pin as output
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  
  ESP_LOGI(TAG, "Blink example started");

  // Blink the LED in an infinite loop
  while (1) {
    // Turn on the LED
    ESP_LOGI(TAG, "LED ON");
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    
    // Turn off the LED
    ESP_LOGI(TAG, "LED OFF");
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}
