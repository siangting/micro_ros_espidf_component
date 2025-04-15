
#include "esp32_led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void led_task(void *parameter) {
    int *state = (int *)parameter;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ESP32_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    while (true) {
        switch (*state) {
            case WAITING_AGENT:
                gpio_set_level(ESP32_LED, 1);
                vTaskDelay(125 / portTICK_PERIOD_MS);
                gpio_set_level(ESP32_LED, 0);
                vTaskDelay(125 / portTICK_PERIOD_MS);
                break;
            case AGENT_CONNECTED:
                gpio_set_level(ESP32_LED, 1);
                break;
            default:
                gpio_set_level(ESP32_LED, 0);
                break;
        }
    }
}
