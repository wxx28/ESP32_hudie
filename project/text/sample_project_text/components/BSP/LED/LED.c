#include "LED.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp_led";

static int s_gpio = -1;
static bool s_active_low;
static bool s_on;

esp_err_t led_init(int gpio_num, bool active_low)
{
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), ESP_ERR_INVALID_ARG, TAG,
                        "invalid output gpio %d", gpio_num);

    s_gpio = gpio_num;
    s_active_low = active_low;
    s_on = false;

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio_config failed");

    led_set(false);
    ESP_LOGI(TAG, "led on gpio %d, %s", gpio_num, active_low ? "active low" : "active high");
    return ESP_OK;
}

void led_set(bool on)
{
    if (s_gpio < 0) {
        return;
    }
    s_on = on;
    gpio_set_level((gpio_num_t)s_gpio, (on != s_active_low) ? 1 : 0);
}

void led_toggle(void)
{
    led_set(!s_on);
}

bool led_get(void)
{
    return s_on;
}
