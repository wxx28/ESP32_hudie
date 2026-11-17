#include "LED.h"

#include "board_v1.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char *TAG = "bsp_led";

#define LED_TICK_MS 50

static volatile led_pattern_t s_pattern = LED_PATTERN_OFF;
static esp_timer_handle_t s_timer;
static uint32_t s_tick;

void led_set(bool on)
{
#if CONFIG_BSP_LED_ACTIVE_HIGH
    gpio_set_level(BOARD_PIN_LED, on ? 1 : 0);
#else
    gpio_set_level(BOARD_PIN_LED, on ? 0 : 1);
#endif
}

/* 每种节奏用 20 拍（1 s）一个周期的位图表示，bit = 1 表示亮 */
static bool pattern_on(led_pattern_t p, uint32_t t)
{
    switch (p) {
    case LED_PATTERN_ON:
        return true;
    case LED_PATTERN_SLOW:
        return (t % 20u) < 10u;
    case LED_PATTERN_DOUBLE: {
        const uint32_t k = t % 20u;
        return k == 0u || k == 1u || k == 4u || k == 5u;
    }
    case LED_PATTERN_FAST:
        return (t % 4u) < 2u;
    case LED_PATTERN_RAPID:
        return (t % 2u) == 0u;
    case LED_PATTERN_SOS: {
        const uint32_t k = t % 40u;
        if (k < 18u) {
            return (k % 6u) < 4u;          /* 三长 */
        }
        if (k < 30u) {
            return ((k - 18u) % 4u) < 2u;  /* 三短 */
        }
        return false;
    }
    default:
        return false;
    }
}

static void led_tick_cb(void *arg)
{
    (void)arg;
    led_set(pattern_on(s_pattern, s_tick++));
}

esp_err_t led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BOARD_PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio");
    led_set(false);

    const esp_timer_create_args_t args = {.callback = led_tick_cb, .name = "led"};
    ESP_RETURN_ON_ERROR(esp_timer_create(&args, &s_timer), TAG, "timer");
    return esp_timer_start_periodic(s_timer, LED_TICK_MS * 1000);
}

void led_set_pattern(led_pattern_t pattern)
{
    s_pattern = pattern;
}
