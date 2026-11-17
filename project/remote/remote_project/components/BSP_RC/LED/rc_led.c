#include "rc_led.h"

#include <stdbool.h>
#include <stdint.h>

#include "board_rc.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_timer.h"

static const char *TAG = "rc_led";

static volatile rc_led_pattern_t s_p = RC_LED_OFF;
static volatile uint8_t s_blip;
static esp_timer_handle_t s_timer;
static uint32_t s_t;

static void set_level(bool on)
{
    gpio_set_level(RC_PIN_LED, on ? 0 : 1); /* 低电平点亮 */
}

static bool on_at(rc_led_pattern_t p, uint32_t t)
{
    switch (p) {
    case RC_LED_ON:
        return true;
    case RC_LED_SLOW:
        return (t % 20u) < 10u;
    case RC_LED_FAST:
        return (t % 4u) < 2u;
    case RC_LED_RAPID:
        return (t % 2u) == 0u;
    case RC_LED_DOUBLE: {
        const uint32_t k = t % 20u;
        return k == 0u || k == 1u || k == 4u || k == 5u;
    }
    default:
        return false;
    }
}

static void tick(void *arg)
{
    (void)arg;
    if (s_blip > 0u) {
        /* 确认闪：和当前状态取反，保证看得见 */
        set_level(!on_at(s_p, s_t));
        s_blip--;
    } else {
        set_level(on_at(s_p, s_t));
    }
    s_t++;
}

esp_err_t rc_led_init(void)
{
    gpio_config_t cfg = {.pin_bit_mask = 1ULL << RC_PIN_LED, .mode = GPIO_MODE_OUTPUT};
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio");
    set_level(false);
    const esp_timer_create_args_t a = {.callback = tick, .name = "rc_led"};
    ESP_RETURN_ON_ERROR(esp_timer_create(&a, &s_timer), TAG, "timer");
    return esp_timer_start_periodic(s_timer, 50 * 1000);
}

void rc_led_set(rc_led_pattern_t p)
{
    if (p != RC_LED_BLIP) {
        s_p = p;
    }
}

void rc_led_blip(void)
{
    s_blip = 3;
}
