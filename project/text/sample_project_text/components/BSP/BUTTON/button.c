#include "button.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

/* 电平需连续保持该时长才视为稳定状态，覆盖机械按键 1~10 ms 的典型抖动区间 */
#define BUTTON_DEBOUNCE_MS 20

static const char *TAG = "bsp_button";

static gpio_num_t s_gpio = GPIO_NUM_NC;
static bool s_active_low;
static button_press_cb_t s_press_cb;
static esp_timer_handle_t s_debounce_timer;
static bool s_stable_pressed;

static bool button_is_pressed(void)
{
    int level = gpio_get_level(s_gpio);
    return s_active_low ? (level == 0) : (level == 1);
}

static void debounce_timeout_cb(void *arg)
{
    (void)arg;
    bool pressed = button_is_pressed();
    /* 抖动后回到原状态，不算一次动作，也就不会重复触发 */
    if (pressed == s_stable_pressed) {
        return;
    }
    s_stable_pressed = pressed;
    if (pressed && s_press_cb != NULL) {
        s_press_cb();
    }
}

static void button_isr_handler(void *arg)
{
    (void)arg;
    /* 每个边沿都重新计时，直到电平安静 BUTTON_DEBOUNCE_MS 后才采样判断 */
    esp_timer_stop(s_debounce_timer);
    esp_timer_start_once(s_debounce_timer, (uint64_t)BUTTON_DEBOUNCE_MS * 1000);
}

esp_err_t button_init(int gpio_num, bool active_low, button_press_cb_t on_press)
{
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(gpio_num), ESP_ERR_INVALID_ARG, TAG, "invalid gpio %d", gpio_num);

    s_gpio = (gpio_num_t)gpio_num;
    s_active_low = active_low;
    s_press_cb = on_press;

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio_config failed");

    /* 上电就按住时先记为已按下，必须松开再按才会计一次 */
    s_stable_pressed = button_is_pressed();

    const esp_timer_create_args_t timer_args = {
        .callback = debounce_timeout_cb,
        .name = "btn_debounce",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_debounce_timer), TAG, "timer create failed");

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "isr service install failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(s_gpio, button_isr_handler, NULL), TAG, "isr add failed");

    ESP_LOGI(TAG, "button on gpio %d, %s", gpio_num, active_low ? "active low" : "active high");
    return ESP_OK;
}
