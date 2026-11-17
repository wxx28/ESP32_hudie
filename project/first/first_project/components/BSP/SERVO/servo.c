#include "servo.h"

#include <math.h>

#include "board_v1.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "sdkconfig.h"

static const char *TAG = "bsp_servo";

#define SERVO_TIMER      LEDC_TIMER_0
#define SERVO_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_DUTY_MAX   (1u << 14)
#define SERVO_PERIOD_US  (1000000u / CONFIG_BSP_SERVO_PWM_HZ)

#ifdef CONFIG_BSP_SERVO_OPTO_INVERT
#define SERVO_INVERT 1
#else
#define SERVO_INVERT 0
#endif

static const int s_gpio[SERVO_COUNT] = {BOARD_PIN_SERVO_LEFT, BOARD_PIN_SERVO_RIGHT};
static bool s_inited;

esp_err_t servo_init(void)
{
    const ledc_timer_config_t timer = {
        .speed_mode = SERVO_MODE,
        .duty_resolution = SERVO_RESOLUTION,
        .timer_num = SERVO_TIMER,
        .freq_hz = CONFIG_BSP_SERVO_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "timer");

    const uint32_t mid_us = (CONFIG_BSP_SERVO_MIN_US + CONFIG_BSP_SERVO_MAX_US) / 2u;
    for (int i = 0; i < SERVO_COUNT; ++i) {
        ledc_channel_config_t ch = {
            .gpio_num = s_gpio[i],
            .speed_mode = SERVO_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = SERVO_TIMER,
            .duty = (mid_us * SERVO_DUTY_MAX) / SERVO_PERIOD_US,
            .hpoint = 0,
#if CONFIG_BSP_SERVO_OPTO_INVERT
            .flags.output_invert = 1,
#endif
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "channel %d", i);
    }
    s_inited = true;
    ESP_LOGI(TAG, "servo %d Hz, %d~%d us, opto invert %s", CONFIG_BSP_SERVO_PWM_HZ, CONFIG_BSP_SERVO_MIN_US,
             CONFIG_BSP_SERVO_MAX_US, SERVO_INVERT ? "on" : "off");
    return ESP_OK;
}

esp_err_t servo_set_pulse_us(servo_id_t id, uint32_t pulse_us)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "not inited");
    ESP_RETURN_ON_FALSE(id < SERVO_COUNT, ESP_ERR_INVALID_ARG, TAG, "bad id");
    if (pulse_us < CONFIG_BSP_SERVO_MIN_US) {
        pulse_us = CONFIG_BSP_SERVO_MIN_US;
    }
    if (pulse_us > CONFIG_BSP_SERVO_MAX_US) {
        pulse_us = CONFIG_BSP_SERVO_MAX_US;
    }
    const uint32_t duty = (pulse_us * SERVO_DUTY_MAX) / SERVO_PERIOD_US;
    ESP_RETURN_ON_ERROR(ledc_set_duty(SERVO_MODE, (ledc_channel_t)id, duty), TAG, "duty");
    return ledc_update_duty(SERVO_MODE, (ledc_channel_t)id);
}

esp_err_t servo_set_angle(servo_id_t id, float angle_deg)
{
    if (!isfinite(angle_deg)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (angle_deg < 0.0f) {
        angle_deg = 0.0f;
    }
    if (angle_deg > 180.0f) {
        angle_deg = 180.0f;
    }
    const float span = (float)(CONFIG_BSP_SERVO_MAX_US - CONFIG_BSP_SERVO_MIN_US);
    const uint32_t us = CONFIG_BSP_SERVO_MIN_US + (uint32_t)(angle_deg * span / 180.0f + 0.5f);
    return servo_set_pulse_us(id, us);
}
