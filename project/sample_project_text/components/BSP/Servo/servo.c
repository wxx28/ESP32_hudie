#include "servo.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#define SERVO_PWM_FREQUENCY_HZ 50U
#define SERVO_PWM_PERIOD_US (1000000U / SERVO_PWM_FREQUENCY_HZ)
#define SERVO_PWM_DUTY_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_PWM_DUTY_MAX (1U << SERVO_PWM_DUTY_RESOLUTION)

static const gpio_num_t s_servo_gpios[] = {
    GPIO_NUM_4,
    GPIO_NUM_5,
};

esp_err_t servo_init(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_PWM_DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        return err;
    }

    for (int i = 0; i < 2; ++i) {
        const ledc_channel_config_t channel_config = {
            .gpio_num = s_servo_gpios[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        err = ledc_channel_config(&channel_config);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

esp_err_t servo_set_pulse_us(servo_channel_t channel, uint32_t pulse_us)
{
    if ((channel != SERVO_CHANNEL_PWM0 && channel != SERVO_CHANNEL_PWM1) ||
        pulse_us < SERVO_MIN_PULSE_US || pulse_us > SERVO_MAX_PULSE_US) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t duty = (pulse_us * SERVO_PWM_DUTY_MAX) / SERVO_PWM_PERIOD_US;
    const ledc_channel_t ledc_channel = (ledc_channel_t)channel;

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_channel, duty);
    if (err != ESP_OK) {
        return err;
    }

    return ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_channel);
}
