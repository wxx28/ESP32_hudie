#include "servo_motion.h"

#include <math.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "servo.h"

#define SERVO_MOTION_UPDATE_PERIOD_MS 10
#define SERVO_MOTION_JOYSTICK_TIMEOUT_US 300000
#define SERVO_MOTION_TASK_STACK_SIZE 3072
#define SERVO_MOTION_TASK_PRIORITY 5
#define SERVO_MOTION_PI 3.14159265358979323846f

static const char *TAG = "servo_motion";

typedef struct {
    float amplitude_deg;
    float differential_deg;
    float frequency_hz;
    float phase_deg;
    float center_deg;
    bool running;
    bool joystick_active;
    int64_t start_time_us;
    int64_t last_joystick_us;
} servo_motion_state_t;

static servo_motion_state_t s_state = {
    .amplitude_deg = SERVO_MOTION_MIN_AMPLITUDE_DEG,
    .differential_deg = 0.0f,
    .frequency_hz = 2.0f,
    .phase_deg = 0.0f,
    .center_deg = 90.0f,
    .running = false,
};
static SemaphoreHandle_t s_state_mutex;

static float clampf(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

/* 单边最大摆幅：A + |D| 不能超过 min(k, 180 - k)。 */
static float center_span(float center_deg)
{
    return fminf(center_deg, 180.0f - center_deg);
}

static void clamp_mix(float *amplitude_deg, float *differential_deg, float center_deg)
{
    const float span = center_span(center_deg);
    const float max_diff = fmaxf(0.0f, span - SERVO_MOTION_MIN_AMPLITUDE_DEG);

    *differential_deg = clampf(*differential_deg, -max_diff, max_diff);

    const float max_amp =
        fminf(SERVO_MOTION_MAX_AMPLITUDE_DEG, span - fabsf(*differential_deg));
    *amplitude_deg = clampf(*amplitude_deg, SERVO_MOTION_MIN_AMPLITUDE_DEG,
                            fmaxf(SERVO_MOTION_MIN_AMPLITUDE_DEG, max_amp));
}

static servo_motion_params_t state_to_params(const servo_motion_state_t *state)
{
    return (servo_motion_params_t){
        .amplitude_deg = state->amplitude_deg,
        .differential_deg = state->differential_deg,
        .frequency_hz = state->frequency_hz,
        .phase_deg = state->phase_deg,
        .center_deg = state->center_deg,
        .running = state->running,
        .joystick_active = state->joystick_active,
    };
}

static servo_motion_state_t state_snapshot(void)
{
    servo_motion_state_t snapshot;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (s_state.running && s_state.joystick_active &&
        esp_timer_get_time() - s_state.last_joystick_us > SERVO_MOTION_JOYSTICK_TIMEOUT_US) {
        s_state.running = false;
        s_state.joystick_active = false;
        ESP_LOGW(TAG, "Joystick timeout, stopping and recentering");
    }
    snapshot = s_state;
    xSemaphoreGive(s_state_mutex);
    return snapshot;
}

bool servo_motion_params_valid(const servo_motion_params_t *params)
{
    if (params == NULL) {
        return false;
    }

    return isfinite(params->amplitude_deg) &&
           params->amplitude_deg >= SERVO_MOTION_MIN_AMPLITUDE_DEG &&
           params->amplitude_deg <= SERVO_MOTION_MAX_AMPLITUDE_DEG &&
           isfinite(params->differential_deg) &&
           isfinite(params->frequency_hz) &&
           params->frequency_hz >= SERVO_MOTION_MIN_FREQUENCY_HZ &&
           params->frequency_hz <= SERVO_MOTION_MAX_FREQUENCY_HZ &&
           isfinite(params->phase_deg) &&
           params->phase_deg >= SERVO_MOTION_MIN_PHASE_DEG &&
           params->phase_deg <= SERVO_MOTION_MAX_PHASE_DEG &&
           isfinite(params->center_deg) &&
           params->center_deg >= SERVO_MOTION_MIN_CENTER_DEG &&
           params->center_deg <= SERVO_MOTION_MAX_CENTER_DEG &&
           params->amplitude_deg + fabsf(params->differential_deg) <=
               center_span(params->center_deg);
}

static uint32_t angle_to_pulse_us(float angle_deg)
{
    const float angle = fminf(180.0f, fmaxf(0.0f, angle_deg));
    const float pulse_span = (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    return SERVO_MIN_PULSE_US + (uint32_t)(angle * pulse_span / 180.0f + 0.5f);
}

/* 差动混控：GPIO4 用 A - D，GPIO5 用 A + D。 */
static void servo_angles_for_wave(const servo_motion_state_t *state, float wave,
                                  float *pwm0_deg, float *pwm1_deg)
{
    *pwm0_deg = state->center_deg + (state->amplitude_deg - state->differential_deg) * wave;
    *pwm1_deg = state->center_deg - (state->amplitude_deg + state->differential_deg) * wave;
}

static void servo_motion_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    float centered_deg = -1.0f;
    (void)arg;

    while (true) {
        const servo_motion_state_t state = state_snapshot();

        if (state.running) {
            const float elapsed_s =
                (float)(esp_timer_get_time() - state.start_time_us) / 1000000.0f;
            const float phase_rad =
                2.0f * SERVO_MOTION_PI * state.frequency_hz * elapsed_s +
                state.phase_deg * SERVO_MOTION_PI / 180.0f;
            const float wave = sinf(phase_rad);
            float pwm0_deg = 0.0f;
            float pwm1_deg = 0.0f;

            servo_angles_for_wave(&state, wave, &pwm0_deg, &pwm1_deg);
            ESP_ERROR_CHECK(
                servo_set_pulse_us(SERVO_CHANNEL_PWM0, angle_to_pulse_us(pwm0_deg)));
            ESP_ERROR_CHECK(
                servo_set_pulse_us(SERVO_CHANNEL_PWM1, angle_to_pulse_us(pwm1_deg)));
            centered_deg = -1.0f;
        } else if (state.center_deg != centered_deg) {
            ESP_ERROR_CHECK(servo_set_pulse_us(SERVO_CHANNEL_PWM0,
                                               angle_to_pulse_us(state.center_deg)));
            ESP_ERROR_CHECK(servo_set_pulse_us(SERVO_CHANNEL_PWM1,
                                               angle_to_pulse_us(state.center_deg)));
            centered_deg = state.center_deg;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SERVO_MOTION_UPDATE_PERIOD_MS));
    }
}

esp_err_t servo_motion_init(void)
{
    if (s_state_mutex != NULL) {
        return ESP_OK;
    }

    const esp_err_t err = servo_init();
    if (err != ESP_OK) {
        return err;
    }

    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(servo_motion_task, "servo_motion", SERVO_MOTION_TASK_STACK_SIZE, NULL,
                    SERVO_MOTION_TASK_PRIORITY, NULL) != pdPASS) {
        vSemaphoreDelete(s_state_mutex);
        s_state_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

servo_motion_params_t servo_motion_get_params(void)
{
    if (s_state_mutex == NULL) {
        return state_to_params(&s_state);
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    const servo_motion_params_t params = state_to_params(&s_state);
    xSemaphoreGive(s_state_mutex);
    return params;
}

esp_err_t servo_motion_set_params(const servo_motion_params_t *params)
{
    if (!servo_motion_params_valid(params)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (params->running && !s_state.running) {
        s_state.start_time_us = esp_timer_get_time();
    }
    s_state.amplitude_deg = params->amplitude_deg;
    s_state.differential_deg = params->differential_deg;
    s_state.frequency_hz = params->frequency_hz;
    s_state.phase_deg = params->phase_deg;
    s_state.center_deg = params->center_deg;
    s_state.running = params->running;
    s_state.joystick_active = false;
    xSemaphoreGive(s_state_mutex);

    return ESP_OK;
}

esp_err_t servo_motion_set_config(float frequency_hz, float phase_deg, float center_deg)
{
    if (!isfinite(frequency_hz) ||
        frequency_hz < SERVO_MOTION_MIN_FREQUENCY_HZ ||
        frequency_hz > SERVO_MOTION_MAX_FREQUENCY_HZ ||
        !isfinite(phase_deg) ||
        phase_deg < SERVO_MOTION_MIN_PHASE_DEG ||
        phase_deg > SERVO_MOTION_MAX_PHASE_DEG ||
        !isfinite(center_deg) ||
        center_deg < SERVO_MOTION_MIN_CENTER_DEG ||
        center_deg > SERVO_MOTION_MAX_CENTER_DEG) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_state_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_state.frequency_hz = frequency_hz;
    s_state.phase_deg = phase_deg;
    s_state.center_deg = center_deg;
    clamp_mix(&s_state.amplitude_deg, &s_state.differential_deg, center_deg);
    xSemaphoreGive(s_state_mutex);

    return ESP_OK;
}

esp_err_t servo_motion_set_joystick(float x, float y, bool active)
{
    if (s_state_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!active) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        s_state.running = false;
        s_state.joystick_active = false;
        xSemaphoreGive(s_state_mutex);
        return ESP_OK;
    }

    if (!isfinite(x) || !isfinite(y) || fabsf(x) > 1.0f || fabsf(y) > 1.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    float amplitude_deg = SERVO_MOTION_JOYSTICK_AMPLITUDE_MID_DEG +
                          SERVO_MOTION_JOYSTICK_AMPLITUDE_SPAN_DEG * y;
    float differential_deg = SERVO_MOTION_MAX_DIFFERENTIAL_DEG * x;
    clamp_mix(&amplitude_deg, &differential_deg, s_state.center_deg);

    if (!s_state.running) {
        s_state.start_time_us = esp_timer_get_time();
    }
    s_state.amplitude_deg = amplitude_deg;
    s_state.differential_deg = differential_deg;
    s_state.running = true;
    s_state.joystick_active = true;
    s_state.last_joystick_us = esp_timer_get_time();
    xSemaphoreGive(s_state_mutex);

    return ESP_OK;
}
