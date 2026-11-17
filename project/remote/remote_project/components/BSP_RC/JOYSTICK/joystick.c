#include "joystick.h"

#include <stdbool.h>
#include <stdlib.h>

#include "board_rc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "joystick";

/* ESP32-C3：IO0~IO4 = ADC1 通道 0~4 */
enum { AX_PITCH = 0, AX_ROLL, AX_THR, AX_YAW, AX_COUNT };
static const adc_channel_t s_chan[AX_COUNT] = {
    (adc_channel_t)RC_PIN_JOY_PITCH,
    (adc_channel_t)RC_PIN_JOY_ROLL,
    (adc_channel_t)RC_PIN_JOY_THROTTLE,
    (adc_channel_t)RC_PIN_JOY_YAW,
};

#define AVG_N 4

/* bool Kconfig 关闭时宏不存在，这里统一成 0/1 */
#ifdef CONFIG_RC_INVERT_ROLL
#define INV_ROLL true
#else
#define INV_ROLL false
#endif
#ifdef CONFIG_RC_INVERT_PITCH
#define INV_PITCH true
#else
#define INV_PITCH false
#endif
#ifdef CONFIG_RC_INVERT_YAW
#define INV_YAW true
#else
#define INV_YAW false
#endif

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_bat_cali;
static int s_center[AX_COUNT];
static int s_hist[AX_COUNT][AVG_N];
static int s_hist_idx;
static float s_bat_mv;

static int read_raw(adc_channel_t ch)
{
    int v = 0;
    adc_oneshot_read(s_adc, ch, &v);
    return v;
}

esp_err_t joystick_init(void)
{
    adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit, &s_adc), TAG, "adc unit");
    adc_oneshot_chan_cfg_t cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    for (int i = 0; i < AX_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, s_chan[i], &cfg), TAG, "ch %d", i);
    }
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, (adc_channel_t)RC_PIN_ADC_BAT, &cfg), TAG, "bat");

    adc_cali_curve_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1,
        .chan = (adc_channel_t)RC_PIN_ADC_BAT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal, &s_bat_cali) != ESP_OK) {
        s_bat_cali = NULL;
    }

    /* 中位：100 次平均 */
    long sum[AX_COUNT] = {0};
    for (int n = 0; n < 100; ++n) {
        for (int i = 0; i < AX_COUNT; ++i) {
            sum[i] += read_raw(s_chan[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    for (int i = 0; i < AX_COUNT; ++i) {
        s_center[i] = (int)(sum[i] / 100);
        for (int k = 0; k < AVG_N; ++k) {
            s_hist[i][k] = s_center[i];
        }
    }
    ESP_LOGI(TAG, "摇杆中位 pitch=%d roll=%d yaw=%d（油门 raw=%d）", s_center[AX_PITCH], s_center[AX_ROLL],
             s_center[AX_YAW], s_center[AX_THR]);
    if (abs(s_center[AX_ROLL] - 2048) > 700 || abs(s_center[AX_PITCH] - 2048) > 700 ||
        abs(s_center[AX_YAW] - 2048) > 700) {
        ESP_LOGW(TAG, "中位偏离很大：开机时是不是碰了摇杆？请松手后重启遥控器");
    }
    return ESP_OK;
}

static int16_t centered(int raw, int center, bool invert)
{
    int v = (raw - center) * 1000 / CONFIG_RC_JOY_SPAN_COUNTS;
    if (invert) {
        v = -v;
    }
    const int dz = CONFIG_RC_JOY_DEADZONE_PERMILLE;
    if (abs(v) < dz) {
        v = 0;
    } else {
        /* 死区外重新拉伸，保证满杆仍是 ±1000 */
        v = (v > 0 ? v - dz : v + dz) * 1000 / (1000 - dz);
    }
    if (v > 1000) {
        v = 1000;
    }
    if (v < -1000) {
        v = -1000;
    }
    return (int16_t)v;
}

void joystick_read(joystick_values_t *out)
{
    int avg[AX_COUNT];
    for (int i = 0; i < AX_COUNT; ++i) {
        s_hist[i][s_hist_idx] = read_raw(s_chan[i]);
        int s = 0;
        for (int k = 0; k < AVG_N; ++k) {
            s += s_hist[i][k];
        }
        avg[i] = s / AVG_N;
    }
    s_hist_idx = (s_hist_idx + 1) % AVG_N;

    int thr = avg[AX_THR] * 1000 / CONFIG_RC_THROTTLE_FULL_COUNTS;
#if CONFIG_RC_INVERT_THROTTLE
    thr = 1000 - thr;
#endif
    if (thr < 0) {
        thr = 0;
    }
    if (thr > 1000) {
        thr = 1000;
    }
    out->throttle = (int16_t)thr;
    out->roll = centered(avg[AX_ROLL], s_center[AX_ROLL], INV_ROLL);
    out->pitch = centered(avg[AX_PITCH], s_center[AX_PITCH], INV_PITCH);
    out->yaw = centered(avg[AX_YAW], s_center[AX_YAW], INV_YAW);
}

uint32_t joystick_read_battery_mv(void)
{
    const int raw = read_raw((adc_channel_t)RC_PIN_ADC_BAT);
    int mv = 0;
    if (s_bat_cali == NULL || adc_cali_raw_to_voltage(s_bat_cali, raw, &mv) != ESP_OK) {
        mv = raw * 3100 / 4095;
    }
    const float v = (float)mv * RC_BAT_DIVIDER;
    s_bat_mv = (s_bat_mv <= 0.0f) ? v : s_bat_mv + 0.05f * (v - s_bat_mv);
    return (uint32_t)s_bat_mv;
}
