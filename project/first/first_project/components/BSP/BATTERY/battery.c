#include "battery.h"

#include "board_v1.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp_battery";

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static float s_filtered_mv;
static bool s_primed;

esp_err_t battery_init(void)
{
    adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit, &s_adc), TAG, "adc unit");

    adc_oneshot_chan_cfg_t ch = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_0, &ch), TAG, "adc channel");

    adc_cali_curve_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_0,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) != ESP_OK) {
        s_cali = NULL;
        ESP_LOGW(TAG, "芯片没有 ADC 校准数据，电压按原始值估算，误差可能较大");
    }
    return ESP_OK;
}

static uint32_t read_once_mv(void)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc, ADC_CHANNEL_0, &raw) != ESP_OK) {
        return 0;
    }
    int mv = 0;
    if (s_cali != NULL && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
        return (uint32_t)((float)mv * BOARD_BAT_DIVIDER);
    }
    return (uint32_t)((float)raw * 3100.0f / 4095.0f * BOARD_BAT_DIVIDER);
}

uint32_t battery_update_mv(void)
{
    if (s_adc == NULL) {
        return 0;
    }
    const float mv = (float)read_once_mv();
    if (!s_primed) {
        s_filtered_mv = mv;
        s_primed = true;
    } else {
        /* 以 50 Hz 调用时约 0.5 s 时间常数 */
        s_filtered_mv += 0.04f * (mv - s_filtered_mv);
    }
    return (uint32_t)s_filtered_mv;
}

uint32_t battery_get_mv(void)
{
    return (uint32_t)s_filtered_mv;
}
