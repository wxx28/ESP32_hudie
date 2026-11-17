#include "fc_input.h"

#include <string.h>

#include "battery.h"
#include "esp_log.h"
#include "fc_link.h"
#include "mpu6500.h"

static const char *TAG = "fc_input";

static bool s_imu_ok;
static bool s_prev_calibrate_flag;
static bool s_calibrate_request;
static uint32_t s_bat_div;

static float ch_norm(int16_t v)
{
    float f = (float)v / 1000.0f;
    if (f > 1.0f) {
        f = 1.0f;
    }
    if (f < -1.0f) {
        f = -1.0f;
    }
    return f;
}

fc_input_status_t fc_input_init(void)
{
    fc_input_status_t st = FC_INPUT_STATUS_OK;

    if (battery_init() != ESP_OK) {
        ESP_LOGE(TAG, "电池 ADC 初始化失败");
        st = FC_INPUT_STATUS_HARDWARE_ERROR;
    }

    s_imu_ok = (mpu6500_init() == ESP_OK);
    if (!s_imu_ok) {
        ESP_LOGE(TAG, "MPU6500 初始化失败，飞控将停在自检状态");
        return FC_INPUT_STATUS_HARDWARE_ERROR;
    }

    /* 上电零偏校准：最多重试 5 次，每次 1 s */
    for (int i = 0; i < 5; ++i) {
        if (mpu6500_calibrate_gyro(1000) == ESP_OK) {
            return st;
        }
    }
    ESP_LOGW(TAG, "陀螺零偏校准一直失败（板子在动？），使用零偏 0；可用遥控器重新校准");
    return st;
}

fc_input_status_t fc_input_collect(uint64_t now_us, algo_input_t *in)
{
    if (in == NULL) {
        return FC_INPUT_STATUS_INVALID_ARG;
    }
    memset(in, 0, sizeof(*in));
    in->now_us = now_us;

    /* ---------- IMU ---------- */
    mpu6500_sample_t s;
    if (s_imu_ok && mpu6500_read(&s) == ESP_OK) {
        in->imu.timestamp_us = s.timestamp_us;
        in->imu.gyro_rad_s.x = s.gyro_rad_s[0];
        in->imu.gyro_rad_s.y = s.gyro_rad_s[1];
        in->imu.gyro_rad_s.z = s.gyro_rad_s[2];
        in->imu.accel_m_s2.x = s.accel_m_s2[0];
        in->imu.accel_m_s2.y = s.accel_m_s2[1];
        in->imu.accel_m_s2.z = s.accel_m_s2[2];
        in->imu.valid = true;
    } else {
        in->imu.timestamp_us = now_us;
        in->imu.valid = false;
    }

    /* ---------- 遥控 ---------- */
    fc_link_rc_t rc;
    fc_link_get_rc(&rc);
    if (rc.rx_time_us == 0u) {
        in->rc.age_ms = UINT32_MAX;
        in->rc.valid = false;
    } else {
        const uint64_t age_us = (now_us > rc.rx_time_us) ? (now_us - rc.rx_time_us) : 0u;
        in->rc.age_ms = (age_us / 1000u > UINT32_MAX) ? UINT32_MAX : (uint32_t)(age_us / 1000u);
        in->rc.valid = in->rc.age_ms < FC_INPUT_RC_VALID_MS;

        const rc_frame_t *f = &rc.frame;
        in->rc.throttle = (float)f->ch[RC_CH_THROTTLE] / 1000.0f;
        if (in->rc.throttle < 0.0f) {
            in->rc.throttle = 0.0f;
        }
        if (in->rc.throttle > 1.0f) {
            in->rc.throttle = 1.0f;
        }
        in->rc.roll = ch_norm(f->ch[RC_CH_ROLL]);
        in->rc.pitch = ch_norm(f->ch[RC_CH_PITCH]);
        in->rc.yaw = ch_norm(f->ch[RC_CH_YAW]);
        in->rc.arm_request = (f->flags & RC_FLAG_ARMED) != 0u;
        in->rc.stabilize_request = (f->flags & RC_FLAG_MANUAL) == 0u;

        const bool cal = in->rc.valid && (f->flags & RC_FLAG_CALIBRATE) != 0u;
        if (cal && !s_prev_calibrate_flag) {
            s_calibrate_request = true;
        }
        s_prev_calibrate_flag = cal;
    }

    /* ---------- 健康 ---------- */
    /* 电池 ADC 50 Hz 采一次即可（400 Hz 任务每 8 拍） */
    if ((s_bat_div++ % 8u) == 0u) {
        battery_update_mv();
    }
    const uint32_t vbat = battery_get_mv();
    in->health.imu_ready = s_imu_ok && mpu6500_is_ready();
    in->health.power_good = vbat >= FC_INPUT_BAT_LAND_MV;
    in->health.servo_fault[0] = false;
    in->health.servo_fault[1] = false;
    in->health.critical_output_fault = false;
    in->health.fault_flags = 0u;
    return FC_INPUT_STATUS_OK;
}

bool fc_input_take_calibrate_request(void)
{
    const bool r = s_calibrate_request;
    s_calibrate_request = false;
    return r;
}

fc_input_status_t fc_input_calibrate(void)
{
    if (!s_imu_ok) {
        return FC_INPUT_STATUS_HARDWARE_ERROR;
    }
    return (mpu6500_calibrate_gyro(1000) == ESP_OK) ? FC_INPUT_STATUS_OK : FC_INPUT_STATUS_HARDWARE_ERROR;
}

uint32_t fc_input_battery_mv(void)
{
    return battery_get_mv();
}
