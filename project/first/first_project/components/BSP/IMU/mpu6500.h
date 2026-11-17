#pragma once

/*
 * MPU6500 SPI 驱动（V1 板 U13，INT 未接，轮询）
 *
 * 量程：陀螺 ±1000 dps，加速度 ±8 g；片内 DLPF 41 Hz，采样 1 kHz。
 * 输出已经换算成 SI 单位，并按 Kconfig 的安装方向转到机体系
 * （X 前 / Y 右 / Z 下），陀螺已减去上电零偏。
 *
 * 只负责采样，不做滤波和姿态（那是 algo 的事）。
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gyro_rad_s[3];   /* 机体系角速度 */
    float accel_m_s2[3];   /* 机体系比力（水平静止时约为 0, 0, -9.81） */
    float temp_c;
    uint64_t timestamp_us;
} mpu6500_sample_t;

/* 初始化 SPI 总线和芯片，检查 WHO_AM_I。 */
esp_err_t mpu6500_init(void);

/* 读取一次传感器（约 150 us）。 */
esp_err_t mpu6500_read(mpu6500_sample_t *out);

/*
 * 陀螺零偏校准：要求板子静止，采样 duration_ms。
 * 检测到晃动（方差过大）返回 ESP_ERR_INVALID_STATE，零偏保持不变。
 */
esp_err_t mpu6500_calibrate_gyro(uint32_t duration_ms);

bool mpu6500_is_ready(void);

#ifdef __cplusplus
}
#endif
