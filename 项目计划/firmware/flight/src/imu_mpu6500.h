/**
 * @file  imu_mpu6500.h
 * @brief MPU6500 SPI 驱动（接口契约，实现待补）
 * 负责人：传感器/算法组
 *
 * 硬件要点（docs/00）：
 *  - SPI：SCLK=IO6, MOSI=IO5, MISO=IO4, CS=IO7；
 *  - INT 未接 MCU -> 只能轮询，采样 400Hz；
 *  - 量程：陀螺 ±1000dps、加速度 ±8g；DLPF 20Hz。
 */
#pragma once
#include <stdint.h>

struct ImuSample {
    float acc_g[3];     /* g */
    float gyr_dps[3];   /* deg/s */
    float temp_c;
};

bool imu_init(int sclk, int mosi, int miso, int cs);
bool imu_read(ImuSample& out);
void imu_zero_bias(float gyr_bias_dps[3]);  /* 静止 2s 均值，写入 NVS */
