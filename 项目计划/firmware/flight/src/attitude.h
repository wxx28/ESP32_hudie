/**
 * @file  attitude.h
 * @brief Mahony 互补滤波（接口契约，实现待补）
 * 负责人：算法组
 *
 * 设计（docs/01）：Kp=1.5, Ki=0.01, 400Hz；磁力计默认关闭。
 */
#pragma once
#include "imu_mpu6500.h"

struct Attitude {
    float roll_deg, pitch_deg, yaw_deg;
};

class AttitudeEstimator {
public:
    void reset();
    void update(const ImuSample& s, float dt);
    Attitude get() const;
private:
    float q_[4] = {1, 0, 0, 0};
    float kp_ = 1.5f, ki_ = 0.01f;
    float e_int_[3] = {0, 0, 0};
};
