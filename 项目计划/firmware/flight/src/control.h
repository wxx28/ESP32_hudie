/**
 * @file  control.h
 * @brief 串级 PID + 相位前馈（接口契约，实现待补）
 * 负责人：算法组
 *
 * 设计（docs/01）：角速度环 400Hz（P0.8 I0.08 D0.002），角度环 100Hz（P3.0），
 * 前馈 u_ff(psi) 用 2 阶傅里叶系数（台架辨识后写入 config.h）。
 */
#pragma once
#include "attitude.h"

struct RcCmd {           /* 归一化 -1..1，ch0 油门 0..1 */
    float throttle, pitch, roll, yaw;
    bool armed;
};

struct CtrlOut {
    float amp;           /* 拍翅幅度 0..1 -> ±PHI_MAX_DEG */
    float offset_deg;    /* 两翼平均行程偏置（俯仰） */
    float diff_deg;      /* 左右差动（滚转） */
    float phase_deg;     /* 左右相位差（偏航） */
    float hz;            /* 拍翅频率 */
};

void control_init();
CtrlOut control_update(const Attitude& att, const RcCmd& rc, float dt);
