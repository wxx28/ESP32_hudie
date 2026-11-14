/**
 * @file  mixer.h
 * @brief 相位发生器 + 四自由度分配 + 四杆逆解（接口契约，实现待补）
 * 负责人：算法组 / 机构组（inv_link 参数）
 */
#pragma once
#include "control.h"
#include "config.h"

struct ServoCmd {
    float deg[2];        /* 舵机目标角 ±SERVO_TRAVEL_DEG */
};

void mixer_init(float hz);
void mixer_update(const CtrlOut& c, ServoCmd& out);
float mixer_phase_rad();        /* 当前拍翅相位，供前馈/同步 */

/* 四杆逆解：期望翼角 -> 舵机角（查表或解析）；机试后标定写入表 */
float inv_link_deg(float wing_deg, int side);
