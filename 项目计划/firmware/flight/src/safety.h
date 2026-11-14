/**
 * @file  safety.h
 * @brief 状态机：自检 / 校准 / 待解锁 / 飞行 / 迫降 / 故障（接口契约，实现待补）
 * 负责人：飞控组
 */
#pragma once
#include "control.h"

enum FlightState : uint8_t {
    ST_SELFTEST = 0, ST_CALIB = 1, ST_ARMED_WAIT = 2,
    ST_FLYING = 3, ST_LANDING = 4, ST_FAULT = 5
};

void safety_init();
void safety_step(float dt);
FlightState safety_state();
bool safety_can_arm();
RcCmd safety_limit(const RcCmd& in);   /* 限幅/软启动/失控接管 */
