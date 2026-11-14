/**
 * @file  servo_out.h
 * @brief LEDC 舵机输出（含光耦反相）（接口契约，实现待补）
 * 负责人：电控组
 *
 * 硬件：PWM_L=IO1 经 U9、PWM_R=IO8 经 U10，EL357 反相 -> duty = period - pulse。
 */
#pragma once
#include "mixer.h"

void servo_out_init();                 /* 配置 LEDC，先输出中位 */
void servo_out_write(const ServoCmd& cmd);
void servo_out_center();               /* 中位，用于装配/失控 */
void servo_out_disable();              /* 关闭 PWM（舵机保持最后位置） */
