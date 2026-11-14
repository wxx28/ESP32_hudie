/**
 * @file  rc_espnow.h
 * @brief ESP-NOW 遥控接收 + 丢包统计 + 绑定（接口契约，实现待补）
 * 负责人：链路组
 *
 * 设计（docs/03）：50Hz 控制帧；300ms 幅度归零；1s 迫降；MAC 存 NVS。
 */
#pragma once
#include "shared/protocol.h"

void rc_init();
bool rc_poll(rc_frame_t& out);      /* 取最新一帧；无新帧返回 false */
uint32_t rc_last_rx_ms();
float rc_loss_pct();
void rc_bind_start(uint32_t duration_ms);
