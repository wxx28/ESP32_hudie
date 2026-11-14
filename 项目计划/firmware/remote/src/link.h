/**
 * @file  link.h
 * @brief ESP-NOW 发送/接收/绑定（接口契约，实现待补）
 * 负责人：链路组
 *
 * 设计（docs/03）：50Hz 控制帧、20Hz 遥测、MAC 存 NVS、绑定键流程。
 */
#pragma once
#include "shared/protocol.h"
#include "joystick.h"

void link_init();
void link_send(const JoyState& joy, uint8_t flags);
bool link_poll_telemetry(tele_frame_t& out);
bool link_is_bound();
void link_enter_bind_mode();
