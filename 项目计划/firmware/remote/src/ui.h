/**
 * @file  ui.h
 * @brief OLED 状态页 + 蜂鸣器（接口契约，实现待补）
 * 负责人：链路组
 * 页面内容：飞机电压/姿态/丢包 + 遥控器电量/信号。
 */
#pragma once

struct UiState {
    float telem_vbat_v;
    float telem_roll_deg, telem_pitch_deg;
    uint8_t telem_state;
    uint8_t loss_pct;
    float tx_bat_v;
    bool link_ok;
};

void ui_init();
void ui_update(const UiState& s);   /* 10Hz 刷新即可 */
void ui_beep(uint16_t ms);
