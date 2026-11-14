/**
 * @file  joystick.h
 * @brief 摇杆采样/校准/死区/expo（接口契约，实现待补）
 * 负责人：链路组
 */
#pragma once

struct JoyState {
    float throttle;   /* 0..1 */
    float pitch;      /* -1..1 */
    float roll;       /* -1..1 */
    float yaw;        /* -1..1 */
    bool  arm;
    uint8_t mode;     /* 0/1/2 */
};

void joystick_init();
void joystick_calibrate();      /* 上电记录中位与两端极限 */
JoyState joystick_read();
