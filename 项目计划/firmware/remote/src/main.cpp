/**
 * @file  main.cpp
 * @brief 遥控器最小可编译骨架（独立无线手持设备）。
 *
 * 当前状态：串口 + 心跳；模块按 TODO 接入（见 joystick.h / ui.h / link.h）。
 * 全程不插线：USB 只用于充电/烧录。
 */
#include <Arduino.h>
#include "config.h"

void setup() {
    Serial.begin(115200);
    delay(50);
    Serial.println("bfly remote skeleton v0.1");
    /* TODO: joystick_init() */
    /* TODO: link_init()      ESP-NOW 配对 + 50Hz 发送 */
    /* TODO: ui_init()        OLED 状态页 + 蜂鸣器 */
}

void loop() {
    /* TODO: joy = joystick_read(); link_send(joy); ui_update(); */
    delay(20);   /* 骨架占位；正式版用 50Hz 定时，不用 delay */
}
