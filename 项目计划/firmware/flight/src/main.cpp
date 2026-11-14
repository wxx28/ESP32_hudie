/**
 * @file  main.cpp
 * @brief 飞控最小可编译骨架。
 *
 * 当前状态：只跑启动自检 + 心跳，功能按下面 TODO 接入（见 src/*.h 接口契约）。
 * 实现顺序建议：
 *   1) imu_mpu6500 + attitude   -> 串口打印姿态，倾斜板子验证
 *   2) servo_out                 -> 舵机中位/行程/反相验证（务必先测反相！）
 *   3) mixer + control           -> 台架拍翅、推力-频率曲线
 *   4) rc_espnow + telemetry     -> 联机、失控保护
 *   5) safety                    -> 状态机（自检/校准/待解锁/飞行/迫降）
 *
 * 单核注意：loop() 内禁止 delay()/Serial.print()/长 I2C。
 */
#include <Arduino.h>
#include <esp_timer.h>
#include "config.h"

static volatile bool g_tick = false;

static void on_tick(void*) { g_tick = true; }

void setup() {
    Serial.begin(115200);
    delay(50);                      /* 仅启动期允许 */
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);    /* 上电自检灯 */
    Serial.println("bfly flight skeleton v0.1");

    /* TODO 1: imu_mpu6500_init(PIN_SPI_*, PIN_IMU_CS) */
    /* TODO 2: servo_out_init()  -> 两舵机回中（注意 SERVO_INVERT） */
    /* TODO 3: rc_espnow_init()  -> 绑定 MAC + 接收回调 */
    /* TODO 4: safety_init()     -> 进入自检状态 */

    static esp_timer_handle_t s_ctrl_timer = nullptr;
    esp_timer_create_args_t args = {};
    args.callback = &on_tick;
    args.name = "ctrl";
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_ctrl_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_ctrl_timer, 1000000 / CONTROL_HZ));
}

void loop() {
    if (!g_tick) return;
    g_tick = false;

    /* 400Hz 控制拍：每拍只做一次 IMU 读取 + 姿态/控制/混控（TODO 接入） */
    /* TODO: imu_read / attitude_update / control_update / mixer_update / servo_write */
}
