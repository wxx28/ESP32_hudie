/**
 * @file  config.h
 * @brief 遥控器参数入口（引脚 / 摇杆标定 / 通道映射）
 * 依据 docs/03：ADC 只用 ADC1（IO32-39），避开与 WiFi 冲突的 ADC2。
 */
#pragma once

/* ---------------- 引脚（ESP32 经典双核） ---------------- */
#define PIN_JOY_LX      34   /* 左摇杆 X（油门）ADC1 */
#define PIN_JOY_LY      35
#define PIN_JOY_RX      32
#define PIN_JOY_RY      33
#define PIN_SW_MODE     25   /* 三段：模式 */
#define PIN_SW_ARM      26   /* 解锁 */
#define PIN_BTN_BIND    27
#define PIN_OLED_SDA    21
#define PIN_OLED_SCL    22
#define PIN_BUZZER      23
#define PIN_BAT_ADC     36   /* 遥控器电池 100K/100K */

/* ---------------- 摇杆 ---------------- */
#define JOY_ADC_SAMPLES   8
#define JOY_DEADZONE      30     /* LSB */
#define JOY_EXPO          0.3f
#define TX_HZ             50

/* ---------------- 通道映射 ---------------- */
#define CH_THROTTLE       0
#define CH_PITCH          1
#define CH_ROLL           2
#define CH_YAW            3
#define CH_MODE           4
#define CH_ARM            5

/* ---------------- 拍翅频率范围（与飞控一致） ---------------- */
#define FLAP_HZ_MIN       5.5f
#define FLAP_HZ_MAX       8.0f
#define FLAP_HZ_DEFAULT   7.5f

/* ---------------- 遥控器电池 ---------------- */
#define TX_BAT_DIVIDER    2.0f
#define TX_BAT_WARN_V     3.50f
