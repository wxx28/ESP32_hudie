/**
 * @file  config.h
 * @brief 飞控唯一参数入口（引脚 / 设计点 / 限幅）
 *
 * 引脚来源：docs/00-原理图解读与引脚表.md（已核对网络名）
 * 设计点：docs/04-总体结构与装配.md（75 g / 7.5 Hz / ±60°）
 * 改参数只改这里；改引脚必须同步修正文档。
 */
#pragma once

/* ---------------- 引脚 ---------------- */
#define PIN_I2C_SCL     2    /* IO2  SCL  LIS2MDL/SPL06 */
#define PIN_I2C_SDA     3    /* IO3  SDA */
#define PIN_SPI_SCLK    6    /* IO6  MPU6500 SCLK */
#define PIN_SPI_MOSI    5    /* IO5  MPU6500 SDI  */
#define PIN_SPI_MISO    4    /* IO4  MPU6500 SDO  */
#define PIN_IMU_CS      7    /* IO7  MPU6500 nCS  */
#define PIN_SERVO_L     1    /* IO1  PWM2 -> U9(EL357) -> PWM_L  （反相！） */
#define PIN_SERVO_R     8    /* IO8  PWM1 -> U10(EL357) -> PWM_R */
#define PIN_ADC_BAT     0    /* IO0  电池 100K/100K 分压 */
#define PIN_LED         10   /* IO10 高有效（Vbat_out 供电） */
#define PIN_BOOT        9    /* IO9  10K 上拉 + 按键 */
#define PIN_SBUS_RX     20   /* IO20 RXD0，SBUS 经 Q1 反相 */
#define PIN_UART_TX     21   /* IO21 TXD0 */

/* ---------------- 设计点（docs/04） ---------------- */
#define DESIGN_MASS_G        75.0f
#define WING_SPAN_MM         490.0f
#define FLAP_HZ_DEFAULT      7.5f
#define FLAP_HZ_MIN          5.5f
#define FLAP_HZ_MAX          8.0f
#define PHI_MAX_DEG          60.0f   /* 翼角半行程 */
#define PHI_LIMIT_MECH_DEG   68.0f   /* 机械限位 */
#define GEAR_RATIO           4.0f    /* 摇臂 20 / 舵机臂 5 */
#define SERVO_TRAVEL_DEG     (PHI_MAX_DEG / GEAR_RATIO)   /* ±15° */

/* ---------------- 舵机（含光耦反相） ---------------- */
#define SERVO_PWM_HZ         333
#define SERVO_PERIOD_US      (1000000 / SERVO_PWM_HZ)
#define SERVO_MIN_US         500
#define SERVO_MID_US         1500
#define SERVO_MAX_US         2500
#define SERVO_INVERT         1        /* EL357 输出反相：duty = period - pulse */
#define SERVO_SLEW_DEG_PER_TICK 3.0f  /* 每控制拍最多动 3° */

/* ---------------- 采样/控制节拍 ---------------- */
#define IMU_HZ               400
#define CONTROL_HZ           400
#define ATTITUDE_HZ          100
#define TELEM_HZ             20
#define SBUS_HZ              50

/* ---------------- 电池 ---------------- */
#define BAT_DIVIDER          2.0f     /* Vbat = 2 x Vadc */
#define BAT_WARN_V           3.50f
#define BAT_LAND_V           3.40f

/* ---------------- 失控保护 ---------------- */
#define RC_TIMEOUT_MS        300      /* 幅度归零 */
#define RC_LAND_MS           1000     /* 迫降 */
