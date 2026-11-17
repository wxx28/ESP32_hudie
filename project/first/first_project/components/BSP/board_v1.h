#pragma once

/*
 * 仿生蝴蝶 V1 飞控板（text 工程 / PCB2，48 × 22 mm）引脚定义
 *
 * 唯一引脚来源：蝴蝶V1_text_元器件引脚表.docx（EDA 网表导出，逐脚核对）。
 * ESP32-C3-MINI-1 模组焊盘号 -> GPIO：
 *
 *   焊盘  网络        GPIO   用途
 *   12    ADC_BAT     IO0    电池 1S 电压，R20/R21 100k/100k 分压（×2）
 *   13    PWM2        IO1    左翼舵机，R25 -> U9 光耦 -> PWM_L -> J3.3（反相）
 *   5     SCL         IO2    I2C：LIS2MDL / SPL06，4.7k 上拉
 *   6     SDA         IO3    I2C
 *   18    SDO         IO4    MPU6500 SDO（MISO）
 *   19    SDI         IO5    MPU6500 SDI（MOSI）
 *   20    SCLK        IO6    MPU6500 SCLK
 *   21    CS          IO7    MPU6500 nCS
 *   22    PWM1        IO8    右翼舵机，R27 -> U10 光耦 -> PWM_R -> J2.3（反相）
 *   23    BOOT        IO9    BOOT 键（SW2），10k 上拉，按下为低
 *   16    LED         IO10   R12 -> Q3(SI2302) 栅极，高电平点亮 LED2
 *   26/27 USB_D-/D+   IO18/19 USB-Serial-JTAG（日志/烧录走这里）
 *   30    RXD0        IO20   J6.4 / SBUS（Q1 反相），本版用 ESP-NOW，不使用
 *   31    TXD0        IO21   J6.3，不使用
 *
 * 注意：
 *   - 只有这一个文件允许写 GPIO 号；
 *   - MPU6500 的 INT 没有接到 MCU，只能轮询；
 *   - LED2 由 VBAT_OUT 供电：电源开关 SW3 没打开时即使插 USB 也不亮；
 *   - 舵机电源是 TPS61088 升压的 VBAT_2S（约 8.4 V），只能接 HV 舵机。
 */

#define BOARD_PIN_ADC_BAT     0
#define BOARD_PIN_SERVO_LEFT  1
#define BOARD_PIN_I2C_SCL     2
#define BOARD_PIN_I2C_SDA     3
#define BOARD_PIN_IMU_MISO    4
#define BOARD_PIN_IMU_MOSI    5
#define BOARD_PIN_IMU_SCLK    6
#define BOARD_PIN_IMU_CS      7
#define BOARD_PIN_SERVO_RIGHT 8
#define BOARD_PIN_BOOT_KEY    9
#define BOARD_PIN_LED         10

/* ADC_BAT 分压比：Vbat = Vadc × 2 */
#define BOARD_BAT_DIVIDER     2.0f
