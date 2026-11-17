#pragma once

/*
 * 康师傅 4 代 EDU 遥控器（c3mini V2.19.SchDoc）引脚定义
 *
 * 唯一引脚来源：EDU版本遥控器/硬件电路/c3mini-V2.19.SchDoc（程序解析网表）
 * 与原 MicroPython 代码（Kang_Joystick.py / Kang_Key.py / Kang_RC.py）交叉核对。
 *
 *   GPIO  原理图网络   原理图按键名   本固件用途
 *   IO0   ADC_BAT      -              遥控器电池，R20/R21 62k/62k（×2）
 *   IO1   ADC_ROLL     右摇杆 J2.5    俯仰（原代码就是这样用的，网络名标反了）
 *   IO2   ADC_PITCH    右摇杆 J2.2    滚转
 *   IO3   ADC_THRUST   左摇杆 J1.2    油门
 *   IO4   ADC_YAW      左摇杆 J1.5    偏航
 *   IO5   KEY1         right          滚转微调 +
 *   IO6   KEY2         left           滚转微调 -（长按：微调清零）
 *   IO7   KEY3         down           俯仰微调 -；开机按住 = 进入绑定
 *   IO8   KEY4         up             俯仰微调 +
 *   IO9   KEY0         stop（与 BOOT 并联） 解锁 / 上锁
 *   IO10  LED          DS1 绿灯       状态灯，低电平点亮
 *   IO20  KEY5         up_right       长按 1 s（上锁时）= 飞控陀螺校准
 *   IO21  KEY6         up_left        双击 = 切换 增稳 / 手动
 *   IO18/19 USB D-/D+  Type-C         USB-Serial-JTAG 日志与烧录
 *
 * 注意：
 *   - 开机时按住 stop 会进入 ESP32 下载模式（IO9 是启动脚）；
 *   - IO20/IO21 是 UART0，本固件日志走 USB，所以可以当普通按键用；
 *   - Type-C 没有 CC 下拉电阻，C 对 C 线可能不供电，用 A 转 C 线。
 */

#define RC_PIN_ADC_BAT      0
#define RC_PIN_JOY_PITCH    1
#define RC_PIN_JOY_ROLL     2
#define RC_PIN_JOY_THROTTLE 3
#define RC_PIN_JOY_YAW      4
#define RC_PIN_KEY_RIGHT    5
#define RC_PIN_KEY_LEFT     6
#define RC_PIN_KEY_DOWN     7
#define RC_PIN_KEY_UP       8
#define RC_PIN_KEY_STOP     9
#define RC_PIN_LED          10
#define RC_PIN_KEY_UP_RIGHT 20
#define RC_PIN_KEY_UP_LEFT  21

#define RC_BAT_DIVIDER      2.0f
