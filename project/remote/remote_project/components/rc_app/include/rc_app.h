#pragma once

/*
 * EDU 遥控器应用层
 *
 * 操作：
 *   stop 键（单击）       ：解锁 / 上锁。解锁只在油门 < 5% 时生效
 *   up_left 键（双击）    ：切换 增稳 / 手动（默认手动；飞控未整定时增稳自动退回手动）
 *   up_right 键（长按 1 s）：上锁时让飞控重新校准陀螺（飞机要放平静止）
 *   left / right 单击     ：滚转微调 -/+（长按 left：微调清零）
 *   up / down 单击        ：俯仰微调 +/-
 *   开机时按住 down        ：进入绑定（没绑定过也会自动进入）
 *
 * 状态灯：
 *   10 Hz 快闪 = 绑定中；5 Hz = 收不到飞控回传；1 Hz = 上锁正常；
 *   常亮 = 已解锁；双闪 = 电量低（遥控器 < 3.5 V 或飞控 < 3.5 V）
 */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rc_app_start(void);

#ifdef __cplusplus
}
#endif
