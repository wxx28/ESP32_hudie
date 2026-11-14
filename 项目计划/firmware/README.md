# 固件目录（骨架已建，功能未实现）

> 状态：**目录与接口已搭好，功能实现待评审后开始**。目前只有 `main.cpp` 会参与编译（最小可编译骨架），
> 各模块 `.h` 是**接口契约**，由对应负责人实现 `.cpp` 后接入 `main.cpp`。

## 两个固件

| 目录 | 目标板 | 作用 |
|---|---|---|
| `flight/` | 机上 ESP32-C3-MINI（板载） | 传感器 → 姿态 → 控制 → 混控 → 舵机；ESP-NOW 收遥控、回遥测；失控保护 |
| `remote/` | 遥控器 ESP32-DevKitC / C3 SuperMini | 双摇杆采样 → ESP-NOW 发送；接收遥测显示在 OLED |

## 共同约束（来自 docs/00 与 docs/03）

1. 舵机 PWM **必须反相**（EL357 光耦），`servo_out` 负责；
2. 协议头 `shared/protocol.h` **两端共用，禁止各自改**；改动必须走"协议冻结"评审；
3. `config.h` 是唯一参数入口（引脚/设计点/限幅），不要散落在 .cpp 里；
4. 机上 C3 是**单核**：主循环禁止 `delay()`、`Serial.print`、长 I2C 事务。

## 烧录

```bash
cd flight  && pio run -t upload     # 机上
cd remote  && pio run -t upload     # 遥控器
```
