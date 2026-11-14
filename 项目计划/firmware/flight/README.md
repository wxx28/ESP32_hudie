# flight · 机上飞控（ESP32-C3）

## 模块与负责人（填名字）

| 文件 | 职责 | 负责人 | 状态 |
|---|---|---|---|
| `src/imu_mpu6500.*` | SPI 驱动、量程、零偏 | （待填） | 接口已定义 |
| `src/attitude.*` | Mahony 姿态解算 | （待填） | 接口已定义 |
| `src/control.*` | 串级 PID + 前馈 | （待填） | 接口已定义 |
| `src/mixer.*` | 相位发生器、四自由度分配、逆解 | （待填） | 接口已定义 |
| `src/servo_out.*` | LEDC 反相输出 | （待填） | 接口已定义 |
| `src/rc_espnow.*` | ESP-NOW 接收/绑定/丢包 | （待填） | 接口已定义 |
| `src/telemetry.*` | 20Hz 遥测 | （待填） | 接口已定义 |
| `src/safety.*` | 状态机与限幅 | （待填） | 接口已定义 |

## 实现顺序（建议）

1. `imu_mpu6500` + `attitude`：串口打印 roll/pitch，倾斜板子校验；
2. `servo_out`：中位/行程/**反相**验证（示波器或舵机测试仪）；
3. `mixer` + `control`：台架拍翅，测推力-频率曲线；
4. `rc_espnow` + `telemetry`：联机、丢包率、失控保护；
5. `safety`：状态机收口。

## 不要动

- `include/config.h` 的引脚映射（改必须同步 docs/00）；
- `../shared/protocol.h`（协议改动走评审）。
