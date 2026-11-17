# 仿生蝴蝶 V1 飞控 + EDU 遥控器固件

适配硬件：

| 设备 | 硬件 | 引脚来源 |
|---|---|---|
| 飞控 | 自制 V1 板（text 工程 / PCB2，48×22 mm，ESP32-C3-MINI-1） | `蝴蝶V1_text_元器件引脚表.docx` → `first/first_project/components/BSP/board_v1.h` |
| 遥控器 | 康师傅 4 代 EDU 遥控器（c3mini V2.19） | `c3mini-V2.19.SchDoc` → `remote/remote_project/components/BSP_RC/board_rc.h` |

两边都是 ESP-IDF 工程（本机用 v5.4 编译通过，兼容 5.5 / 6.x 的 ESP-NOW 回调签名）。

```
project/
├─ common/hudie_link/            两端共用：协议帧(18/24/12 字节 + CRC16) / ESP-NOW / 绑定 NVS
├─ first/first_project/          飞控
│  ├─ components/BSP/            V1 板硬件：MPU6500(SPI)、舵机(LEDC 光耦反相)、电池 ADC、状态灯
│  ├─ components/Middlewares/algo/  算法（滤波/Mahony/串级PID/混控/相位/波形 + 本次补齐的连杆/安全/顶层）
│  ├─ components/flight_control/ 应用层：400 Hz 调度、输入组装、输出、无线链路、参数
│  ├─ main/main.c                只做配置 + 初始化 + 启动
│  └─ tests/                     主机测试（gcc 直接跑，不需要硬件）
└─ remote/remote_project/        遥控器
   ├─ components/BSP_RC/         摇杆 ADC、按键、绿灯
   └─ components/rc_app/         50 Hz 发送、绑定、解锁保护、微调、遥测
```

---

## 1. 编译与烧录

```bash
cd project/first/first_project      # 飞控
idf.py set-target esp32c3
idf.py build
idf.py -p COMx flash monitor

cd project/remote/remote_project    # 遥控器
idf.py set-target esp32c3
idf.py build
idf.py -p COMy flash monitor
```

日志都走 **USB-Serial-JTAG**（Type-C 直接连电脑），UART0 不用。

> 烧不进去：飞控按住 BOOT 键再插 USB；遥控器按住 stop 键再插 USB（stop 和 BOOT 是同一个脚）。

---

## 2. 第一次使用：绑定

1. 遥控器**按住 down 键开机**（从没绑定过会自动进入），绿灯 10 Hz 快闪；
2. 飞控上电。没绑定过的飞控随时接受绑定；已绑定过的飞控只在上电后 15 s 内接受；
3. 遥控器灯变成 1 Hz 慢闪 = 绑定成功。MAC 存在 NVS，之后开机自动连接，**不用再改代码填 MAC**。

---

## 3. 台架测试顺序（先不装翅膀！）

固件默认 **`FC_MECH_CONFIRMED = n`**：飞控能跑、能连遥控、遥控器能看到姿态，但**拒绝解锁扑翼**。按下面顺序验证完再打开：

| 步骤 | 做什么 | 看什么 |
|---|---|---|
| 1 | 飞控放平上电，静止 2 s | 日志 `MPU6500 ok`、`陀螺零偏 ...`；灯由 10 Hz 变双闪/慢闪 |
| 2 | 把飞控**机头抬起** | 日志 `pitch` 变**正**；不对就改 menuconfig →「IMU 安装方向」 |
| 3 | 把飞控**右侧压低** | 日志 `roll` 变**正** |
| 4 | 遥控器推杆 | 遥控器日志的姿态/信号/丢包正常；推油门上锁时舵机**不动** |
| 5 | menuconfig →「蝴蝶飞控参数」→ 打开 `FC_MECH_CONFIRMED`，重新烧 | — |
| 6 | 油门拉到底 → 单击 stop 解锁（灯常亮）→ 慢慢推油门 | 两个舵机同时摆，左右对称；油门越大越快 |
| 7 | 打右滚杆 | 左翼摆幅变大、右翼变小（右滚） |
| 8 | 关掉遥控器 | 0.3 s 后幅度减半，1 s 后进入失控保护：幅度在 2 s 内收到 35%，**不会突然停翅** |
| 9 | 重新打开遥控器 | 先单击 stop **上锁**，飞控才会回到待机；再解锁才能继续飞 |

左右方向不对：menuconfig 里改 `FC_SERVO_RIGHT_MIRRORED`；中位不对：改 `FC_SERVO_LEFT/RIGHT_NEUTRAL_DEG`，或者飞行中用遥控器 left/right/up/down 微调。

---

## 4. 遥控器操作

| 按键 | 操作 | 作用 |
|---|---|---|
| stop | 单击 | 解锁 / 上锁（**油门 > 5% 时拒绝解锁**） |
| up_left | 双击 | 切换 增稳 / 手动（默认手动） |
| up_right | 上锁时长按 1 s | 飞控重新校准陀螺（飞机要放平静止） |
| left / right | 单击 | 滚转微调 −/+ 1%；**长按 left 清零** |
| up / down | 单击 | 俯仰微调 +/− 1% |
| down | 按住开机 | 重新绑定 |

> 开机时**不要碰摇杆**（上电自动取中位），也**不要按住 stop**（会进下载模式）。

## 5. 指示灯

| 灯 | 飞控（LED2） | 遥控器（绿灯） |
|---|---|---|
| 10 Hz 快闪 | 自检 / 校准中 | 绑定中 |
| 双闪 | 待机，没连上遥控器 | 电量低（遥控器或飞控 < 3.5 V） |
| 1 Hz 慢闪 | 待机，遥控正常 | 上锁，链路正常 |
| 常亮 | 已解锁 | 已解锁 |
| 5 Hz | 失联降级 / 失控保护 / 电池低 | 收不到飞控回传 |
| 三长三短 | 急停 / 故障（上锁后可清除） | — |

> 飞控 LED2 由电池开关后的 VBAT_OUT 供电：**电源开关没打开时，插 USB 灯也不亮**。

---

## 6. 安全机制

| 条件 | 动作 |
|---|---|
| 上电时遥控器已在解锁状态 | 不起飞，必须"上锁 → 解锁" |
| 解锁时油门 > 5% | 拒绝（两端都检查） |
| 机构参数未确认 / 连杆不可用 | 拒绝解锁 |
| 遥控 300 ms 没更新 | 降级：幅度 50%、差动 20% |
| 遥控 1 s 没更新 | 失控保护：保持最后油门、杆量回中，幅度 2 s 斜坡到 35%，禁止差动 |
| 姿态 > 45° 或角速度 > 300°/s 持续 0.5 s | 急停，立即回中位 |
| 飞行中 IMU 读不到 | 急停 |
| 电池 < 3.4 V | 不能解锁；飞行中进入降级 |
| 算法内部错误 | 急停 |

## 7. 增稳

默认 `FC_ENABLE_STABILIZE = n`：PID 增益为 0，遥控器切到增稳也会自动退回手动直通（安全状态机判断"未整定"）。
打开后使用 `fc_config.c` 里的起步增益，**必须系绳试飞**，按 `项目计划/docs/01` 第 7 节逐级整定。

---

## 8. 主机测试

```bash
cd project/first/first_project
gcc -std=c11 -O2 -Icomponents/Middlewares/algo tests/test_api.c components/Middlewares/algo/*.c -lm -o test_api && ./test_api
```

`test_api.c` 覆盖：默认配置永不解锁、上电解锁位不起飞、油门高拒绝解锁、扑翼与滚转差动、
300 ms 降级 / 1 s 失控保护 / 幅度斜坡 / 禁止差动、恢复后必须重新上锁、姿态翻转急停与清除、四连杆逆解。
原有的 `test_attitude.c`、`test_control.c`、`test_mixer.c` 也全部通过。

## 9. 本次改动对原算法接口的影响

- `algo_rc_command_t` 新增 `stabilize_request`（遥控模式键）；
- `algo_linkage_config_t` 新增 `direct_drive`（默认 false，不影响原有逻辑）；
- 补齐了原来是空文件的 `algo_api.c`、`algo_safety.c/h`、`algo_linkage.c/h`、`flight_control` 全部 `.c`、`BSP/LED`；
- 原 `fc_input.h` 里放的其实是 fc_output 的接口，已原样移到 `fc_output.h`，`fc_input.h` 改为输入接口。
- 协议帧与 `项目计划/firmware/shared/protocol.h` 逐字节一致（C 版本在 `common/hudie_link/include/hudie_protocol.h`）。

## 10. 还没做 / 需要实测

- 气压计 SPL06、磁力计 LIS2MDL 还没读（I2C 已引出，`alt_cm` 遥测恒为 0）；
- 前馈、偏航闭环、增益调度未启用；
- 舵机速度限制了扑翼频率，默认 2.5–5.5 Hz，实测舵机后在 menuconfig 里调；
- ESP32-C3 ADC 有个体差异，电池电压请用万用表核一次。
