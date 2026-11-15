# ESP32-C3 ESP-NOW 双板通信示例

两块 ESP32-C3 烧同一份固件，通过 ESP-NOW 直连（不需要路由器 / AP）：

- 按 A 板按键 → 广播 `LED_TOGGLE` → **B 板 LED 翻转** → B 板回 `LED_STATE` ACK
- A 板按键瞬间自身 LED 闪一下，表示指令已交给发送驱动
- 收包按源 MAC 过滤，广播回环不会让自己控制自己

## 接线

| 功能 | 引脚 | 电平 |
| --- | --- | --- |
| LED | GPIO3 | 低电平点亮（LED 阳极接 3V3，阴极经限流电阻接 GPIO3） |
| 按键 | GPIO0 | 低电平按下（按键另一端接 GND，内部上拉） |

> GPIO0 是 ESP32-C3 的 BOOT strapping 脚，复位瞬间不要按住按键，否则会进入下载模式。

引脚、电平、信道都可在 `idf.py menuconfig` → **HUDI ESP-NOW 双板通信** 中修改；两块板必须使用**同一信道**。

## 编译与烧录

```bash
idf.py build
idf.py -p COM3 flash monitor   # 烧第一块板
idf.py -p COM5 flash monitor   # 换成另一块板的串口再烧一次
```

两块板烧的是同一份 `build/sample_project_text.bin`，无需区分发送端 / 接收端。

## 预期日志

```
I (312) espnow_link: ready: mac 60:55:f9:xx:xx:01, channel 1, peer ff:ff:ff:ff:ff:ff
I (312) app: 本机 MAC 60:55:f9:xx:xx:01, 信道 1
I (412) app: 按键按下 -> 请求对端翻转 LED
I (418) app: 收到 60:55:f9:xx:xx:02 的翻转指令 (seq=0, RSSI -42 dBm) -> 本板 LED ON
I (421) app: 对端 60:55:f9:xx:xx:02 确认 LED = ON (seq=0, RSSI -45 dBm)
```

## 常见问题

- **完全收不到对方数据**：检查两块板的 `CONFIG_APP_ESPNOW_CHANNEL` 是否一致（改完要重新 build + flash）。
- **只收到自己发的包**：不会发生，`recv_handler` 已按源 MAC 丢弃本机帧。
- **RSSI 正常但偶发丢包**：ESP-NOW 广播没有链路层重传，干扰大时会丢；需要可靠传输可改为点对点单播（`espnow_link_config_t.peer_mac` 传入对端 MAC 即可）。
