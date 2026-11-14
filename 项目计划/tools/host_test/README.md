# host_test · PC 端算法自测

不依赖硬件，用 g++ 编译验证核心数学：

| 测试 | 验证内容 |
|---|---|
| `test_attitude.cpp` | Mahony 收敛：静态 1g 输入 -> 姿态 0°；倾斜 30° -> 输出 30° |
| `test_mixer.cpp` | 混控范围：油门/滚转/俯仰/偏航极值下舵机角不超 ±15° |
| `test_protocol.cpp` | 帧尺寸（18/24/12）、CRC 正反例、丢包统计 |

```bash
cd tools/host_test
g++ -std=c++17 -I../../firmware/shared -I../../firmware/flight/src     test_protocol.cpp -o t && ./t
```

注意：这些测试只测纯算法/协议，不包含 Arduino 依赖。
