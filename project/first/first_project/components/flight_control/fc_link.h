#ifndef FC_LINK_H
#define FC_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_types.h"
#include "hudie_protocol.h"

/*
 * 飞控侧无线链路：接收遥控器控制帧、发送遥测、处理绑定
 *
 * 绑定规则：
 *   - 已绑定：只接受绑定 MAC 的控制帧；
 *   - 上电后 FC_LINK_BIND_WINDOW_MS 内（且未解锁）接受遥控器的绑定请求，
 *     存入 NVS 并单播回确认；
 *   - 从未绑定过：一直处于可绑定状态。
 *
 * 收包回调在 Wi-Fi 任务中运行，只做校验和拷贝。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define FC_LINK_CHANNEL         1u
#define FC_LINK_BIND_WINDOW_MS  15000u

typedef struct {
    rc_frame_t frame;
    uint64_t rx_time_us;    /* 0 表示从未收到 */
    int8_t rssi;
    uint8_t loss_pct;
} fc_link_rc_t;

bool fc_link_init(void);

/* 线程安全地取最新一帧遥控数据。 */
void fc_link_get_rc(fc_link_rc_t *out);

/* 是否已绑定遥控器。 */
bool fc_link_is_bound(void);

/* 设置是否允许接受绑定请求（飞控在解锁时关闭）。 */
void fc_link_allow_bind(bool allow);

/* 发送一帧遥测（20 Hz 调用）。 */
void fc_link_send_telemetry(const algo_output_t *out, uint32_t vbat_mv);

#ifdef __cplusplus
}
#endif

#endif /* FC_LINK_H */
