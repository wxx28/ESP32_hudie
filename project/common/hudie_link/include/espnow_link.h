#pragma once

/*
 * ESP-NOW 收发封装（在 project/text 的 espnow_link 基础上扩展）
 *
 * 与原版的区别：
 *   - 始终注册广播对端，绑定阶段用广播；
 *   - 绑定成功后用 espnow_link_set_peer() 切换为单播，单播有 MAC 层确认；
 *   - 兼容 ESP-IDF 5.x（mac_addr 回调）与 5.5+/6.x（esp_now_send_info_t 回调）。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESPNOW_LINK_MAX_DATA_LEN 250

/* 收包回调：在 Wi-Fi 任务中执行，只能做拷贝，不能阻塞。 */
typedef void (*espnow_link_recv_cb_t)(const uint8_t *src_mac, const uint8_t *data, size_t len, int8_t rssi);
/* 发送结果回调：单播时 success 表示对端 MAC 层已确认。 */
typedef void (*espnow_link_send_cb_t)(bool success);

typedef struct {
    uint8_t channel;                 /* 两端必须一致，0 表示信道 1 */
    espnow_link_recv_cb_t recv_cb;
    espnow_link_send_cb_t send_cb;
} espnow_link_config_t;

/* 初始化 NVS / Wi-Fi(STA) / ESP-NOW，并注册广播对端。 */
esp_err_t espnow_link_init(const espnow_link_config_t *config);

/* 设置单播对端；mac 为 NULL 时回到广播。可重复调用。 */
esp_err_t espnow_link_set_peer(const uint8_t mac[6]);

/* 发给当前对端（未设置时为广播）。 */
esp_err_t espnow_link_send(const void *data, size_t len);

/* 强制广播（绑定阶段用）。 */
esp_err_t espnow_link_send_broadcast(const void *data, size_t len);

esp_err_t espnow_link_get_mac(uint8_t mac[6]);

/* 当前是否已有单播对端。 */
bool espnow_link_has_peer(void);

#ifdef __cplusplus
}
#endif
