#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 单包最大长度（esp_now.h 中 ESP_NOW_MAX_DATA_LEN 为 250，这里保持一致） */
#define ESPNOW_LINK_MAX_DATA_LEN 250

typedef void (*espnow_link_recv_cb_t)(const uint8_t *src_mac, const uint8_t *data, size_t len, int8_t rssi);
typedef void (*espnow_link_send_cb_t)(bool success);

typedef struct {
    uint8_t channel;                     /**< 固定 Wi-Fi 信道，通信双方必须一致（0 表示默认信道 1） */
    const uint8_t *peer_mac;             /**< 目标 MAC，NULL 表示广播 FF:FF:FF:FF:FF:FF */
    espnow_link_recv_cb_t recv_cb;       /**< 收包回调（已过滤掉本机自己发出的广播包） */
    espnow_link_send_cb_t send_cb;       /**< 发送完成回调，广播模式下仅代表已交给驱动 */
} espnow_link_config_t;

/** 初始化 NVS / Wi-Fi / ESP-NOW 并注册对端 */
esp_err_t espnow_link_init(const espnow_link_config_t *config);

/** 向配置的目标地址发送一帧数据 */
esp_err_t espnow_link_send(const uint8_t *data, size_t len);

/** 读取本机 STA MAC */
esp_err_t espnow_link_get_mac(uint8_t mac[6]);

/** 读取当前使用的目标 MAC（广播时为 FF:FF:FF:FF:FF:FF） */
const uint8_t *espnow_link_get_peer_mac(void);

#ifdef __cplusplus
}
#endif
