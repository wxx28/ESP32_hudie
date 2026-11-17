#include "espnow_link.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "espnow_link";

static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t s_own_mac[6];
static uint8_t s_peer_mac[6];
static bool s_has_peer;
static uint8_t s_channel;
static espnow_link_recv_cb_t s_recv_cb;
static espnow_link_send_cb_t s_send_cb;
static bool s_initialized;

static esp_err_t nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        err = nvs_flash_init();
    }
    return err;
}

static void recv_handler(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (info == NULL || info->src_addr == NULL || data == NULL || len <= 0) {
        return;
    }
    /* 广播会回环到自己，按源 MAC 丢弃 */
    if (memcmp(info->src_addr, s_own_mac, sizeof(s_own_mac)) == 0) {
        return;
    }
    int8_t rssi = (info->rx_ctrl != NULL) ? (int8_t)info->rx_ctrl->rssi : 0;
    if (s_recv_cb != NULL) {
        s_recv_cb(info->src_addr, data, (size_t)len, rssi);
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void send_handler(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
#else
static void send_handler(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;
#endif
    if (s_send_cb != NULL) {
        s_send_cb(status == ESP_NOW_SEND_SUCCESS);
    }
}

static esp_err_t add_peer(const uint8_t mac[6])
{
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = s_channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    return (err == ESP_ERR_ESPNOW_EXIST) ? ESP_OK : err;
}

esp_err_t espnow_link_init(const espnow_link_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is NULL");
    ESP_RETURN_ON_FALSE(!s_initialized, ESP_ERR_INVALID_STATE, TAG, "already initialized");

    ESP_RETURN_ON_ERROR(nvs_init(), TAG, "nvs init failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_cfg), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "set storage failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");
    /* 关省电，保证收包延迟稳定 */
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "disable ps failed");

    s_channel = (config->channel != 0) ? config->channel : 1;
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE), TAG, "set channel failed");
    ESP_RETURN_ON_ERROR(esp_wifi_get_mac(WIFI_IF_STA, s_own_mac), TAG, "get mac failed");

    s_recv_cb = config->recv_cb;
    s_send_cb = config->send_cb;

    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "esp_now init failed");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(recv_handler), TAG, "register recv cb failed");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(send_handler), TAG, "register send cb failed");
    ESP_RETURN_ON_ERROR(add_peer(s_broadcast_mac), TAG, "add broadcast peer failed");

    s_initialized = true;
    ESP_LOGI(TAG, "ready: mac " MACSTR ", channel %u", MAC2STR(s_own_mac), s_channel);
    return ESP_OK;
}

esp_err_t espnow_link_set_peer(const uint8_t mac[6])
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    if (s_has_peer && (mac == NULL || memcmp(mac, s_peer_mac, 6) != 0)) {
        esp_now_del_peer(s_peer_mac);
        s_has_peer = false;
    }
    if (mac == NULL) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(add_peer(mac), TAG, "add peer failed");
    memcpy(s_peer_mac, mac, 6);
    s_has_peer = true;
    ESP_LOGI(TAG, "peer set to " MACSTR, MAC2STR(s_peer_mac));
    return ESP_OK;
}

esp_err_t espnow_link_send(const void *data, size_t len)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(data != NULL && len > 0 && len <= ESPNOW_LINK_MAX_DATA_LEN, ESP_ERR_INVALID_ARG, TAG,
                        "invalid payload");
    return esp_now_send(s_has_peer ? s_peer_mac : s_broadcast_mac, (const uint8_t *)data, len);
}

esp_err_t espnow_link_send_broadcast(const void *data, size_t len)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(data != NULL && len > 0 && len <= ESPNOW_LINK_MAX_DATA_LEN, ESP_ERR_INVALID_ARG, TAG,
                        "invalid payload");
    return esp_now_send(s_broadcast_mac, (const uint8_t *)data, len);
}

esp_err_t espnow_link_get_mac(uint8_t mac[6])
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    memcpy(mac, s_own_mac, 6);
    return ESP_OK;
}

bool espnow_link_has_peer(void)
{
    return s_has_peer;
}
