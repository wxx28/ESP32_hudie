#include "hudie_bind.h"

#include <string.h>

#include "nvs.h"

#define BIND_NS  "hudie_bind"
#define BIND_KEY "peer_mac"

bool hudie_bind_load(uint8_t mac[6])
{
    nvs_handle_t h;
    if (nvs_open(BIND_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t len = 6;
    esp_err_t err = nvs_get_blob(h, BIND_KEY, mac, &len);
    nvs_close(h);
    return err == ESP_OK && len == 6;
}

esp_err_t hudie_bind_save(const uint8_t mac[6])
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(BIND_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, BIND_KEY, mac, 6);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t hudie_bind_clear(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(BIND_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(h, BIND_KEY);
    if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}
