#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "LED.h"
#include "button.h"
#include "espnow_link.h"

static const char *TAG = "app";

#define APP_PROTO_MAGIC 0x48554431u /* "HUD1" */
#define APP_PROTO_VERSION 1

typedef enum {
    MSG_LED_TOGGLE = 1, /* 请求对端翻转 LED */
    MSG_LED_STATE = 2,  /* 对端回 ACK，携带翻转后的状态 */
} app_msg_type_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint8_t seq;
    uint8_t led;
    uint32_t uptime_ms;
} app_msg_t;

static uint8_t s_seq;
static esp_timer_handle_t s_blink_timer;
static bool s_blink_restore_state;

static uint32_t uptime_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void send_msg(app_msg_type_t type, bool led_state)
{
    app_msg_t msg = {
        .magic = APP_PROTO_MAGIC,
        .version = APP_PROTO_VERSION,
        .type = (uint8_t)type,
        .seq = s_seq++,
        .led = led_state ? 1 : 0,
        .uptime_ms = uptime_ms(),
    };

    esp_err_t err = espnow_link_send((const uint8_t *)&msg, sizeof(msg));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "发送失败: %s", esp_err_to_name(err));
    }
}

static void blink_restore_cb(void *arg)
{
    (void)arg;
    led_set(s_blink_restore_state);
}

static void blink_once(void)
{
    if (s_blink_timer == NULL) {
        return;
    }
    s_blink_restore_state = led_get();
    led_set(true);
    esp_timer_stop(s_blink_timer);
    esp_timer_start_once(s_blink_timer, (uint64_t)CONFIG_APP_LED_ACK_BLINK_MS * 1000);
}

static void on_button_pressed(void)
{
    ESP_LOGI(TAG, "按键按下 -> 请求对端翻转 LED");
    blink_once();
    send_msg(MSG_LED_TOGGLE, led_get());
}

static void on_link_recv(const uint8_t *src_mac, const uint8_t *data, size_t len, int8_t rssi)
{
    if (len != sizeof(app_msg_t)) {
        ESP_LOGW(TAG, "忽略长度异常的数据: %u 字节", (unsigned)len);
        return;
    }

    app_msg_t msg;
    memcpy(&msg, data, sizeof(msg));
    if (msg.magic != APP_PROTO_MAGIC || msg.version != APP_PROTO_VERSION) {
        return;
    }

    switch (msg.type) {
    case MSG_LED_TOGGLE:
        led_toggle();
        ESP_LOGI(TAG, "收到 " MACSTR " 的翻转指令 (seq=%u, RSSI %d dBm) -> 本板 LED %s",
                 MAC2STR(src_mac), msg.seq, rssi, led_get() ? "ON" : "OFF");
        send_msg(MSG_LED_STATE, led_get());
        break;
    case MSG_LED_STATE:
        ESP_LOGI(TAG, "对端 " MACSTR " 确认 LED = %s (seq=%u, RSSI %d dBm)",
                 MAC2STR(src_mac), msg.led ? "ON" : "OFF", msg.seq, rssi);
        break;
    default:
        ESP_LOGW(TAG, "未知消息类型 %u", msg.type);
        break;
    }
}

static void on_link_send(bool success)
{
    if (!success) {
        ESP_LOGW(TAG, "发送未被确认（广播无重传）");
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(led_init(CONFIG_APP_LED_GPIO, CONFIG_APP_LED_ACTIVE_LOW));

    const esp_timer_create_args_t blink_args = {
        .callback = blink_restore_cb,
        .name = "led_blink",
    };
    ESP_ERROR_CHECK(esp_timer_create(&blink_args, &s_blink_timer));

    espnow_link_config_t cfg = {
        .channel = CONFIG_APP_ESPNOW_CHANNEL,
        .peer_mac = NULL, /* 广播 */
        .recv_cb = on_link_recv,
        .send_cb = on_link_send,
    };
    ESP_ERROR_CHECK(espnow_link_init(&cfg));

    uint8_t mac[6];
    ESP_ERROR_CHECK(espnow_link_get_mac(mac));
    ESP_LOGI(TAG, "本机 MAC " MACSTR ", 信道 %d", MAC2STR(mac), CONFIG_APP_ESPNOW_CHANNEL);
    ESP_LOGI(TAG, "把同一份固件烧到第二块板，按 GPIO%d 按键即可翻转对面板的 LED",
             CONFIG_APP_BUTTON_GPIO);

    ESP_ERROR_CHECK(button_init(CONFIG_APP_BUTTON_GPIO, CONFIG_APP_BUTTON_ACTIVE_LOW, on_button_pressed));

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

