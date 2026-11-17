#include "rc_app.h"

#include <string.h>

#include "espnow_link.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hudie_bind.h"
#include "hudie_protocol.h"
#include "joystick.h"
#include "keys.h"
#include "nvs.h"
#include "rc_led.h"

static const char *TAG = "rc";

#define RC_CHANNEL          1u      /* 必须和飞控 FC_LINK_CHANNEL 一致 */
#define TX_PERIOD_MS        20      /* 50 Hz */
#define BIND_PERIOD_MS      200
#define TELEM_TIMEOUT_MS    1000
#define ARM_THROTTLE_MAX    50      /* 0..1000，即 5% */
#define TRIM_STEP           10      /* 1% */
#define TRIM_MAX            200
#define CALIBRATE_HOLD_MS   1000
#define LOW_BAT_MV          3500

/* ---------- 共享状态（收包回调写，任务读） ---------- */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static tele_frame_t s_tele;
static int64_t s_tele_time_us;
static volatile bool s_bind_accepted;
static uint8_t s_accept_mac[6];

/* ---------- 遥控器状态 ---------- */
static uint8_t s_peer[6];
static bool s_bound;
static bool s_binding;
static bool s_armed;
static bool s_manual = true;
static int16_t s_trim_roll, s_trim_pitch;
static bool s_trim_dirty;
static int64_t s_trim_changed_us;
static int64_t s_calibrate_until_us;
static uint8_t s_seq;

static void on_recv(const uint8_t *src, const uint8_t *data, size_t len, int8_t rssi)
{
    (void)rssi;
    if (len == sizeof(bind_frame_t) && hudie_proto_check(data, len, sizeof(bind_frame_t), TYPE_BIND)) {
        const bind_frame_t *b = (const bind_frame_t *)data;
        if (b->cmd == BIND_CMD_ACCEPT && s_binding) {
            memcpy(s_accept_mac, src, 6);
            s_bind_accepted = true;
        }
        return;
    }
    if (len == sizeof(tele_frame_t) && hudie_proto_check(data, len, sizeof(tele_frame_t), TYPE_TELEM)) {
        if (!s_bound || memcmp(src, s_peer, 6) != 0) {
            return;
        }
        portENTER_CRITICAL(&s_lock);
        memcpy(&s_tele, data, sizeof(s_tele));
        s_tele_time_us = esp_timer_get_time();
        portEXIT_CRITICAL(&s_lock);
    }
}

/* ---------- 微调存 NVS ---------- */
static void trims_load(void)
{
    nvs_handle_t h;
    if (nvs_open("rc_trim", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i16(h, "roll", &s_trim_roll);
        nvs_get_i16(h, "pitch", &s_trim_pitch);
        nvs_close(h);
    }
}

static void trims_save(void)
{
    nvs_handle_t h;
    if (nvs_open("rc_trim", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i16(h, "roll", s_trim_roll);
        nvs_set_i16(h, "pitch", s_trim_pitch);
        nvs_commit(h);
        nvs_close(h);
    }
}

static int16_t clamp16(int v, int lo, int hi)
{
    return (int16_t)(v < lo ? lo : (v > hi ? hi : v));
}

static void adjust_trim(int16_t *t, int delta)
{
    *t = clamp16(*t + delta, -TRIM_MAX, TRIM_MAX);
    s_trim_dirty = true;
    s_trim_changed_us = esp_timer_get_time();
    rc_led_blip();
    ESP_LOGI(TAG, "微调 roll=%d pitch=%d", s_trim_roll, s_trim_pitch);
}

/* ---------- 按键 ---------- */
static void handle_keys(const joystick_values_t *js)
{
    const int64_t now = esp_timer_get_time();

    if (keys_take_event(KEY_STOP) == KEY_EVT_CLICK) {
        if (s_armed) {
            s_armed = false;
            ESP_LOGI(TAG, "上锁");
        } else if (js->throttle > ARM_THROTTLE_MAX) {
            ESP_LOGW(TAG, "油门没拉到底（%d%%），拒绝解锁", js->throttle / 10);
            rc_led_blip();
        } else {
            s_armed = true;
            ESP_LOGI(TAG, "解锁");
        }
    }

    if (keys_take_event(KEY_UP_LEFT) == KEY_EVT_DOUBLE) {
        s_manual = !s_manual;
        rc_led_blip();
        ESP_LOGI(TAG, "模式：%s", s_manual ? "手动" : "增稳（飞控未整定时自动退回手动）");
    }

    if (keys_take_event(KEY_UP_RIGHT) == KEY_EVT_LONG) {
        if (!s_armed) {
            s_calibrate_until_us = now + CALIBRATE_HOLD_MS * 1000;
            rc_led_blip();
            ESP_LOGI(TAG, "请求飞控陀螺校准（飞机放平别动）");
        }
    }

    switch (keys_take_event(KEY_LEFT)) {
    case KEY_EVT_CLICK:
        adjust_trim(&s_trim_roll, -TRIM_STEP);
        break;
    case KEY_EVT_LONG:
        s_trim_roll = 0;
        s_trim_pitch = 0;
        adjust_trim(&s_trim_roll, 0);
        break;
    default:
        break;
    }
    if (keys_take_event(KEY_RIGHT) == KEY_EVT_CLICK) {
        adjust_trim(&s_trim_roll, TRIM_STEP);
    }
    if (keys_take_event(KEY_UP) == KEY_EVT_CLICK) {
        adjust_trim(&s_trim_pitch, TRIM_STEP);
    }
    if (keys_take_event(KEY_DOWN) == KEY_EVT_CLICK) {
        adjust_trim(&s_trim_pitch, -TRIM_STEP);
    }

    /* 微调停止变化 2 s 后再写 flash，减少擦写 */
    if (s_trim_dirty && now - s_trim_changed_us > 2000000) {
        s_trim_dirty = false;
        trims_save();
    }
}

/* ---------- 绑定 ---------- */
static void bind_step(void)
{
    if (s_bind_accepted) {
        s_bind_accepted = false;
        memcpy(s_peer, s_accept_mac, 6);
        hudie_bind_save(s_peer);
        espnow_link_set_peer(s_peer);
        s_bound = true;
        s_binding = false;
        ESP_LOGI(TAG, "绑定成功：飞控 " MACSTR, MAC2STR(s_peer));
        return;
    }
    bind_frame_t b = {.cmd = BIND_CMD_REQUEST};
    espnow_link_get_mac(b.mac);
    hudie_proto_finalize(&b, sizeof(b), TYPE_BIND);
    espnow_link_send_broadcast(&b, sizeof(b));
}

/* ---------- 主循环 ---------- */
static void rc_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    uint32_t n = 0;
    uint32_t key_div = 0;
    uint32_t vbat = 0;

    while (true) {
        /* 按键 10 ms 一次，发送 20 ms 一次 */
        keys_poll();
        if ((key_div++ & 1u) == 0u) {
            vTaskDelayUntil(&last, pdMS_TO_TICKS(TX_PERIOD_MS / 2));
            continue;
        }

        joystick_values_t js;
        joystick_read(&js);
        handle_keys(&js);
        if ((n % 10u) == 0u) {
            vbat = joystick_read_battery_mv();
        }

        const int64_t now = esp_timer_get_time();

        if (s_binding) {
            if ((n % (BIND_PERIOD_MS / TX_PERIOD_MS)) == 0u) {
                bind_step();
            }
            rc_led_set(RC_LED_RAPID);
        } else {
            rc_frame_t f = {0};
            f.seq = s_seq++;
            f.flags = (s_armed ? RC_FLAG_ARMED : 0u) | (s_manual ? RC_FLAG_MANUAL : 0u) |
                      ((now < s_calibrate_until_us) ? RC_FLAG_CALIBRATE : 0u);
            f.ch[RC_CH_THROTTLE] = js.throttle;
            f.ch[RC_CH_ROLL] = clamp16(js.roll + s_trim_roll, -1000, 1000);
            f.ch[RC_CH_PITCH] = clamp16(js.pitch + s_trim_pitch, -1000, 1000);
            f.ch[RC_CH_YAW] = js.yaw;
            f.ch[RC_CH_MODE] = s_manual ? 1000 : 0;
            f.ch[RC_CH_ARM] = s_armed ? 1000 : 0;
            hudie_proto_finalize(&f, sizeof(f), TYPE_RC);
            espnow_link_send(&f, sizeof(f));

            tele_frame_t t;
            int64_t t_time;
            portENTER_CRITICAL(&s_lock);
            t = s_tele;
            t_time = s_tele_time_us;
            portEXIT_CRITICAL(&s_lock);
            const bool link_ok = t_time != 0 && (now - t_time) < TELEM_TIMEOUT_MS * 1000;
            const bool low_bat = (vbat > 1000u && vbat < LOW_BAT_MV) ||
                                 (link_ok && t.vbat_mv > 1000u && t.vbat_mv < LOW_BAT_MV);

            if (!link_ok) {
                rc_led_set(RC_LED_FAST);
            } else if (low_bat) {
                rc_led_set(RC_LED_DOUBLE);
            } else {
                rc_led_set(s_armed ? RC_LED_ON : RC_LED_SLOW);
            }

            if ((n % 50u) == 0u) {
                static const char *st[] = {"自检", "校准", "待机", "飞行", "失控保护", "故障"};
                if (link_ok) {
                    ESP_LOGI(TAG, "飞控[%s] 电池 %umV 横滚 %.1f° 俯仰 %.1f° 信号 -%udBm 丢包 %u%% | 遥控 %s %s 油门 %d%% 电池 %umV",
                             t.state < 6u ? st[t.state] : "?", t.vbat_mv, t.roll / 10.0f, t.pitch / 10.0f, t.rssi_dbm,
                             t.loss_pct, s_armed ? "解锁" : "上锁", s_manual ? "手动" : "增稳", js.throttle / 10,
                             (unsigned)vbat);
                } else {
                    ESP_LOGW(TAG, "收不到飞控回传 | 遥控 %s 油门 %d%% 电池 %umV", s_armed ? "解锁" : "上锁",
                             js.throttle / 10, (unsigned)vbat);
                }
            }
        }
        n++;
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TX_PERIOD_MS / 2));
    }
}

esp_err_t rc_app_start(void)
{
    ESP_ERROR_CHECK(rc_led_init());
    rc_led_set(RC_LED_RAPID);
    ESP_ERROR_CHECK(keys_init());
    const bool bind_key = keys_raw_down(KEY_DOWN);

    ESP_ERROR_CHECK(joystick_init());

    espnow_link_config_t cfg = {.channel = RC_CHANNEL, .recv_cb = on_recv, .send_cb = NULL};
    ESP_ERROR_CHECK(espnow_link_init(&cfg));
    uint8_t mac[6];
    espnow_link_get_mac(mac);
    ESP_LOGI(TAG, "遥控器 MAC " MACSTR "，信道 %u", MAC2STR(mac), (unsigned)RC_CHANNEL);

    trims_load();

    if (!bind_key && hudie_bind_load(s_peer)) {
        s_bound = true;
        espnow_link_set_peer(s_peer);
        ESP_LOGI(TAG, "已绑定飞控 " MACSTR "（重新绑定：按住 down 开机）", MAC2STR(s_peer));
    } else {
        s_binding = true;
        ESP_LOGI(TAG, "进入绑定：请在 15 s 内给飞控上电（飞控未绑定过则随时可以）");
    }

    /* 上电默认上锁：开机后发出的都是上锁帧，飞控需要看到"上锁 -> 解锁"的跳变才会解锁 */
    s_armed = false;
    return xTaskCreate(rc_task, "rc", 4096, NULL, 10, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
