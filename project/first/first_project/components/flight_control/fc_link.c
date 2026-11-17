#include "fc_link.h"

#include <math.h>
#include <string.h>

#include "espnow_link.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "hudie_bind.h"

static const char *TAG = "fc_link";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static fc_link_rc_t s_rc;
static uint8_t s_peer[6];
static volatile bool s_bound;
static volatile bool s_allow_bind = true;
static volatile bool s_bind_pending;
static uint8_t s_bind_mac[6];
static uint8_t s_tele_seq;

/* 丢包统计：每 50 帧（约 1 s）结算一次 */
static uint8_t s_last_seq;
static bool s_have_seq;
static uint32_t s_win_expected, s_win_got;
static uint8_t s_loss_pct;

static void on_recv(const uint8_t *src, const uint8_t *data, size_t len, int8_t rssi)
{
    if (len == sizeof(bind_frame_t) && hudie_proto_check(data, len, sizeof(bind_frame_t), TYPE_BIND)) {
        const bind_frame_t *b = (const bind_frame_t *)data;
        if (b->cmd == BIND_CMD_REQUEST && s_allow_bind &&
            (!s_bound || esp_timer_get_time() < (int64_t)FC_LINK_BIND_WINDOW_MS * 1000)) {
            memcpy(s_bind_mac, src, 6);
            s_bind_pending = true;   /* NVS 写入放到任务里做 */
        }
        return;
    }

    if (len != sizeof(rc_frame_t) || !hudie_proto_check(data, len, sizeof(rc_frame_t), TYPE_RC)) {
        return;
    }
    if (!s_bound || memcmp(src, s_peer, 6) != 0) {
        return; /* 只听绑定的遥控器 */
    }

    const rc_frame_t *f = (const rc_frame_t *)data;
    if (s_have_seq) {
        const uint8_t gap = (uint8_t)(f->seq - s_last_seq);
        if (gap == 0u || gap > 100u) {
            return; /* 重复帧或乱序 */
        }
        s_win_expected += gap;
    } else {
        s_win_expected += 1u;
        s_have_seq = true;
    }
    s_last_seq = f->seq;
    s_win_got++;
    if (s_win_expected >= 50u) {
        s_loss_pct = (uint8_t)(100u - (s_win_got * 100u) / s_win_expected);
        s_win_expected = 0u;
        s_win_got = 0u;
    }

    portENTER_CRITICAL(&s_lock);
    s_rc.frame = *f;
    s_rc.rx_time_us = (uint64_t)esp_timer_get_time();
    s_rc.rssi = rssi;
    s_rc.loss_pct = s_loss_pct;
    portEXIT_CRITICAL(&s_lock);
}

bool fc_link_init(void)
{
    espnow_link_config_t cfg = {
        .channel = FC_LINK_CHANNEL,
        .recv_cb = on_recv,
        .send_cb = NULL,
    };
    if (espnow_link_init(&cfg) != ESP_OK) {
        return false;
    }
    uint8_t mac[6];
    espnow_link_get_mac(mac);
    ESP_LOGI(TAG, "飞控 MAC " MACSTR "，信道 %u", MAC2STR(mac), (unsigned)FC_LINK_CHANNEL);

    if (hudie_bind_load(s_peer)) {
        s_bound = true;
        espnow_link_set_peer(s_peer);
        ESP_LOGI(TAG, "已绑定遥控器 " MACSTR "（上电 %u s 内可重新绑定）", MAC2STR(s_peer),
                 (unsigned)(FC_LINK_BIND_WINDOW_MS / 1000u));
    } else {
        ESP_LOGW(TAG, "尚未绑定遥控器：遥控器按住 down 键开机即可绑定");
    }
    return true;
}

/* 在飞控任务的低频分支里处理绑定（写 NVS、回确认） */
static void process_bind(void)
{
    if (!s_bind_pending) {
        return;
    }
    s_bind_pending = false;
    uint8_t mac[6];
    memcpy(mac, s_bind_mac, 6);

    if (!s_bound || memcmp(mac, s_peer, 6) != 0) {
        if (hudie_bind_save(mac) != ESP_OK) {
            ESP_LOGE(TAG, "绑定信息写入 NVS 失败");
            return;
        }
        portENTER_CRITICAL(&s_lock);
        memcpy(s_peer, mac, 6);
        memset(&s_rc, 0, sizeof(s_rc));
        s_have_seq = false;
        portEXIT_CRITICAL(&s_lock);
        espnow_link_set_peer(mac);
        s_bound = true;
        ESP_LOGI(TAG, "绑定遥控器 " MACSTR, MAC2STR(mac));
    }

    bind_frame_t ack = {.cmd = BIND_CMD_ACCEPT};
    espnow_link_get_mac(ack.mac);
    hudie_proto_finalize(&ack, sizeof(ack), TYPE_BIND);
    espnow_link_send(&ack, sizeof(ack));
}

void fc_link_get_rc(fc_link_rc_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *out = s_rc;
    portEXIT_CRITICAL(&s_lock);
}

bool fc_link_is_bound(void)
{
    return s_bound;
}

void fc_link_allow_bind(bool allow)
{
    s_allow_bind = allow;
}

static int16_t to_decideg(float rad)
{
    float d = rad * 572.957795f;
    if (!isfinite(d)) {
        return 0;
    }
    if (d > 32767.0f) {
        d = 32767.0f;
    }
    if (d < -32768.0f) {
        d = -32768.0f;
    }
    return (int16_t)d;
}

static int16_t to_dps(float rad_s)
{
    return to_decideg(rad_s) / 10;
}

static uint8_t tele_state(safety_state_t s)
{
    switch (s) {
    case ALGO_SAFETY_STANDBY:
        return TELE_STATE_STANDBY;
    case ALGO_SAFETY_ARMED:
    case ALGO_SAFETY_DEGRADED:
        return TELE_STATE_FLYING;
    case ALGO_SAFETY_FAILSAFE:
        return TELE_STATE_FAILSAFE;
    case ALGO_SAFETY_ESTOP:
        return TELE_STATE_FAULT;
    default:
        return TELE_STATE_SELFTEST;
    }
}

void fc_link_send_telemetry(const algo_output_t *out, uint32_t vbat_mv)
{
    process_bind();
    if (!s_bound || out == NULL) {
        return;
    }
    fc_link_rc_t rc;
    fc_link_get_rc(&rc);

    tele_frame_t t = {0};
    t.seq = s_tele_seq++;
    t.state = tele_state(out->safety.state);
    t.roll = to_decideg(out->attitude.roll_rad);
    t.pitch = to_decideg(out->attitude.pitch_rad);
    t.yaw = to_decideg(out->attitude.yaw_rad);
    t.gyro[0] = to_dps(out->filter.gyro_rad_s.x);
    t.gyro[1] = to_dps(out->filter.gyro_rad_s.y);
    t.gyro[2] = to_dps(out->filter.gyro_rad_s.z);
    t.vbat_mv = (uint16_t)(vbat_mv > 65535u ? 65535u : vbat_mv);
    t.alt_cm = 0;
    t.rssi_dbm = (uint8_t)(rc.rssi < 0 ? -rc.rssi : rc.rssi);
    t.loss_pct = rc.loss_pct;
    hudie_proto_finalize(&t, sizeof(t), TYPE_TELEM);
    espnow_link_send(&t, sizeof(t));
}
