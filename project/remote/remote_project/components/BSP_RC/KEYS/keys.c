#include "keys.h"

#include "board_rc.h"
#include "driver/gpio.h"
#include "esp_check.h"

static const char *TAG = "keys";

#define POLL_MS        10
#define DEBOUNCE_TICKS 2    /* 20 ms */
#define DOUBLE_TICKS   40   /* 400 ms */
#define LONG_TICKS     80   /* 800 ms */

static const int s_pin[KEY_COUNT] = {
    [KEY_STOP] = RC_PIN_KEY_STOP,
    [KEY_UP] = RC_PIN_KEY_UP,
    [KEY_DOWN] = RC_PIN_KEY_DOWN,
    [KEY_LEFT] = RC_PIN_KEY_LEFT,
    [KEY_RIGHT] = RC_PIN_KEY_RIGHT,
    [KEY_UP_LEFT] = RC_PIN_KEY_UP_LEFT,
    [KEY_UP_RIGHT] = RC_PIN_KEY_UP_RIGHT,
};

typedef struct {
    bool stable;            /* 消抖后的状态：true = 按下 */
    uint8_t same_count;
    uint16_t held_ticks;
    bool long_sent;
    bool waiting_second;    /* 第一下已松开，等双击 */
    uint16_t wait_ticks;
    key_event_t event;
} key_state_t;

static key_state_t s_key[KEY_COUNT];

esp_err_t keys_init(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < KEY_COUNT; ++i) {
        mask |= 1ULL << s_pin[i];
    }
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio");
    for (int i = 0; i < KEY_COUNT; ++i) {
        /* 上电已按住的键：必须松开再按才算 */
        s_key[i].stable = keys_raw_down((key_id_t)i);
        s_key[i].long_sent = s_key[i].stable;
    }
    return ESP_OK;
}

bool keys_raw_down(key_id_t id)
{
    return gpio_get_level(s_pin[id]) == 0;
}

void keys_poll(void)
{
    for (int i = 0; i < KEY_COUNT; ++i) {
        key_state_t *k = &s_key[i];
        const bool raw = keys_raw_down((key_id_t)i);

        if (raw == k->stable) {
            k->same_count = 0;
        } else if (++k->same_count >= DEBOUNCE_TICKS) {
            k->same_count = 0;
            k->stable = raw;
            if (raw) {
                /* 按下 */
                k->held_ticks = 0;
                if (k->waiting_second) {
                    k->waiting_second = false;
                    k->event = KEY_EVT_DOUBLE;
                    k->long_sent = true; /* 这次按下不再产生 CLICK/LONG */
                } else {
                    k->long_sent = false;
                }
            } else if (!k->long_sent) {
                /* 短按松开：进入双击等待 */
                k->waiting_second = true;
                k->wait_ticks = 0;
            }
        }

        if (k->stable) {
            if (k->held_ticks < UINT16_MAX) {
                k->held_ticks++;
            }
            if (!k->long_sent && k->held_ticks >= LONG_TICKS) {
                k->long_sent = true;
                k->event = KEY_EVT_LONG;
            }
        } else if (k->waiting_second && ++k->wait_ticks >= DOUBLE_TICKS) {
            k->waiting_second = false;
            k->event = KEY_EVT_CLICK;
        }
    }
}

key_event_t keys_take_event(key_id_t id)
{
    const key_event_t e = s_key[id].event;
    s_key[id].event = KEY_EVT_NONE;
    return e;
}

bool keys_is_down(key_id_t id)
{
    return s_key[id].stable;
}
