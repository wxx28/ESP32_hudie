#include "fc_output.h"

#include <math.h>
#include <string.h>

#include "servo.h"

/* 逻辑舵机目标 -> BSP。电气反相在 BSP 里做，这里不处理。 */

typedef struct {
    float neutral;
    float min;
    float max;
} side_limit_t;

static side_limit_t s_side[ALGO_WING_COUNT];
static servo_target_t s_last;
static bool s_inited;

static fc_output_status_t write_two(float left_deg, float right_deg)
{
    const esp_err_t e1 = servo_set_angle(SERVO_LEFT, left_deg);
    const esp_err_t e2 = servo_set_angle(SERVO_RIGHT, right_deg);
    memset(&s_last, 0, sizeof(s_last));
    s_last.angle_deg[ALGO_WING_LEFT] = left_deg;
    s_last.angle_deg[ALGO_WING_RIGHT] = right_deg;
    s_last.reachable[0] = s_last.reachable[1] = true;
    s_last.valid = (e1 == ESP_OK && e2 == ESP_OK);
    return s_last.valid ? FC_OUTPUT_STATUS_OK : FC_OUTPUT_STATUS_HARDWARE_ERROR;
}

fc_output_status_t fc_output_init(const algo_linkage_config_t *linkage_config)
{
    if (linkage_config == NULL) {
        return FC_OUTPUT_STATUS_INVALID_ARG;
    }
    for (int w = 0; w < ALGO_WING_COUNT; ++w) {
        const algo_linkage_side_config_t *s = &linkage_config->side[w];
        if (!isfinite(s->servo_neutral_deg) || !isfinite(s->servo_min_deg) || !isfinite(s->servo_max_deg) ||
            s->servo_min_deg > s->servo_neutral_deg || s->servo_neutral_deg > s->servo_max_deg) {
            return FC_OUTPUT_STATUS_INVALID_CONFIG;
        }
        s_side[w].neutral = s->servo_neutral_deg;
        s_side[w].min = s->servo_min_deg;
        s_side[w].max = s->servo_max_deg;
    }
    if (servo_init() != ESP_OK) {
        return FC_OUTPUT_STATUS_HARDWARE_ERROR;
    }
    s_inited = true;
    return fc_output_write_safe();
}

fc_output_status_t fc_output_write_safe(void)
{
    if (!s_inited) {
        return FC_OUTPUT_STATUS_NOT_INITIALIZED;
    }
    return write_two(s_side[ALGO_WING_LEFT].neutral, s_side[ALGO_WING_RIGHT].neutral);
}

static bool angle_ok(int w, float a)
{
    return isfinite(a) && a >= s_side[w].min && a <= s_side[w].max;
}

fc_output_status_t fc_output_write(const algo_output_t *output)
{
    if (!s_inited) {
        return FC_OUTPUT_STATUS_NOT_INITIALIZED;
    }
    if (output == NULL) {
        fc_output_write_safe();
        return FC_OUTPUT_STATUS_INVALID_ARG;
    }

    /* 禁止扑翼：两侧中位 */
    if (!output->safety.allow_flap || output->safety.immediate_stop) {
        return fc_output_write_safe();
    }

    const servo_target_t *t = &output->servo;
    const bool ok = output->valid && t->valid && t->reachable[ALGO_WING_LEFT] && t->reachable[ALGO_WING_RIGHT] &&
                    angle_ok(ALGO_WING_LEFT, t->angle_deg[ALGO_WING_LEFT]) &&
                    angle_ok(ALGO_WING_RIGHT, t->angle_deg[ALGO_WING_RIGHT]);
    if (!ok) {
        /* 不允许一侧继续扑、另一侧停 */
        fc_output_write_safe();
        return FC_OUTPUT_STATUS_INVALID_OUTPUT;
    }

    const fc_output_status_t st = write_two(t->angle_deg[ALGO_WING_LEFT], t->angle_deg[ALGO_WING_RIGHT]);
    if (st == FC_OUTPUT_STATUS_OK) {
        s_last.saturated[0] = t->saturated[0];
        s_last.saturated[1] = t->saturated[1];
    }
    return st;
}

fc_output_status_t fc_output_get_last(servo_target_t *target)
{
    if (target == NULL) {
        return FC_OUTPUT_STATUS_INVALID_ARG;
    }
    if (!s_inited) {
        return FC_OUTPUT_STATUS_NOT_INITIALIZED;
    }
    *target = s_last;
    return FC_OUTPUT_STATUS_OK;
}
