#include "algo_linkage.h"

#include <math.h>
#include <string.h>

#define LINKAGE_RAD_TO_DEG 57.29577951308232f
#define LINKAGE_PI 3.14159265358979323846f

static bool side_limits_valid(const algo_linkage_side_config_t *s)
{
    return isfinite(s->servo_neutral_deg) &&
           isfinite(s->servo_min_deg) &&
           isfinite(s->servo_max_deg) &&
           isfinite(s->servo_direction) &&
           (s->servo_direction == 1.0f || s->servo_direction == -1.0f) &&
           s->servo_min_deg <= s->servo_neutral_deg &&
           s->servo_neutral_deg <= s->servo_max_deg;
}

static bool side_geometry_valid(const algo_linkage_side_config_t *s)
{
    return isfinite(s->ground_length_mm) && s->ground_length_mm > 0.0f &&
           isfinite(s->input_length_mm) && s->input_length_mm > 0.0f &&
           isfinite(s->coupler_length_mm) && s->coupler_length_mm > 0.0f &&
           isfinite(s->output_length_mm) && s->output_length_mm > 0.0f &&
           isfinite(s->ground_angle_rad) &&
           isfinite(s->wing_zero_rad) &&
           (s->assembly_sign == 1.0f || s->assembly_sign == -1.0f);
}

bool linkage_config_is_valid(const algo_linkage_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    for (int w = 0; w < ALGO_WING_COUNT; ++w) {
        if (!side_limits_valid(&config->side[w])) {
            return false;
        }
        if (config->enabled && !side_geometry_valid(&config->side[w])) {
            return false;
        }
    }
    return true;
}

/*
 * 四连杆：给定摇杆绝对角 theta4，求曲柄绝对角 theta2。
 *
 *   O2 = (0, 0)，O4 = d (cos g, sin g)
 *   B  = O4 + c (cos theta4, sin theta4)
 *   A 在以 O2 为圆心、半径 a 的圆上，且 |A - B| = b
 *
 * 两圆求交，sign 选分支。无交点返回 false。
 */
static bool four_bar_crank(const algo_linkage_side_config_t *s, float theta4, float *theta2)
{
    const float bx = s->ground_length_mm * cosf(s->ground_angle_rad) + s->output_length_mm * cosf(theta4);
    const float by = s->ground_length_mm * sinf(s->ground_angle_rad) + s->output_length_mm * sinf(theta4);
    const float dist = sqrtf(bx * bx + by * by);
    const float a = s->input_length_mm;
    const float b = s->coupler_length_mm;

    if (!(dist > 1e-6f) || dist > a + b || dist < fabsf(a - b)) {
        return false;
    }

    /* 在 O2->B 方向上的投影长度与垂直距离 */
    const float along = (a * a - b * b + dist * dist) / (2.0f * dist);
    float h2 = a * a - along * along;
    if (h2 < 0.0f) {
        h2 = 0.0f;
    }
    const float h = sqrtf(h2);
    const float ux = bx / dist;
    const float uy = by / dist;
    const float ax = along * ux - s->assembly_sign * h * uy;
    const float ay = along * uy + s->assembly_sign * h * ux;

    *theta2 = atan2f(ay, ax);
    return isfinite(*theta2);
}

static float wrap_pi(float x)
{
    while (x > LINKAGE_PI) {
        x -= 2.0f * LINKAGE_PI;
    }
    while (x < -LINKAGE_PI) {
        x += 2.0f * LINKAGE_PI;
    }
    return x;
}

linkage_status_t linkage_init(linkage_state_t *state, const algo_linkage_config_t *config)
{
    if (state == NULL || config == NULL) {
        return LINKAGE_STATUS_INVALID_ARG;
    }
    memset(state, 0, sizeof(*state));
    if (!linkage_config_is_valid(config)) {
        return LINKAGE_STATUS_INVALID_CONFIG;
    }
    state->config = *config;

    if (config->enabled) {
        for (int w = 0; w < ALGO_WING_COUNT; ++w) {
            const algo_linkage_side_config_t *s = &config->side[w];
            if (!four_bar_crank(s, s->wing_zero_rad, &state->crank_zero_rad[w])) {
                /* 机翼零位本身不可装配：几何参数填错 */
                return LINKAGE_STATUS_INVALID_CONFIG;
            }
        }
    }

    state->initialized = true;
    linkage_neutral(state, &state->current);
    return LINKAGE_STATUS_OK;
}

void linkage_neutral(const linkage_state_t *state, servo_target_t *output)
{
    if (output == NULL) {
        return;
    }
    memset(output, 0, sizeof(*output));
    if (state == NULL) {
        return;
    }
    for (int w = 0; w < ALGO_WING_COUNT; ++w) {
        output->angle_deg[w] = state->config.side[w].servo_neutral_deg;
        output->reachable[w] = true;
    }
    output->valid = true;
}

linkage_status_t linkage_update(linkage_state_t *state, const wing_target_t *wing, servo_target_t *output)
{
    if (output == NULL) {
        return LINKAGE_STATUS_INVALID_ARG;
    }
    memset(output, 0, sizeof(*output));
    if (state == NULL || wing == NULL) {
        return LINKAGE_STATUS_INVALID_ARG;
    }
    if (!state->initialized) {
        return LINKAGE_STATUS_NOT_INITIALIZED;
    }
    if (!wing->valid) {
        state->current = *output;
        return LINKAGE_STATUS_OK;
    }

    bool all_ok = true;
    for (int w = 0; w < ALGO_WING_COUNT; ++w) {
        const algo_linkage_side_config_t *s = &state->config.side[w];
        float servo_deg = s->servo_neutral_deg;
        bool reachable = false;

        if (isfinite(wing->angle_rad[w])) {
            if (state->config.enabled) {
                float theta2;
                if (four_bar_crank(s, s->wing_zero_rad + wing->angle_rad[w], &theta2)) {
                    servo_deg = s->servo_neutral_deg +
                                s->servo_direction * wrap_pi(theta2 - state->crank_zero_rad[w]) * LINKAGE_RAD_TO_DEG;
                    reachable = true;
                }
            } else if (state->config.direct_drive) {
                servo_deg = s->servo_neutral_deg + s->servo_direction * wing->angle_rad[w] * LINKAGE_RAD_TO_DEG;
                reachable = true;
            }
        }

        output->saturated[w] = wing->saturated[w];
        if (reachable) {
            if (servo_deg < s->servo_min_deg) {
                servo_deg = s->servo_min_deg;
                output->saturated[w] = true;
            } else if (servo_deg > s->servo_max_deg) {
                servo_deg = s->servo_max_deg;
                output->saturated[w] = true;
            }
        } else {
            servo_deg = s->servo_neutral_deg;
            all_ok = false;
        }

        output->angle_deg[w] = servo_deg;
        output->reachable[w] = reachable;
    }

    output->valid = all_ok;
    state->current = *output;
    return LINKAGE_STATUS_OK;
}
