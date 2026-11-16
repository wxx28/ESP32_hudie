#include "algo_waveform.h"

#include <math.h>
#include <string.h>

#define WAVEFORM_TWO_PI_F  6.28318530717958647692f

/* ============================================================
 * 通用辅助函数
 * ============================================================ */

static float waveform_clampf(
    float value,
    float min_value,
    float max_value
)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static void waveform_clear_output(
    wing_target_t *output
)
{
    memset(output, 0, sizeof(*output));
}

static void waveform_clear_command(
    flap_wave_cmd_t *command
)
{
    memset(command, 0, sizeof(*command));
}

/* ============================================================
 * 配置检查
 * ============================================================ */

static bool waveform_config_is_valid(
    const algo_waveform_config_t *config
)
{
    uint32_t wing;

    if (config == NULL) {
        return false;
    }

    for (wing = 0u; wing < (uint32_t)ALGO_WING_COUNT; ++wing) {
        if (!isfinite(config->min_wing_angle_rad[wing]) ||
            !isfinite(config->max_wing_angle_rad[wing]) ||
            !isfinite(config->max_wing_rate_rad_s[wing])) {
            return false;
        }

        if (config->min_wing_angle_rad[wing] >=
            config->max_wing_angle_rad[wing]) {
            return false;
        }

        if (config->max_wing_rate_rad_s[wing] <= 0.0f) {
            return false;
        }
    }

    return true;
}

/* ============================================================
 * 波形参数检查
 *
 * command_seq 不参与检查，因为它由 waveform 在提交时生成。
 * ============================================================ */

static bool waveform_command_values_are_valid(
    const flap_wave_cmd_t *command
)
{
    uint32_t wing;

    if (!isfinite(command->frequency_hz) ||
        (command->frequency_hz <= 0.0f)) {
        return false;
    }

    if (!isfinite(command->shape_k) ||
        (command->shape_k < 0.0f) ||
        (command->shape_k > 1.0f)) {
        return false;
    }

    for (wing = 0u; wing < (uint32_t)ALGO_WING_COUNT; ++wing) {
        if (!isfinite(command->amplitude_rad[wing]) ||
            (command->amplitude_rad[wing] < 0.0f)) {
            return false;
        }

        if (!isfinite(command->center_rad[wing])) {
            return false;
        }

        if (!isfinite(command->up_down_bias[wing]) ||
            (command->up_down_bias[wing] < -1.0f) ||
            (command->up_down_bias[wing] > 1.0f)) {
            return false;
        }
    }

    return true;
}

static bool waveform_phase_is_valid(
    const phase_out_t *phase
)
{
    if (!phase->valid) {
        return false;
    }

    if (!isfinite(phase->psi_rad) ||
        (phase->psi_rad < 0.0f) ||
        (phase->psi_rad >= WAVEFORM_TWO_PI_F)) {
        return false;
    }

    if (!isfinite(phase->f_hz) ||
        (phase->f_hz <= 0.0f)) {
        return false;
    }

    return true;
}

/* ============================================================
 * 整拍提交
 * ============================================================ */

static void waveform_commit_command(
    waveform_state_t *state,
    const flap_wave_cmd_t *pending_command
)
{
    uint32_t next_sequence =
        state->active_command.command_seq + 1u;

    /* 0 保留给初始化和无有效命令状态。 */
    if (next_sequence == 0u) {
        next_sequence = 1u;
    }

    state->active_command = *pending_command;
    state->active_command.command_seq = next_sequence;
    state->active_command.valid = true;
}

/* ============================================================
 * 初始化
 * ============================================================ */

waveform_status_t waveform_init(
    waveform_state_t *state,
    const algo_waveform_config_t *config
)
{
    algo_waveform_config_t config_copy;

    if (state == NULL) {
        return WAVEFORM_STATUS_INVALID_ARG;
    }

    state->initialized = false;

    if (config == NULL) {
        return WAVEFORM_STATUS_INVALID_ARG;
    }

    if (!waveform_config_is_valid(config)) {
        return WAVEFORM_STATUS_INVALID_CONFIG;
    }

    config_copy = *config;

    memset(state, 0, sizeof(*state));

    state->config = config_copy;
    state->initialized = true;

    waveform_clear_command(&state->active_command);

    return WAVEFORM_STATUS_OK;
}

/* ============================================================
 * 复位
 * ============================================================ */

waveform_status_t waveform_reset(
    waveform_state_t *state
)
{
    if (state == NULL) {
        return WAVEFORM_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return WAVEFORM_STATUS_NOT_INITIALIZED;
    }

    waveform_clear_command(&state->active_command);

    return WAVEFORM_STATUS_OK;
}

/* ============================================================
 * 更新波形输出
 * ============================================================ */

waveform_status_t waveform_update(
    waveform_state_t *state,
    const phase_out_t *phase,
    const flap_wave_cmd_t *pending_command,
    wing_target_t *output
)
{
    float sine_value;
    float cosine_value;
    float sine_squared;
    float shape_k;
    float wave;
    float wave_derivative;
    float phase_rate_rad_s;
    uint32_t wing;

    if (output == NULL) {
        return WAVEFORM_STATUS_INVALID_ARG;
    }

    waveform_clear_output(output);

    if (state == NULL) {
        return WAVEFORM_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return WAVEFORM_STATUS_NOT_INITIALIZED;
    }

    if ((phase == NULL) || (pending_command == NULL)) {
        return WAVEFORM_STATUS_INVALID_ARG;
    }

    /* 相位无效时不能生成目标，也不能依据它提交新参数。 */
    if (!waveform_phase_is_valid(phase)) {
        return WAVEFORM_STATUS_INVALID_PHASE;
    }

    /* ========================================================
     * 只在整拍边界处理待提交参数
     * ======================================================== */

    if (phase->beat_start) {
        if (!pending_command->valid) {
            /*
             * 控制参数已经失效，不能无限沿用上一次有效幅度。
             * valid=false 属于运行状态，不作为接口错误。
             */
            waveform_clear_command(&state->active_command);
            return WAVEFORM_STATUS_OK;
        }

        if (!waveform_command_values_are_valid(pending_command)) {
            waveform_clear_command(&state->active_command);
            return WAVEFORM_STATUS_INVALID_COMMAND;
        }

        waveform_commit_command(state, pending_command);
    }

    /* 尚未获得有效整拍参数。 */
    if (!state->active_command.valid) {
        return WAVEFORM_STATUS_OK;
    }

    /*
     * 每次使用前重新检查当前生效参数，
     * 防止状态损坏后继续生成输出。
     */
    if (!waveform_command_values_are_valid(
            &state->active_command)) {
        waveform_clear_command(&state->active_command);
        return WAVEFORM_STATUS_INVALID_COMMAND;
    }

    /* ========================================================
     * 计算公共波形及其相位导数
     *
     * wave = (1+k)*s - k*s^3
     *
     * d_wave_d_psi = c*(1+k-3*k*s^2)
     * ======================================================== */

    sine_value = sinf(phase->psi_rad);
    cosine_value = cosf(phase->psi_rad);
    sine_squared = sine_value * sine_value;
    shape_k = state->active_command.shape_k;

    wave =
        (1.0f + shape_k) * sine_value -
        shape_k * sine_value * sine_squared;

    wave_derivative =
        cosine_value *
        (1.0f + shape_k - 3.0f * shape_k * sine_squared);

    /* 实际相位角速度 = 2π * 实际扑翼频率。 */
    phase_rate_rad_s =
        WAVEFORM_TWO_PI_F * phase->f_hz;

    if (!isfinite(wave) ||
        !isfinite(wave_derivative) ||
        !isfinite(phase_rate_rad_s)) {
        waveform_clear_output(output);
        return WAVEFORM_STATUS_NUMERIC_ERROR;
    }

    /* ========================================================
     * 分别计算左右翼目标
     * ======================================================== */

    for (wing = 0u; wing < (uint32_t)ALGO_WING_COUNT; ++wing) {
        float amplitude =
            state->active_command.amplitude_rad[wing];

        float center =
            state->active_command.center_rad[wing];

        float bias =
            state->active_command.up_down_bias[wing];

        float lobe_scale;
        float lobe_amplitude;
        float angle;
        float rate;
        float limited_value;

        /*
         * s>=0：上拍；
         * s<0 ：下拍。
         * 正偏置增强下拍、减弱上拍。
         */
        if (sine_value >= 0.0f) {
            lobe_scale = 1.0f - bias;
        } else {
            lobe_scale = 1.0f + bias;
        }

        lobe_amplitude = amplitude * lobe_scale;

        angle = center + lobe_amplitude * wave;

        rate =
            lobe_amplitude *
            wave_derivative *
            phase_rate_rad_s;

        if (!isfinite(angle) || !isfinite(rate)) {
            waveform_clear_output(output);
            return WAVEFORM_STATUS_NUMERIC_ERROR;
        }

        output->saturated[wing] = false;

        /* 翼角限幅。 */
        if (state->config.angle_limit_enable) {
            limited_value = waveform_clampf(
                angle,
                state->config.min_wing_angle_rad[wing],
                state->config.max_wing_angle_rad[wing]
            );

            if (limited_value != angle) {
                output->saturated[wing] = true;
            }

            angle = limited_value;
        }

        /* 翼角速度限幅。 */
        if (state->config.rate_limit_enable) {
            limited_value = waveform_clampf(
                rate,
                -state->config.max_wing_rate_rad_s[wing],
                state->config.max_wing_rate_rad_s[wing]
            );

            if (limited_value != rate) {
                output->saturated[wing] = true;
            }

            rate = limited_value;
        }

        output->angle_rad[wing] = angle;
        output->rate_rad_s[wing] = rate;
    }

    output->valid = true;

    return WAVEFORM_STATUS_OK;
}

/* ============================================================
 * 获取当前生效参数
 * ============================================================ */

waveform_status_t waveform_get_active(
    const waveform_state_t *state,
    flap_wave_cmd_t *output
)
{
    if (output == NULL) {
        return WAVEFORM_STATUS_INVALID_ARG;
    }

    waveform_clear_command(output);

    if (state == NULL) {
        return WAVEFORM_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return WAVEFORM_STATUS_NOT_INITIALIZED;
    }

    *output = state->active_command;

    return WAVEFORM_STATUS_OK;
}