#include "algo_filter.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define FILTER_TWO_PI_F                 6.28318530717958647692f
#define FILTER_STANDARD_GRAVITY_M_S2    9.80665f
#define FILTER_VIBRATION_TAU_S          0.20f
#define FILTER_DT_RELATIVE_TOLERANCE    0.001f

/* ============================================================
 * 通用输出处理
 * ============================================================ */

static void filter_clear_output(filter_out_t *output)
{
    if (output == NULL) {
        return;
    }

    memset(output, 0, sizeof(*output));
}

static float filter_vec3_get(
    const algo_vec3_t *vector,
    uint32_t axis
)
{
    switch (axis) {
        case 0u:
            return vector->x;

        case 1u:
            return vector->y;

        default:
            return vector->z;
    }
}

static void filter_vec3_set(
    algo_vec3_t *vector,
    uint32_t axis,
    float value
)
{
    switch (axis) {
        case 0u:
            vector->x = value;
            break;

        case 1u:
            vector->y = value;
            break;

        default:
            vector->z = value;
            break;
    }
}

static bool filter_vec3_is_finite(const algo_vec3_t *vector)
{
    return isfinite(vector->x) &&
           isfinite(vector->y) &&
           isfinite(vector->z);
}

static float filter_vec3_magnitude(const algo_vec3_t *vector)
{
    return sqrtf(
        vector->x * vector->x +
        vector->y * vector->y +
        vector->z * vector->z
    );
}

static float filter_clamp_float(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

/* ============================================================
 * 配置检查
 * ============================================================ */

static bool filter_config_is_valid(
    const algo_filter_config_t *config,
    float nominal_dt_s
)
{
    float nyquist_hz;

    if (config == NULL) {
        return false;
    }

    nyquist_hz = 0.5f / nominal_dt_s;

    if (!isfinite(config->gyro_lpf_cutoff_hz) ||
        !isfinite(config->accel_lpf_cutoff_hz)) {
        return false;
    }

    if (config->gyro_lpf_cutoff_hz <= 0.0f ||
        config->gyro_lpf_cutoff_hz >= nyquist_hz) {
        return false;
    }

    if (config->accel_lpf_cutoff_hz <= 0.0f ||
        config->accel_lpf_cutoff_hz >= nyquist_hz) {
        return false;
    }

    if (config->flap_notch_enable) {
        if (!isfinite(config->flap_notch_q) ||
            config->flap_notch_q <= 0.0f) {
            return false;
        }
    }

    if (config->second_harmonic_notch_enable) {
        if (!isfinite(config->second_harmonic_notch_q) ||
            config->second_harmonic_notch_q <= 0.0f) {
            return false;
        }
    }

    if (!isfinite(config->notch_retune_threshold_hz) ||
        config->notch_retune_threshold_hz < 0.0f) {
        return false;
    }

    if (!isfinite(config->gravity_tolerance_m_s2) ||
        config->gravity_tolerance_m_s2 <= 0.0f) {
        return false;
    }

    if (!isfinite(config->min_accel_trust) ||
        !isfinite(config->max_accel_trust)) {
        return false;
    }

    if (config->min_accel_trust < 0.0f ||
        config->max_accel_trust > 1.0f ||
        config->max_accel_trust < config->min_accel_trust) {
        return false;
    }

    return true;
}

/* ============================================================
 * 一阶低通
 * ============================================================ */

static void filter_lowpass_set_cutoff(
    filter_lowpass_t *filter,
    float cutoff_hz,
    float dt_s
)
{
    float frequency_times_dt;

    /*
     * 一阶 RC 低通的离散形式：
     *
     * RC    = 1 / (2πfc)
     * alpha = dt / (RC + dt)
     *       = 2πfc·dt / (1 + 2πfc·dt)
     */
    frequency_times_dt =
        FILTER_TWO_PI_F * cutoff_hz * dt_s;

    filter->alpha =
        frequency_times_dt / (1.0f + frequency_times_dt);
}

static void filter_lowpass_reset(filter_lowpass_t *filter)
{
    memset(&filter->value, 0, sizeof(filter->value));
    filter->primed = false;
}

static void filter_lowpass_update(
    filter_lowpass_t *filter,
    const algo_vec3_t *input,
    algo_vec3_t *output
)
{
    uint32_t axis;

    if (!filter->primed) {
        filter->value = *input;
        filter->primed = true;
        *output = *input;
        return;
    }

    for (axis = 0u; axis < ALGO_AXIS_COUNT; ++axis) {
        float input_value;
        float previous_value;
        float output_value;

        input_value = filter_vec3_get(input, axis);
        previous_value = filter_vec3_get(&filter->value, axis);

        output_value =
            previous_value +
            filter->alpha * (input_value - previous_value);

        filter_vec3_set(&filter->value, axis, output_value);
        filter_vec3_set(output, axis, output_value);
    }
}

/* ============================================================
 * 二阶陷波器
 * ============================================================ */

static void filter_biquad_set_passthrough(
    filter_biquad_coeff_t *coeff
)
{
    coeff->b0 = 1.0f;
    coeff->b1 = 0.0f;
    coeff->b2 = 0.0f;
    coeff->a1 = 0.0f;
    coeff->a2 = 0.0f;
}

static bool filter_biquad_set_notch(
    filter_biquad_coeff_t *coeff,
    float center_hz,
    float q,
    float dt_s
)
{
    float nyquist_hz;
    float omega;
    float sine_omega;
    float cosine_omega;
    float alpha;
    float a0;
    float a1;
    float a2;

    nyquist_hz = 0.5f / dt_s;

    if (!isfinite(center_hz) ||
        !isfinite(q) ||
        center_hz <= 0.0f ||
        center_hz >= nyquist_hz ||
        q <= 0.0f) {
        filter_biquad_set_passthrough(coeff);
        return false;
    }

    omega = FILTER_TWO_PI_F * center_hz * dt_s;

    sine_omega = sinf(omega);
    cosine_omega = cosf(omega);

    alpha = sine_omega / (2.0f * q);

    /*
     * RBJ 二阶陷波：
     *
     * b0 = 1
     * b1 = -2cos(ω)
     * b2 = 1
     *
     * a0 = 1 + alpha
     * a1 = -2cos(ω)
     * a2 = 1 - alpha
     */
    a0 = 1.0f + alpha;
    a1 = -2.0f * cosine_omega;
    a2 = 1.0f - alpha;

    coeff->b0 = 1.0f / a0;
    coeff->b1 = a1 / a0;
    coeff->b2 = 1.0f / a0;
    coeff->a1 = a1 / a0;
    coeff->a2 = a2 / a0;

    return true;
}

static void filter_biquad_reset_axis(
    filter_biquad_state_t *state
)
{
    state->x1 = 0.0f;
    state->x2 = 0.0f;
    state->y1 = 0.0f;
    state->y2 = 0.0f;
}

static void filter_biquad_prime_axis(
    filter_biquad_state_t *state,
    float input
)
{
    state->x1 = input;
    state->x2 = input;
    state->y1 = input;
    state->y2 = input;
}

static float filter_biquad_process_axis(
    const filter_biquad_coeff_t *coeff,
    filter_biquad_state_t *state,
    float input,
    bool prime
)
{
    float output;

    if (prime) {
        filter_biquad_prime_axis(state, input);
        return input;
    }

    output =
        coeff->b0 * input +
        coeff->b1 * state->x1 +
        coeff->b2 * state->x2 -
        coeff->a1 * state->y1 -
        coeff->a2 * state->y2;

    state->x2 = state->x1;
    state->x1 = input;

    state->y2 = state->y1;
    state->y1 = output;

    return output;
}

static void filter_biquad_process_vector(
    const filter_biquad_coeff_t *coeff,
    filter_biquad_state_t states[ALGO_AXIS_COUNT],
    const algo_vec3_t *input,
    algo_vec3_t *output,
    bool prime
)
{
    uint32_t axis;

    for (axis = 0u; axis < ALGO_AXIS_COUNT; ++axis) {
        float input_value;
        float output_value;

        input_value = filter_vec3_get(input, axis);

        output_value = filter_biquad_process_axis(
            coeff,
            &states[axis],
            input_value,
            prime
        );

        filter_vec3_set(output, axis, output_value);
    }
}

static void filter_biquad_prime_vector(
    filter_biquad_state_t states[ALGO_AXIS_COUNT],
    const algo_vec3_t *input
)
{
    uint32_t axis;

    for (axis = 0u; axis < ALGO_AXIS_COUNT; ++axis) {
        filter_biquad_prime_axis(
            &states[axis],
            filter_vec3_get(input, axis)
        );
    }
}

/* ============================================================
 * 系数更新
 * ============================================================ */

static void filter_update_notch_coefficients(
    filter_state_t *state,
    float flap_frequency_hz,
    float dt_s
)
{
    bool valid_center;

    if (state->config.flap_notch_enable) {
        valid_center = filter_biquad_set_notch(
            &state->flap_notch_coeff,
            flap_frequency_hz,
            state->config.flap_notch_q,
            dt_s
        );

        state->flap_notch_center_hz =
            valid_center ? flap_frequency_hz : 0.0f;
    } else {
        filter_biquad_set_passthrough(
            &state->flap_notch_coeff
        );

        state->flap_notch_center_hz = 0.0f;
    }

    if (state->config.second_harmonic_notch_enable) {
        float harmonic_frequency_hz =
            2.0f * flap_frequency_hz;

        valid_center = filter_biquad_set_notch(
            &state->harmonic_notch_coeff,
            harmonic_frequency_hz,
            state->config.second_harmonic_notch_q,
            dt_s
        );

        state->harmonic_notch_center_hz =
            valid_center ? harmonic_frequency_hz : 0.0f;
    } else {
        filter_biquad_set_passthrough(
            &state->harmonic_notch_coeff
        );

        state->harmonic_notch_center_hz = 0.0f;
    }

    state->last_notch_update_frequency_hz =
        flap_frequency_hz;
}

/* ============================================================
 * 加速度可信度
 * ============================================================ */

static float filter_calculate_accel_trust(
    const filter_state_t *state,
    const algo_vec3_t *accel
)
{
    float magnitude;
    float deviation;
    float excess;
    float trust;

    magnitude = filter_vec3_magnitude(accel);

    deviation = fabsf(
        magnitude - FILTER_STANDARD_GRAVITY_M_S2
    );

    if (deviation <= state->config.gravity_tolerance_m_s2) {
        trust = state->config.max_accel_trust;
    } else {
        excess =
            deviation - state->config.gravity_tolerance_m_s2;

        /*
         * 超过容忍范围后平滑降低可信度：
         *
         * trust = tolerance / (tolerance + excess)
         *
         * 不直接阶跃到零，避免姿态修正权重在边界反复跳变。
         */
        trust =
            state->config.max_accel_trust *
            state->config.gravity_tolerance_m_s2 /
            (state->config.gravity_tolerance_m_s2 + excess);
    }

    return filter_clamp_float(
        trust,
        state->config.min_accel_trust,
        state->config.max_accel_trust
    );
}

/* ============================================================
 * 初始化与复位
 * ============================================================ */

filter_status_t filter_init(
    filter_state_t *state,
    const algo_filter_config_t *config,
    float nominal_dt_s
)
{
    if (state == NULL || config == NULL) {
        return FILTER_STATUS_INVALID_ARG;
    }

    if (!isfinite(nominal_dt_s) || nominal_dt_s <= 0.0f) {
        state->initialized = false;
        return FILTER_STATUS_INVALID_DT;
    }

    if (!filter_config_is_valid(config, nominal_dt_s)) {
        state->initialized = false;
        return FILTER_STATUS_INVALID_CONFIG;
    }

    memset(state, 0, sizeof(*state));

    state->config = *config;
    state->nominal_dt_s = nominal_dt_s;
    state->coefficient_dt_s = nominal_dt_s;
    state->initialized = true;

    return filter_reset(state);
}

filter_status_t filter_reset(filter_state_t *state)
{
    uint32_t axis;

    if (state == NULL) {
        return FILTER_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return FILTER_STATUS_NOT_INITIALIZED;
    }

    filter_lowpass_reset(&state->gyro_lowpass);
    filter_lowpass_reset(&state->accel_lowpass);

    filter_lowpass_set_cutoff(
        &state->gyro_lowpass,
        state->config.gyro_lpf_cutoff_hz,
        state->nominal_dt_s
    );

    filter_lowpass_set_cutoff(
        &state->accel_lowpass,
        state->config.accel_lpf_cutoff_hz,
        state->nominal_dt_s
    );

    filter_biquad_set_passthrough(
        &state->flap_notch_coeff
    );

    filter_biquad_set_passthrough(
        &state->harmonic_notch_coeff
    );

    for (axis = 0u; axis < ALGO_AXIS_COUNT; ++axis) {
        filter_biquad_reset_axis(
            &state->flap_notch_state[axis]
        );

        filter_biquad_reset_axis(
            &state->harmonic_notch_state[axis]
        );
    }

    state->coefficient_dt_s = state->nominal_dt_s;

    state->flap_notch_center_hz = 0.0f;
    state->harmonic_notch_center_hz = 0.0f;
    state->last_notch_update_frequency_hz = 0.0f;

    state->vibration_rms_rad_s = 0.0f;
    state->accel_trust = 0.0f;

    return FILTER_STATUS_OK;
}

/* ============================================================
 * 样本更新
 * ============================================================ */

filter_status_t filter_update(
    filter_state_t *state,
    const algo_imu_sample_t *sample,
    const phase_out_t *phase,
    float dt_s,
    filter_out_t *output
)
{
    algo_vec3_t gyro_lowpass;
    algo_vec3_t gyro_after_flap_notch;
    algo_vec3_t gyro_filtered;
    algo_vec3_t vibration_difference;
    bool prime_gyro_filters;
    bool dt_changed;
    bool notch_retune_required;
    bool any_notch_enabled;
    float vibration_mean_square;
    float vibration_alpha;
    uint32_t axis;

    if (output != NULL) {
        filter_clear_output(output);
    }

    if (state == NULL || sample == NULL || output == NULL) {
        return FILTER_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return FILTER_STATUS_NOT_INITIALIZED;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f) {
        return FILTER_STATUS_INVALID_DT;
    }

    any_notch_enabled =
        state->config.flap_notch_enable ||
        state->config.second_harmonic_notch_enable;

    if (any_notch_enabled) {
        if (phase == NULL ||
            !phase->valid ||
            !isfinite(phase->f_hz) ||
            phase->f_hz <= 0.0f) {
            return FILTER_STATUS_INVALID_PHASE;
        }
    }

    /*
     * 无效样本不作为接口错误返回，但也不能更新滤波历史。
     */
    if (!sample->valid ||
        !filter_vec3_is_finite(&sample->gyro_rad_s) ||
        !filter_vec3_is_finite(&sample->accel_m_s2)) {
        return FILTER_STATUS_OK;
    }

    dt_changed =
        fabsf(dt_s - state->coefficient_dt_s) >
        state->coefficient_dt_s *
        FILTER_DT_RELATIVE_TOLERANCE;

    if (dt_changed) {
        filter_lowpass_set_cutoff(
            &state->gyro_lowpass,
            state->config.gyro_lpf_cutoff_hz,
            dt_s
        );

        filter_lowpass_set_cutoff(
            &state->accel_lowpass,
            state->config.accel_lpf_cutoff_hz,
            dt_s
        );

        state->coefficient_dt_s = dt_s;
    }

    if (any_notch_enabled) {
        notch_retune_required =
            dt_changed ||
            fabsf(
                phase->f_hz -
                state->last_notch_update_frequency_hz
            ) >= state->config.notch_retune_threshold_hz;

        if (notch_retune_required) {
            filter_update_notch_coefficients(
                state,
                phase->f_hz,
                dt_s
            );
        }
    }

    /*
     * 首个有效样本用于初始化所有滤波级，避免从零启动。
     */
    prime_gyro_filters =
        !state->gyro_lowpass.primed;

    filter_lowpass_update(
        &state->gyro_lowpass,
        &sample->gyro_rad_s,
        &gyro_lowpass
    );

    filter_lowpass_update(
        &state->accel_lowpass,
        &sample->accel_m_s2,
        &output->accel_m_s2
    );

    gyro_after_flap_notch = gyro_lowpass;

    /*
     * 基频陷波。
     *
     * 如果频率超过当前奈奎斯特范围，该级自动旁路，
     * 同时刷新历史，防止恢复有效频率时接入旧数据。
     */
    if (state->config.flap_notch_enable &&
        state->flap_notch_center_hz > 0.0f) {
        filter_biquad_process_vector(
            &state->flap_notch_coeff,
            state->flap_notch_state,
            &gyro_lowpass,
            &gyro_after_flap_notch,
            prime_gyro_filters
        );
    } else {
        filter_biquad_prime_vector(
            state->flap_notch_state,
            &gyro_lowpass
        );
    }

    gyro_filtered = gyro_after_flap_notch;

    /* 二次谐波陷波。 */
    if (state->config.second_harmonic_notch_enable &&
        state->harmonic_notch_center_hz > 0.0f) {
        filter_biquad_process_vector(
            &state->harmonic_notch_coeff,
            state->harmonic_notch_state,
            &gyro_after_flap_notch,
            &gyro_filtered,
            prime_gyro_filters
        );
    } else {
        filter_biquad_prime_vector(
            state->harmonic_notch_state,
            &gyro_after_flap_notch
        );
    }

    output->gyro_rad_s = gyro_filtered;

    /*
     * 振动强度估计：
     *
     * 使用原始陀螺与最终滤波结果之差，覆盖低通和陷波
     * 所移除的高频及扑翼相干成分。
     */
    vibration_difference.x =
        sample->gyro_rad_s.x - gyro_filtered.x;

    vibration_difference.y =
        sample->gyro_rad_s.y - gyro_filtered.y;

    vibration_difference.z =
        sample->gyro_rad_s.z - gyro_filtered.z;

    vibration_mean_square =
        state->vibration_rms_rad_s *
        state->vibration_rms_rad_s;

    vibration_mean_square +=
        vibration_difference.x * vibration_difference.x +
        vibration_difference.y * vibration_difference.y +
        vibration_difference.z * vibration_difference.z;

    vibration_mean_square *= 0.5f;

    vibration_alpha =
        dt_s / (FILTER_VIBRATION_TAU_S + dt_s);

    vibration_mean_square =
        state->vibration_rms_rad_s *
            state->vibration_rms_rad_s +
        vibration_alpha *
        (vibration_mean_square -
         state->vibration_rms_rad_s *
             state->vibration_rms_rad_s);

    if (vibration_mean_square < 0.0f) {
        vibration_mean_square = 0.0f;
    }

    state->vibration_rms_rad_s =
        sqrtf(vibration_mean_square);

    state->accel_trust =
        filter_calculate_accel_trust(
            state,
            &output->accel_m_s2
        );

    output->vibration_rms_rad_s =
        state->vibration_rms_rad_s;

    output->accel_trust =
        state->accel_trust;

    output->gyro_valid = true;
    output->accel_valid = true;
    output->valid = true;

    return FILTER_STATUS_OK;
}

/* ============================================================
 * 当前输出快照
 * ============================================================ */

filter_status_t filter_get(
    const filter_state_t *state,
    filter_out_t *output
)
{
    uint32_t axis;

    if (output != NULL) {
        filter_clear_output(output);
    }

    if (state == NULL || output == NULL) {
        return FILTER_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return FILTER_STATUS_NOT_INITIALIZED;
    }

    output->gyro_rad_s = state->gyro_lowpass.value;

    /*
     * 取最后一级启用陷波器的 y1 作为最终输出。
     * 首个样本初始化后，y1 同样保存了正确结果。
     */
    if (state->config.flap_notch_enable) {
        for (axis = 0u; axis < ALGO_AXIS_COUNT; ++axis) {
            filter_vec3_set(
                &output->gyro_rad_s,
                axis,
                state->flap_notch_state[axis].y1
            );
        }
    }

    if (state->config.second_harmonic_notch_enable) {
        for (axis = 0u; axis < ALGO_AXIS_COUNT; ++axis) {
            filter_vec3_set(
                &output->gyro_rad_s,
                axis,
                state->harmonic_notch_state[axis].y1
            );
        }
    }

    output->accel_m_s2 = state->accel_lowpass.value;

    output->vibration_rms_rad_s =
        state->vibration_rms_rad_s;

    output->accel_trust =
        state->accel_trust;

    output->gyro_valid =
        state->gyro_lowpass.primed;

    output->accel_valid =
        state->accel_lowpass.primed;

    output->valid =
        output->gyro_valid &&
        output->accel_valid;

    return FILTER_STATUS_OK;
}