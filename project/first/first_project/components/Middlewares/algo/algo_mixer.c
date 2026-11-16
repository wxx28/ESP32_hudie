/*
 * 控制分配（混控）模块实现
 *
 * 将控制器输出的归一化需求（升力/滚转/俯仰）映射为
 * 扑翼波形参数（频率/左右翼幅度/上下拍偏置/中位/形状），
 * 并根据安全指令对输出进行限幅或直接禁止扑翼。
 *
 * 模块级职责、映射约定和接口契约见 algo_mixer.h。
 */

#include "algo_mixer.h"

#include <math.h>
#include <string.h>

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

/*
 * 浮点限幅：将 value 钳制到 [min_value, max_value]。
 *
 * 调用者负责保证传入的都是有限值（本模块在使用前
 * 均已通过 isfinite 检查），因此这里不再做 NaN 处理。
 */
static float mixer_clampf(
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

/*
 * 校验混控配置和频率范围是否合法。
 *
 * 检查规则：
 *   - 指针不能为空；
 *   - 第一版未实现偏航分配，yaw_mixer_enable 必须为 false；
 *   - 所有参与计算的增益、限幅参数必须为有限值且非负；
 *   - max_up_down_bias 和 default_shape_k 限定在 [0, 1]；
 *   - 左右翼最大幅度必须为正，中位必须为有限值；
 *   - 最小频率必须为正，最大频率不得小于最小频率。
 *
 * 返回 true 表示配置可用。
 */
static bool mixer_config_is_valid(
    const algo_mixer_config_t *config,
    const algo_phase_config_t *phase_config
)
{
    uint32_t wing;

    if ((config == NULL) || (phase_config == NULL)) {
        return false;
    }

    /* 第一版暂不实现偏航控制分配。 */
    if (config->yaw_mixer_enable) {
        return false;
    }

    if (!isfinite(config->lift_amplitude_gain_rad) ||
        (config->lift_amplitude_gain_rad < 0.0f)) {
        return false;
    }

    if (!isfinite(config->roll_differential_gain_rad) ||
        (config->roll_differential_gain_rad < 0.0f)) {
        return false;
    }

    if (!isfinite(config->max_differential_rad) ||
        (config->max_differential_rad < 0.0f)) {
        return false;
    }

    if (!isfinite(config->pitch_up_down_gain) ||
        (config->pitch_up_down_gain < 0.0f)) {
        return false;
    }

    if (!isfinite(config->max_up_down_bias) ||
        (config->max_up_down_bias < 0.0f) ||
        (config->max_up_down_bias > 1.0f)) {
        return false;
    }

    if (!isfinite(config->max_center_offset_rad) ||
        (config->max_center_offset_rad < 0.0f)) {
        return false;
    }

    if (!isfinite(config->default_shape_k) ||
        (config->default_shape_k < 0.0f) ||
        (config->default_shape_k > 1.0f)) {
        return false;
    }

    for (wing = 0u; wing < (uint32_t)ALGO_WING_COUNT; ++wing) {
        if (!isfinite(config->max_amplitude_rad[wing]) ||
            (config->max_amplitude_rad[wing] <= 0.0f)) {
            return false;
        }

        if (!isfinite(config->center_rad[wing])) {
            return false;
        }
    }

    /* 混控只检查自己使用的频率范围。 */
    if (!isfinite(phase_config->min_frequency_hz) ||
        !isfinite(phase_config->max_frequency_hz)) {
        return false;
    }

    if (phase_config->min_frequency_hz <= 0.0f) {
        return false;
    }

    if (phase_config->max_frequency_hz <
        phase_config->min_frequency_hz) {
        return false;
    }

    return true;
}

/*
 * 生成零幅度波形参数。
 *
 * 中位和形状仍使用配置值，避免安全输出包含未初始化的数据。
 */
static void mixer_set_safe_command(
    const mixer_state_t *state,
    bool valid,
    flap_wave_cmd_t *output
)
{
    uint32_t wing;

    memset(output, 0, sizeof(*output));

    output->frequency_hz = state->phase_config.min_frequency_hz;
    output->shape_k = state->config.default_shape_k;
    output->command_seq = 0u;

    for (wing = 0u; wing < (uint32_t)ALGO_WING_COUNT; ++wing) {
        output->amplitude_rad[wing] = 0.0f;
        output->center_rad[wing] = state->config.center_rad[wing];
        output->up_down_bias[wing] = 0.0f;
    }

    output->valid = valid;
}

/*
 * 检查控制器输出的四个通道是否都是有限值。
 *
 * 任一通道为 NaN 或无穷大时返回 false，
 * 调用方应拒绝本次输入并输出安全参数。
 */
static bool mixer_control_is_finite(
    const control_out_t *control
)
{
    return isfinite(control->lift) &&
           isfinite(control->roll) &&
           isfinite(control->pitch) &&
           isfinite(control->yaw);
}

/*
 * 检查已生成的波形参数是否全部为有限值。
 *
 * 作为输出前的最后一道防线：即使极端配置导致
 * 乘法溢出，也不会把非有限值传给下游 waveform。
 */
static bool mixer_output_is_finite(
    const flap_wave_cmd_t *output
)
{
    uint32_t wing;

    if (!isfinite(output->frequency_hz) ||
        !isfinite(output->shape_k)) {
        return false;
    }

    for (wing = 0u; wing < (uint32_t)ALGO_WING_COUNT; ++wing) {
        if (!isfinite(output->amplitude_rad[wing]) ||
            !isfinite(output->center_rad[wing]) ||
            !isfinite(output->up_down_bias[wing])) {
            return false;
        }
    }

    return true;
}

/* ============================================================
 * 初始化
 * ============================================================ */

mixer_status_t mixer_init(
    mixer_state_t *state,
    const algo_mixer_config_t *config,
    const algo_phase_config_t *phase_config
)
{
    algo_mixer_config_t config_copy;
    algo_phase_config_t phase_config_copy;

    if (state == NULL) {
        return MIXER_STATUS_INVALID_ARG;
    }

    /* 防止重新初始化失败后继续使用旧状态。 */
    state->initialized = false;

    if ((config == NULL) || (phase_config == NULL)) {
        return MIXER_STATUS_INVALID_ARG;
    }

    if (!mixer_config_is_valid(config, phase_config)) {
        return MIXER_STATUS_INVALID_CONFIG;
    }

    /*
     * 先复制到局部变量，再清空状态。
     * 这样即使调用者传入的是 state 内部的配置副本，
     * 也不会在复制前被 memset 清除。
     */
    config_copy = *config;
    phase_config_copy = *phase_config;

    memset(state, 0, sizeof(*state));

    state->config = config_copy;
    state->phase_config = phase_config_copy;
    state->initialized = true;

    /* 初始化后先填入安全波形参数，保证 last_command 内容确定。 */
    mixer_set_safe_command(
        state,
        false,
        &state->last_command
    );

    return MIXER_STATUS_OK;
}

/* ============================================================
 * 复位
 * ============================================================ */

mixer_status_t mixer_reset(
    mixer_state_t *state
)
{
    if (state == NULL) {
        return MIXER_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return MIXER_STATUS_NOT_INITIALIZED;
    }

    /* 保留配置，仅把最近一次结果恢复为安全状态。 */
    mixer_set_safe_command(
        state,
        false,
        &state->last_command
    );

    return MIXER_STATUS_OK;
}

/* ============================================================
 * 更新控制分配
 * ============================================================ */

mixer_status_t mixer_update(
    mixer_state_t *state,
    const control_out_t *control,
    const safety_directive_t *safety,
    flap_wave_cmd_t *output
)
{
    float lift;                                        /* 限幅后的升力需求，[0, 1] */
    float roll;                                        /* 限幅后的滚转需求，[-1, 1] */
    float pitch;                                       /* 限幅后的俯仰需求，[-1, 1] */
    float amplitude_limit;                             /* 安全幅度系数，[0, 1] */
    float differential_limit;                          /* 安全差动系数，[0, 1] */
    float base_amplitude;                              /* 由升力决定的基础扑翼幅度（rad） */
    float roll_offset;                                 /* 单侧滚转差动幅度（rad） */
    float max_roll_offset;                             /* 允许的最大单侧差动（rad） */
    float pitch_bias;                                  /* 上下拍不对称偏置 */
    float max_amplitude[ALGO_WING_COUNT];              /* 各翼当前允许的最大幅度（rad） */
    uint32_t wing;

    if (output == NULL) {
        return MIXER_STATUS_INVALID_ARG;
    }

    /* 任何返回路径都保证 output 内容确定。 */
    memset(output, 0, sizeof(*output));

    if (state == NULL) {
        return MIXER_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return MIXER_STATUS_NOT_INITIALIZED;
    }

    /* 默认先填入零幅度输出。 */
    mixer_set_safe_command(state, false, output);

    if ((control == NULL) || (safety == NULL)) {
        state->last_command = *output;
        return MIXER_STATUS_INVALID_ARG;
    }

    /* 安全限制本身不能包含非法数值。 */
    if (!isfinite(safety->amplitude_limit) ||
        !isfinite(safety->differential_limit)) {
        state->last_command = *output;
        return MIXER_STATUS_INVALID_INPUT;
    }

    /*
     * 控制模块已经声明结果无效时，不读取其中的控制量，
     * 直接生成零幅度输出。
     */
    if (!control->valid) {
        state->last_command = *output;
        return MIXER_STATUS_OK;
    }

    if (!mixer_control_is_finite(control)) {
        state->last_command = *output;
        return MIXER_STATUS_INVALID_INPUT;
    }

    /*
     * 有有效控制数据，但安全状态机当前禁止扑翼。
     * 输出结构仍然有效，只是幅度和偏置均为零。
     */
    if (!safety->allow_flap || safety->immediate_stop) {
        output->valid = true;
        state->last_command = *output;
        return MIXER_STATUS_OK;
    }

    /* 归一化控制需求限幅：超界按饱和处理，不作为接口错误。 */
    lift = mixer_clampf(control->lift, 0.0f, 1.0f);
    roll = mixer_clampf(control->roll, -1.0f, 1.0f);
    pitch = mixer_clampf(control->pitch, -1.0f, 1.0f);

    amplitude_limit = mixer_clampf(
        safety->amplitude_limit,
        0.0f,
        1.0f
    );

    differential_limit = mixer_clampf(
        safety->differential_limit,
        0.0f,
        1.0f
    );

    /* 总升力映射为目标频率。 */
    output->frequency_hz =
        state->phase_config.min_frequency_hz +
        lift *
        (state->phase_config.max_frequency_hz -
         state->phase_config.min_frequency_hz);

    /* 总升力映射为基础扑翼幅度。 */
    base_amplitude =
        state->config.lift_amplitude_gain_rad * lift;

    /*
     * 正滚转需求：
     * 左翼增加幅度，右翼减小幅度，形成右滚控制力矩。
     */
    max_roll_offset =
        state->config.max_differential_rad *
        differential_limit;

    roll_offset = mixer_clampf(
        roll * state->config.roll_differential_gain_rad,
        -max_roll_offset,
        max_roll_offset
    );

    /* 安全状态机通过幅度系数收缩各翼的幅度上限。 */
    max_amplitude[ALGO_WING_LEFT] =
        state->config.max_amplitude_rad[ALGO_WING_LEFT] *
        amplitude_limit;

    max_amplitude[ALGO_WING_RIGHT] =
        state->config.max_amplitude_rad[ALGO_WING_RIGHT] *
        amplitude_limit;

    /* 左右翼幅度 = 基础幅度 ± 滚转差动，并分别限幅。 */
    output->amplitude_rad[ALGO_WING_LEFT] = mixer_clampf(
        base_amplitude + roll_offset,
        0.0f,
        max_amplitude[ALGO_WING_LEFT]
    );

    output->amplitude_rad[ALGO_WING_RIGHT] = mixer_clampf(
        base_amplitude - roll_offset,
        0.0f,
        max_amplitude[ALGO_WING_RIGHT]
    );

    /* 正俯仰需求映射为相对增强下拍。 */
    pitch_bias = mixer_clampf(
        pitch * state->config.pitch_up_down_gain,
        -state->config.max_up_down_bias,
        state->config.max_up_down_bias
    );

    output->up_down_bias[ALGO_WING_LEFT] = pitch_bias;
    output->up_down_bias[ALGO_WING_RIGHT] = pitch_bias;

    /* 第一版不主动调整扑翼中位。 */
    for (wing = 0u; wing < (uint32_t)ALGO_WING_COUNT; ++wing) {
        output->center_rad[wing] =
            state->config.center_rad[wing];
    }

    output->shape_k = state->config.default_shape_k;

    /* 版本号由 waveform 在整拍提交时递增。 */
    output->command_seq = 0u;

    /* 防止极端配置导致乘法溢出后输出非有限值。 */
    if (!mixer_output_is_finite(output)) {
        mixer_set_safe_command(state, false, output);
        state->last_command = *output;
        return MIXER_STATUS_INVALID_INPUT;
    }

    output->valid = true;
    state->last_command = *output;

    return MIXER_STATUS_OK;
}

/* ============================================================
 * 获取最近一次混控结果
 * ============================================================ */

mixer_status_t mixer_get(
    const mixer_state_t *state,
    flap_wave_cmd_t *output
)
{
    if (output == NULL) {
        return MIXER_STATUS_INVALID_ARG;
    }

    memset(output, 0, sizeof(*output));

    if (state == NULL) {
        return MIXER_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return MIXER_STATUS_NOT_INITIALIZED;
    }

    /* 直接返回最近一次 mixer_update / init / reset 留下的结果。 */
    *output = state->last_command;

    return MIXER_STATUS_OK;
}
