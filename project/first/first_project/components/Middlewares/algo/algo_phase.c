#include "algo_phase.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

/* 2π，使用 double 精度字面量，降低长期相位累计误差。 */
#define PHASE_TWO_PI    6.28318530717958647692

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

/*
 * 将输出快照清零并标记为无效。
 *
 * 所有对外函数在参数检查失败时也先调用本函数，
 * 保证调用者拿到的永远是定义明确的数据。
 */
static void phase_clear_output(phase_out_t *output)
{
    if (output == NULL) {
        return;
    }

    output->psi_rad = 0.0f;
    output->f_hz = 0.0f;
    output->beat_start = false;
    output->beat_count = 0u;
    output->valid = false;
}

/*
 * 用发生器当前状态填充输出快照。
 * beat_start 由调用方根据本次更新是否越拍决定。
 */
static void phase_fill_output(
    const phase_gen_t *generator,
    bool beat_start,
    phase_out_t *output
)
{
    output->psi_rad = (float)generator->psi_rad;
    output->f_hz = generator->f_hz;
    output->beat_start = beat_start;
    output->beat_count = generator->beat_count;
    output->valid = true;
}

/*
 * 检查相位发生器配置是否合法：
 *   - 全部参数为有限值；
 *   - 频率下限为正；
 *   - 上限不低于下限；
 *   - 默认频率在 [min, max] 内；
 *   - slew 为正。
 */
static bool phase_config_is_valid(
    const algo_phase_config_t *config
)
{
    if (config == NULL) {
        return false;
    }

    if (!isfinite(config->default_frequency_hz) ||
        !isfinite(config->min_frequency_hz) ||
        !isfinite(config->max_frequency_hz) ||
        !isfinite(config->slew_hz_per_s)) {
        return false;
    }

    if (config->min_frequency_hz <= 0.0f) {
        return false;
    }

    if (config->max_frequency_hz < config->min_frequency_hz) {
        return false;
    }

    if (config->default_frequency_hz < config->min_frequency_hz ||
        config->default_frequency_hz > config->max_frequency_hz) {
        return false;
    }

    if (config->slew_hz_per_s <= 0.0f) {
        return false;
    }

    return true;
}

/* ============================================================
 * 初始化与复位
 * ============================================================ */

/*
 * 初始化相位发生器。
 *
 * 成功后：
 *   psi_rad       = 0
 *   f_hz          = default_frequency_hz
 *   f_target_hz   = default_frequency_hz
 *   beat_count    = 0
 *
 * 配置内容被复制进发生器，调用者无需保持 config 存活。
 */
phase_status_t phase_init(
    phase_gen_t *generator,
    const algo_phase_config_t *config
)
{
    if (generator == NULL) {
        return PHASE_STATUS_INVALID_ARG;
    }

    if (!phase_config_is_valid(config)) {
        generator->initialized = false;
        return PHASE_STATUS_INVALID_CONFIG;
    }

    generator->psi_rad = 0.0;

    generator->f_hz = config->default_frequency_hz;
    generator->f_target_hz = config->default_frequency_hz;

    generator->default_frequency_hz = config->default_frequency_hz;
    generator->min_frequency_hz = config->min_frequency_hz;
    generator->max_frequency_hz = config->max_frequency_hz;
    generator->slew_hz_per_s = config->slew_hz_per_s;

    generator->beat_count = 0u;
    generator->initialized = true;

    return PHASE_STATUS_OK;
}

/*
 * 复位到初始状态：零相位、默认频率、零拍数。
 * 保留初始化时的频率范围和 slew 配置。
 */
phase_status_t phase_reset(
    phase_gen_t *generator
)
{
    if (generator == NULL) {
        return PHASE_STATUS_INVALID_ARG;
    }

    if (!generator->initialized) {
        return PHASE_STATUS_NOT_INITIALIZED;
    }

    generator->psi_rad = 0.0;
    generator->f_hz = generator->default_frequency_hz;
    generator->f_target_hz = generator->default_frequency_hz;
    generator->beat_count = 0u;

    return PHASE_STATUS_OK;
}

/* ============================================================
 * 目标频率设置
 * ============================================================ */

/*
 * 设置目标频率。
 *
 * 仅修改目标值，实际频率由后续 phase_update() 按 slew 渐变逼近；
 * 不会重置相位，也不会立即跳变频率。
 *
 * 有限值会被钳位到 [min_frequency_hz, max_frequency_hz]；
 * NaN、无穷大和非正值被拒绝，且不改变原目标。
 */
phase_status_t phase_set_freq(
    phase_gen_t *generator,
    float target_frequency_hz
)
{
    if (generator == NULL) {
        return PHASE_STATUS_INVALID_ARG;
    }

    if (!generator->initialized) {
        return PHASE_STATUS_NOT_INITIALIZED;
    }

    if (!isfinite(target_frequency_hz) ||
        target_frequency_hz <= 0.0f) {
        return PHASE_STATUS_INVALID_FREQUENCY;
    }

    if (target_frequency_hz < generator->min_frequency_hz) {
        target_frequency_hz = generator->min_frequency_hz;
    }

    if (target_frequency_hz > generator->max_frequency_hz) {
        target_frequency_hz = generator->max_frequency_hz;
    }

    generator->f_target_hz = target_frequency_hz;

    return PHASE_STATUS_OK;
}

/* ============================================================
 * 相位推进
 * ============================================================ */

/*
 * 推进一个时间周期，分三步：
 *   1. 按 slew 限制把实际频率向目标频率逼近；
 *   2. 用本周期平均频率积分相位；
 *   3. 处理整拍回绕并累计拍数。
 *
 * 任何一步失败都不推进内部状态，输出清零且 valid = false。
 */
phase_status_t phase_update(
    phase_gen_t *generator,
    float dt_s,
    phase_out_t *output
)
{
    float previous_frequency_hz;
    float frequency_error_hz;
    float max_frequency_change_hz;
    float average_frequency_hz;
    double next_phase_rad;
    double completed_cycles;
    double remaining_counter_space;
    bool beat_start;

    /* 先清零输出，保证任何返回路径下 output 都定义明确。 */
    if (output != NULL) {
        phase_clear_output(output);
    }

    if (generator == NULL || output == NULL) {
        return PHASE_STATUS_INVALID_ARG;
    }

    if (!generator->initialized) {
        return PHASE_STATUS_NOT_INITIALIZED;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f) {
        return PHASE_STATUS_INVALID_DT;
    }

    previous_frequency_hz = generator->f_hz;

    /*
     * 第一步：根据 slew 限制更新当前实际频率。
     */
    frequency_error_hz =
        generator->f_target_hz - generator->f_hz;

    max_frequency_change_hz =
        generator->slew_hz_per_s * dt_s;

    if (frequency_error_hz > max_frequency_change_hz) {
        generator->f_hz += max_frequency_change_hz;
    } else if (frequency_error_hz < -max_frequency_change_hz) {
        generator->f_hz -= max_frequency_change_hz;
    } else {
        generator->f_hz = generator->f_target_hz;
    }

    /*
     * 第二步：按本周期平均频率推进相位。
     *
     * 频率在本周期内近似线性变化，因此：
     *
     * Δψ = 2π × (f_start + f_end) / 2 × dt
     */
    average_frequency_hz =
        0.5f * (previous_frequency_hz + generator->f_hz);

    next_phase_rad =
        generator->psi_rad +
        PHASE_TWO_PI *
        (double)average_frequency_hz *
        (double)dt_s;

    /*
     * 第三步：处理整拍回绕。
     *
     * 使用 floor() 而不是固定 if，可以支持一次更新跨越多拍。
     */
    completed_cycles = floor(next_phase_rad / PHASE_TWO_PI);

    next_phase_rad -= completed_cycles * PHASE_TWO_PI;

    /* 防止浮点舍入造成边界值略小于零。 */
    if (next_phase_rad < 0.0) {
        next_phase_rad += PHASE_TWO_PI;
        completed_cycles -= 1.0;
    }

    /* 防止浮点舍入造成结果恰好等于或略大于 2π。 */
    if (next_phase_rad >= PHASE_TWO_PI) {
        next_phase_rad -= PHASE_TWO_PI;
        completed_cycles += 1.0;
    }

    generator->psi_rad = next_phase_rad;

    beat_start = completed_cycles >= 1.0;

    /*
     * 正常飞行中不会接近计数上限。
     * 极端 dt 或超长运行情况下采用饱和处理，避免无符号回绕。
     */
    if (completed_cycles > 0.0) {
        remaining_counter_space =
            (double)(UINT32_MAX - generator->beat_count);

        if (completed_cycles >= remaining_counter_space) {
            generator->beat_count = UINT32_MAX;
        } else {
            generator->beat_count += (uint32_t)completed_cycles;
        }
    }

    phase_fill_output(
        generator,
        beat_start,
        output
    );

    return PHASE_STATUS_OK;
}

/* ============================================================
 * 当前状态快照
 * ============================================================ */

/*
 * 读取当前状态快照，不推进相位、不改变任何内部状态。
 * 快照中的 beat_start 固定为 false（未发生新的越拍）。
 */
phase_status_t phase_get(
    const phase_gen_t *generator,
    phase_out_t *output
)
{
    if (output != NULL) {
        phase_clear_output(output);
    }

    if (generator == NULL || output == NULL) {
        return PHASE_STATUS_INVALID_ARG;
    }

    if (!generator->initialized) {
        return PHASE_STATUS_NOT_INITIALIZED;
    }

    phase_fill_output(
        generator,
        false,
        output
    );

    return PHASE_STATUS_OK;
}
