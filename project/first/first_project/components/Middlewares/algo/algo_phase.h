#ifndef ALGO_PHASE_H
#define ALGO_PHASE_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_config.h"
#include "algo_types.h"

/*
 * 连续扑翼相位发生器
 *
 * 职责：
 *   1. 按真实 dt 连续推进相位；
 *   2. 对目标频率进行变化率限制；
 *   3. 产生整拍事件和累计拍数；
 *   4. 向滤波器提供当前实际扑翼频率。
 *
 * 不负责：
 *   - PID；
 *   - 波形生成；
 *   - 舵机输出；
 *   - 整拍波形参数提交。
 *
 * 相位始终保持在 [0, 2π)，设置目标频率不会重置相位。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PHASE_STATUS_OK = 0,

    PHASE_STATUS_INVALID_ARG = -1,
    PHASE_STATUS_INVALID_CONFIG = -2,
    PHASE_STATUS_NOT_INITIALIZED = -3,
    PHASE_STATUS_INVALID_DT = -4,
    PHASE_STATUS_INVALID_FREQUENCY = -5
} phase_status_t;

typedef struct {
    /*
     * 当前相位，使用 double 降低长期累计误差。
     * 始终限制在 [0, 2π)。
     */
    double psi_rad;

    /* 当前实际频率。 */
    float f_hz;

    /* 期望达到的目标频率。 */
    float f_target_hz;

    /* 以下为初始化时复制的配置。 */
    float default_frequency_hz;
    float min_frequency_hz;
    float max_frequency_hz;
    float slew_hz_per_s;

    /* 已完成的整拍数量。 */
    uint32_t beat_count;

    bool initialized;
} phase_gen_t;

/*
 * 初始化相位发生器。
 *
 * 初始化后：
 *   psi_rad       = 0
 *   f_hz          = default_frequency_hz
 *   f_target_hz   = default_frequency_hz
 *   beat_count    = 0
 *
 * 配置会被复制，调用者不需要保证 config 的长期生命周期。
 */
phase_status_t phase_init(
    phase_gen_t *generator,
    const algo_phase_config_t *config
);

/*
 * 复位相位发生器。
 *
 * 保留初始化配置，但恢复：
 *   - 默认频率；
 *   - 零相位；
 *   - 零拍数。
 */
phase_status_t phase_reset(
    phase_gen_t *generator
);

/*
 * 设置目标频率。
 *
 * 有限数值会被限制在 [min_frequency_hz, max_frequency_hz]。
 * NaN 和无穷大将被拒绝，且不改变原目标频率。
 *
 * 当前实际频率在后续 phase_update() 中逐渐接近目标值。
 */
phase_status_t phase_set_freq(
    phase_gen_t *generator,
    float target_frequency_hz
);

/*
 * 推进一个时间周期。
 *
 * dt_s 必须：
 *   - 为有限值；
 *   - 大于零。
 *
 * 频率在一个周期内按 slew 线性变化，相位使用该周期内的
 * 平均频率积分，避免直接采用末端频率造成额外误差。
 *
 * 成功时：
 *   output->psi_rad     为更新后的相位；
 *   output->f_hz        为更新后的实际频率；
 *   output->beat_start  表示本次更新是否越过一个或多个整拍边界；
 *   output->beat_count  为累计完成的整拍数；
 *   output->valid       为 true。
 *
 * 失败时：
 *   不推进内部相位；
 *   output 被清零；
 *   output->valid 为 false。
 */
phase_status_t phase_update(
    phase_gen_t *generator,
    float dt_s,
    phase_out_t *output
);

/*
 * 获取当前状态快照，不推进相位。
 *
 * 快照中的 beat_start 固定为 false。
 */
phase_status_t phase_get(
    const phase_gen_t *generator,
    phase_out_t *output
);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_PHASE_H */