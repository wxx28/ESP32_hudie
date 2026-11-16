#ifndef ALGO_WAVEFORM_H
#define ALGO_WAVEFORM_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_config.h"
#include "algo_types.h"

/*
 * 扑翼波形发生模块
 *
 * 数据流：
 *
 *   mixer 生成待提交的 flap_wave_cmd_t
 *              ↓
 *   phase 更新并产生 beat_start
 *              ↓
 *   waveform 在整拍边界提交新参数
 *              ↓
 *   根据当前相位生成左右翼 angle/rate
 *              ↓
 *   wing_target_t
 *              ↓
 *   algo_linkage
 *
 * 主要职责：
 *   1. 保存当前生效的一整拍波形参数；
 *   2. 只在整拍边界提交新参数，避免拍中突变；
 *   3. 根据实际相位生成左右翼目标角和目标角速度；
 *   4. 应用上下拍不对称和波形形状参数；
 *   5. 执行翼角和翼角速度限幅；
 *   6. 输出饱和状态。
 *
 * 不负责：
 *   - 不计算 PID；
 *   - 不重新进行控制分配；
 *   - 不推进相位；
 *   - 不设置目标扑翼频率；
 *   - 不进行连杆逆解；
 *   - 不直接输出舵机 PWM。
 *
 * 频率约定：
 *   - 波形角速度使用 phase_out_t.f_hz，即实际频率；
 *   - 不使用 flap_wave_cmd_t.frequency_hz 计算角速度；
 *   - 目标频率仍由顶层通过 phase_set_freq() 设置。
 *
 * 本模块不访问硬件、不使用动态内存，也不依赖 Arduino。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 模块返回状态
 * ============================================================ */

typedef enum {
    WAVEFORM_STATUS_OK = 0,

    /* 必要指针为空。 */
    WAVEFORM_STATUS_INVALID_ARG = -1,

    /* 翼角或角速度限幅配置非法。 */
    WAVEFORM_STATUS_INVALID_CONFIG = -2,

    /* 尚未成功调用 waveform_init()。 */
    WAVEFORM_STATUS_NOT_INITIALIZED = -3,

    /* 相位数据无效或包含非法数值。 */
    WAVEFORM_STATUS_INVALID_PHASE = -4,

    /* 整拍边界收到非法待提交参数。 */
    WAVEFORM_STATUS_INVALID_COMMAND = -5,

    /* 波形计算过程中产生非有限值。 */
    WAVEFORM_STATUS_NUMERIC_ERROR = -6
} waveform_status_t;

/* ============================================================
 * 波形模块内部状态
 * ============================================================ */

typedef struct {
    algo_waveform_config_t config;

    /*
     * 当前正在生效的一整拍参数。
     *
     * active_command.valid=false 表示尚未获得有效波形参数，
     * 此时输出无效，不允许生成扑翼目标。
     */
    flap_wave_cmd_t active_command;

    bool initialized;
} waveform_state_t;

/* ============================================================
 * 初始化
 *
 * 配置检查：
 *   - 每侧 min_wing_angle_rad < max_wing_angle_rad；
 *   - 每侧 max_wing_rate_rad_s 必须为有限正值；
 *   - 所有角度和角速度限值必须为有限值；
 *   - angle_limit_enable 和 rate_limit_enable 可独立开关。
 *
 * mechanical_limits_valid：
 *   - false 不属于波形配置格式错误；
 *   - 算法仍可进行主机测试和数学验证；
 *   - 顶层安全状态机不得因此允许正常飞行解锁。
 *
 * 初始化后：
 *   - active_command 清零；
 *   - active_command.valid=false；
 *   - command_seq=0；
 *   - 等待第一次整拍参数提交。
 * ============================================================ */

waveform_status_t waveform_init(
    waveform_state_t *state,
    const algo_waveform_config_t *config
);

/* ============================================================
 * 复位波形模块
 *
 * 保留初始化配置，清除：
 *   - 当前生效波形参数；
 *   - 参数版本号；
 *   - 有效状态。
 *
 * 复位后必须等待新的有效整拍提交，
 * 不得继续沿用复位前的扑翼幅度。
 * ============================================================ */

waveform_status_t waveform_reset(
    waveform_state_t *state
);

/* ============================================================
 * 波形数学约定
 *
 * 记：
 *   psi = 当前相位；
 *   s   = sin(psi)；
 *   c   = cos(psi)；
 *   k   = shape_k，范围 [0, 1]。
 *
 * 基准形状：
 *
 *   wave(psi) = (1 + k) * s - k * s^3
 *
 * 导数：
 *
 *   d_wave_d_psi = c * (1 + k - 3 * k * s^2)
 *
 * shape_k=0 时：
 *   wave=sin(psi)，即基准正弦波。
 *
 * 该形状在 psi=π/2 和 3π/2 时仍分别达到 +1 和 -1，
 * 不改变基准波形的峰值。
 *
 * 上下拍约定：
 *   - s >= 0 的半拍称为上拍；
 *   - s < 0 的半拍称为下拍；
 *   - 正 up_down_bias 增强下拍、减弱上拍；
 *   - 负 up_down_bias 增强上拍、减弱下拍。
 *
 * 对每侧机翼：
 *
 *   上拍比例：
 *     up_scale = 1 - up_down_bias
 *
 *   下拍比例：
 *     down_scale = 1 + up_down_bias
 *
 *   当前半拍比例记为 lobe_scale。
 *
 * 目标角度：
 *
 *   angle = center
 *           + amplitude * lobe_scale * wave
 *
 * 目标角速度：
 *
 *   rate = amplitude * lobe_scale
 *          * d_wave_d_psi
 *          * 2π * phase.f_hz
 *
 * 注意：
 *   - amplitude 必须非负；
 *   - 上下拍交界处角度连续；
 *   - 上下拍幅度不对称时，交界处的目标角速度可以变化；
 *   - 第一版接受该分段波形约定，后续再通过实测优化。
 * ============================================================ */

/* ============================================================
 * 更新波形输出
 *
 * 参数：
 *   state：
 *     已完成初始化的波形状态。
 *
 *   phase：
 *     当前相位发生器输出，不允许为空。
 *
 *   pending_command：
 *     mixer 当前生成的待提交参数，不允许为空。
 *
 *   output：
 *     当前左右翼目标，由本函数完整填充。
 *
 * ------------------------------------------------------------
 * 整拍参数提交
 * ------------------------------------------------------------
 *
 * 仅当 phase->beat_start == true 时：
 *   1. 检查 pending_command；
 *   2. pending_command.valid=true 且参数合法时，
 *      将其复制为新的 active_command；
 *   3. command_seq 不采用 mixer 填入的零，
 *      而是由 waveform 在正式提交时递增；
 *   4. 第一个正式提交的版本号为 1；
 *   5. 版本号溢出后跳过 0，从 1 继续。
 *
 * 非整拍边界：
 *   - 不提交 pending_command；
 *   - 继续使用 active_command；
 *   - 即使 pending_command 内容发生变化，
 *     当前拍的角度和角速度参数也不得拍中切换。
 *
 * 无效待提交参数：
 *   - 非整拍边界不检查其数值，也不提交；
 *   - 整拍边界 valid=false 时，清除 active_command，
 *     输出无效，返回 WAVEFORM_STATUS_OK；
 *   - 整拍边界 valid=true 但包含非法数值时，
 *     清除 active_command，输出无效，
 *     返回 WAVEFORM_STATUS_INVALID_COMMAND；
 *   - 不得在控制参数失效后无限沿用最后一组有效参数。
 *
 * ------------------------------------------------------------
 * 每周期波形计算
 * ------------------------------------------------------------
 *
 * 正常计算要求：
 *   - phase.valid == true；
 *   - active_command.valid == true；
 *   - psi_rad 位于 [0, 2π)；
 *   - f_hz 为有限正值；
 *   - 当前波形参数全部合法。
 *
 * 对左右翼分别计算：
 *   - angle_rad[wing]；
 *   - rate_rad_s[wing]。
 *
 * 翼角限幅：
 *   angle_limit_enable=true 时，将 angle_rad 限制到：
 *     [min_wing_angle_rad, max_wing_angle_rad]
 *
 * 角速度限幅：
 *   rate_limit_enable=true 时，将 rate_rad_s 限制到：
 *     [-max_wing_rate_rad_s, +max_wing_rate_rad_s]
 *
 * 饱和：
 *   - 任意角度或角速度被实际限制时，
 *     output->saturated[wing]=true；
 *   - 饱和不代表输出无效；
 *   - 限幅后的有效输出仍保持 output->valid=true。
 *
 * 无效相位或未提交有效波形：
 *   - 输出角度和角速度清零；
 *   - output->valid=false；
 *   - 不生成扑翼目标。
 *
 * 本函数不需要 dt，因为相位和实际频率已经由 phase 模块计算。
 * ============================================================ */

waveform_status_t waveform_update(
    waveform_state_t *state,
    const phase_out_t *phase,
    const flap_wave_cmd_t *pending_command,
    wing_target_t *output
);

/* ============================================================
 * 获取当前生效参数
 *
 * 不提交新参数，不计算波形，也不修改状态。
 *
 * 可用于：
 *   - 顶层填充 algo_output_t.active_wave；
 *   - 检查整拍提交是否正确；
 *   - 主机单元测试和调试。
 * ============================================================ */

waveform_status_t waveform_get_active(
    const waveform_state_t *state,
    flap_wave_cmd_t *output
);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_WAVEFORM_H */