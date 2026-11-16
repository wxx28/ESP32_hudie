#ifndef ALGO_MIXER_H
#define ALGO_MIXER_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_config.h"
#include "algo_types.h"

/*
 * 控制分配（混控）模块
 *
 * 主要职责：
 *   1. 将 control_out_t 中的归一化控制需求转换为扑翼波形参数；
 *   2. 将总升力需求映射为目标扑翼频率和基础扑翼幅度；
 *   3. 通过左右翼幅度差动实现滚转控制；
 *   4. 通过上下拍不对称实现俯仰控制；
 *   5. 根据安全指令限制或禁止扑翼输出；
 *   6. 对频率、幅度、差动和上下拍偏置进行限幅。
 *
 * 不负责：
 *   - 姿态解算；
 *   - PID 控制；
 *   - 相位推进；
 *   - 波形生成；
 *   - 连杆逆解；
 *   - 舵机电气方向转换和 PWM 输出。
 *
 * 第一版控制分配约定：
 *
 *   总升力：
 *     lift 范围 [0, 1]。
 *     频率在 phase_config 的最小值和最大值之间线性映射。
 *     基础幅度 = lift_amplitude_gain_rad * lift。
 *
 *   滚转：
 *     roll > 0 表示右滚控制需求。
 *     增加左翼幅度、减小右翼幅度：
 *       left  = base + roll_offset
 *       right = base - roll_offset
 *
 *     max_differential_rad 表示单侧 roll_offset 的最大绝对值，
 *     左右翼实际幅度差最大为其两倍。
 *
 *   俯仰：
 *     pitch > 0 映射为正的 up_down_bias，表示相对增强下拍。
 *
 *   偏航：
 *     第一版不参与控制分配。
 *     yaw_mixer_enable 必须保持 false，待台架辨识后再扩展。
 *
 * 波形参数提交：
 *   mixer 每次只生成“待提交参数”；
 *   command_seq 固定填 0；
 *   waveform 在整拍边界提交参数时负责递增 command_seq。
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
    MIXER_STATUS_OK = 0,

    /* 必要指针为空。 */
    MIXER_STATUS_INVALID_ARG = -1,

    /* 混控配置或频率范围非法。 */
    MIXER_STATUS_INVALID_CONFIG = -2,

    /* 尚未成功调用 mixer_init()。 */
    MIXER_STATUS_NOT_INITIALIZED = -3,

    /*
     * 输入包含 NaN、无穷大等非法数值。
     * 此时输出安全波形参数，output->valid 为 false。
     */
    MIXER_STATUS_INVALID_INPUT = -4
} mixer_status_t;

/* ============================================================
 * 混控模块内部状态
 *
 * 配置在初始化时复制，不保存外部配置指针。
 * ============================================================ */

typedef struct {
    algo_mixer_config_t config;

    /*
     * 只使用其中的 min_frequency_hz 和 max_frequency_hz。
     * 相位发生器仍然独立保存和检查自己的完整配置。
     */
    algo_phase_config_t phase_config;

    /* 最近一次生成的待提交波形参数，用于调试和测试。 */
    flap_wave_cmd_t last_command;

    bool initialized;
} mixer_state_t;

/* ============================================================
 * 初始化
 *
 * 参数：
 *   state：
 *     混控模块状态，不允许为空。
 *
 *   config：
 *     控制分配配置，初始化时被完整复制。
 *
 *   phase_config：
 *     提供允许的目标扑翼频率范围，不允许为空。
 *
 * 主要配置检查：
 *   - 所有参与计算的参数必须为有限值；
 *   - 幅度、差动和中位调整参数不能为负；
 *   - 左右翼最大幅度必须大于零；
 *   - max_up_down_bias 必须位于 [0, 1]；
 *   - default_shape_k 第一版限定在 [0, 1]；
 *   - 最小频率必须大于零；
 *   - 最大频率必须大于或等于最小频率；
 *   - 第一版要求 yaw_mixer_enable == false。
 *
 * 初始化后：
 *   - 左右翼幅度为零；
 *   - 上下拍偏置为零；
 *   - 中位使用 config->center_rad；
 *   - 目标频率使用 min_frequency_hz；
 *   - shape_k 使用 default_shape_k；
 *   - command_seq 为零；
 *   - last_command.valid 为 false。
 * ============================================================ */

mixer_status_t mixer_init(
    mixer_state_t *state,
    const algo_mixer_config_t *config,
    const algo_phase_config_t *phase_config
);

/* ============================================================
 * 复位
 *
 * 保留初始化时复制的配置，清除最近一次控制结果。
 *
 * 复位后的波形参数与初始化后的安全状态一致：
 *   - 幅度为零；
 *   - 上下拍偏置为零；
 *   - 中位恢复配置初值；
 *   - 目标频率恢复最小频率；
 *   - command_seq 为零；
 *   - valid 为 false。
 *
 * 返回：
 *   MIXER_STATUS_OK：复位成功；
 *   MIXER_STATUS_INVALID_ARG：state 为空；
 *   MIXER_STATUS_NOT_INITIALIZED：尚未初始化。
 * ============================================================ */

mixer_status_t mixer_reset(
    mixer_state_t *state
);

/* ============================================================
 * 更新控制分配
 *
 * 本函数是无时间递推的映射计算，不需要 dt。
 *
 * 参数：
 *   state：
 *     已完成初始化的混控状态。
 *
 *   control：
 *     控制器输出的归一化需求，不允许为空。
 *
 *   safety：
 *     当前安全指令，不允许为空。
 *
 *   output：
 *     待提交扑翼波形参数，始终由本函数完整填充。
 *
 * 正常映射：
 *   1. 将 lift 限制在 [0, 1]；
 *   2. 将 roll、pitch 限制在 [-1, 1]；
 *   3. 根据 lift 计算目标频率和基础幅度；
 *   4. 根据 roll 计算左右翼幅度差动；
 *   5. 根据 pitch 计算上下拍偏置；
 *   6. 根据 safety 限制最大幅度和最大差动；
 *   7. 对左右翼输出分别进行最终限幅。
 *
 * 安全限制：
 *   - safety->amplitude_limit 和 differential_limit
 *     会被限制在 [0, 1]；
 *
 *   - 单翼允许最大幅度：
 *       max_amplitude_rad[wing] * amplitude_limit
 *
 *   - 允许的最大单侧滚转差动：
 *       max_differential_rad * differential_limit
 *
 *   - allow_flap == false 或 immediate_stop == true 时：
 *       左右翼幅度输出零；
 *       左右翼上下拍偏置输出零；
 *       中位和波形形状保留配置值；
 *       目标频率使用最小频率。
 *
 * 无效控制输入：
 *   如果 control->valid == false：
 *     - 返回 MIXER_STATUS_OK；
 *     - 输出零幅度、零差动、零上下拍偏置；
 *     - output->valid 为 false；
 *     - 由顶层安全状态机决定后续处置。
 *
 * 非法数值：
 *   如果控制量或安全限制包含 NaN、无穷大：
 *     - 返回 MIXER_STATUS_INVALID_INPUT；
 *     - 输出安全波形参数；
 *     - output->valid 为 false。
 *
 * 其他说明：
 *   - 超出有限范围的 lift、roll、pitch 采用限幅处理，
 *     不作为接口错误；
 *   - 第一版 center_rad 使用配置中位，不主动调整中位；
 *   - 第一版 yaw 不参与分配；
 *   - output->command_seq 始终填 0；
 *   - 成功生成可用参数时 output->valid 为 true。
 * ============================================================ */

mixer_status_t mixer_update(
    mixer_state_t *state,
    const control_out_t *control,
    const safety_directive_t *safety,
    flap_wave_cmd_t *output
);

/* ============================================================
 * 获取最近一次混控结果
 *
 * 不重新进行控制分配，也不修改模块状态。
 *
 * 如果尚未获得有效控制输入，返回的 last_command.valid
 * 可能为 false，但接口本身仍可返回 MIXER_STATUS_OK。
 *
 * 返回：
 *   MIXER_STATUS_OK：获取成功；
 *   MIXER_STATUS_INVALID_ARG：state 或 output 为空；
 *   MIXER_STATUS_NOT_INITIALIZED：尚未初始化。
 * ============================================================ */

mixer_status_t mixer_get(
    const mixer_state_t *state,
    flap_wave_cmd_t *output
);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_MIXER_H */