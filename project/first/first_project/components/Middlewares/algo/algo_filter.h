#ifndef ALGO_FILTER_H
#define ALGO_FILTER_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_config.h"
#include "algo_types.h"

/*
 * IMU 滤波模块
 *
 * 主要职责：
 *   1. 陀螺仪低通滤波；
 *   2. 加速度低通滤波；
 *   3. 跟随实际扑翼频率的基频陷波；
 *   4. 预留二次谐波陷波；
 *   5. 计算陀螺振动强度；
 *   6. 计算加速度作为重力参考的可信度。
 *
 * 不负责：
 *   - Mahony 姿态解算；
 *   - PID；
 *   - 扑翼相位生成；
 *   - IMU 寄存器读写；
 *   - 舵机输出。
 *
 * 输入 IMU 数据必须已经完成：
 *   - 量程转换；
 *   - 安装方向转换；
 *   - 单位转换；
 *   - 必要的零偏校准。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 模块返回状态
 * ============================================================ */

typedef enum {
    FILTER_STATUS_OK = 0,

    FILTER_STATUS_INVALID_ARG = -1,
    FILTER_STATUS_INVALID_CONFIG = -2,
    FILTER_STATUS_NOT_INITIALIZED = -3,
    FILTER_STATUS_INVALID_DT = -4,
    FILTER_STATUS_INVALID_PHASE = -5
} filter_status_t;

/* ============================================================
 * 一阶低通滤波器状态
 *
 * 三轴共用同一个 alpha，但分别保存滤波结果。
 * ============================================================ */

typedef struct {
    float alpha;

    algo_vec3_t value;

    /*
     * false：下一有效样本用于初始化输出，避免从零开始产生瞬态；
     * true ：正常进行递推滤波。
     */
    bool primed;
} filter_lowpass_t;

/* ============================================================
 * 二阶陷波器系数
 *
 * 使用归一化形式：
 *
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
 *                 - a1*y[n-1] - a2*y[n-2]
 *
 * a0 默认已经归一化为 1，不需要保存。
 * ============================================================ */

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} filter_biquad_coeff_t;

/* ============================================================
 * 单个二阶滤波器历史状态
 *
 * 每个轴使用独立状态，但可共用同一组陷波系数。
 * ============================================================ */

typedef struct {
    float x1;
    float x2;
    float y1;
    float y2;
} filter_biquad_state_t;

/* ============================================================
 * 滤波模块内部状态
 *
 * 配置在初始化时复制，不保存外部配置指针。
 * ============================================================ */

typedef struct {
    algo_filter_config_t config;

    /* 初始化时的名义采样周期。 */
    float nominal_dt_s;

    /* 上一次系数计算使用的采样周期。 */
    float coefficient_dt_s;

    /* 陀螺仪和加速度低通。 */
    filter_lowpass_t gyro_lowpass;
    filter_lowpass_t accel_lowpass;

    /* 扑翼基频陷波。 */
    filter_biquad_coeff_t flap_notch_coeff;
    filter_biquad_state_t flap_notch_state[ALGO_AXIS_COUNT];

    float flap_notch_center_hz;

    /* 二次谐波陷波，配置关闭时不参与计算。 */
    filter_biquad_coeff_t harmonic_notch_coeff;
    filter_biquad_state_t harmonic_notch_state[ALGO_AXIS_COUNT];

    float harmonic_notch_center_hz;

    /* 最近一次用于更新陷波系数的实际扑翼频率。 */
    float last_notch_update_frequency_hz;

    /* 当前陀螺振动强度估计。 */
    float vibration_rms_rad_s;

    /* 当前加速度可信度。 */
    float accel_trust;

    bool initialized;
} filter_state_t;

/* ============================================================
 * 初始化
 *
 * 参数：
 *   state：
 *     滤波模块状态，不允许为空。
 *
 *   config：
 *     滤波配置，初始化时被完整复制。
 *
 *   nominal_dt_s：
 *     名义采样周期，400 Hz 时传入 0.0025f。
 *
 * 初始化检查：
 *   - 低通截止频率必须大于零并低于奈奎斯特频率；
 *   - 陷波 Q 值必须大于零；
 *   - 频率重整定阈值不能为负；
 *   - 重力容忍范围必须大于零；
 *   - 可信度上下限必须位于 [0, 1]。
 *
 * 初始化后所有历史数据清零，等待第一个有效 IMU 样本。
 * ============================================================ */

filter_status_t filter_init(
    filter_state_t *state,
    const algo_filter_config_t *config,
    float nominal_dt_s
);

/* ============================================================
 * 复位
 *
 * 保留初始化配置和名义采样周期，清除：
 *   - 低通历史；
 *   - 陷波历史；
 *   - 振动估计；
 *   - 加速度可信度；
 *   - 陷波中心频率记录。
 *
 * 不会释放配置，也不会改变陷波启用状态。
 * ============================================================ */

filter_status_t filter_reset(
    filter_state_t *state
);

/* ============================================================
 * 处理一个 IMU 样本
 *
 * 参数：
 *   state：
 *     已完成初始化的滤波状态。
 *
 *   sample：
 *     当前 IMU 样本。
 *
 *   phase：
 *     当前扑翼相位快照。
 *     陷波使用其中的实际 f_hz，而不是目标频率。
 *
 *     当基频和二次谐波陷波全部关闭时，phase 可以为 NULL。
 *     只要启用任意一个陷波，就必须提供有效 phase。
 *
 *   dt_s：
 *     距离上次有效样本的真实时间间隔，必须大于零且为有限值。
 *
 *   output：
 *     本次滤波结果，始终由函数完整填充。
 *
 * 返回：
 *   FILTER_STATUS_OK：
 *     调用流程正常完成。
 *
 *     如果 sample->valid == false：
 *       - output->valid 为 false；
 *       - 不使用坏样本更新滤波历史；
 *       - 仍返回 FILTER_STATUS_OK；
 *       - 由顶层安全状态机决定故障处理。
 *
 *   其他返回值：
 *     表示接口、配置、相位或 dt 错误；
 *     output 被清零，output->valid 为 false。
 *
 * 系数更新：
 *   - dt 发生明显变化时，重新计算低通系数；
 *   - 实际扑翼频率变化超过 notch_retune_threshold_hz 时，
 *     重新计算陷波系数；
 *   - 重新计算系数时应保留滤波历史，避免不必要的输出跳变。
 * ============================================================ */

filter_status_t filter_update(
    filter_state_t *state,
    const algo_imu_sample_t *sample,
    const phase_out_t *phase,
    float dt_s,
    filter_out_t *output
);

/* ============================================================
 * 获取当前状态快照
 *
 * 不处理新样本，也不改变滤波历史。
 *
 * output->gyro_rad_s 和 output->accel_m_s2 返回当前滤波结果；
 * output->valid 表示当前是否已有有效滤波输出。
 * ============================================================ */

filter_status_t filter_get(
    const filter_state_t *state,
    filter_out_t *output
);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_FILTER_H */