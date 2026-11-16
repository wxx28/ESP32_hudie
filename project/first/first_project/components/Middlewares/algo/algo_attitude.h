#ifndef ALGO_ATTITUDE_H
#define ALGO_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_config.h"
#include "algo_types.h"

/*
 * Mahony 姿态解算模块
 *
 * 主要职责：
 *   1. 使用陀螺仪积分更新姿态四元数；
 *   2. 使用加速度计提供的重力方向修正滚转和俯仰；
 *   3. 根据 accel_trust 调整姿态修正强度；
 *   4. 估计陀螺仪慢变零偏；
 *   5. 输出滚转、俯仰和偏航角；
 *   6. 管理初始姿态建立和收敛状态。
 *
 * 不负责：
 *   - IMU 驱动；
 *   - 低通和陷波滤波；
 *   - PID 控制；
 *   - 扑翼相位生成；
 *   - 舵机输出。
 *
 * 坐标约定：
 *   机体系：X 向前，Y 向右，Z 向下；
 *   导航系：NED；
 *   q_bn：将机体系向量旋转到 NED 导航系。
 *
 * 注意：
 *   当前没有磁力计，偏航角只能作为相对角度使用，
 *   长时间运行可能存在慢速漂移。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 模块返回状态
 * ============================================================ */

typedef enum {
    ATTITUDE_STATUS_OK = 0,

    ATTITUDE_STATUS_INVALID_ARG = -1,
    ATTITUDE_STATUS_INVALID_CONFIG = -2,
    ATTITUDE_STATUS_NOT_INITIALIZED = -3,
    ATTITUDE_STATUS_INVALID_DT = -4,

    /* 四元数或内部状态出现非有限值。 */
    ATTITUDE_STATUS_NUMERIC_ERROR = -5
} attitude_status_t;

/* ============================================================
 * 姿态模块内部状态
 *
 * 配置在初始化时复制，不依赖外部配置的生命周期。
 * ============================================================ */

typedef struct {
    algo_attitude_config_t config;

    /*
     * Mahony 积分反馈项，单位 rad/s。
     *
     * 用于修正陀螺慢变误差，不能直接当成经过温度补偿的
     * 完整陀螺零偏模型。
     */
    algo_vec3_t integral_feedback_rad_s;

    /*
     * 当前姿态输出快照。
     *
     * current.q_bn 同时作为下一周期姿态积分的基础。
     */
    attitude_out_t current;

    /* 初始姿态建立后的累计有效运行时间。 */
    float convergence_elapsed_s;

    /*
     * false：尚未利用有效重力参考建立初始滚转和俯仰；
     * true ：已经建立初始姿态，可以正常进行 Mahony 更新。
     */
    bool orientation_initialized;

    /* 四元数归一化周期计数。 */
    uint32_t normalization_counter;

    bool initialized;
} attitude_state_t;

/* ============================================================
 * 初始化
 *
 * 参数：
 *   state：
 *     姿态模块状态，不允许为空。
 *
 *   config：
 *     Mahony 配置，初始化时被完整复制。
 *
 * 配置检查：
 *   - kp、ki 必须为有限非负值；
 *   - integral_limit_rad_s 必须为有限非负值；
 *   - 可信度阈值和 impact_kp_scale 必须位于 [0, 1]；
 *   - convergence_time_s 必须大于零；
 *   - normalization_interval 必须大于零。
 *
 * 初始化后：
 *   - 四元数设为单位四元数；
 *   - 欧拉角清零；
 *   - 积分反馈清零；
 *   - valid = false；
 *   - converged = false；
 *   - 等待有效重力参考建立初始姿态。
 * ============================================================ */

attitude_status_t attitude_init(
    attitude_state_t *state,
    const algo_attitude_config_t *config
);

/* ============================================================
 * 复位
 *
 * 保留初始化配置，清除：
 *   - 当前四元数和欧拉角；
 *   - 积分反馈；
 *   - 收敛计时；
 *   - 初始姿态建立标志；
 *   - 有效和收敛状态。
 *
 * 复位后不会立即报告有效姿态，必须重新获得有效输入。
 * ============================================================ */

attitude_status_t attitude_reset(
    attitude_state_t *state
);

/* ============================================================
 * 处理一个滤波后的 IMU 样本
 *
 * 参数：
 *   state：
 *     已完成初始化的姿态状态。
 *
 *   filtered_imu：
 *     当前滤波结果。
 *
 *     gyro_rad_s：
 *       用于姿态积分和 Mahony 反馈修正。
 *
 *     accel_m_s2：
 *       用于建立重力参考和修正滚转、俯仰。
 *
 *     accel_trust：
 *       重力参考可信度，范围 [0, 1]。
 *       冲击或振动较大时，滤波模块应降低该值。
 *
 *   dt_s：
 *     距离上次有效更新的真实时间间隔，单位秒。
 *     必须为有限正值。
 *
 *   output：
 *     当前姿态输出，由函数完整填充。
 *
 * 正常更新：
 *   1. 检查输入有效性；
 *   2. 必要时利用重力参考建立初始滚转和俯仰；
 *   3. 计算机体系重力估计与测量重力方向的误差；
 *   4. 按 accel_trust 调整比例修正；
 *   5. 更新积分反馈；
 *   6. 积分更新四元数；
 *   7. 周期性归一化四元数；
 *   8. 更新欧拉角和收敛状态。
 *
 * 无效输入：
 *   如果 filtered_imu 无效或数据包含非有限值：
 *     - 返回 ATTITUDE_STATUS_OK；
 *     - output->valid = false；
 *     - 不使用该样本推进姿态；
 *     - 保留上一次内部四元数，等待后续有效样本恢复；
 *     - 由顶层安全状态机决定故障处理。
 *
 * 注意：
 *   - 本模块每个固定控制周期运行，不是每拍运行一次；
 *   - 加速度只用于滚转和俯仰修正；
 *   - 没有磁力计时，偏航角不参与绝对方向修正。
 * ============================================================ */

attitude_status_t attitude_update(
    attitude_state_t *state,
    const filter_out_t *filtered_imu,
    float dt_s,
    attitude_out_t *output
);

/* ============================================================
 * 获取当前姿态快照
 *
 * 不处理新样本，也不改变姿态状态。
 *
 * 如果尚未获得有效姿态：
 *   output->valid = false；
 *   output->converged = false。
 *
 * 该接口可用于调试和顶层输出组装。
 * ============================================================ */

attitude_status_t attitude_get(
    const attitude_state_t *state,
    attitude_out_t *output
);

/* ============================================================
 * 显式请求重新收敛
 *
 * 与完整复位不同：
 *   - 保留当前四元数；
 *   - 保留积分反馈；
 *   - 只清除 converged 和收敛计时。
 *
 * 适用于短时异常后重新确认姿态稳定性，
 * 不适用于严重姿态发散后的完整复位。
 * ============================================================ */

attitude_status_t attitude_restart_convergence(
    attitude_state_t *state
);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_ATTITUDE_H */