#ifndef ALGO_CONTROL_H
#define ALGO_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_config.h"
#include "algo_types.h"

/*
 * 串级姿态控制模块
 *
 * 数据流：
 *
 *   RC 滚转/俯仰指令
 *        ↓
 *   目标姿态角
 *        ↓
 *   姿态角外环 PID
 *        ↓
 *   目标机体系角速度
 *        ↓
 *   角速度内环 PID
 *        ↓
 *   control_out_t
 *        ↓
 *   algo_mixer
 *
 * 控制轴：
 *   - roll：姿态角外环 + X 轴角速度内环；
 *   - pitch：姿态角外环 + Y 轴角速度内环；
 *   - yaw：直接使用 Z 轴角速度内环，不使用绝对偏航角。
 *
 * 输入来源：
 *   - rc：已经归一化的遥控指令；
 *   - attitude：Mahony 姿态解算结果；
 *   - filtered_imu：滤波后的机体系角速度；
 *   - safety：安全状态机的控制许可和复位请求。
 *
 * 输出：
 *   control_out_t.lift：直接来自 rc.throttle；
 *   control_out_t.roll：滚转角速度内环输出；
 *   control_out_t.pitch：俯仰角速度内环输出；
 *   control_out_t.yaw：偏航角速度内环输出，默认关闭。
 *
 * 不负责：
 *   - IMU 滤波；
 *   - Mahony 姿态解算；
 *   - 左右翼控制分配；
 *   - 波形和连杆逆解；
 *   - 舵机 PWM 输出。
 *
 * 坐标和单位：
 *   - 机体系 X 向前、Y 向右、Z 向下；
 *   - 姿态角使用 rad；
 *   - 角速度使用 rad/s；
 *   - PID 输出为归一化控制需求。
 *
 * 本模块不访问硬件、不使用动态内存，也不依赖 Arduino。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 姿态外环只有 roll 和 pitch 两轴。 */
enum {
    CONTROL_ANGLE_AXIS_COUNT = 2
};

/* ============================================================
 * 模块返回状态
 * ============================================================ */

typedef enum {
    CONTROL_STATUS_OK = 0,

    /* 必要指针为空。 */
    CONTROL_STATUS_INVALID_ARG = -1,

    /* PID 或控制限幅配置非法。 */
    CONTROL_STATUS_INVALID_CONFIG = -2,

    /* 尚未成功调用 control_init()。 */
    CONTROL_STATUS_NOT_INITIALIZED = -3,

    /* dt 非有限值或小于等于零。 */
    CONTROL_STATUS_INVALID_DT = -4,

    /* 有效输入中包含 NaN、无穷大等非法数值。 */
    CONTROL_STATUS_INVALID_INPUT = -5,

    /* PID 运算过程中产生非有限值。 */
    CONTROL_STATUS_NUMERIC_ERROR = -6
} control_status_t;

/* ============================================================
 * 单个 PID 的历史状态
 *
 * 角度外环和角速度内环共用该状态结构。
 *
 * 微分约定：
 *   - 对测量值微分，而不是对设定值微分；
 *   - 避免遥控目标突变造成微分冲击；
 *   - 第一个有效样本的微分输出为零；
 *   - 微分结果经过一阶低通滤波。
 * ============================================================ */

typedef struct {
    /* 当前积分项。 */
    float integral;

    /* 上一次有效更新时的测量值。 */
    float previous_measurement;

    /* 上一次低通滤波后的微分项。 */
    float derivative_filtered;

    /*
     * false：下一次有效更新只记录测量值，微分置零；
     * true ：可以正常计算测量值变化率。
     */
    bool primed;
} control_pid_state_t;

/* ============================================================
 * 控制模块内部状态
 *
 * 配置在初始化时复制，不保存外部配置指针。
 * ============================================================ */

typedef struct {
    algo_control_config_t config;

    /*
     * 姿态角外环：
     *   index 0：roll；
     *   index 1：pitch。
     */
    control_pid_state_t angle_pid[CONTROL_ANGLE_AXIS_COUNT];

    /*
     * 角速度内环：
     *   ALGO_AXIS_X：roll rate；
     *   ALGO_AXIS_Y：pitch rate；
     *   ALGO_AXIS_Z：yaw rate。
     */
    control_pid_state_t rate_pid[ALGO_AXIS_COUNT];

    /* 最近一次控制输出，用于 control_get()。 */
    control_out_t current;

    bool initialized;
} control_state_t;

/* ============================================================
 * 初始化
 *
 * 参数：
 *   state：控制器状态，不允许为空；
 *   config：串级 PID 配置，初始化时被完整复制。
 *
 * 配置检查：
 *   - kp、ki、kd 必须为有限非负值；
 *   - integral_limit 必须为有限非负值；
 *   - derivative_lpf_cutoff_hz 必须为有限正值；
 *   - 最大姿态角和最大目标角速度必须为有限正值；
 *   - max_roll_output、max_pitch_output、max_yaw_output
 *     必须位于 [0, 1]；
 *   - 全零 PID 增益属于合法配置；
 *   - yaw_control_enable=false 时，偏航输出固定为零。
 *
 * 初始化后：
 *   - 所有 PID 积分、微分和测量历史清零；
 *   - current 的控制量全部为零；
 *   - current.valid=false；
 *   - 等待第一个有效控制周期。
 * ============================================================ */

control_status_t control_init(
    control_state_t *state,
    const algo_control_config_t *config
);

/* ============================================================
 * 复位控制器
 *
 * 保留初始化时复制的配置，清除：
 *   - 滚转和俯仰姿态外环历史；
 *   - 三轴角速度内环历史；
 *   - 所有积分项；
 *   - 所有微分滤波状态；
 *   - 最近一次控制输出。
 *
 * 复位后 current.valid=false。
 *
 * 安全状态机可通过 safety->reset_controller 请求同样的复位。
 * ============================================================ */

control_status_t control_reset(
    control_state_t *state
);

/* ============================================================
 * 执行一个控制周期
 *
 * 参数：
 *   state：
 *     已完成初始化的控制器状态。
 *
 *   rc：
 *     已归一化的遥控指令。
 *
 *   attitude：
 *     当前 Mahony 姿态输出。
 *
 *   filtered_imu：
 *     当前滤波结果，使用其中的 gyro_rad_s。
 *
 *   safety：
 *     当前安全指令。
 *     本模块只使用 allow_stabilization 和 reset_controller。
 *     是否允许扑翼仍由 mixer 和输出层统一检查。
 *
 *   dt_s：
 *     距离上次有效更新的真实时间间隔，必须为有限正值。
 *
 *   output：
 *     当前控制输出，由本函数完整填充。
 *
 * ------------------------------------------------------------
 * 闭环模式：safety->allow_stabilization == true
 * ------------------------------------------------------------
 *
 * 滚转和俯仰：
 *   1. rc.roll/pitch 乘以最大目标姿态角；
 *   2. 目标姿态角减去当前姿态角，进入姿态外环；
 *   3. 外环输出目标机体系角速度；
 *   4. 目标角速度减去滤波后的陀螺角速度；
 *   5. 角速度内环输出归一化 roll/pitch 控制需求。
 *
 * 偏航：
 *   1. 不使用 attitude->yaw_rad 做绝对偏航闭环；
 *   2. rc.yaw 直接映射为目标 Z 轴角速度；
 *   3. 与 gyro_rad_s.z 比较后进入偏航角速度内环；
 *   4. yaw_control_enable=false 时输出固定为零。
 *
 * 闭环输入要求：
 *   - rc.valid == true；
 *   - attitude.valid == true；
 *   - attitude.converged == true；
 *   - filtered_imu.valid == true；
 *   - filtered_imu.gyro_valid == true。
 *
 * 条件不满足时：
 *   - 返回 CONTROL_STATUS_OK；
 *   - 输出零控制量；
 *   - output->valid=false；
 *   - 清除 PID 历史，禁止继续使用旧积分；
 *   - 不推进控制器状态。
 *
 * ------------------------------------------------------------
 * 手动模式：safety->allow_stabilization == false
 * ------------------------------------------------------------
 *
 * 仅要求 rc.valid == true：
 *   - lift 直接来自 rc.throttle；
 *   - roll/pitch 直接来自归一化 RC 指令；
 *   - 分别受 max_roll_output/max_pitch_output 限制；
 *   - 不读取姿态角和陀螺仪数值；
 *   - 不运行 PID；
 *   - 清除 PID 历史，防止恢复闭环时沿用旧积分。
 *
 * 手动模式不是安全解锁接口。
 * safety.allow_flap 仍由 mixer 和输出层单独检查。
 *
 * ------------------------------------------------------------
 * 通用处理约定
 * ------------------------------------------------------------
 *
 *   - lift 限制在 [0, 1]；
 *   - RC 三轴指令限制在 [-1, 1]；
 *   - PID 微分对测量值计算，并进行低通滤波；
 *   - 输出饱和且误差继续扩大时，按配置停止积分；
 *   - safety->reset_controller=true 时，先清除 PID 历史；
 *   - 输入为有限值但超出归一化范围时进行限幅；
 *   - 输入包含 NaN 或无穷大时返回 INVALID_INPUT；
 *   - output->valid 只表示控制数据有效，
 *     不表示 PID 已经整定，也不表示允许飞行；
 *   - 全零 PID 增益下，有效控制周期的 output->valid
 *     仍为 true，但姿态修正输出为零。
 *
 * 无效运行输入返回 CONTROL_STATUS_OK；
 * 接口、dt、非法数值和内部数值错误使用其他返回状态。
 * ============================================================ */

control_status_t control_update(
    control_state_t *state,
    const algo_rc_command_t *rc,
    const attitude_out_t *attitude,
    const filter_out_t *filtered_imu,
    const safety_directive_t *safety,
    float dt_s,
    control_out_t *output
);

/* ============================================================
 * 获取最近一次控制输出
 *
 * 不执行新的控制计算，也不改变 PID 历史。
 *
 * 如果尚未获得有效控制输出：
 *   output->valid=false，控制量为零。
 * ============================================================ */

control_status_t control_get(
    const control_state_t *state,
    control_out_t *output
);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_CONTROL_H */