#ifndef ALGO_TYPES_H
#define ALGO_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 公共数据契约
 *
 * 本头文件定义算法各模块之间传递的全部数据结构，
 * 是模块间唯一的接口约定，各模块不得私自扩展字段语义。
 *
 * 数据流向（单周期）：
 *   algo_input_t
 *     -> 滤波 (filter_out_t)
 *     -> 姿态估计 (attitude_out_t)
 *     -> 安全状态机 (safety_directive_t)
 *     -> 控制器 (control_out_t)
 *     -> 混控 (flap_wave_cmd_t)
 *     -> 相位发生器 (phase_out_t)
 *     -> 波形发生器 (wing_target_t)
 *     -> 连杆逆解 (servo_target_t)
 *     -> algo_output_t
 *
 * 坐标系：
 *   机体系：X 向前，Y 向右，Z 向下
 *   导航系：NED
 *
 * 姿态正方向：
 *   roll  ：右滚为正
 *   pitch ：抬头为正
 *   yaw   ：向右偏航为正
 *
 * 单位约定：
 *   内部角度统一使用 rad
 *   角速度统一使用 rad/s
 *   加速度统一使用 m/s^2
 *   频率统一使用 Hz
 *   时间统一使用 us 或 ms，由字段后缀标明
 *
 * 舵机角度只在最终 ServoTarget 中使用 deg。
 */

#ifdef __cplusplus
extern "C" {
#endif

enum {
    ALGO_AXIS_COUNT = 3,    /* 三轴：X / Y / Z */
    ALGO_WING_COUNT = 2     /* 双翼：左 / 右 */
};

/* 机体三轴索引，用于按轴访问向量分量。 */
typedef enum {
    ALGO_AXIS_X = 0,    /* 前向轴 */
    ALGO_AXIS_Y = 1,    /* 右向轴 */
    ALGO_AXIS_Z = 2     /* 下向轴 */
} algo_axis_t;

/* 左右翼索引，所有按翼区分的数据均用此枚举做数组下标。 */
typedef enum {
    ALGO_WING_LEFT = 0,
    ALGO_WING_RIGHT = 1
} algo_wing_t;

/* ============================================================
 * 基础数学类型
 * ============================================================ */

/* 三维向量，具体物理含义由使用处决定（角速度、加速度等）。 */
typedef struct {
    float x;
    float y;
    float z;
} algo_vec3_t;

/* 四元数，w 为实部。使用时必须保持单位长度。 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} algo_quat_t;

/* ============================================================
 * 故障标志
 *
 * 可以多位置位，由安全状态机综合判断严重等级。
 * ============================================================ */

typedef enum {
    ALGO_FAULT_NONE               = 0,          /* 无故障 */
    ALGO_FAULT_RC_LOST            = (1u << 0),  /* 遥控信号丢失或长时间无效 */
    ALGO_FAULT_IMU_INVALID        = (1u << 1),  /* IMU 数据无效或超时 */
    ALGO_FAULT_ATTITUDE_INVALID   = (1u << 2),  /* 姿态解算结果不可用 */
    ALGO_FAULT_PHASE_INVALID      = (1u << 3),  /* 相位发生器状态异常 */
    ALGO_FAULT_SERVO_LEFT         = (1u << 4),  /* 左翼舵机故障 */
    ALGO_FAULT_SERVO_RIGHT        = (1u << 5),  /* 右翼舵机故障 */
    ALGO_FAULT_POWER              = (1u << 6),  /* 电源异常（欠压/过流等） */
    ALGO_FAULT_LINKAGE            = (1u << 7),  /* 连杆机构异常（卡滞、逆解失败等） */
    ALGO_FAULT_OUTPUT_SATURATED   = (1u << 8),  /* 输出持续饱和，控制能力不足 */
    ALGO_FAULT_CONFIG             = (1u << 9),  /* 配置参数非法 */
    ALGO_FAULT_INTERNAL           = (1u << 10)  /* 算法内部不可恢复错误 */
} algo_fault_flags_t;

/* ============================================================
 * 顶层输入
 *
 * 由 BSP / 硬件适配层填充，算法层不直接访问硬件。
 * ============================================================ */

/* 已经完成量程转换和安装方向转换的 IMU 样本。 */
typedef struct {
    uint64_t timestamp_us;      /* 采样时刻，us */

    algo_vec3_t gyro_rad_s;     /* 三轴角速度，rad/s */
    algo_vec3_t accel_m_s2;     /* 三轴加速度，m/s^2 */

    /* 本次采样是否完整、有效。 */
    bool valid;
} algo_imu_sample_t;

/* 已经完成通道映射、死区处理和归一化的遥控指令。 */
typedef struct {
    /* 总升力/油门，范围 [0, 1]。 */
    float throttle;

    /* 三轴指令，范围 [-1, 1]。 */
    float roll;
    float pitch;
    float yaw;

    /* 当前解锁开关请求，由安全状态机判断状态变化。 */
    bool arm_request;

    /*
     * 增稳模式请求（遥控器模式键）。
     * 只是请求；是否真正闭环由安全状态机根据姿态和整定状态决定。
     */
    bool stabilize_request;

    /* 当前遥控数据是否有效。 */
    bool valid;

    /* 距离最近一次有效遥控数据的时间。 */
    uint32_t age_ms;
} algo_rc_command_t;

/*
 * 由硬件适配层提供的健康信息。
 *
 * servo_fault 表示已经检测到的故障；
 * 如果硬件没有舵机反馈，应保持 false，不能宣称具备堵转检测能力。
 */
typedef struct {
    bool imu_ready;             /* IMU 已就绪（完成初始化且数据连续） */
    bool power_good;            /* 电源状态正常 */

    bool servo_fault[ALGO_WING_COUNT];  /* 各翼舵机检测到的故障 */

    /* 需要立即限制或停止输出的严重硬件故障。 */
    bool critical_output_fault;

    /* 其他未归类的故障标志，透传给安全状态机。 */
    uint32_t fault_flags;
} algo_system_health_t;

/* 单个控制周期内，算法收到的全部外部输入。 */
typedef struct {
    uint64_t now_us;            /* 当前时刻，us，作为本周期统一时间基准 */

    algo_imu_sample_t imu;      /* IMU 采样 */
    algo_rc_command_t rc;       /* 遥控指令 */
    algo_system_health_t health;/* 硬件健康状态 */
} algo_input_t;

/* ============================================================
 * 相位模块输出
 *
 * 相位发生器产生扑翼周期节拍，供波形发生器查波形。
 * phase_gen_t 是模块私有状态，继续保留在 algo_phase.h。
 * ============================================================ */

typedef struct {
    /* 当前相位，范围 [0, 2π)。 */
    float psi_rad;

    /* slew 限制后的当前实际频率，不是目标频率。 */
    float f_hz;

    /* 本次更新是否越过整拍边界。 */
    bool beat_start;

    /* 已完成的整拍数量。 */
    uint32_t beat_count;

    bool valid;
} phase_out_t;

/* ============================================================
 * 滤波模块输出
 *
 * 对原始 IMU 数据做低通/陷波处理，并评估数据质量。
 * ============================================================ */

typedef struct {
    algo_vec3_t gyro_rad_s;     /* 滤波后角速度，rad/s */
    algo_vec3_t accel_m_s2;     /* 滤波后加速度，m/s^2 */

    /*
     * 加速度作为重力参考的可信度，范围 [0, 1]。
     * 振动或冲击较大时降低，Mahony 据此调整修正强度。
     */
    float accel_trust;

    /* 陀螺振动强度估计，用于诊断和故障判断。 */
    float vibration_rms_rad_s;

    bool gyro_valid;            /* 陀螺通道有效 */
    bool accel_valid;           /* 加速度计通道有效 */
    bool valid;                 /* 整体输出有效 */
} filter_out_t;

/* ============================================================
 * 姿态模块输出
 *
 * 姿态解算（Mahony 类算法）的结果，供控制器做闭环反馈。
 * ============================================================ */

typedef struct {
    /* 机体系到 NED 导航系的旋转四元数，必须保持归一化。 */
    algo_quat_t q_bn;

    float roll_rad;             /* 滚转角，rad */
    float pitch_rad;            /* 俯仰角，rad */
    float yaw_rad;              /* 偏航角，rad */

    /* 姿态是否可用于控制。 */
    bool valid;

    /* 是否已经完成初始收敛。 */
    bool converged;

    /* 本次姿态修正实际使用的重力参考可信度。 */
    float accel_trust;
} attitude_out_t;

/* ============================================================
 * 控制器输出
 *
 * 这里只表达虚拟力和力矩需求，不直接表达翼角或舵机角。
 * 具体如何映射到扑翼动作由混控决定。
 * ============================================================ */

typedef struct {
    /* 总升力需求，范围 [0, 1]。 */
    float lift;

    /* 滚转控制需求，范围 [-1, 1]。 */
    float roll;

    /* 俯仰控制需求，范围 [-1, 1]。 */
    float pitch;

    /* 偏航控制需求，范围 [-1, 1]，第一版可以始终为 0。 */
    float yaw;

    bool valid;
} control_out_t;

/* ============================================================
 * 混控输出：一整拍波形参数
 *
 * 把控制需求映射为扑翼波形的幅度/中位/不对称等参数。
 * mixer 生成待提交参数；
 * waveform 在整拍边界将其切换为当前生效参数。
 * ============================================================ */

typedef struct {
    /* 相位发生器的目标频率，实际频率仍受 slew 限制。 */
    float frequency_hz;

    /* 左右翼扑翼幅度，单位 rad，必须非负。 */
    float amplitude_rad[ALGO_WING_COUNT];

    /* 左右翼运动中位，单位 rad。 */
    float center_rad[ALGO_WING_COUNT];

    /*
     * 上下拍不对称控制量，范围 [-1, 1]。
     *
     * 正值表示相对增强下拍；
     * 负值表示相对增强上拍；
     * 具体波形映射由 waveform 统一实现。
     */
    float up_down_bias[ALGO_WING_COUNT];

    /*
     * 波形形状参数。
     * 0 表示基准正弦波，其余有效范围由配置限定。
     */
    float shape_k;

    /* 参数版本号，由 waveform 在正式提交时递增。 */
    uint32_t command_seq;

    bool valid;
} flap_wave_cmd_t;

/* ============================================================
 * 波形模块输出：左右翼目标
 *
 * 根据生效的波形参数和当前相位，生成两翼的目标角度与角速度。
 * ============================================================ */

typedef struct {
    float angle_rad[ALGO_WING_COUNT];       /* 两翼目标角度，rad */
    float rate_rad_s[ALGO_WING_COUNT];      /* 两翼目标角速度，rad/s */

    /* 是否发生幅度、速度或其他输出限幅。 */
    bool saturated[ALGO_WING_COUNT];

    bool valid;
} wing_target_t;

/* ============================================================
 * 连杆逆解输出：逻辑舵机目标
 *
 * 把翼角经连杆机构反解为舵机角，这是算法输出的最后一环。
 * ============================================================ */

typedef struct {
    /*
     * 左右逻辑舵机角，单位 deg。
     * 不包含 IO1/IO8 的电气反相，也不包含 PWM 脉宽转换。
     */
    float angle_deg[ALGO_WING_COUNT];

    /* 目标位置是否存在有效连杆逆解。 */
    bool reachable[ALGO_WING_COUNT];

    /* 是否因机械行程或舵机限位而饱和。 */
    bool saturated[ALGO_WING_COUNT];

    bool valid;
} servo_target_t;

/* ============================================================
 * 安全状态机
 *
 * 汇总所有输入健康和故障信息，决定系统当前允许哪些行为。
 * 状态迁移大致为：
 *   BOOT -> SELFTEST -> STANDBY -> ARMED
 *   任意状态在故障时可进入 DEGRADED / FAILSAFE / ESTOP
 * ============================================================ */

typedef enum {
    ALGO_SAFETY_BOOT = 0,   /* 上电初始，尚未自检 */
    ALGO_SAFETY_SELFTEST,   /* 自检中（传感器、舵机行程等） */
    ALGO_SAFETY_STANDBY,    /* 自检通过，等待解锁 */
    ALGO_SAFETY_ARMED,      /* 已解锁，允许正常扑翼输出 */
    ALGO_SAFETY_DEGRADED,   /* 降级运行：有非致命故障，限幅输出 */
    ALGO_SAFETY_FAILSAFE,   /* 失控保护：如遥控失联，进入受控处置 */
    ALGO_SAFETY_ESTOP       /* 急停：严重故障，立即停止输出 */
} safety_state_t;

typedef struct {
    safety_state_t state;

    /* 是否允许产生扑翼输出。 */
    bool allow_flap;

    /* 是否允许姿态闭环修正。 */
    bool allow_stabilization;

    /*
     * 严重故障立即限制输出。
     * 普通失联和受控降幅不能简单复用该标志。
     */
    bool immediate_stop;

    /* 请求控制器清零积分和微分历史。 */
    bool reset_controller;

    /* 请求姿态估计器重新收敛。 */
    bool reset_attitude;

    /* 相对于配置最大幅度的限制，范围 [0, 1]。 */
    float amplitude_limit;

    /* 相对于配置最大差动量的限制，范围 [0, 1]。 */
    float differential_limit;

    uint32_t fault_flags;
} safety_directive_t;

/* ============================================================
 * 顶层算法输出
 *
 * 单个控制周期内全部模块输出的汇总，BSP 据此驱动硬件，
 * 也可用于日志记录和在线调试。
 * ============================================================ */

typedef struct {
    uint64_t timestamp_us;

    /* 最终逻辑舵机输出，BSP 只负责电气转换和写入。 */
    servo_target_t servo;

    /* 当前生效的波形参数，不是等待提交的参数。 */
    flap_wave_cmd_t active_wave;

    wing_target_t wing;             /* 两翼目标角度/角速度 */

    phase_out_t phase;              /* 相位发生器输出 */
    filter_out_t filter;            /* 滤波结果 */
    attitude_out_t attitude;        /* 姿态估计 */
    control_out_t control;          /* 控制器需求 */
    safety_directive_t safety;      /* 安全指令 */

    /* 本周期所有模块故障标志的汇总。 */
    uint32_t fault_flags;

    /* 整个算法输出是否有效。 */
    bool valid;
} algo_output_t;

#ifdef __cplusplus
}
#endif

#endif /* ALGO_TYPES_H */
