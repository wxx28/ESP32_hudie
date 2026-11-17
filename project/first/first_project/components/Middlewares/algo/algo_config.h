#ifndef ALGO_CONFIG_H
#define ALGO_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_types.h"

/*
 * 算法集中配置
 *
 * 注意：
 *   1. 这里的数值只是初始设计值，不代表已经完成飞行整定。
 *   2. 所有内部角度使用 rad，舵机接口角度使用 deg。
 *   3. 几何尺寸使用 mm。
 *   4. 修改配置后必须经过合法性检查，才能进入 ARMED。
 */

/* 配置结构版本号，用于识别存储/升级配置时的布局变化。 */
#define ALGO_CONFIG_VERSION    1u

/* 角度转弧度，便于默认值宏中以 deg 书写、以 rad 存储。 */
#define ALGO_DEG_TO_RAD(deg) \
    ((deg) * 0.01745329251994329577f)

/* ============================================================
 * 调度配置
 *
 * 决定各模块的运行频率和周期容忍度。
 * ============================================================ */

typedef struct {
    /* 快速任务周期：400 Hz。 */
    float fast_dt_s;

    /* 控制器分频：2 表示每两个快速周期运行一次，即 200 Hz。 */
    uint32_t control_divider;

    /* 健康状态分频：4 表示每四个快速周期运行一次，即 100 Hz。 */
    uint32_t health_divider;

    /* 单次更新允许的最大 dt，超过后认为调度异常。 */
    float max_dt_s;
} algo_scheduler_config_t;

/* ============================================================
 * 相位发生器配置
 *
 * 相位发生器决定扑翼节拍的快慢，频率变化受 slew 平滑。
 * ============================================================ */

typedef struct {
    float default_frequency_hz; /* 上电默认扑翼频率，Hz */
    float min_frequency_hz;     /* 允许的最小频率，Hz */
    float max_frequency_hz;     /* 允许的最大频率，Hz */

    /* 目标频率变化率限制，单位 Hz/s。 */
    float slew_hz_per_s;
} algo_phase_config_t;

/* ============================================================
 * IMU 滤波配置
 *
 * 低通 + 跟随扑翼频率的陷波，并评估加速度可信度。
 * ============================================================ */

typedef struct {
    float gyro_lpf_cutoff_hz;   /* 陀螺低通截止频率，Hz */
    float accel_lpf_cutoff_hz;  /* 加速度计低通截止频率，Hz */

    /* 是否启用跟随实际扑翼频率的陷波。 */
    bool flap_notch_enable;
    float flap_notch_q;         /* 基波陷波品质因数，越大陷波越窄 */

    /* 二次谐波陷波，第一版可以关闭。 */
    bool second_harmonic_notch_enable;
    float second_harmonic_notch_q;  /* 二次谐波陷波品质因数 */

    /* 实际频率变化超过该值后，重新计算陷波系数。 */
    float notch_retune_threshold_hz;

    /*
     * 加速度模长偏离重力加速度的容忍范围。
     * 用于降低冲击段的姿态修正权重。
     */
    float gravity_tolerance_m_s2;

    /* 加速度作为重力参考时的最低、最高可信度。 */
    float min_accel_trust;
    float max_accel_trust;
} algo_filter_config_t;

/* ============================================================
 * Mahony 姿态解算配置
 * ============================================================ */

typedef struct {
    float kp;                   /* 比例增益：姿态修正强度 */
    float ki;                   /* 积分增益：陀螺零偏估计强度 */

    /* 陀螺零偏积分项限制。 */
    float integral_limit_rad_s;

    /* 低于该可信度时，不使用加速度进行姿态修正。 */
    float min_correction_trust;

    /* 冲击段比例修正系数缩放。 */
    float impact_trust_threshold;   /* 低于该可信度视为冲击段 */
    float impact_kp_scale;          /* 冲击段 kp 缩放系数 */

    /* 初始收敛判定时间。 */
    float convergence_time_s;

    /* 四元数归一化间隔，单位为快速周期数。 */
    uint32_t normalization_interval;
} algo_attitude_config_t;

/* ============================================================
 * PID 通用配置
 *
 * 角度外环和角速度内环共用此结构。
 * ============================================================ */

typedef struct {
    float kp;   /* 比例增益 */
    float ki;   /* 积分增益 */
    float kd;   /* 微分增益 */

    /* 积分项绝对值上限。 */
    float integral_limit;

    /* D 项低通滤波截止频率。 */
    float derivative_lpf_cutoff_hz;
} algo_pid_config_t;

typedef struct {
    /*
     * 姿态外环：
     *   index 0：roll
     *   index 1：pitch
     *
     * 输入为姿态角误差，输出为目标角速度。
     */
    algo_pid_config_t angle_pid[2];

    /*
     * 角速度内环：
     *   index 0：roll
     *   index 1：pitch
     *   index 2：yaw
     *
     * 输出为归一化控制量。
     */
    algo_pid_config_t rate_pid[ALGO_AXIS_COUNT];

    /* 最大滚转、俯仰目标角。 */
    float max_roll_angle_rad;
    float max_pitch_angle_rad;

    /* 三轴最大目标角速度。 */
    float max_rate_rad_s[ALGO_AXIS_COUNT];

    /* 归一化控制输出限幅。 */
    float max_roll_output;
    float max_pitch_output;
    float max_yaw_output;

    /* 第一版默认关闭偏航闭环。 */
    bool yaw_control_enable;

    /* 输出饱和且误差继续增大时，停止积分。 */
    bool anti_windup_enable;
} algo_control_config_t;

/* ============================================================
 * 控制分配配置
 *
 * 把控制器的归一化需求映射为两翼的扑翼波形参数。
 * ============================================================ */

typedef struct {
    /* 总升力到基础幅度的映射增益。 */
    float lift_amplitude_gain_rad;

    /* 左右翼允许的最大扑翼幅度。 */
    float max_amplitude_rad[ALGO_WING_COUNT];

    /* 滚转到左右幅度差动的映射增益。 */
    float roll_differential_gain_rad;

    /* 最大左右幅度差动。 */
    float max_differential_rad;

    /* 俯仰到上下拍不对称的映射增益。 */
    float pitch_up_down_gain;

    /* 上下拍不对称控制量上限，范围 [0, 1]。 */
    float max_up_down_bias;

    /* 最大中位调整量。 */
    float max_center_offset_rad;

    /* 左右翼中位初值。 */
    float center_rad[ALGO_WING_COUNT];

    /* 默认波形参数。 */
    float default_shape_k;

    /*
     * 偏航控制分配预留。
     * 未完成台架辨识前必须保持关闭。
     */
    bool yaw_mixer_enable;
} algo_mixer_config_t;

/* ============================================================
 * 波形输出与机械限制
 *
 * 对波形发生器输出的翼角和翼角速度做硬限幅。
 * ============================================================ */

typedef struct {
    /* 左右翼绝对角度范围。 */
    float min_wing_angle_rad[ALGO_WING_COUNT];
    float max_wing_angle_rad[ALGO_WING_COUNT];

    /* 左右翼最大目标角速度。 */
    float max_wing_rate_rad_s[ALGO_WING_COUNT];

    bool angle_limit_enable;    /* 是否启用翼角限幅 */
    bool rate_limit_enable;     /* 是否启用翼角速度限幅 */

    /*
     * 机械限位是否已根据真实机构确认。
     * false 时安全自检不得允许正常飞行解锁。
     */
    bool mechanical_limits_valid;
} algo_waveform_config_t;

/* ============================================================
 * 单侧四连杆配置
 *
 * input  ：舵机曲柄
 * coupler：连杆
 * output ：机翼摇臂
 * ground ：两固定转轴之间的距离
 * ============================================================ */

typedef struct {
    float ground_length_mm;     /* 机架杆长度，mm */
    float input_length_mm;      /* 舵机曲柄长度，mm */
    float coupler_length_mm;    /* 连杆长度，mm */
    float output_length_mm;     /* 机翼摇臂长度，mm */

    /* 固定转轴连线相对参考坐标的方向。 */
    float ground_angle_rad;

    /* 四连杆装配分支，通常取 +1 或 -1。 */
    float assembly_sign;

    /* 舵机安装方向，通常取 +1 或 -1。 */
    float servo_direction;

    /* 舵机逻辑中位和限位。 */
    float servo_neutral_deg;
    float servo_min_deg;
    float servo_max_deg;

    /* 机翼零位对应的安装偏置。 */
    float wing_zero_rad;
} algo_linkage_side_config_t;

typedef struct {
    /*
     * 几何参数未测量前保持 false。
     * false 时只能用于接口和开环测试，不能宣称逆解有效。
     */
    bool enabled;

    /*
     * 直驱模式：机翼直接装在舵机摆臂上（康师傅 V1 机构），
     * 舵机角 = servo_neutral_deg + servo_direction × 翼角(deg)。
     *
     * 仅在 enabled == false 时生效；两者都为 false 时逆解输出不可达，
     * 安全状态机不允许正常扑翼。
     */
    bool direct_drive;

    /* 左右翼各自的四连杆参数。 */
    algo_linkage_side_config_t side[ALGO_WING_COUNT];
} algo_linkage_config_t;

/* ============================================================
 * 安全状态机配置
 * ============================================================ */

typedef struct {
    uint32_t selftest_timeout_ms;   /* 自检最长允许时间，超时视为失败 */

    /* 短时遥控异常判定时间。 */
    uint32_t rc_degraded_timeout_ms;

    /* 持续失联进入 FAILSAFE 的时间。 */
    uint32_t rc_failsafe_timeout_ms;

    /* 遥控恢复后，重新建立有效状态所需时间。 */
    uint32_t rc_recover_time_ms;

    /* 姿态异常阈值。 */
    float max_attitude_angle_rad;   /* 超过该姿态角判定异常 */
    float max_body_rate_rad_s;      /* 超过该角速度判定异常 */

    /* 降级模式输出限制。 */
    float degraded_amplitude_limit;
    float degraded_differential_limit;

    /* 失联迫降初始限制，后续必须根据下降率实测修改。 */
    float failsafe_amplitude_limit;
    float failsafe_differential_limit;

    /* 普通失控降幅时间，不适用于严重硬件故障。 */
    float failsafe_ramp_time_s;

    /* 机械参数未确认时是否禁止飞行解锁。 */
    bool require_valid_mechanical_limits;
} algo_safety_config_t;

/* ============================================================
 * 顶层配置
 *
 * 汇总所有子模块配置，运行时整体验证后使用。
 * ============================================================ */

typedef struct {
    uint32_t version;                       /* 对应 ALGO_CONFIG_VERSION */

    algo_scheduler_config_t scheduler;      /* 调度 */
    algo_phase_config_t phase;              /* 相位发生器 */
    algo_filter_config_t filter;            /* IMU 滤波 */
    algo_attitude_config_t attitude;        /* 姿态解算 */
    algo_control_config_t control;          /* 串级 PID 控制 */
    algo_mixer_config_t mixer;              /* 控制分配 */
    algo_waveform_config_t waveform;        /* 波形与机械限幅 */
    algo_linkage_config_t linkage;          /* 连杆逆解 */
    algo_safety_config_t safety;            /* 安全状态机 */
} algo_config_t;

/* ============================================================
 * 默认配置
 *
 * 使用方式：
 *   algo_config_t config = ALGO_CONFIG_DEFAULT;
 *
 * PID 增益默认全部为零。
 * 完成台架辨识和单轴整定后，再逐项启用。
 * ============================================================ */

/* 快速周期 2.5 ms（400 Hz），控制 200 Hz，健康 100 Hz。 */
#define ALGO_SCHEDULER_CONFIG_DEFAULT \
    { \
        .fast_dt_s = 0.0025f, \
        .control_divider = 2u, \
        .health_divider = 4u, \
        .max_dt_s = 0.010f \
    }

/* 默认扑翼 7.5 Hz，范围 3~10 Hz，频率每秒最多变化 1 Hz。 */
#define ALGO_PHASE_CONFIG_DEFAULT \
    { \
        .default_frequency_hz = 7.5f, \
        .min_frequency_hz = 3.0f, \
        .max_frequency_hz = 10.0f, \
        .slew_hz_per_s = 1.0f \
    }

/* 低通高于扑翼基频（7.5 Hz）以保留控制带宽；重力容忍 3.43 ≈ 0.35g。 */
#define ALGO_FILTER_CONFIG_DEFAULT \
    { \
        .gyro_lpf_cutoff_hz = 20.0f, \
        .accel_lpf_cutoff_hz = 10.0f, \
        .flap_notch_enable = true, \
        .flap_notch_q = 4.0f, \
        .second_harmonic_notch_enable = false, \
        .second_harmonic_notch_q = 4.0f, \
        .notch_retune_threshold_hz = 0.15f, \
        .gravity_tolerance_m_s2 = 3.43f, \
        .min_accel_trust = 0.0f, \
        .max_accel_trust = 1.0f \
    }

/* Mahony 增益为常见起步值，需结合实测陀螺噪声调整。 */
#define ALGO_ATTITUDE_CONFIG_DEFAULT \
    { \
        .kp = 1.5f, \
        .ki = 0.01f, \
        .integral_limit_rad_s = 0.5f, \
        .min_correction_trust = 0.05f, \
        .impact_trust_threshold = 0.35f, \
        .impact_kp_scale = 0.05f, \
        .convergence_time_s = 2.0f, \
        .normalization_interval = 10u \
    }

/* 全零 PID：整定完成前控制器不产生有效输出。 */
#define ALGO_PID_CONFIG_ZERO \
    { \
        .kp = 0.0f, \
        .ki = 0.0f, \
        .kd = 0.0f, \
        .integral_limit = 0.0f, \
        .derivative_lpf_cutoff_hz = 15.0f \
    }

/* 姿态目标限 30°，角速度限 120°/s（yaw 60°/s）；偏航闭环默认关闭。 */
#define ALGO_CONTROL_CONFIG_DEFAULT \
    { \
        .angle_pid = { \
            ALGO_PID_CONFIG_ZERO, \
            ALGO_PID_CONFIG_ZERO \
        }, \
        .rate_pid = { \
            ALGO_PID_CONFIG_ZERO, \
            ALGO_PID_CONFIG_ZERO, \
            ALGO_PID_CONFIG_ZERO \
        }, \
        .max_roll_angle_rad = ALGO_DEG_TO_RAD(30.0f), \
        .max_pitch_angle_rad = ALGO_DEG_TO_RAD(30.0f), \
        .max_rate_rad_s = { \
            ALGO_DEG_TO_RAD(120.0f), \
            ALGO_DEG_TO_RAD(120.0f), \
            ALGO_DEG_TO_RAD(60.0f) \
        }, \
        .max_roll_output = 1.0f, \
        .max_pitch_output = 1.0f, \
        .max_yaw_output = 0.0f, \
        .yaw_control_enable = false, \
        .anti_windup_enable = true \
    }

/* 满升力对应 ±15° 基础幅度，滚转差动和俯仰不对称均为保守起步值。 */
#define ALGO_MIXER_CONFIG_DEFAULT \
    { \
        .lift_amplitude_gain_rad = ALGO_DEG_TO_RAD(15.0f), \
        .max_amplitude_rad = { \
            ALGO_DEG_TO_RAD(15.0f), \
            ALGO_DEG_TO_RAD(15.0f) \
        }, \
        .roll_differential_gain_rad = ALGO_DEG_TO_RAD(5.0f), \
        .max_differential_rad = ALGO_DEG_TO_RAD(5.0f), \
        .pitch_up_down_gain = 0.20f, \
        .max_up_down_bias = 0.30f, \
        .max_center_offset_rad = ALGO_DEG_TO_RAD(5.0f), \
        .center_rad = { 0.0f, 0.0f }, \
        .default_shape_k = 0.25f, \
        .yaw_mixer_enable = false \
    }

/* 翼角硬限位 ±45°，角速度限 630°/s；限位未经实测确认。 */
#define ALGO_WAVEFORM_CONFIG_DEFAULT \
    { \
        .min_wing_angle_rad = { \
            ALGO_DEG_TO_RAD(-45.0f), \
            ALGO_DEG_TO_RAD(-45.0f) \
        }, \
        .max_wing_angle_rad = { \
            ALGO_DEG_TO_RAD(45.0f), \
            ALGO_DEG_TO_RAD(45.0f) \
        }, \
        .max_wing_rate_rad_s = { \
            ALGO_DEG_TO_RAD(630.0f), \
            ALGO_DEG_TO_RAD(630.0f) \
        }, \
        .angle_limit_enable = true, \
        .rate_limit_enable = true, \
        .mechanical_limits_valid = false \
    }

/* 几何参数全部置零占位：测量完成前逆解不可用。 */
#define ALGO_LINKAGE_SIDE_CONFIG_DEFAULT \
    { \
        .ground_length_mm = 0.0f, \
        .input_length_mm = 0.0f, \
        .coupler_length_mm = 0.0f, \
        .output_length_mm = 0.0f, \
        .ground_angle_rad = 0.0f, \
        .assembly_sign = 1.0f, \
        .servo_direction = 1.0f, \
        .servo_neutral_deg = 90.0f, \
        .servo_min_deg = 0.0f, \
        .servo_max_deg = 180.0f, \
        .wing_zero_rad = 0.0f \
    }

/* 逆解默认关闭。 */
#define ALGO_LINKAGE_CONFIG_DEFAULT \
    { \
        .enabled = false, \
        .direct_drive = false, \
        .side = { \
            ALGO_LINKAGE_SIDE_CONFIG_DEFAULT, \
            ALGO_LINKAGE_SIDE_CONFIG_DEFAULT \
        } \
    }

/*
 * 遥控异常 300 ms 降级、1 s 失联；失联迫降幅度限 35%、禁止差动，
 * 用 2 s 斜坡缓慢收小输出，避免姿态突变。
 */
#define ALGO_SAFETY_CONFIG_DEFAULT \
    { \
        .selftest_timeout_ms = 3000u, \
        .rc_degraded_timeout_ms = 300u, \
        .rc_failsafe_timeout_ms = 1000u, \
        .rc_recover_time_ms = 500u, \
        .max_attitude_angle_rad = ALGO_DEG_TO_RAD(45.0f), \
        .max_body_rate_rad_s = ALGO_DEG_TO_RAD(300.0f), \
        .degraded_amplitude_limit = 0.50f, \
        .degraded_differential_limit = 0.20f, \
        .failsafe_amplitude_limit = 0.35f, \
        .failsafe_differential_limit = 0.0f, \
        .failsafe_ramp_time_s = 2.0f, \
        .require_valid_mechanical_limits = true \
    }

/* 汇总默认配置。 */
#define ALGO_CONFIG_DEFAULT \
    { \
        .version = ALGO_CONFIG_VERSION, \
        .scheduler = ALGO_SCHEDULER_CONFIG_DEFAULT, \
        .phase = ALGO_PHASE_CONFIG_DEFAULT, \
        .filter = ALGO_FILTER_CONFIG_DEFAULT, \
        .attitude = ALGO_ATTITUDE_CONFIG_DEFAULT, \
        .control = ALGO_CONTROL_CONFIG_DEFAULT, \
        .mixer = ALGO_MIXER_CONFIG_DEFAULT, \
        .waveform = ALGO_WAVEFORM_CONFIG_DEFAULT, \
        .linkage = ALGO_LINKAGE_CONFIG_DEFAULT, \
        .safety = ALGO_SAFETY_CONFIG_DEFAULT \
    }

#endif /* ALGO_CONFIG_H */
