/*
 * algo_control.c
 *
 * 串级姿态控制模块实现：
 *
 *   RC 指令 → 目标姿态角 → 姿态角外环 PID → 目标角速度
 *           → 角速度内环 PID → 归一化控制输出 → algo_mixer
 *
 * 模块职责、坐标系约定和接口语义详见 algo_control.h。
 */

#include "algo_control.h"

#include <math.h>    /* isfinite */
#include <string.h>  /* memset */

/* 2*pi 的单精度浮点近似，用于微分低通滤波器截止频率的换算。 */
#define CONTROL_TWO_PI_F  6.28318530717958647692f

/* ============================================================
 * 通用辅助函数
 * ============================================================ */

/*
 * 将 value 限制在 [min_value, max_value] 区间内。
 * 调用方保证 min_value <= max_value。
 */
static float control_clampf(
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
 * 将输出结构清零并设置有效标志。
 * 所有出错或待机路径都通过它产出确定的“零输出”，
 * 避免调用方读到未初始化数据。
 */
static void control_set_zero_output(
    control_out_t *output,
    bool valid
)
{
    memset(output, 0, sizeof(*output));
    output->valid = valid;
}

/* 清零单个 PID 的积分、微分滤波和测量历史。 */
static void control_reset_pid_state(
    control_pid_state_t *state
)
{
    memset(state, 0, sizeof(*state));
}

/*
 * 清零控制器内全部 PID 的历史状态：
 *   - 滚转、俯仰两个姿态角外环；
 *   - X、Y、Z 三个角速度内环。
 * 配置不受影响。
 */
static void control_clear_pid_history(
    control_state_t *state
)
{
    uint32_t axis;

    for (axis = 0u; axis < CONTROL_ANGLE_AXIS_COUNT; ++axis) {
        control_reset_pid_state(&state->angle_pid[axis]);
    }

    for (axis = 0u; axis < (uint32_t)ALGO_AXIS_COUNT; ++axis) {
        control_reset_pid_state(&state->rate_pid[axis]);
    }
}

/* ============================================================
 * 配置检查
 * ============================================================ */

/*
 * 检查单个 PID 配置是否合法：
 *   - 所有字段必须是有限值（排除 NaN 和无穷大）；
 *   - kp、ki、kd、integral_limit 不允许为负；
 *   - 微分低通截止频率必须为正（全零增益仍属合法）。
 */
static bool control_pid_config_is_valid(
    const algo_pid_config_t *config
)
{
    /* 排除 NaN 和无穷大。 */
    if (!isfinite(config->kp) ||
        !isfinite(config->ki) ||
        !isfinite(config->kd) ||
        !isfinite(config->integral_limit) ||
        !isfinite(config->derivative_lpf_cutoff_hz)) {
        return false;
    }

    /* 增益和积分限幅不允许为负。 */
    if ((config->kp < 0.0f) ||
        (config->ki < 0.0f) ||
        (config->kd < 0.0f) ||
        (config->integral_limit < 0.0f)) {
        return false;
    }

    /* 截止频率必须为正，否则低通系数计算会出现除零。 */
    if (config->derivative_lpf_cutoff_hz <= 0.0f) {
        return false;
    }

    return true;
}

/*
 * 检查整套控制配置是否合法：
 *   - 全部外环、内环 PID 配置合法；
 *   - 三轴最大目标角速度为有限正值；
 *   - 最大滚转、俯仰目标姿态角为有限正值；
 *   - 三路输出限幅位于 [0, 1]（归一化控制需求）。
 */
static bool control_config_is_valid(
    const algo_control_config_t *config
)
{
    uint32_t axis;

    if (config == NULL) {
        return false;
    }

    /* 姿态角外环：roll、pitch 两轴。 */
    for (axis = 0u; axis < CONTROL_ANGLE_AXIS_COUNT; ++axis) {
        if (!control_pid_config_is_valid(&config->angle_pid[axis])) {
            return false;
        }
    }

    /* 角速度内环：X、Y、Z 三轴，附带最大目标角速度检查。 */
    for (axis = 0u; axis < (uint32_t)ALGO_AXIS_COUNT; ++axis) {
        if (!control_pid_config_is_valid(&config->rate_pid[axis])) {
            return false;
        }

        if (!isfinite(config->max_rate_rad_s[axis]) ||
            (config->max_rate_rad_s[axis] <= 0.0f)) {
            return false;
        }
    }

    /* 最大目标姿态角：必须有限且为正。 */
    if (!isfinite(config->max_roll_angle_rad) ||
        !isfinite(config->max_pitch_angle_rad)) {
        return false;
    }

    if ((config->max_roll_angle_rad <= 0.0f) ||
        (config->max_pitch_angle_rad <= 0.0f)) {
        return false;
    }

    /* 输出限幅：归一化控制需求，必须位于 [0, 1]。 */
    if (!isfinite(config->max_roll_output) ||
        !isfinite(config->max_pitch_output) ||
        !isfinite(config->max_yaw_output)) {
        return false;
    }

    if ((config->max_roll_output < 0.0f) ||
        (config->max_roll_output > 1.0f) ||
        (config->max_pitch_output < 0.0f) ||
        (config->max_pitch_output > 1.0f) ||
        (config->max_yaw_output < 0.0f) ||
        (config->max_yaw_output > 1.0f)) {
        return false;
    }

    return true;
}

/* ============================================================
 * 输入和输出数值检查
 * ============================================================ */

/* 检查遥控指令四个通道是否都是有限值。 */
static bool control_rc_is_finite(
    const algo_rc_command_t *rc
)
{
    return isfinite(rc->throttle) &&
           isfinite(rc->roll) &&
           isfinite(rc->pitch) &&
           isfinite(rc->yaw);
}

/* 检查闭环所需反馈量（姿态角和三轴陀螺角速度）是否都是有限值。 */
static bool control_feedback_is_finite(
    const attitude_out_t *attitude,
    const filter_out_t *filtered_imu
)
{
    return isfinite(attitude->roll_rad) &&
           isfinite(attitude->pitch_rad) &&
           isfinite(filtered_imu->gyro_rad_s.x) &&
           isfinite(filtered_imu->gyro_rad_s.y) &&
           isfinite(filtered_imu->gyro_rad_s.z);
}

/* 检查四路控制输出是否都是有限值，用于返回前的最后防线。 */
static bool control_output_is_finite(
    const control_out_t *output
)
{
    return isfinite(output->lift) &&
           isfinite(output->roll) &&
           isfinite(output->pitch) &&
           isfinite(output->yaw);
}

/* ============================================================
 * 通用 PID 更新
 *
 * state->integral 保存已经乘以 ki 的积分贡献，
 * integral_limit 因而直接限制积分对输出的影响。
 *
 * 微分项：
 *   derivative = -(measurement - previous_measurement) / dt
 *
 * 即对测量值微分并带负号，避免目标突变造成微分冲击。
 * ============================================================ */

/*
 * 执行一次 PID 计算并返回限幅后的输出。
 *
 * 参数：
 *   state：             该 PID 的历史状态（积分、微分、上次测量值）；
 *   config：            该 PID 的增益和限幅配置；
 *   error：             目标值 - 测量值；
 *   measurement：       当前测量值，用于测量值微分；
 *   output_min/max：    输出限幅区间；
 *   anti_windup_enable：输出饱和时是否启用条件停止积分；
 *   dt_s：              距离上次更新的时间间隔（秒）；
 *   numeric_ok：        输出参数，运算出现非有限值时置 false。
 *
 * 结构特点：
 *   - 微分对测量值计算（微分先行），目标突变不会冲击输出；
 *   - 微分经一阶低通滤波，抑制陀螺噪声放大；
 *   - 积分保存的是 ki*∫e·dt，integral_limit 直接限制其输出贡献；
 *   - 可选条件抗积分饱和：已饱和且误差继续推向饱和时暂停积分。
 */
static float control_pid_update(
    control_pid_state_t *state,
    const algo_pid_config_t *config,
    float error,
    float measurement,
    float output_min,
    float output_max,
    bool anti_windup_enable,
    float dt_s,
    bool *numeric_ok
)
{
    float derivative_raw = 0.0f;
    float derivative_alpha;
    float derivative_term;
    float proportional_term;
    float output_before_integral;
    float output_unsaturated;
    bool allow_integration = true;

    /*
     * 对测量值微分并取负号：
     *   d(error)/dt = d(target)/dt - d(measurement)/dt
     * 忽略目标的变化率，只保留 -d(measurement)/dt，
     * 避免遥控目标阶跃时微分项产生冲击（微分冲击）。
     * 第一个有效样本（primed=false）没有历史可比较，微分保持为零。
     */
    if (state->primed) {
        derivative_raw =
            -(measurement - state->previous_measurement) / dt_s;
    }

    state->previous_measurement = measurement;
    state->primed = true;

    /*
     * 一阶低通：
     * alpha = dt / (dt + RC)
     * RC = 1 / (2*pi*fc)
     */
    derivative_alpha =
        dt_s /
        (dt_s +
         1.0f /
         (CONTROL_TWO_PI_F * config->derivative_lpf_cutoff_hz));

    /* 低通滤波后的微分项：y += alpha * (x - y)。 */
    state->derivative_filtered +=
        derivative_alpha *
        (derivative_raw - state->derivative_filtered);

    proportional_term = config->kp * error;
    derivative_term = config->kd * state->derivative_filtered;

    /*
     * 未加入本周期积分增量前的输出，
     * 用于抗积分饱和判断。
     */
    output_before_integral =
        proportional_term +
        state->integral +
        derivative_term;

    /*
     * 输出已经饱和，并且误差会继续推动积分向饱和方向增长时，
     * 本周期停止积分。
     */
    if (anti_windup_enable) {
        /* 正向饱和且误差仍为正：积分继续增长只会加深饱和。 */
        if ((output_before_integral > output_max) &&
            (error > 0.0f)) {
            allow_integration = false;
        }

        /* 负向饱和且误差仍为负：同理。 */
        if ((output_before_integral < output_min) &&
            (error < 0.0f)) {
            allow_integration = false;
        }
    }

    if ((config->ki == 0.0f) ||
        (config->integral_limit == 0.0f)) {
        /* 积分被配置关闭：强制清零，防止残留历史影响输出。 */
        state->integral = 0.0f;
    } else if (allow_integration) {
        /* 积分项直接保存 ki*∫e·dt，integral_limit 限制其输出贡献。 */
        state->integral += config->ki * error * dt_s;

        state->integral = control_clampf(
            state->integral,
            -config->integral_limit,
            config->integral_limit
        );
    }
    /* 抗饱和触发时 allow_integration=false，本周期积分保持不变。 */

    output_unsaturated =
        proportional_term +
        state->integral +
        derivative_term;

    /* 数值健康检查：任一中间量出现 NaN 或无穷大即上报错误。 */
    if (!isfinite(state->integral) ||
        !isfinite(state->derivative_filtered) ||
        !isfinite(output_unsaturated)) {
        *numeric_ok = false;
        return 0.0f;
    }

    return control_clampf(
        output_unsaturated,
        output_min,
        output_max
    );
}

/* ============================================================
 * 初始化
 * ============================================================ */

/*
 * 初始化控制器：
 *   - 校验配置合法性；
 *   - 将配置完整复制进状态，之后不依赖外部配置内存；
 *   - 清零全部 PID 历史和当前输出；
 *   - 置 initialized 标志。
 */
control_status_t control_init(
    control_state_t *state,
    const algo_control_config_t *config
)
{
    algo_control_config_t config_copy;

    if (state == NULL) {
        return CONTROL_STATUS_INVALID_ARG;
    }

    /* 先清除标志，确保后续任何失败路径都停留在未初始化状态。 */
    state->initialized = false;

    if (config == NULL) {
        return CONTROL_STATUS_INVALID_ARG;
    }

    if (!control_config_is_valid(config)) {
        return CONTROL_STATUS_INVALID_CONFIG;
    }

    /* 先复制再清状态，允许 config 指向 state->config 自身。 */
    config_copy = *config;

    memset(state, 0, sizeof(*state));

    state->config = config_copy;
    state->initialized = true;

    /* 初始输出为零，valid=false，等待第一个有效控制周期。 */
    control_set_zero_output(&state->current, false);

    return CONTROL_STATUS_OK;
}

/* ============================================================
 * 复位
 * ============================================================ */

/*
 * 复位控制器：保留配置，清除全部 PID 历史和最近输出。
 * 安全状态机也可通过 safety->reset_controller 触发同样的复位。
 */
control_status_t control_reset(
    control_state_t *state
)
{
    if (state == NULL) {
        return CONTROL_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return CONTROL_STATUS_NOT_INITIALIZED;
    }

    control_clear_pid_history(state);
    control_set_zero_output(&state->current, false);

    return CONTROL_STATUS_OK;
}

/* ============================================================
 * 周期控制更新
 * ============================================================ */

/*
 * 执行一个控制周期，完整填充 output。
 *
 * 主流程：
 *   1. 参数和数值检查（任何失败都保证 output 为确定的零输出）；
 *   2. 处理安全复位请求和遥控有效性；
 *   3. 手动模式：RC 直通，不运行 PID；
 *   4. 闭环模式：姿态角外环 → 角速度内环，输出归一化控制需求。
 *
 * 所有异常路径都会清除 PID 历史并输出零控制量，
 * 防止故障恢复后沿用旧积分造成输出突变。
 */
control_status_t control_update(
    control_state_t *state,
    const algo_rc_command_t *rc,
    const attitude_out_t *attitude,
    const filter_out_t *filtered_imu,
    const safety_directive_t *safety,
    float dt_s,
    control_out_t *output
)
{
    float rc_roll;
    float rc_pitch;
    float rc_yaw;
    float roll_angle_target;
    float pitch_angle_target;
    float roll_angle_error;
    float pitch_angle_error;
    float roll_rate_target;
    float pitch_rate_target;
    float yaw_rate_target = 0.0f;
    float roll_rate_error;
    float pitch_rate_error;
    float yaw_rate_error;
    bool numeric_ok = true;

    if (output == NULL) {
        return CONTROL_STATUS_INVALID_ARG;
    }

    /*
     * 先把输出清零：后续任何提前返回路径，
     * 调用方拿到的都是确定的零控制量。
     */
    control_set_zero_output(output, false);

    if (state == NULL) {
        return CONTROL_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return CONTROL_STATUS_NOT_INITIALIZED;
    }

    if ((rc == NULL) ||
        (attitude == NULL) ||
        (filtered_imu == NULL) ||
        (safety == NULL)) {
        state->current = *output;
        return CONTROL_STATUS_INVALID_ARG;
    }

    if (!isfinite(dt_s) || (dt_s <= 0.0f)) {
        state->current = *output;
        return CONTROL_STATUS_INVALID_DT;
    }

    /* 安全状态机请求复位：清 PID 历史，本周期仍继续正常计算。 */
    if (safety->reset_controller) {
        control_clear_pid_history(state);
    }

    /*
     * 遥控无效属于运行状态，不属于接口错误。
     * 清除 PID 历史，防止恢复后使用旧积分。
     */
    if (!rc->valid) {
        control_clear_pid_history(state);
        state->current = *output;
        return CONTROL_STATUS_OK;
    }

    /* 遥控数据有效但包含 NaN/无穷大：上报非法输入。 */
    if (!control_rc_is_finite(rc)) {
        control_clear_pid_history(state);
        state->current = *output;
        return CONTROL_STATUS_INVALID_INPUT;
    }

    /* 三轴杆量限幅到 [-1, 1]，容忍上游轻微的归一化越界。 */
    rc_roll = control_clampf(rc->roll, -1.0f, 1.0f);
    rc_pitch = control_clampf(rc->pitch, -1.0f, 1.0f);
    rc_yaw = control_clampf(rc->yaw, -1.0f, 1.0f);

    /* ========================================================
     * 手动模式：只使用 RC，不读取姿态反馈
     * ======================================================== */

    if (!safety->allow_stabilization) {
        /* 不运行 PID，同时清历史，防止恢复闭环时沿用旧积分。 */
        control_clear_pid_history(state);

        /* 油门直接映射为升力，限制在 [0, 1]。 */
        output->lift = control_clampf(
            rc->throttle,
            0.0f,
            1.0f
        );

        /* 杆量按各自输出限幅直通。 */
        output->roll = control_clampf(
            rc_roll,
            -state->config.max_roll_output,
            state->config.max_roll_output
        );

        output->pitch = control_clampf(
            rc_pitch,
            -state->config.max_pitch_output,
            state->config.max_pitch_output
        );

        if (state->config.yaw_control_enable) {
            output->yaw = control_clampf(
                rc_yaw,
                -state->config.max_yaw_output,
                state->config.max_yaw_output
            );
        } else {
            output->yaw = 0.0f;
        }

        if (!control_output_is_finite(output)) {
            control_set_zero_output(output, false);
            state->current = *output;
            return CONTROL_STATUS_NUMERIC_ERROR;
        }

        output->valid = true;
        state->current = *output;

        return CONTROL_STATUS_OK;
    }

    /* ========================================================
     * 闭环模式：检查姿态和陀螺反馈
     * ======================================================== */

    /*
     * 反馈未就绪（姿态未收敛或陀螺无效）属于运行状态：
     * 输出零控制量并清 PID 历史，返回 OK。
     */
    if (!attitude->valid ||
        !attitude->converged ||
        !filtered_imu->valid ||
        !filtered_imu->gyro_valid) {
        control_clear_pid_history(state);
        state->current = *output;
        return CONTROL_STATUS_OK;
    }

    /* 反馈有效标志齐备但数值含 NaN/无穷大：上报非法输入。 */
    if (!control_feedback_is_finite(attitude, filtered_imu)) {
        control_clear_pid_history(state);
        state->current = *output;
        return CONTROL_STATUS_INVALID_INPUT;
    }

    /* ========================================================
     * 姿态角外环
     *
     * RC 杆量 × 最大目标姿态角 = 目标姿态角；
     * 外环 PID 输出目标机体系角速度，
     * 限幅为该轴最大目标角速度。
     * ======================================================== */

    roll_angle_target =
        rc_roll * state->config.max_roll_angle_rad;

    pitch_angle_target =
        rc_pitch * state->config.max_pitch_angle_rad;

    roll_angle_error =
        roll_angle_target - attitude->roll_rad;

    pitch_angle_error =
        pitch_angle_target - attitude->pitch_rad;

    /* 滚转姿态角外环：输出 X 轴目标角速度（rad/s）。 */
    roll_rate_target = control_pid_update(
        &state->angle_pid[0],
        &state->config.angle_pid[0],
        roll_angle_error,
        attitude->roll_rad,
        -state->config.max_rate_rad_s[ALGO_AXIS_X],
        state->config.max_rate_rad_s[ALGO_AXIS_X],
        state->config.anti_windup_enable,
        dt_s,
        &numeric_ok
    );

    /* 俯仰姿态角外环：输出 Y 轴目标角速度（rad/s）。 */
    pitch_rate_target = control_pid_update(
        &state->angle_pid[1],
        &state->config.angle_pid[1],
        pitch_angle_error,
        attitude->pitch_rad,
        -state->config.max_rate_rad_s[ALGO_AXIS_Y],
        state->config.max_rate_rad_s[ALGO_AXIS_Y],
        state->config.anti_windup_enable,
        dt_s,
        &numeric_ok
    );

    /* ========================================================
     * 偏航只使用角速度目标，不使用绝对偏航角
     * ======================================================== */

    if (state->config.yaw_control_enable) {
        /* RC 偏航杆量直接映射为 Z 轴目标角速度。 */
        yaw_rate_target =
            rc_yaw *
            state->config.max_rate_rad_s[ALGO_AXIS_Z];
    } else {
        /* 偏航通道关闭：清掉对应内环历史，防止重新使能时残留积分。 */
        control_reset_pid_state(
            &state->rate_pid[ALGO_AXIS_Z]
        );
    }

    /* ========================================================
     * 角速度内环
     *
     * 目标角速度 - 滤波后陀螺角速度 = 角速度误差；
     * 内环 PID 输出归一化控制需求，
     * 限幅为对应通道的最大输出。
     * ======================================================== */

    roll_rate_error =
        roll_rate_target -
        filtered_imu->gyro_rad_s.x;

    pitch_rate_error =
        pitch_rate_target -
        filtered_imu->gyro_rad_s.y;

    /* 滚转角速度内环（X 轴陀螺）。 */
    output->roll = control_pid_update(
        &state->rate_pid[ALGO_AXIS_X],
        &state->config.rate_pid[ALGO_AXIS_X],
        roll_rate_error,
        filtered_imu->gyro_rad_s.x,
        -state->config.max_roll_output,
        state->config.max_roll_output,
        state->config.anti_windup_enable,
        dt_s,
        &numeric_ok
    );

    /* 俯仰角速度内环（Y 轴陀螺）。 */
    output->pitch = control_pid_update(
        &state->rate_pid[ALGO_AXIS_Y],
        &state->config.rate_pid[ALGO_AXIS_Y],
        pitch_rate_error,
        filtered_imu->gyro_rad_s.y,
        -state->config.max_pitch_output,
        state->config.max_pitch_output,
        state->config.anti_windup_enable,
        dt_s,
        &numeric_ok
    );

    if (state->config.yaw_control_enable) {
        /* 偏航角速度内环（Z 轴陀螺）。 */
        yaw_rate_error =
            yaw_rate_target -
            filtered_imu->gyro_rad_s.z;

        output->yaw = control_pid_update(
            &state->rate_pid[ALGO_AXIS_Z],
            &state->config.rate_pid[ALGO_AXIS_Z],
            yaw_rate_error,
            filtered_imu->gyro_rad_s.z,
            -state->config.max_yaw_output,
            state->config.max_yaw_output,
            state->config.anti_windup_enable,
            dt_s,
            &numeric_ok
        );
    } else {
        output->yaw = 0.0f;
    }

    /* 升力不参与闭环：直接来自油门，限制在 [0, 1]。 */
    output->lift = control_clampf(
        rc->throttle,
        0.0f,
        1.0f
    );

    /*
     * 任一 PID 出现数值错误，或最终输出含非有限值：
     * 清 PID 历史并输出零控制量，上报数值错误。
     */
    if (!numeric_ok || !control_output_is_finite(output)) {
        control_clear_pid_history(state);
        control_set_zero_output(output, false);
        state->current = *output;
        return CONTROL_STATUS_NUMERIC_ERROR;
    }

    output->valid = true;
    state->current = *output;

    return CONTROL_STATUS_OK;
}

/* ============================================================
 * 获取最近一次控制输出
 * ============================================================ */

/*
 * 返回最近一次控制周期缓存的输出。
 * 不执行新的控制计算，也不改变 PID 历史。
 */
control_status_t control_get(
    const control_state_t *state,
    control_out_t *output
)
{
    if (output == NULL) {
        return CONTROL_STATUS_INVALID_ARG;
    }

    /* 失败路径同样保证调用方拿到确定的零输出。 */
    control_set_zero_output(output, false);

    if (state == NULL) {
        return CONTROL_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return CONTROL_STATUS_NOT_INITIALIZED;
    }

    *output = state->current;

    return CONTROL_STATUS_OK;
}
