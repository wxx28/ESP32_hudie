#include "algo_attitude.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ATTITUDE_PI_F               3.14159265358979323846f
#define ATTITUDE_TWO_PI_F           6.28318530717958647692f
#define ATTITUDE_MIN_NORM           1.0e-6f
#define ATTITUDE_MIN_ACCEL_NORM     1.0e-3f

/* ============================================================
 * 通用辅助函数
 * ============================================================ */

static float attitude_clamp_float(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

static bool attitude_vec3_is_finite(
    const algo_vec3_t *vector
)
{
    return isfinite(vector->x) &&
           isfinite(vector->y) &&
           isfinite(vector->z);
}

static bool attitude_quaternion_is_finite(
    const algo_quat_t *quaternion
)
{
    return isfinite(quaternion->w) &&
           isfinite(quaternion->x) &&
           isfinite(quaternion->y) &&
           isfinite(quaternion->z);
}

static float attitude_vec3_magnitude(
    const algo_vec3_t *vector
)
{
    return sqrtf(
        vector->x * vector->x +
        vector->y * vector->y +
        vector->z * vector->z
    );
}

static algo_vec3_t attitude_vec3_cross(
    const algo_vec3_t *left,
    const algo_vec3_t *right
)
{
    algo_vec3_t result;

    result.x =
        left->y * right->z -
        left->z * right->y;

    result.y =
        left->z * right->x -
        left->x * right->z;

    result.z =
        left->x * right->y -
        left->y * right->x;

    return result;
}

static void attitude_clear_output(
    attitude_out_t *output
)
{
    if (output == NULL) {
        return;
    }

    memset(output, 0, sizeof(*output));

    output->q_bn.w = 1.0f;
    output->q_bn.x = 0.0f;
    output->q_bn.y = 0.0f;
    output->q_bn.z = 0.0f;

    output->valid = false;
    output->converged = false;
}

/* ============================================================
 * 配置检查
 * ============================================================ */

static bool attitude_config_is_valid(
    const algo_attitude_config_t *config
)
{
    if (config == NULL) {
        return false;
    }

    if (!isfinite(config->kp) ||
        !isfinite(config->ki) ||
        config->kp < 0.0f ||
        config->ki < 0.0f) {
        return false;
    }

    if (!isfinite(config->integral_limit_rad_s) ||
        config->integral_limit_rad_s < 0.0f) {
        return false;
    }

    if (!isfinite(config->min_correction_trust) ||
        config->min_correction_trust < 0.0f ||
        config->min_correction_trust > 1.0f) {
        return false;
    }

    if (!isfinite(config->impact_trust_threshold) ||
        config->impact_trust_threshold < 0.0f ||
        config->impact_trust_threshold > 1.0f) {
        return false;
    }

    if (!isfinite(config->impact_kp_scale) ||
        config->impact_kp_scale < 0.0f ||
        config->impact_kp_scale > 1.0f) {
        return false;
    }

    if (!isfinite(config->convergence_time_s) ||
        config->convergence_time_s <= 0.0f) {
        return false;
    }

    if (config->normalization_interval == 0u) {
        return false;
    }

    return true;
}

/* ============================================================
 * 四元数运算
 * ============================================================ */

static bool attitude_quaternion_normalize(
    algo_quat_t *quaternion
)
{
    float norm_squared;
    float inverse_norm;

    if (!attitude_quaternion_is_finite(quaternion)) {
        return false;
    }

    norm_squared =
        quaternion->w * quaternion->w +
        quaternion->x * quaternion->x +
        quaternion->y * quaternion->y +
        quaternion->z * quaternion->z;

    if (!isfinite(norm_squared) ||
        norm_squared < ATTITUDE_MIN_NORM) {
        return false;
    }

    inverse_norm = 1.0f / sqrtf(norm_squared);

    quaternion->w *= inverse_norm;
    quaternion->x *= inverse_norm;
    quaternion->y *= inverse_norm;
    quaternion->z *= inverse_norm;

    return true;
}

static void attitude_quaternion_from_euler(
    float roll_rad,
    float pitch_rad,
    float yaw_rad,
    algo_quat_t *quaternion
)
{
    float half_roll;
    float half_pitch;
    float half_yaw;

    float cr;
    float sr;
    float cp;
    float sp;
    float cy;
    float sy;

    half_roll = 0.5f * roll_rad;
    half_pitch = 0.5f * pitch_rad;
    half_yaw = 0.5f * yaw_rad;

    cr = cosf(half_roll);
    sr = sinf(half_roll);

    cp = cosf(half_pitch);
    sp = sinf(half_pitch);

    cy = cosf(half_yaw);
    sy = sinf(half_yaw);

    /*
     * q_bn = q_yaw × q_pitch × q_roll
     *
     * 旋转顺序：Z-Y-X
     * 机体系到 NED 导航系。
     */
    quaternion->w =
        cy * cp * cr +
        sy * sp * sr;

    quaternion->x =
        cy * cp * sr -
        sy * sp * cr;

    quaternion->y =
        cy * sp * cr +
        sy * cp * sr;

    quaternion->z =
        sy * cp * cr -
        cy * sp * sr;
}

static void attitude_update_euler_angles(
    attitude_state_t *state
)
{
    const algo_quat_t *q = &state->current.q_bn;

    float sin_pitch;
    float r00;
    float r10;
    float r20;
    float r21;
    float r22;

    /*
     * q_bn 对应的机体系到导航系旋转矩阵元素。
     */
    r00 =
        1.0f - 2.0f *
        (q->y * q->y + q->z * q->z);

    r10 =
        2.0f *
        (q->x * q->y + q->w * q->z);

    r20 =
        2.0f *
        (q->x * q->z - q->w * q->y);

    r21 =
        2.0f *
        (q->y * q->z + q->w * q->x);

    r22 =
        1.0f - 2.0f *
        (q->x * q->x + q->y * q->y);

    state->current.roll_rad =
        atan2f(r21, r22);

    sin_pitch = attitude_clamp_float(
        -r20,
        -1.0f,
        1.0f
    );

    state->current.pitch_rad =
        asinf(sin_pitch);

    state->current.yaw_rad =
        atan2f(r10, r00);
}

static bool attitude_integrate_gyro(
    attitude_state_t *state,
    const algo_vec3_t *gyro_rad_s,
    float dt_s
)
{
    const algo_quat_t q = state->current.q_bn;

    algo_quat_t derivative;
    algo_quat_t updated;

    float half_dt = 0.5f * dt_s;

    /*
     * q_bn 对机体角速度的微分：
     *
     * q_dot = 0.5 × q_bn ⊗ [0, ω_body]
     */
    derivative.w =
        -q.x * gyro_rad_s->x
        -q.y * gyro_rad_s->y
        -q.z * gyro_rad_s->z;

    derivative.x =
         q.w * gyro_rad_s->x
        +q.y * gyro_rad_s->z
        -q.z * gyro_rad_s->y;

    derivative.y =
         q.w * gyro_rad_s->y
        +q.z * gyro_rad_s->x
        -q.x * gyro_rad_s->z;

    derivative.z =
         q.w * gyro_rad_s->z
        +q.x * gyro_rad_s->y
        -q.y * gyro_rad_s->x;

    updated.w =
        q.w + half_dt * derivative.w;

    updated.x =
        q.x + half_dt * derivative.x;

    updated.y =
        q.y + half_dt * derivative.y;

    updated.z =
        q.z + half_dt * derivative.z;

    if (!attitude_quaternion_normalize(&updated)) {
        return false;
    }

    state->current.q_bn = updated;

    state->normalization_counter++;

    if (state->normalization_counter >=
        state->config.normalization_interval) {
        state->normalization_counter = 0u;
    }

    return true;
}

/* ============================================================
 * 初始姿态建立
 * ============================================================ */

static bool attitude_initialize_from_gravity(
    attitude_state_t *state,
    const algo_vec3_t *accel_m_s2
)
{
    algo_vec3_t normalized_accel;

    float magnitude;
    float roll_rad;
    float pitch_rad;
    float sin_pitch;

    magnitude = attitude_vec3_magnitude(accel_m_s2);

    if (!isfinite(magnitude) ||
        magnitude < ATTITUDE_MIN_ACCEL_NORM) {
        return false;
    }

    normalized_accel.x =
        accel_m_s2->x / magnitude;

    normalized_accel.y =
        accel_m_s2->y / magnitude;

    normalized_accel.z =
        accel_m_s2->z / magnitude;

    /*
     * 加速度计测量的是比力。
     *
     * 在机体水平、机体系与 NED 重合时：
     *
     * accel = (0, 0, -g)
     *
     * 因此：
     * ax = sin(pitch)
     * ay = -sin(roll) × cos(pitch)
     * az = -cos(roll) × cos(pitch)
     */
    sin_pitch = attitude_clamp_float(
        normalized_accel.x,
        -1.0f,
        1.0f
    );

    pitch_rad = asinf(sin_pitch);

    roll_rad = atan2f(
        -normalized_accel.y,
        -normalized_accel.z
    );

    attitude_quaternion_from_euler(
        roll_rad,
        pitch_rad,
        0.0f,
        &state->current.q_bn
    );

    if (!attitude_quaternion_normalize(
            &state->current.q_bn)) {
        return false;
    }

    state->orientation_initialized = true;
    state->convergence_elapsed_s = 0.0f;

    state->current.valid = true;
    state->current.converged = false;

    attitude_update_euler_angles(state);

    return true;
}

/* ============================================================
 * Mahony 重力误差
 * ============================================================ */

static bool attitude_calculate_gravity_error(
    const attitude_state_t *state,
    const algo_vec3_t *accel_m_s2,
    algo_vec3_t *error
)
{
    const algo_quat_t *q = &state->current.q_bn;

    algo_vec3_t measured_specific_force;
    algo_vec3_t estimated_specific_force;

    float magnitude;

    float r20;
    float r21;
    float r22;

    magnitude = attitude_vec3_magnitude(accel_m_s2);

    if (!isfinite(magnitude) ||
        magnitude < ATTITUDE_MIN_ACCEL_NORM) {
        return false;
    }

    measured_specific_force.x =
        accel_m_s2->x / magnitude;

    measured_specific_force.y =
        accel_m_s2->y / magnitude;

    measured_specific_force.z =
        accel_m_s2->z / magnitude;

    /*
     * 导航系中的重力方向为 +Z，静止时比力方向为 -Z。
     *
     * 将导航系 (0, 0, -1) 旋转到机体系，得到估计比力。
     *
     * q_bn 旋转矩阵的第三行为：
     *   (r20, r21, r22)
     *
     * 因此估计比力取其负值。
     */
    r20 =
        2.0f *
        (q->x * q->z - q->w * q->y);

    r21 =
        2.0f *
        (q->y * q->z + q->w * q->x);

    r22 =
        1.0f - 2.0f *
        (q->x * q->x + q->y * q->y);

    estimated_specific_force.x = -r20;
    estimated_specific_force.y = -r21;
    estimated_specific_force.z = -r22;

    /*
     * 测量方向 × 估计方向。
     *
     * 该误差方向与 q_bn 的机体角速度修正方向一致。
     */
    *error = attitude_vec3_cross(
        &measured_specific_force,
        &estimated_specific_force
    );

    return attitude_vec3_is_finite(error);
}

/* ============================================================
 * 积分反馈限制
 * ============================================================ */

static void attitude_limit_integral_feedback(
    attitude_state_t *state
)
{
    float magnitude;
    float scale;

    magnitude = attitude_vec3_magnitude(
        &state->integral_feedback_rad_s
    );

    if (!isfinite(magnitude)) {
        memset(
            &state->integral_feedback_rad_s,
            0,
            sizeof(state->integral_feedback_rad_s)
        );

        return;
    }

    if (magnitude > state->config.integral_limit_rad_s &&
        magnitude > 0.0f) {
        scale =
            state->config.integral_limit_rad_s /
            magnitude;

        state->integral_feedback_rad_s.x *= scale;
        state->integral_feedback_rad_s.y *= scale;
        state->integral_feedback_rad_s.z *= scale;
    }
}

/* ============================================================
 * 初始化与复位
 * ============================================================ */

attitude_status_t attitude_init(
    attitude_state_t *state,
    const algo_attitude_config_t *config
)
{
    if (state == NULL || config == NULL) {
        return ATTITUDE_STATUS_INVALID_ARG;
    }

    if (!attitude_config_is_valid(config)) {
        state->initialized = false;
        return ATTITUDE_STATUS_INVALID_CONFIG;
    }

    memset(state, 0, sizeof(*state));

    state->config = *config;
    state->initialized = true;

    return attitude_reset(state);
}

attitude_status_t attitude_reset(
    attitude_state_t *state
)
{
    if (state == NULL) {
        return ATTITUDE_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return ATTITUDE_STATUS_NOT_INITIALIZED;
    }

    memset(
        &state->integral_feedback_rad_s,
        0,
        sizeof(state->integral_feedback_rad_s)
    );

    attitude_clear_output(&state->current);

    state->convergence_elapsed_s = 0.0f;
    state->orientation_initialized = false;
    state->normalization_counter = 0u;

    return ATTITUDE_STATUS_OK;
}

/* ============================================================
 * 姿态更新
 * ============================================================ */

attitude_status_t attitude_update(
    attitude_state_t *state,
    const filter_out_t *filtered_imu,
    float dt_s,
    attitude_out_t *output
)
{
    algo_vec3_t gravity_error;
    algo_vec3_t corrected_gyro;

    bool gyro_valid;
    bool accel_valid;
    bool correction_enabled;

    float accel_trust;
    float kp;
    float ki;

    if (output != NULL) {
        attitude_clear_output(output);
    }

    if (state == NULL ||
        filtered_imu == NULL ||
        output == NULL) {
        return ATTITUDE_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return ATTITUDE_STATUS_NOT_INITIALIZED;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f) {
        return ATTITUDE_STATUS_INVALID_DT;
    }

    gyro_valid =
        filtered_imu->gyro_valid &&
        attitude_vec3_is_finite(
            &filtered_imu->gyro_rad_s
        );

    accel_valid =
        filtered_imu->accel_valid &&
        attitude_vec3_is_finite(
            &filtered_imu->accel_m_s2
        ) &&
        isfinite(filtered_imu->accel_trust);

    accel_trust = accel_valid
        ? attitude_clamp_float(
              filtered_imu->accel_trust,
              0.0f,
              1.0f
          )
        : 0.0f;

    /*
     * 陀螺无效时不能继续积分。
     *
     * 加速度无效但陀螺有效时，允许短时间仅陀螺更新；
     * 如果初始姿态尚未建立，则仍不能报告有效姿态。
     */
    if (!gyro_valid ||
        (!state->orientation_initialized && !accel_valid)) {
        state->current.valid = false;
        state->current.converged = false;
        state->convergence_elapsed_s = 0.0f;

        *output = state->current;
        output->valid = false;
        output->converged = false;

        return ATTITUDE_STATUS_OK;
    }

    if (!attitude_quaternion_is_finite(
            &state->current.q_bn)) {
        attitude_reset(state);

        *output = state->current;
        return ATTITUDE_STATUS_NUMERIC_ERROR;
    }

    /*
     * 初始姿态必须由可信重力参考建立。
     */
    if (!state->orientation_initialized) {
        if (accel_trust <
            state->config.min_correction_trust) {
            state->current.valid = false;
            state->current.converged = false;

            *output = state->current;
            return ATTITUDE_STATUS_OK;
        }

        if (!attitude_initialize_from_gravity(
                state,
                &filtered_imu->accel_m_s2)) {
            state->current.valid = false;
            state->current.converged = false;

            *output = state->current;
            return ATTITUDE_STATUS_OK;
        }
    }

    corrected_gyro = filtered_imu->gyro_rad_s;

    /*
     * 加入已有的 Mahony 积分反馈。
     */
    corrected_gyro.x +=
        state->integral_feedback_rad_s.x;

    corrected_gyro.y +=
        state->integral_feedback_rad_s.y;

    corrected_gyro.z +=
        state->integral_feedback_rad_s.z;

    correction_enabled =
        accel_valid &&
        accel_trust >=
        state->config.min_correction_trust;

    if (correction_enabled) {
        if (attitude_calculate_gravity_error(
                state,
                &filtered_imu->accel_m_s2,
                &gravity_error)) {
            kp = state->config.kp * accel_trust;
            ki = state->config.ki * accel_trust;

            /*
             * 冲击段进一步降低比例修正，防止瞬时加速度
             * 被误认为真实重力方向。
             */
            if (accel_trust <
                state->config.impact_trust_threshold) {
                kp *= state->config.impact_kp_scale;
            }

            corrected_gyro.x +=
                kp * gravity_error.x;

            corrected_gyro.y +=
                kp * gravity_error.y;

            corrected_gyro.z +=
                kp * gravity_error.z;

            if (state->config.integral_limit_rad_s > 0.0f) {
                state->integral_feedback_rad_s.x +=
                    ki * gravity_error.x * dt_s;

                state->integral_feedback_rad_s.y +=
                    ki * gravity_error.y * dt_s;

                state->integral_feedback_rad_s.z +=
                    ki * gravity_error.z * dt_s;

                attitude_limit_integral_feedback(state);
            }
        }
    }

    if (!attitude_integrate_gyro(
            state,
            &corrected_gyro,
            dt_s)) {
        attitude_reset(state);

        *output = state->current;
        return ATTITUDE_STATUS_NUMERIC_ERROR;
    }

    attitude_update_euler_angles(state);

    /*
     * 只有持续获得可信重力参考时，才累计收敛时间。
     */
    if (correction_enabled) {
        state->convergence_elapsed_s += dt_s;
    }

    if (state->convergence_elapsed_s >=
        state->config.convergence_time_s) {
        state->current.converged = true;
    }

    state->current.accel_trust = accel_trust;
    state->current.valid = true;

    *output = state->current;

    return ATTITUDE_STATUS_OK;
}

/* ============================================================
 * 当前姿态快照
 * ============================================================ */

attitude_status_t attitude_get(
    const attitude_state_t *state,
    attitude_out_t *output
)
{
    if (output != NULL) {
        attitude_clear_output(output);
    }

    if (state == NULL || output == NULL) {
        return ATTITUDE_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return ATTITUDE_STATUS_NOT_INITIALIZED;
    }

    *output = state->current;

    return ATTITUDE_STATUS_OK;
}

/* ============================================================
 * 重新收敛
 * ============================================================ */

attitude_status_t attitude_restart_convergence(
    attitude_state_t *state
)
{
    if (state == NULL) {
        return ATTITUDE_STATUS_INVALID_ARG;
    }

    if (!state->initialized) {
        return ATTITUDE_STATUS_NOT_INITIALIZED;
    }

    state->convergence_elapsed_s = 0.0f;
    state->current.converged = false;

    return ATTITUDE_STATUS_OK;
}