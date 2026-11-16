/*
 * Standalone C99 tests for algo_control.c.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "algo_control.h"

#define TEST_FLOAT_TOLERANCE 1.0e-6f

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            printf("    CHECK failed at %s:%d: %s\n",                         \
                   __FILE__, __LINE__, #condition);                             \
            return false;                                                       \
        }                                                                       \
    } while (0)

#define CHECK_STATUS(expression, expected_status)                               \
    do {                                                                        \
        control_status_t actual_status_ = (expression);                         \
        control_status_t expected_status_ = (expected_status);                  \
        if (actual_status_ != expected_status_) {                               \
            printf("    STATUS failed at %s:%d: got %d, expected %d\n",       \
                   __FILE__, __LINE__, (int)actual_status_,                     \
                   (int)expected_status_);                                      \
            return false;                                                       \
        }                                                                       \
    } while (0)

#define CHECK_FLOAT(actual_expression, expected_expression)                     \
    do {                                                                        \
        float actual_value_ = (actual_expression);                              \
        float expected_value_ = (expected_expression);                          \
        if (!isfinite(actual_value_) ||                                         \
            fabsf(actual_value_ - expected_value_) > TEST_FLOAT_TOLERANCE) {    \
            printf("    FLOAT failed at %s:%d: got %.9g, expected %.9g\n",    \
                   __FILE__, __LINE__, (double)actual_value_,                   \
                   (double)expected_value_);                                    \
            return false;                                                       \
        }                                                                       \
    } while (0)

typedef struct {
    control_state_t state;
    algo_control_config_t config;
    algo_rc_command_t rc;
    attitude_out_t attitude;
    filter_out_t imu;
    safety_directive_t safety;
    control_out_t output;
} test_fixture_t;

static void fixture_make(test_fixture_t *fixture, bool closed_loop)
{
    const algo_control_config_t default_config = ALGO_CONTROL_CONFIG_DEFAULT;

    memset(fixture, 0, sizeof(*fixture));
    fixture->config = default_config;

    fixture->rc.valid = true;
    fixture->attitude.valid = true;
    fixture->attitude.converged = true;
    fixture->imu.valid = true;
    fixture->imu.gyro_valid = true;
    fixture->safety.allow_stabilization = closed_loop;
}

static bool pid_is_zero(const control_pid_state_t *pid)
{
    return pid->integral == 0.0f &&
           pid->previous_measurement == 0.0f &&
           pid->derivative_filtered == 0.0f &&
           !pid->primed;
}

static bool history_is_zero(const control_state_t *state)
{
    uint32_t axis;

    for (axis = 0u; axis < CONTROL_ANGLE_AXIS_COUNT; ++axis) {
        if (!pid_is_zero(&state->angle_pid[axis])) {
            return false;
        }
    }

    for (axis = 0u; axis < (uint32_t)ALGO_AXIS_COUNT; ++axis) {
        if (!pid_is_zero(&state->rate_pid[axis])) {
            return false;
        }
    }

    return true;
}

static void history_seed(control_state_t *state)
{
    uint32_t axis;

    for (axis = 0u; axis < CONTROL_ANGLE_AXIS_COUNT; ++axis) {
        state->angle_pid[axis].integral = 0.01f * (float)(axis + 1u);
        state->angle_pid[axis].previous_measurement = 0.1f;
        state->angle_pid[axis].derivative_filtered = 0.2f;
        state->angle_pid[axis].primed = true;
    }

    for (axis = 0u; axis < (uint32_t)ALGO_AXIS_COUNT; ++axis) {
        state->rate_pid[axis].integral = 0.02f * (float)(axis + 1u);
        state->rate_pid[axis].previous_measurement = 0.3f;
        state->rate_pid[axis].derivative_filtered = 0.4f;
        state->rate_pid[axis].primed = true;
    }
}

static bool output_is_zero_invalid(const control_out_t *output)
{
    return output->lift == 0.0f &&
           output->roll == 0.0f &&
           output->pitch == 0.0f &&
           output->yaw == 0.0f &&
           !output->valid;
}

static bool outputs_equal(const control_out_t *lhs, const control_out_t *rhs)
{
    return lhs->lift == rhs->lift &&
           lhs->roll == rhs->roll &&
           lhs->pitch == rhs->pitch &&
           lhs->yaw == rhs->yaw &&
           lhs->valid == rhs->valid;
}

static control_status_t fixture_update(test_fixture_t *fixture, float dt_s)
{
    return control_update(&fixture->state,
                          &fixture->rc,
                          &fixture->attitude,
                          &fixture->imu,
                          &fixture->safety,
                          dt_s,
                          &fixture->output);
}

static bool test_init_and_reset(void)
{
    test_fixture_t fixture;
    control_state_t uninitialized;
    algo_control_config_t invalid_config;

    memset(&uninitialized, 0, sizeof(uninitialized));
    CHECK_STATUS(control_reset(NULL), CONTROL_STATUS_INVALID_ARG);
    CHECK_STATUS(control_reset(&uninitialized),
                 CONTROL_STATUS_NOT_INITIALIZED);

    fixture_make(&fixture, false);
    CHECK_STATUS(control_init(NULL, &fixture.config),
                 CONTROL_STATUS_INVALID_ARG);
    CHECK_STATUS(control_init(&fixture.state, NULL),
                 CONTROL_STATUS_INVALID_ARG);

    invalid_config = fixture.config;
    invalid_config.rate_pid[ALGO_AXIS_Y].derivative_lpf_cutoff_hz = 0.0f;
    CHECK_STATUS(control_init(&fixture.state, &invalid_config),
                 CONTROL_STATUS_INVALID_CONFIG);
    CHECK(!fixture.state.initialized);

    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);
    CHECK(fixture.state.initialized);
    CHECK(history_is_zero(&fixture.state));
    CHECK(output_is_zero_invalid(&fixture.state.current));

    /* 初始化复制配置，不保存调用者的配置指针。 */
    fixture.config.max_roll_output = 0.1f;
    CHECK_FLOAT(fixture.state.config.max_roll_output, 1.0f);

    CHECK_STATUS(control_get(&fixture.state, &fixture.output),
                 CONTROL_STATUS_OK);
    CHECK(output_is_zero_invalid(&fixture.output));

    fixture.rc.throttle = 0.4f;
    fixture.rc.roll = 0.3f;
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);
    CHECK(fixture.output.valid);
    history_seed(&fixture.state);

    CHECK_STATUS(control_reset(&fixture.state), CONTROL_STATUS_OK);
    CHECK(fixture.state.initialized);
    CHECK(history_is_zero(&fixture.state));
    CHECK(output_is_zero_invalid(&fixture.state.current));
    CHECK_FLOAT(fixture.state.config.max_roll_output, 1.0f);

    return true;
}

static bool test_manual_mapping_and_clamping(void)
{
    test_fixture_t fixture;

    fixture_make(&fixture, false);
    fixture.config.max_roll_output = 0.4f;
    fixture.config.max_pitch_output = 0.3f;
    fixture.config.max_yaw_output = 0.2f;
    fixture.config.yaw_control_enable = true;
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);

    /* 手动模式不得读取无效反馈。 */
    fixture.attitude.valid = false;
    fixture.attitude.converged = false;
    fixture.attitude.roll_rad = NAN;
    fixture.attitude.pitch_rad = NAN;
    fixture.imu.valid = false;
    fixture.imu.gyro_valid = false;
    fixture.imu.gyro_rad_s.x = NAN;
    fixture.imu.gyro_rad_s.y = NAN;
    fixture.imu.gyro_rad_s.z = NAN;

    fixture.rc.throttle = 0.65f;
    fixture.rc.roll = 0.5f;
    fixture.rc.pitch = -0.25f;
    fixture.rc.yaw = 0.75f;
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);

    /*
     * roll=clamp(0.5,-0.4,0.4)=0.4
     * pitch=clamp(-0.25,-0.3,0.3)=-0.25
     * yaw=clamp(0.75,-0.2,0.2)=0.2
     */
    CHECK(fixture.output.valid);
    CHECK_FLOAT(fixture.output.lift, 0.65f);
    CHECK_FLOAT(fixture.output.roll, 0.4f);
    CHECK_FLOAT(fixture.output.pitch, -0.25f);
    CHECK_FLOAT(fixture.output.yaw, 0.2f);
    CHECK(history_is_zero(&fixture.state));

    fixture.rc.throttle = 1.7f;
    fixture.rc.roll = -2.0f;
    fixture.rc.pitch = 2.0f;
    fixture.rc.yaw = -3.0f;
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.output.lift, 1.0f);
    CHECK_FLOAT(fixture.output.roll, -0.4f);
    CHECK_FLOAT(fixture.output.pitch, 0.3f);
    CHECK_FLOAT(fixture.output.yaw, -0.2f);

    fixture.rc.throttle = -0.25f;
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.output.lift, 0.0f);
    CHECK(history_is_zero(&fixture.state));

    return true;
}

static bool test_zero_gain_closed_loop(void)
{
    test_fixture_t fixture;
    control_out_t cached;

    fixture_make(&fixture, true);
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);

    fixture.rc.throttle = 0.4f;
    fixture.rc.roll = 0.6f;
    fixture.rc.pitch = -0.4f;
    fixture.rc.yaw = 0.9f;
    fixture.attitude.roll_rad = 0.1f;
    fixture.attitude.pitch_rad = -0.2f;
    fixture.imu.gyro_rad_s.x = 0.3f;
    fixture.imu.gyro_rad_s.y = -0.5f;
    fixture.imu.gyro_rad_s.z = 0.7f;

    CHECK_STATUS(fixture_update(&fixture, 0.005f), CONTROL_STATUS_OK);
    CHECK(fixture.output.valid);
    CHECK_FLOAT(fixture.output.lift, 0.4f);
    CHECK_FLOAT(fixture.output.roll, 0.0f);
    CHECK_FLOAT(fixture.output.pitch, 0.0f);
    CHECK_FLOAT(fixture.output.yaw, 0.0f);

    CHECK_STATUS(control_get(&fixture.state, &cached), CONTROL_STATUS_OK);
    CHECK(outputs_equal(&cached, &fixture.output));

    return true;
}

static bool test_attitude_outer_loop_sign(void)
{
    test_fixture_t fixture;

    fixture_make(&fixture, true);
    fixture.config.max_roll_angle_rad = 0.5f;
    fixture.config.max_pitch_angle_rad = 0.4f;
    fixture.config.angle_pid[0].kp = 2.0f;
    fixture.config.angle_pid[1].kp = 3.0f;
    fixture.config.rate_pid[ALGO_AXIS_X].kp = 1.0f;
    fixture.config.rate_pid[ALGO_AXIS_Y].kp = 1.0f;
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);

    fixture.rc.roll = 0.2f;
    fixture.rc.pitch = -0.5f;
    fixture.attitude.roll_rad = 0.04f;
    fixture.attitude.pitch_rad = 0.1f;

    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);

    /*
     * 首次更新 D 项和所有 I 项为零，陀螺反馈为零：
     * roll target=0.2*0.5=0.10 rad
     * angle error=0.10-0.04=0.06 rad
     * rate target=2*0.06=0.12 rad/s
     * inner output=1*(0.12-0)=0.12
     *
     * pitch target=-0.5*0.4=-0.20 rad
     * angle error=-0.20-0.10=-0.30 rad
     * rate target=3*(-0.30)=-0.90 rad/s
     * inner output=-0.90
     */
    CHECK_FLOAT(fixture.output.roll, 0.12f);
    CHECK_FLOAT(fixture.output.pitch, -0.90f);
    CHECK_FLOAT(fixture.output.yaw, 0.0f);

    return true;
}

static bool test_rate_inner_loop_sign(void)
{
    test_fixture_t fixture;

    fixture_make(&fixture, true);
    fixture.config.rate_pid[ALGO_AXIS_X].kp = 0.5f;
    fixture.config.rate_pid[ALGO_AXIS_Y].kp = 0.25f;
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);

    fixture.imu.gyro_rad_s.x = 0.8f;
    fixture.imu.gyro_rad_s.y = -1.2f;
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);

    /*
     * 外环增益为零，因此目标角速度为零，首次更新 D=0：
     * roll error=0-0.8=-0.8，output=0.5*(-0.8)=-0.4
     * pitch error=0-(-1.2)=1.2，output=0.25*1.2=0.3
     */
    CHECK_FLOAT(fixture.output.roll, -0.4f);
    CHECK_FLOAT(fixture.output.pitch, 0.3f);

    return true;
}

static bool test_yaw_enable_switch(void)
{
    test_fixture_t fixture;

    fixture_make(&fixture, false);
    fixture.config.max_yaw_output = 0.8f;
    fixture.config.max_rate_rad_s[ALGO_AXIS_Z] = 2.0f;
    fixture.config.rate_pid[ALGO_AXIS_Z].kp = 0.5f;
    fixture.config.yaw_control_enable = false;
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);

    fixture.rc.yaw = 0.5f;
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.output.yaw, 0.0f);

    fixture.safety.allow_stabilization = true;
    fixture.imu.gyro_rad_s.z = 0.2f;
    history_seed(&fixture.state);
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.output.yaw, 0.0f);
    CHECK(pid_is_zero(&fixture.state.rate_pid[ALGO_AXIS_Z]));

    fixture.config.yaw_control_enable = true;
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);
    fixture.safety.allow_stabilization = false;
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.output.yaw, 0.5f);

    fixture.safety.allow_stabilization = true;

    /* 控制器不读取绝对偏航角。 */
    fixture.attitude.yaw_rad = NAN;
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);

    /*
     * target=0.5*2=1.0 rad/s
     * error=1.0-0.2=0.8 rad/s
     * 首次更新 D=0、I=0，因此 yaw=0.5*0.8=0.4。
     */
    CHECK_FLOAT(fixture.output.yaw, 0.4f);

    return true;
}

static bool test_unready_inputs_clear_history(void)
{
    test_fixture_t fixture;
    int test_case;

    for (test_case = 0; test_case < 5; ++test_case) {
        fixture_make(&fixture, true);
        CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                     CONTROL_STATUS_OK);
        history_seed(&fixture.state);
        fixture.output.lift = 1.0f;
        fixture.output.valid = true;

        switch (test_case) {
        case 0:
            fixture.rc.valid = false;
            break;
        case 1:
            fixture.attitude.valid = false;
            break;
        case 2:
            fixture.attitude.converged = false;
            break;
        case 3:
            fixture.imu.valid = false;
            break;
        default:
            fixture.imu.gyro_valid = false;
            break;
        }

        CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);
        CHECK(output_is_zero_invalid(&fixture.output));
        CHECK(history_is_zero(&fixture.state));
        CHECK(output_is_zero_invalid(&fixture.state.current));
    }

    /* 手动模式下，无效 RC 也必须产生安全输出。 */
    fixture_make(&fixture, false);
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);
    history_seed(&fixture.state);
    fixture.rc.valid = false;
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);
    CHECK(output_is_zero_invalid(&fixture.output));
    CHECK(history_is_zero(&fixture.state));

    return true;
}

static bool test_invalid_dt_and_nonfinite_inputs(void)
{
    test_fixture_t fixture;
    static const float invalid_dt[] = {0.0f, -0.01f, NAN, INFINITY};
    uint32_t index;

    fixture_make(&fixture, true);
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);

    for (index = 0u;
         index < (uint32_t)(sizeof(invalid_dt) / sizeof(invalid_dt[0]));
         ++index) {
        fixture.output.lift = 1.0f;
        fixture.output.valid = true;
        CHECK_STATUS(fixture_update(&fixture, invalid_dt[index]),
                     CONTROL_STATUS_INVALID_DT);
        CHECK(output_is_zero_invalid(&fixture.output));
        CHECK(output_is_zero_invalid(&fixture.state.current));
    }

    /* 每个有效 RC 通道都必须是有限值。 */
    for (index = 0u; index < 4u; ++index) {
        float *channels[4];

        fixture_make(&fixture, true);
        channels[0] = &fixture.rc.throttle;
        channels[1] = &fixture.rc.roll;
        channels[2] = &fixture.rc.pitch;
        channels[3] = &fixture.rc.yaw;
        *channels[index] = (index == 3u) ? INFINITY : NAN;

        CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                     CONTROL_STATUS_OK);
        history_seed(&fixture.state);
        CHECK_STATUS(fixture_update(&fixture, 0.01f),
                     CONTROL_STATUS_INVALID_INPUT);
        CHECK(output_is_zero_invalid(&fixture.output));
        CHECK(history_is_zero(&fixture.state));
    }

    /* 闭环使用的所有姿态角和角速度都必须是有限值。 */
    for (index = 0u; index < 5u; ++index) {
        float *feedback[5];

        fixture_make(&fixture, true);
        feedback[0] = &fixture.attitude.roll_rad;
        feedback[1] = &fixture.attitude.pitch_rad;
        feedback[2] = &fixture.imu.gyro_rad_s.x;
        feedback[3] = &fixture.imu.gyro_rad_s.y;
        feedback[4] = &fixture.imu.gyro_rad_s.z;
        *feedback[index] = NAN;

        CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                     CONTROL_STATUS_OK);
        CHECK_STATUS(fixture_update(&fixture, 0.01f),
                     CONTROL_STATUS_INVALID_INPUT);
        CHECK(output_is_zero_invalid(&fixture.output));
        CHECK(history_is_zero(&fixture.state));
    }

    return true;
}

static bool test_integral_anti_windup_and_reset(void)
{
    test_fixture_t fixture;
    static const float expected_integral[] = {
        0.04f, 0.08f, 0.12f, 0.15f
    };
    uint32_t index;

    fixture_make(&fixture, true);
    fixture.config.rate_pid[ALGO_AXIS_X].kp = 0.0f;
    fixture.config.rate_pid[ALGO_AXIS_X].ki = 2.0f;
    fixture.config.rate_pid[ALGO_AXIS_X].integral_limit = 0.15f;
    fixture.config.max_roll_output = 1.0f;
    fixture.config.anti_windup_enable = true;
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);

    fixture.imu.gyro_rad_s.x = -0.2f;

    /*
     * 外环输出为零，因此目标角速度为零，角速度误差为 0.2 rad/s。
     * 每次 dt=0.1 s 的更新增加：
     * ki*e*dt=2*0.2*0.1=0.04。
     * 第四次原始积分值 0.16 被限制为 0.15。
     */
    for (index = 0u; index < 4u; ++index) {
        CHECK_STATUS(fixture_update(&fixture, 0.1f), CONTROL_STATUS_OK);
        CHECK_FLOAT(fixture.state.rate_pid[ALGO_AXIS_X].integral,
                    expected_integral[index]);
        CHECK_FLOAT(fixture.output.roll, expected_integral[index]);
    }

    CHECK_STATUS(control_reset(&fixture.state), CONTROL_STATUS_OK);
    CHECK(history_is_zero(&fixture.state));
    CHECK_STATUS(fixture_update(&fixture, 0.1f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.output.roll, 0.04f);

    CHECK_STATUS(fixture_update(&fixture, 0.1f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.output.roll, 0.08f);

    fixture.safety.reset_controller = true;
    CHECK_STATUS(fixture_update(&fixture, 0.1f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.output.roll, 0.04f);
    CHECK_FLOAT(fixture.state.rate_pid[ALGO_AXIS_X].integral, 0.04f);

    /*
     * P=2*1=2，已经超过 +/-0.5 输出限幅。
     * 误差仍指向饱和方向，因此抗积分饱和应阻止积分增长。
     */
    fixture_make(&fixture, true);
    fixture.config.rate_pid[ALGO_AXIS_X].kp = 2.0f;
    fixture.config.rate_pid[ALGO_AXIS_X].ki = 1.0f;
    fixture.config.rate_pid[ALGO_AXIS_X].integral_limit = 10.0f;
    fixture.config.max_roll_output = 0.5f;
    fixture.config.anti_windup_enable = true;
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);

    fixture.imu.gyro_rad_s.x = -1.0f;
    CHECK_STATUS(fixture_update(&fixture, 0.1f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.output.roll, 0.5f);
    CHECK_FLOAT(fixture.state.rate_pid[ALGO_AXIS_X].integral, 0.0f);

    CHECK_STATUS(fixture_update(&fixture, 0.1f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.state.rate_pid[ALGO_AXIS_X].integral, 0.0f);

    /* 负向饱和同样不得继续积分。 */
    fixture.imu.gyro_rad_s.x = 1.0f;
    CHECK_STATUS(fixture_update(&fixture, 0.1f), CONTROL_STATUS_OK);
    CHECK_FLOAT(fixture.output.roll, -0.5f);
    CHECK_FLOAT(fixture.state.rate_pid[ALGO_AXIS_X].integral, 0.0f);

    return true;
}

static bool test_control_get(void)
{
    test_fixture_t fixture;
    control_state_t snapshot;
    control_out_t cached;

    fixture_make(&fixture, false);
    memset(&fixture.state, 0, sizeof(fixture.state));
    cached.lift = 1.0f;
    cached.valid = true;

    CHECK_STATUS(control_get(NULL, &cached), CONTROL_STATUS_INVALID_ARG);
    CHECK(output_is_zero_invalid(&cached));
    CHECK_STATUS(control_get(&fixture.state, NULL),
                 CONTROL_STATUS_INVALID_ARG);
    CHECK_STATUS(control_get(&fixture.state, &cached),
                 CONTROL_STATUS_NOT_INITIALIZED);
    CHECK(output_is_zero_invalid(&cached));

    fixture.config.max_roll_output = 0.5f;
    fixture.config.max_pitch_output = 0.5f;
    CHECK_STATUS(control_init(&fixture.state, &fixture.config),
                 CONTROL_STATUS_OK);

    fixture.rc.throttle = 0.33f;
    fixture.rc.roll = 0.2f;
    fixture.rc.pitch = -0.1f;
    CHECK_STATUS(fixture_update(&fixture, 0.01f), CONTROL_STATUS_OK);

    snapshot = fixture.state;
    CHECK_STATUS(control_get(&fixture.state, &cached), CONTROL_STATUS_OK);
    CHECK(outputs_equal(&cached, &fixture.output));
    CHECK(memcmp(&snapshot, &fixture.state, sizeof(snapshot)) == 0);

    /* 只修改输入而不执行更新时，缓存输出不能改变。 */
    fixture.rc.throttle = 0.99f;
    fixture.rc.roll = -1.0f;
    CHECK_STATUS(control_get(&fixture.state, &cached), CONTROL_STATUS_OK);
    CHECK(outputs_equal(&cached, &fixture.output));
    CHECK_FLOAT(cached.lift, 0.33f);
    CHECK_FLOAT(cached.roll, 0.2f);

    CHECK_STATUS(control_reset(&fixture.state), CONTROL_STATUS_OK);
    CHECK_STATUS(control_get(&fixture.state, &cached), CONTROL_STATUS_OK);
    CHECK(output_is_zero_invalid(&cached));

    return true;
}

typedef bool (*test_function_t)(void);

typedef struct {
    const char *name;
    test_function_t function;
} named_test_t;

int main(void)
{
    static const named_test_t tests[] = {
        {"init/reset/config copy", test_init_and_reset},
        {"manual mapping and clamping", test_manual_mapping_and_clamping},
        {"zero-gain closed loop", test_zero_gain_closed_loop},
        {"attitude outer-loop signs", test_attitude_outer_loop_sign},
        {"rate inner-loop signs", test_rate_inner_loop_sign},
        {"yaw enable switch", test_yaw_enable_switch},
        {"unready inputs clear history", test_unready_inputs_clear_history},
        {"invalid dt and non-finite inputs",
         test_invalid_dt_and_nonfinite_inputs},
        {"integral/anti-windup/reset",
         test_integral_anti_windup_and_reset},
        {"control_get cache", test_control_get}
    };

    const uint32_t test_count =
        (uint32_t)(sizeof(tests) / sizeof(tests[0]));

    uint32_t passed = 0u;
    uint32_t index;

    for (index = 0u; index < test_count; ++index) {
        bool ok = tests[index].function();

        if (ok) {
            ++passed;
            printf("PASS: %s\n", tests[index].name);
        } else {
            printf("FAIL: %s\n", tests[index].name);
        }
    }

    printf("Summary: %u/%u tests passed.\n",
           (unsigned int)passed,
           (unsigned int)test_count);

    return (passed == test_count) ? EXIT_SUCCESS : EXIT_FAILURE;
}