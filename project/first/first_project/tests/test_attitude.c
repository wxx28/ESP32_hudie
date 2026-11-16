/*
 * Mahony 姿态解算模块主机端单元测试
 *
 * 在 PC 上编译运行（不依赖 ESP32 硬件），验证 algo_attitude 的
 * 接口契约和融合行为。断言失败只计数不中断，跑完全部用例后
 * 通过进程退出码报告总体结果（0 表示全部通过）。
 *
 * 测试条件约定：
 *   - 更新周期 2.5 ms（400 Hz，与快速任务一致）；
 *   - 加速度计静置时只感受重力，机体水平时读数为 (0, 0, -g)；
 *   - 机体右滚 θ 时重力在体轴上的投影为 (0, -g·sinθ, -g·cosθ)；
 *   - 机体抬头 θ 时投影为 (g·sinθ, 0, -g·cosθ)；
 *   - 默认配置见 algo_config.h：kp = 1.5，ki = 0.01，
 *     收敛判定时间 2 s，因此 10 秒仿真足够收敛并置位 converged。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "algo_attitude.h"

/* 更新周期 2.5 ms，对应 400 Hz。 */
#define TEST_DT_S               0.0025f

/* 标准重力加速度，用于构造静置时的加速度计读数。 */
#define TEST_GRAVITY_M_S2       9.80665f

/* 弧度角度互转系数。 */
#define TEST_RAD_TO_DEG         57.29577951308232f
#define TEST_DEG_TO_RAD         0.017453292519943295f

/* 400 Hz 下的仿真步数：1 秒和 10 秒。 */
#define TEST_STEPS_1_SECOND     400
#define TEST_STEPS_10_SECONDS   4000

/* 累计失败断言数，main 据此决定退出码。 */
static int g_test_failures = 0;

/* ============================================================
 * 测试辅助函数
 * ============================================================ */

/*
 * 构造一帧滤波后的 IMU 采样。
 *
 * 参数：
 *   gx/gy/gz   陀螺角速度，rad/s；
 *   ax/ay/az   加速度计读数，m/s^2；
 *   accel_trust 加速度可信度（0 表示本帧关闭加速度修正）。
 *
 * 振动指标填零，陀螺和加速度均标记有效。
 */
static void test_set_sample(
    filter_out_t *sample,
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az,
    float accel_trust
)
{
    memset(sample, 0, sizeof(*sample));

    sample->gyro_rad_s.x = gx;
    sample->gyro_rad_s.y = gy;
    sample->gyro_rad_s.z = gz;

    sample->accel_m_s2.x = ax;
    sample->accel_m_s2.y = ay;
    sample->accel_m_s2.z = az;

    sample->accel_trust = accel_trust;
    sample->vibration_rms_rad_s = 0.0f;

    sample->gyro_valid = true;
    sample->accel_valid = true;
    sample->valid = true;
}

/*
 * 用同一帧采样连续运行 step_count 次姿态更新。
 *
 * 任一步返回非 OK 状态时打印失败位置并返回 false，
 * 由调用者决定终止当前用例。
 */
static bool test_run_updates(
    attitude_state_t *state,
    const filter_out_t *sample,
    int step_count,
    attitude_out_t *output
)
{
    int step;
    attitude_status_t status;

    for (step = 0; step < step_count; ++step) {
        status = attitude_update(
            state,
            sample,
            TEST_DT_S,
            output
        );

        if (status != ATTITUDE_STATUS_OK) {
            printf(
                "attitude_update failed at step %d, status = %d\n",
                step,
                (int)status
            );

            return false;
        }
    }

    return true;
}

/*
 * 浮点近似断言：|actual - expected| <= tolerance 视为通过。
 * 失败时打印误差和容差并累计失败数。
 */
static void test_check_close(
    const char *test_name,
    float actual,
    float expected,
    float tolerance
)
{
    float error = fabsf(actual - expected);

    if (error <= tolerance) {
        printf(
            "[PASS] %s: actual = %.6f, expected = %.6f\n",
            test_name,
            actual,
            expected
        );
    } else {
        printf(
            "[FAIL] %s: actual = %.6f, expected = %.6f, "
            "error = %.6f, tolerance = %.6f\n",
            test_name,
            actual,
            expected,
            error,
            tolerance
        );

        g_test_failures++;
    }
}

/* 布尔条件断言。 */
static void test_check_true(
    const char *test_name,
    bool condition
)
{
    if (condition) {
        printf("[PASS] %s\n", test_name);
    } else {
        printf("[FAIL] %s\n", test_name);
        g_test_failures++;
    }
}

/*
 * 计算四元数模长。
 * 用于验证姿态四元数始终保持单位长度（模长应为 1）。
 */
static float test_quaternion_norm(
    const algo_quat_t *quaternion
)
{
    return sqrtf(
        quaternion->w * quaternion->w +
        quaternion->x * quaternion->x +
        quaternion->y * quaternion->y +
        quaternion->z * quaternion->z
    );
}

/*
 * 打印一次姿态输出，便于断言失败时从日志定位实际收敛值。
 * 该打印不参与判定。
 */
static void test_print_attitude(
    const char *test_name,
    const attitude_out_t *output
)
{
    printf(
        "%s: roll = %.4f deg, pitch = %.4f deg, "
        "yaw = %.4f deg, trust = %.3f, "
        "valid = %d, converged = %d\n",
        test_name,
        output->roll_rad * TEST_RAD_TO_DEG,
        output->pitch_rad * TEST_RAD_TO_DEG,
        output->yaw_rad * TEST_RAD_TO_DEG,
        output->accel_trust,
        (int)output->valid,
        (int)output->converged
    );
}

/*
 * 使用默认配置初始化姿态解算模块。
 * 初始化失败时打印状态并返回 false，由调用者终止当前用例。
 */
static bool test_initialize_attitude(
    attitude_state_t *state
)
{
    algo_attitude_config_t config =
        ALGO_ATTITUDE_CONFIG_DEFAULT;

    attitude_status_t status =
        attitude_init(state, &config);

    if (status != ATTITUDE_STATUS_OK) {
        printf(
            "attitude_init failed, status = %d\n",
            (int)status
        );

        return false;
    }

    return true;
}

/* ============================================================
 * 测试1：水平静置
 *
 * 陀螺零输入，加速度计读数 (0, 0, -g)，仿真 10 秒。
 * 期望三轴姿态角均收敛到 0 度（容差 0.1 度），输出有效、
 * 已收敛，且姿态四元数保持单位模长。
 * ============================================================ */

static void test_level_attitude(void)
{
    attitude_state_t state;
    attitude_out_t output;
    filter_out_t sample;
    bool update_ok;

    printf("\n===== Test 1: Level attitude =====\n");

    if (!test_initialize_attitude(&state)) {
        g_test_failures++;
        return;
    }

    /* 水平静置：无角速度，重力完全落在 -Z 轴上。 */
    test_set_sample(
        &sample,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -TEST_GRAVITY_M_S2,
        1.0f
    );

    update_ok = test_run_updates(
        &state,
        &sample,
        TEST_STEPS_10_SECONDS,
        &output
    );

    test_check_true("Level update status", update_ok);

    if (!update_ok) {
        return;
    }

    test_print_attitude("Level", &output);

    test_check_close(
        "Level roll",
        output.roll_rad * TEST_RAD_TO_DEG,
        0.0f,
        0.1f
    );

    test_check_close(
        "Level pitch",
        output.pitch_rad * TEST_RAD_TO_DEG,
        0.0f,
        0.1f
    );

    /* 无磁力计参与时偏航角保持初值 0。 */
    test_check_close(
        "Level yaw",
        output.yaw_rad * TEST_RAD_TO_DEG,
        0.0f,
        0.1f
    );

    test_check_true(
        "Level valid",
        output.valid
    );

    /* 仿真 10 秒，超过默认的 2 秒收敛判定时间。 */
    test_check_true(
        "Level converged",
        output.converged
    );

    /* 姿态四元数必须始终归一化。 */
    test_check_close(
        "Level quaternion norm",
        test_quaternion_norm(&output.q_bn),
        1.0f,
        1.0e-4f
    );
}

/* ============================================================
 * 测试2：右滚30度
 *
 * 陀螺零输入，加速度计读数对应机体右滚 30 度时的重力投影
 * (0, -g·sin30°, -g·cos30°)，仿真 10 秒。
 * 期望 roll 收敛到 +30 度，pitch 保持 0 度。
 * ============================================================ */

static void test_roll_30_degrees(void)
{
    attitude_state_t state;
    attitude_out_t output;
    filter_out_t sample;
    bool update_ok;

    float angle = 30.0f * TEST_DEG_TO_RAD;

    printf("\n===== Test 2: Roll +30 degrees =====\n");

    if (!test_initialize_attitude(&state)) {
        g_test_failures++;
        return;
    }

    /* 右滚 30 度静置时的重力投影。 */
    test_set_sample(
        &sample,
        0.0f, 0.0f, 0.0f,
        0.0f,
        -TEST_GRAVITY_M_S2 * sinf(angle),
        -TEST_GRAVITY_M_S2 * cosf(angle),
        1.0f
    );

    update_ok = test_run_updates(
        &state,
        &sample,
        TEST_STEPS_10_SECONDS,
        &output
    );

    test_check_true("Roll update status", update_ok);

    if (!update_ok) {
        return;
    }

    test_print_attitude("Roll +30", &output);

    test_check_close(
        "Roll angle",
        output.roll_rad * TEST_RAD_TO_DEG,
        30.0f,
        0.1f
    );

    /* 纯滚转场景下俯仰通道不应被激励。 */
    test_check_close(
        "Roll case pitch",
        output.pitch_rad * TEST_RAD_TO_DEG,
        0.0f,
        0.1f
    );

    test_check_true(
        "Roll case valid",
        output.valid
    );

    test_check_true(
        "Roll case converged",
        output.converged
    );
}

/* ============================================================
 * 测试3：抬头30度
 *
 * 陀螺零输入，加速度计读数对应机体抬头 30 度时的重力投影
 * (g·sin30°, 0, -g·cos30°)，仿真 10 秒。
 * 期望 pitch 收敛到 +30 度，roll 保持 0 度。
 * ============================================================ */

static void test_pitch_30_degrees(void)
{
    attitude_state_t state;
    attitude_out_t output;
    filter_out_t sample;
    bool update_ok;

    float angle = 30.0f * TEST_DEG_TO_RAD;

    printf("\n===== Test 3: Pitch +30 degrees =====\n");

    if (!test_initialize_attitude(&state)) {
        g_test_failures++;
        return;
    }

    /* 抬头 30 度静置时的重力投影。 */
    test_set_sample(
        &sample,
        0.0f, 0.0f, 0.0f,
        TEST_GRAVITY_M_S2 * sinf(angle),
        0.0f,
        -TEST_GRAVITY_M_S2 * cosf(angle),
        1.0f
    );

    update_ok = test_run_updates(
        &state,
        &sample,
        TEST_STEPS_10_SECONDS,
        &output
    );

    test_check_true("Pitch update status", update_ok);

    if (!update_ok) {
        return;
    }

    test_print_attitude("Pitch +30", &output);

    test_check_close(
        "Pitch angle",
        output.pitch_rad * TEST_RAD_TO_DEG,
        30.0f,
        0.1f
    );

    /* 纯俯仰场景下滚转通道不应被激励。 */
    test_check_close(
        "Pitch case roll",
        output.roll_rad * TEST_RAD_TO_DEG,
        0.0f,
        0.1f
    );

    test_check_true(
        "Pitch case valid",
        output.valid
    );

    test_check_true(
        "Pitch case converged",
        output.converged
    );
}

/* ============================================================
 * 测试4：纯陀螺积分
 *
 * 先在水平参考下更新 10 步建立初始姿态，然后绕 X 轴输入
 * 0.1 rad/s 恒定角速度并令 accel_trust = 0 关闭加速度修正，
 * 仿真 1 秒。
 *
 * 期望 roll = 0.1 rad/s × 1 s = 0.1 rad ≈ 5.73 度，
 * pitch 保持 0 度。容差 0.05 度，覆盖积分离散误差。
 * ============================================================ */

static void test_gyro_integration(void)
{
    attitude_state_t state;
    attitude_out_t output;
    filter_out_t sample;
    bool update_ok;

    printf("\n===== Test 4: Gyro integration =====\n");

    if (!test_initialize_attitude(&state)) {
        g_test_failures++;
        return;
    }

    /* 先建立水平初始姿态。 */
    test_set_sample(
        &sample,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -TEST_GRAVITY_M_S2,
        1.0f
    );

    update_ok = test_run_updates(
        &state,
        &sample,
        10,
        &output
    );

    if (!update_ok) {
        g_test_failures++;
        return;
    }

    /*
     * 绕X轴以0.1 rad/s旋转1秒。
     * 关闭加速度修正，只检查陀螺积分。
     */
    test_set_sample(
        &sample,
        0.1f, 0.0f, 0.0f,
        0.0f, 0.0f, -TEST_GRAVITY_M_S2,
        0.0f
    );

    update_ok = test_run_updates(
        &state,
        &sample,
        TEST_STEPS_1_SECOND,
        &output
    );

    test_check_true("Gyro update status", update_ok);

    if (!update_ok) {
        return;
    }

    test_print_attitude("Gyro integration", &output);

    test_check_close(
        "Gyro roll integration",
        output.roll_rad * TEST_RAD_TO_DEG,
        0.1f * TEST_RAD_TO_DEG,
        0.05f
    );

    test_check_close(
        "Gyro pitch unchanged",
        output.pitch_rad * TEST_RAD_TO_DEG,
        0.0f,
        0.05f
    );
}

/* ============================================================
 * 测试5：Mahony动态收敛方向
 *
 * 先在水平参考下仿真 1 秒建立水平姿态，随后保持陀螺为零，
 * 把加速度参考切换为右滚 30 度的重力投影，再仿真 10 秒。
 *
 * 期望 Mahony 修正项把姿态从 0 度逐渐拉回 +30 度
 * （容差放宽到 0.2 度，允许收敛过程残留的微小误差），
 * pitch 保持 0 度。验证修正方向正确、不会发散或反向。
 * ============================================================ */

static void test_dynamic_roll_convergence(void)
{
    attitude_state_t state;
    attitude_out_t output;
    filter_out_t sample;
    bool update_ok;

    float angle = 30.0f * TEST_DEG_TO_RAD;

    printf("\n===== Test 5: Dynamic roll convergence =====\n");

    if (!test_initialize_attitude(&state)) {
        g_test_failures++;
        return;
    }

    /* 先建立水平姿态。 */
    test_set_sample(
        &sample,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -TEST_GRAVITY_M_S2,
        1.0f
    );

    update_ok = test_run_updates(
        &state,
        &sample,
        TEST_STEPS_1_SECOND,
        &output
    );

    if (!update_ok) {
        g_test_failures++;
        return;
    }

    /*
     * 模拟重力参考突然对应右滚30度。
     * Mahony应从当前水平姿态逐渐向+30度收敛。
     */
    test_set_sample(
        &sample,
        0.0f, 0.0f, 0.0f,
        0.0f,
        -TEST_GRAVITY_M_S2 * sinf(angle),
        -TEST_GRAVITY_M_S2 * cosf(angle),
        1.0f
    );

    update_ok = test_run_updates(
        &state,
        &sample,
        TEST_STEPS_10_SECONDS,
        &output
    );

    test_check_true(
        "Dynamic convergence update status",
        update_ok
    );

    if (!update_ok) {
        return;
    }

    test_print_attitude("Dynamic roll", &output);

    test_check_close(
        "Dynamic roll convergence",
        output.roll_rad * TEST_RAD_TO_DEG,
        30.0f,
        0.2f
    );

    test_check_close(
        "Dynamic roll pitch",
        output.pitch_rad * TEST_RAD_TO_DEG,
        0.0f,
        0.2f
    );
}

/* ============================================================
 * 测试6：无效输入保护
 *
 * 先在水平参考下更新 100 步建立姿态并记录四元数，
 * 然后送入 valid = false 的样本。
 *
 * 期望：接口仍返回 OK，但输出标记无效，且内部姿态四元数
 * 保持不变（无效采样不得污染姿态状态）。
 * ============================================================ */

static void test_invalid_input_protection(void)
{
    attitude_state_t state;
    attitude_out_t output;
    filter_out_t sample;
    algo_quat_t quaternion_before;

    attitude_status_t status;

    printf("\n===== Test 6: Invalid input protection =====\n");

    if (!test_initialize_attitude(&state)) {
        g_test_failures++;
        return;
    }

    test_set_sample(
        &sample,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -TEST_GRAVITY_M_S2,
        1.0f
    );

    if (!test_run_updates(
            &state,
            &sample,
            100,
            &output)) {
        g_test_failures++;
        return;
    }

    /* 记录无效采样前的姿态四元数，用于对比。 */
    quaternion_before = state.current.q_bn;

    /* 构造无效样本。 */
    memset(&sample, 0, sizeof(sample));
    sample.valid = false;
    sample.gyro_valid = false;
    sample.accel_valid = false;

    status = attitude_update(
        &state,
        &sample,
        TEST_DT_S,
        &output
    );

    test_check_true(
        "Invalid sample returns OK",
        status == ATTITUDE_STATUS_OK
    );

    test_check_true(
        "Invalid sample output invalid",
        !output.valid
    );

    /* 无效样本不得改变姿态状态：四元数四个分量逐一对比。 */
    test_check_close(
        "Invalid sample preserves quaternion w",
        state.current.q_bn.w,
        quaternion_before.w,
        1.0e-6f
    );

    test_check_close(
        "Invalid sample preserves quaternion x",
        state.current.q_bn.x,
        quaternion_before.x,
        1.0e-6f
    );

    test_check_close(
        "Invalid sample preserves quaternion y",
        state.current.q_bn.y,
        quaternion_before.y,
        1.0e-6f
    );

    test_check_close(
        "Invalid sample preserves quaternion z",
        state.current.q_bn.z,
        quaternion_before.z,
        1.0e-6f
    );
}

/* ============================================================
 * 测试7：异常dt保护
 *
 * 分别送入 dt = 0 和 dt = NaN。
 * 期望均返回 ATTITUDE_STATUS_INVALID_DT，输出标记无效，
 * 不进行本次姿态递推。
 * ============================================================ */

static void test_invalid_dt_protection(void)
{
    attitude_state_t state;
    attitude_out_t output;
    filter_out_t sample;
    attitude_status_t status;

    printf("\n===== Test 7: Invalid dt protection =====\n");

    if (!test_initialize_attitude(&state)) {
        g_test_failures++;
        return;
    }

    test_set_sample(
        &sample,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -TEST_GRAVITY_M_S2,
        1.0f
    );

    status = attitude_update(
        &state,
        &sample,
        0.0f,
        &output
    );

    test_check_true(
        "Zero dt rejected",
        status == ATTITUDE_STATUS_INVALID_DT
    );

    test_check_true(
        "Zero dt output invalid",
        !output.valid
    );

    status = attitude_update(
        &state,
        &sample,
        NAN,
        &output
    );

    test_check_true(
        "NaN dt rejected",
        status == ATTITUDE_STATUS_INVALID_DT
    );

    test_check_true(
        "NaN dt output invalid",
        !output.valid
    );
}

/* ============================================================
 * 测试入口
 *
 * 依次运行全部用例；g_test_failures 为零时退出码为
 * EXIT_SUCCESS，否则打印失败数并返回 EXIT_FAILURE。
 * ============================================================ */

int main(void)
{
    printf("Mahony attitude unit tests\n");

    test_level_attitude();
    test_roll_30_degrees();
    test_pitch_30_degrees();
    test_gyro_integration();
    test_dynamic_roll_convergence();
    test_invalid_input_protection();
    test_invalid_dt_protection();

    printf("\n========================================\n");

    if (g_test_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return EXIT_SUCCESS;
    }

    printf("TESTS FAILED: %d\n", g_test_failures);
    return EXIT_FAILURE;
}
