#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "algo_attitude.h"

#define TEST_DT_S               0.0025f
#define TEST_GRAVITY_M_S2       9.80665f
#define TEST_RAD_TO_DEG         57.29577951308232f
#define TEST_DEG_TO_RAD         0.017453292519943295f

#define TEST_STEPS_1_SECOND     400
#define TEST_STEPS_10_SECONDS   4000

static int g_test_failures = 0;

/* ============================================================
 * 测试辅助函数
 * ============================================================ */

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

    test_check_true(
        "Level converged",
        output.converged
    );

    test_check_close(
        "Level quaternion norm",
        test_quaternion_norm(&output.q_bn),
        1.0f,
        1.0e-4f
    );
}

/* ============================================================
 * 测试2：右滚30度
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