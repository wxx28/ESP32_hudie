/*
 * 控制分配（混控）模块主机端单元测试
 *
 * 在 PC 上编译运行（不依赖 ESP32 硬件），验证 algo_mixer.c 的
 * 接口契约和映射关系。断言失败只计数不中断，跑完全部用例后
 * 通过进程退出码报告总体结果（0 表示全部通过）。
 *
 * 期望值推导所依据的默认配置（见 algo_config.h）：
 *   - 频率范围：3 ~ 10 Hz；
 *   - 升力幅度增益：15 deg/rad，左右翼最大幅度均为 15 deg；
 *   - 滚转差动增益：5 deg，最大单侧差动 5 deg；
 *   - 俯仰偏置增益：0.20，最大偏置 0.30；
 *   - 默认波形形状 shape_k = 0.25。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "algo_mixer.h"

/* 角度转弧度，与 ALGO_DEG_TO_RAD 保持一致，便于用 deg 书写期望值。 */
#define TEST_DEG_TO_RAD     0.017453292519943295f

/* 浮点比较容差：对 float 精度的弧度值足够宽松，对逻辑错误足够严格。 */
#define TEST_TOLERANCE      1.0e-5f

/* 累计失败断言数，main 据此决定退出码。 */
static int g_test_failures = 0;

/* ============================================================
 * 测试辅助函数
 * ============================================================ */

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

/* 接口返回状态断言。 */
static void test_check_status(
    const char *test_name,
    mixer_status_t actual,
    mixer_status_t expected
)
{
    if (actual == expected) {
        printf("[PASS] %s\n", test_name);
    } else {
        printf(
            "[FAIL] %s: actual = %d, expected = %d\n",
            test_name,
            (int)actual,
            (int)expected
        );

        g_test_failures++;
    }
}

/*
 * 填入一组默认控制输入：四通道全零、标记有效。
 * 各用例在此基础上只修改关注的通道。
 */
static void test_set_default_control(
    control_out_t *control
)
{
    memset(control, 0, sizeof(*control));

    control->lift = 0.0f;
    control->roll = 0.0f;
    control->pitch = 0.0f;
    control->yaw = 0.0f;
    control->valid = true;
}

/*
 * 填入一组默认安全指令：已解锁、允许扑翼、无输出限制。
 * 各用例在此基础上只修改关注的字段。
 */
static void test_set_default_safety(
    safety_directive_t *safety
)
{
    memset(safety, 0, sizeof(*safety));

    safety->state = ALGO_SAFETY_ARMED;
    safety->allow_flap = true;
    safety->allow_stabilization = true;
    safety->immediate_stop = false;

    safety->amplitude_limit = 1.0f;
    safety->differential_limit = 1.0f;
}

/*
 * 使用默认配置初始化混控模块。
 * 初始化失败时打印状态并返回 false，由调用者终止当前用例。
 */
static bool test_initialize_default(
    mixer_state_t *state
)
{
    algo_mixer_config_t mixer_config =
        ALGO_MIXER_CONFIG_DEFAULT;

    algo_phase_config_t phase_config =
        ALGO_PHASE_CONFIG_DEFAULT;

    mixer_status_t status = mixer_init(
        state,
        &mixer_config,
        &phase_config
    );

    if (status != MIXER_STATUS_OK) {
        printf(
            "mixer_init failed, status = %d\n",
            (int)status
        );

        return false;
    }

    return true;
}

/* ============================================================
 * 测试1：初始化与复位状态
 *
 * 验证初始化后的安全输出：幅度/偏置为零、频率取最小值、
 * 中位和形状取配置值、command_seq 为零、valid 为 false；
 * 以及 mixer_reset 在已初始化状态下可以正常调用。
 * ============================================================ */

static void test_initialization(void)
{
    mixer_state_t state;
    flap_wave_cmd_t output;
    mixer_status_t status;

    printf("\n===== Test 1: Initialization =====\n");

    if (!test_initialize_default(&state)) {
        g_test_failures++;
        return;
    }

    status = mixer_get(&state, &output);

    test_check_status(
        "Get initial command",
        status,
        MIXER_STATUS_OK
    );

    test_check_true(
        "Initial command invalid",
        !output.valid
    );

    /* 初始频率 = 默认配置最小频率 3 Hz。 */
    test_check_close(
        "Initial frequency",
        output.frequency_hz,
        3.0f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Initial left amplitude",
        output.amplitude_rad[ALGO_WING_LEFT],
        0.0f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Initial right amplitude",
        output.amplitude_rad[ALGO_WING_RIGHT],
        0.0f,
        TEST_TOLERANCE
    );

    /* 初始形状 = 默认配置 shape_k 0.25。 */
    test_check_close(
        "Initial shape",
        output.shape_k,
        0.25f,
        TEST_TOLERANCE
    );

    test_check_true(
        "Initial command sequence zero",
        output.command_seq == 0u
    );

    status = mixer_reset(&state);

    test_check_status(
        "Reset initialized mixer",
        status,
        MIXER_STATUS_OK
    );
}

/* ============================================================
 * 测试2：总升力到频率和幅度
 *
 * 线性映射关系：
 *   频率 = 3 + lift * (10 - 3) Hz
 *   基础幅度 = lift * 15 deg
 * 并验证超出 [0, 1] 的有限输入按饱和限幅处理。
 * ============================================================ */

static void test_lift_mapping(void)
{
    mixer_state_t state;
    control_out_t control;
    safety_directive_t safety;
    flap_wave_cmd_t output;
    mixer_status_t status;

    printf("\n===== Test 2: Lift mapping =====\n");

    if (!test_initialize_default(&state)) {
        g_test_failures++;
        return;
    }

    test_set_default_control(&control);
    test_set_default_safety(&safety);

    /* 零升力：最低频率、零幅度。 */
    control.lift = 0.0f;

    status = mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_status(
        "Zero lift update",
        status,
        MIXER_STATUS_OK
    );

    test_check_true(
        "Zero lift output valid",
        output.valid
    );

    test_check_close(
        "Zero lift frequency",
        output.frequency_hz,
        3.0f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Zero lift amplitude",
        output.amplitude_rad[ALGO_WING_LEFT],
        0.0f,
        TEST_TOLERANCE
    );

    /* 半升力：频率 3 + 0.5*7 = 6.25 Hz，幅度 0.5*15 = 7.5 度。 */
    control.lift = 0.5f;

    status = mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_status(
        "Half lift update",
        status,
        MIXER_STATUS_OK
    );

    test_check_close(
        "Half lift frequency",
        output.frequency_hz,
        6.50f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Half lift left amplitude",
        output.amplitude_rad[ALGO_WING_LEFT],
        7.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    /* 无滚转差动时，右翼幅度与左翼一致。 */
    test_check_close(
        "Half lift right amplitude",
        output.amplitude_rad[ALGO_WING_RIGHT],
        7.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    /* 满升力：频率 10 Hz，幅度 15 度。 */
    control.lift = 1.0f;

    status = mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_status(
        "Full lift update",
        status,
        MIXER_STATUS_OK
    );

    test_check_close(
        "Full lift frequency",
        output.frequency_hz,
        10.0f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Full lift left amplitude",
        output.amplitude_rad[ALGO_WING_LEFT],
        15.0f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    test_check_close(
        "Full lift right amplitude",
        output.amplitude_rad[ALGO_WING_RIGHT],
        15.0f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    /* 超出范围的有限输入应被限幅。 */
    control.lift = 1.5f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    /* lift = 1.5 按 lift = 1.0 饱和，幅度仍是 15 度。 */
    test_check_close(
        "Excess lift clamped",
        output.amplitude_rad[ALGO_WING_LEFT],
        15.0f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    control.lift = -0.5f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    /* 负升力按 lift = 0 饱和，幅度为零。 */
    test_check_close(
        "Negative lift clamped",
        output.amplitude_rad[ALGO_WING_LEFT],
        0.0f,
        TEST_TOLERANCE
    );
}

/* ============================================================
 * 测试3：滚转左右差动
 *
 * 在半升力（基础幅度 7.5 度）下验证：
 *   roll = +1：左 7.5+5 = 12.5 度，右 7.5-5 = 2.5 度（右滚）；
 *   roll = -1：左右互换；
 *   滚转不影响由升力决定的频率。
 * ============================================================ */

static void test_roll_differential(void)
{
    mixer_state_t state;
    control_out_t control;
    safety_directive_t safety;
    flap_wave_cmd_t output;

    printf("\n===== Test 3: Roll differential =====\n");

    if (!test_initialize_default(&state)) {
        g_test_failures++;
        return;
    }

    test_set_default_control(&control);
    test_set_default_safety(&safety);

    control.lift = 0.5f;
    control.roll = 1.0f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_close(
        "Positive roll left amplitude",
        output.amplitude_rad[ALGO_WING_LEFT],
        12.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    test_check_close(
        "Positive roll right amplitude",
        output.amplitude_rad[ALGO_WING_RIGHT],
        2.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    control.roll = -1.0f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_close(
        "Negative roll left amplitude",
        output.amplitude_rad[ALGO_WING_LEFT],
        2.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    test_check_close(
        "Negative roll right amplitude",
        output.amplitude_rad[ALGO_WING_RIGHT],
        12.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    /* 频率只由升力决定：半升力仍为 6.50 Hz。 */
    test_check_close(
        "Roll keeps half lift frequency",
        output.frequency_hz,
        6.50f,
        TEST_TOLERANCE
    );
}

/* ============================================================
 * 测试4：俯仰上下拍偏置
 *
 * 偏置 = pitch * 0.20，左右翼相同：
 *   pitch = +1 → +0.20，pitch = -1 → -0.20；
 * 并验证俯仰不影响左右翼幅度。
 * ============================================================ */

static void test_pitch_bias(void)
{
    mixer_state_t state;
    control_out_t control;
    safety_directive_t safety;
    flap_wave_cmd_t output;

    printf("\n===== Test 4: Pitch bias =====\n");

    if (!test_initialize_default(&state)) {
        g_test_failures++;
        return;
    }

    test_set_default_control(&control);
    test_set_default_safety(&safety);

    control.lift = 0.5f;
    control.pitch = 1.0f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_close(
        "Positive pitch left bias",
        output.up_down_bias[ALGO_WING_LEFT],
        0.20f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Positive pitch right bias",
        output.up_down_bias[ALGO_WING_RIGHT],
        0.20f,
        TEST_TOLERANCE
    );

    control.pitch = -1.0f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_close(
        "Negative pitch left bias",
        output.up_down_bias[ALGO_WING_LEFT],
        -0.20f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Negative pitch right bias",
        output.up_down_bias[ALGO_WING_RIGHT],
        -0.20f,
        TEST_TOLERANCE
    );

    /* 幅度只由升力和滚转决定：半升力仍为 7.5 度。 */
    test_check_close(
        "Pitch does not change left amplitude",
        output.amplitude_rad[ALGO_WING_LEFT],
        7.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    test_check_close(
        "Pitch does not change right amplitude",
        output.amplitude_rad[ALGO_WING_RIGHT],
        7.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );
}

/* ============================================================
 * 测试5：差动、幅度和偏置限幅
 *
 * 故意把映射增益调得比限幅值大，验证配置限幅真正生效：
 *   - 差动增益 10 deg，单侧差动上限 5 deg；
 *   - 俯仰增益 0.50，偏置上限 0.30；
 *   - 满升力 + 差动时左翼触及最大幅度 15 deg。
 * ============================================================ */

static void test_output_saturation(void)
{
    mixer_state_t state;
    algo_mixer_config_t mixer_config =
        ALGO_MIXER_CONFIG_DEFAULT;

    algo_phase_config_t phase_config =
        ALGO_PHASE_CONFIG_DEFAULT;

    control_out_t control;
    safety_directive_t safety;
    flap_wave_cmd_t output;
    mixer_status_t status;

    printf("\n===== Test 5: Output saturation =====\n");

    /*
     * 增大映射增益，确保测试真正触发配置限幅。
     */
    mixer_config.roll_differential_gain_rad =
        10.0f * TEST_DEG_TO_RAD;

    mixer_config.max_differential_rad =
        5.0f * TEST_DEG_TO_RAD;

    mixer_config.pitch_up_down_gain = 0.50f;
    mixer_config.max_up_down_bias = 0.30f;

    status = mixer_init(
        &state,
        &mixer_config,
        &phase_config
    );

    test_check_status(
        "Saturation test init",
        status,
        MIXER_STATUS_OK
    );

    if (status != MIXER_STATUS_OK) {
        return;
    }

    test_set_default_control(&control);
    test_set_default_safety(&safety);

    control.lift = 0.5f;
    control.roll = 1.0f;
    control.pitch = 1.0f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    /* 增益给出 10 度差动，被单侧上限压回 5 度：左 12.5，右 2.5。 */
    test_check_close(
        "Roll differential limited left",
        output.amplitude_rad[ALGO_WING_LEFT],
        12.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    test_check_close(
        "Roll differential limited right",
        output.amplitude_rad[ALGO_WING_RIGHT],
        2.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    /* 增益给出 0.50 偏置，被上限压回 0.30。 */
    test_check_close(
        "Pitch bias limited",
        output.up_down_bias[ALGO_WING_LEFT],
        0.30f,
        TEST_TOLERANCE
    );

    /* 满升力加滚转：左翼触发最大幅度限制。 */
    control.lift = 1.0f;
    control.pitch = 0.0f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    /* 基础 15 + 差动 5 = 20 度，被最大幅度压回 15 度。 */
    test_check_close(
        "Left amplitude saturated",
        output.amplitude_rad[ALGO_WING_LEFT],
        15.0f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    /* 右翼 15 - 5 = 10 度，不受左翼饱和影响。 */
    test_check_close(
        "Right amplitude after saturation",
        output.amplitude_rad[ALGO_WING_RIGHT],
        10.0f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );
}

/* ============================================================
 * 测试6：安全状态限制
 *
 * 验证安全指令对输出的三层作用：
 *   1. amplitude_limit 按比例收缩幅度上限；
 *   2. differential_limit 按比例收缩单侧差动上限；
 *   3. allow_flap = false 或 immediate_stop = true 时
 *      输出零幅度、零偏置、最小频率，但结构仍有效。
 * ============================================================ */

static void test_safety_limits(void)
{
    mixer_state_t state;
    control_out_t control;
    safety_directive_t safety;
    flap_wave_cmd_t output;
    mixer_status_t status;

    printf("\n===== Test 6: Safety limits =====\n");

    if (!test_initialize_default(&state)) {
        g_test_failures++;
        return;
    }

    test_set_default_control(&control);
    test_set_default_safety(&safety);

    /* 最大幅度限制为配置值的 50%。 */
    control.lift = 1.0f;
    safety.amplitude_limit = 0.5f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    /* 满升力基础幅度 15 度，被 15 * 0.5 = 7.5 度上限压住。 */
    test_check_close(
        "Safety limited left amplitude",
        output.amplitude_rad[ALGO_WING_LEFT],
        7.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    test_check_close(
        "Safety limited right amplitude",
        output.amplitude_rad[ALGO_WING_RIGHT],
        7.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    /* 差动限制为 20%，单侧最大差动由 5 度变为 1 度。 */
    test_set_default_safety(&safety);

    control.lift = 0.5f;
    control.roll = 1.0f;
    safety.differential_limit = 0.2f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    /* 基础 7.5 度 ± 1 度：左 8.5，右 6.5。 */
    test_check_close(
        "Safety differential left",
        output.amplitude_rad[ALGO_WING_LEFT],
        8.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    test_check_close(
        "Safety differential right",
        output.amplitude_rad[ALGO_WING_RIGHT],
        6.5f * TEST_DEG_TO_RAD,
        TEST_TOLERANCE
    );

    /* 禁止扑翼。 */
    test_set_default_safety(&safety);

    control.lift = 1.0f;
    control.roll = 1.0f;
    control.pitch = 1.0f;
    safety.allow_flap = false;

    status = mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_status(
        "Disarmed mixer returns OK",
        status,
        MIXER_STATUS_OK
    );

    test_check_true(
        "Disarmed output structurally valid",
        output.valid
    );

    test_check_close(
        "Disarmed left amplitude zero",
        output.amplitude_rad[ALGO_WING_LEFT],
        0.0f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Disarmed right amplitude zero",
        output.amplitude_rad[ALGO_WING_RIGHT],
        0.0f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Disarmed pitch bias zero",
        output.up_down_bias[ALGO_WING_LEFT],
        0.0f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Disarmed frequency minimum",
        output.frequency_hz,
        3.0f,
        TEST_TOLERANCE
    );

    /* 急停。 */
    safety.allow_flap = true;
    safety.immediate_stop = true;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_close(
        "Emergency stop left amplitude",
        output.amplitude_rad[ALGO_WING_LEFT],
        0.0f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Emergency stop right amplitude",
        output.amplitude_rad[ALGO_WING_RIGHT],
        0.0f,
        TEST_TOLERANCE
    );
}

/* ============================================================
 * 测试7：无效输入保护
 *
 * 覆盖三类异常输入：
 *   1. control->valid == false：忽略控制量，返回 OK，
 *      输出零幅度且 valid == false；
 *   2. 控制量或安全限制为 NaN/Inf：返回 INVALID_INPUT，
 *      输出安全参数；
 *   3. 空指针：返回 INVALID_ARG。
 * ============================================================ */

static void test_invalid_inputs(void)
{
    mixer_state_t state;
    control_out_t control;
    safety_directive_t safety;
    flap_wave_cmd_t output;
    mixer_status_t status;

    printf("\n===== Test 7: Invalid inputs =====\n");

    if (!test_initialize_default(&state)) {
        g_test_failures++;
        return;
    }

    test_set_default_control(&control);
    test_set_default_safety(&safety);

    /* 控制模块声明输出无效。 */
    control.valid = false;
    control.lift = NAN;

    status = mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_status(
        "Invalid control returns OK",
        status,
        MIXER_STATUS_OK
    );

    test_check_true(
        "Invalid control output invalid",
        !output.valid
    );

    /* NaN 控制量必须不被读取：幅度仍为零。 */
    test_check_close(
        "Invalid control amplitude zero",
        output.amplitude_rad[ALGO_WING_LEFT],
        0.0f,
        TEST_TOLERANCE
    );

    /* 有效控制状态下出现 NaN。 */
    test_set_default_control(&control);
    control.roll = NAN;

    status = mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_status(
        "NaN control rejected",
        status,
        MIXER_STATUS_INVALID_INPUT
    );

    test_check_true(
        "NaN control output invalid",
        !output.valid
    );

    /* 安全限制出现无穷大。 */
    test_set_default_control(&control);
    safety.amplitude_limit = INFINITY;

    status = mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    test_check_status(
        "Infinite safety limit rejected",
        status,
        MIXER_STATUS_INVALID_INPUT
    );

    test_check_true(
        "Infinite safety output invalid",
        !output.valid
    );

    /* 空指针保护。 */
    test_set_default_safety(&safety);

    status = mixer_update(
        &state,
        NULL,
        &safety,
        &output
    );

    test_check_status(
        "Null control rejected",
        status,
        MIXER_STATUS_INVALID_ARG
    );

    status = mixer_update(
        &state,
        &control,
        &safety,
        NULL
    );

    test_check_status(
        "Null output rejected",
        status,
        MIXER_STATUS_INVALID_ARG
    );
}

/* ============================================================
 * 测试8：非法配置保护
 *
 * 验证 mixer_init 拒绝四类非法配置：
 *   1. 启用第一版未实现的偏航混控；
 *   2. 最大幅度为零（必须为正）；
 *   3. 最小频率为零（必须为正）；
 *   4. 配置参数为 NaN。
 * 均应返回 MIXER_STATUS_INVALID_CONFIG。
 * ============================================================ */

static void test_invalid_configs(void)
{
    mixer_state_t state;
    algo_mixer_config_t mixer_config;
    algo_phase_config_t phase_config;
    mixer_status_t status;

    printf("\n===== Test 8: Invalid configs =====\n");

    /* 第一版不允许启用尚未实现的偏航混控。 */
    {
        algo_mixer_config_t local_mixer =
            ALGO_MIXER_CONFIG_DEFAULT;

        algo_phase_config_t local_phase =
            ALGO_PHASE_CONFIG_DEFAULT;

        local_mixer.yaw_mixer_enable = true;

        status = mixer_init(
            &state,
            &local_mixer,
            &local_phase
        );

        test_check_status(
            "Yaw mixer rejected",
            status,
            MIXER_STATUS_INVALID_CONFIG
        );
    }

    /* 最大幅度为零。 */
    mixer_config =
        (algo_mixer_config_t)ALGO_MIXER_CONFIG_DEFAULT;

    phase_config =
        (algo_phase_config_t)ALGO_PHASE_CONFIG_DEFAULT;

    mixer_config.max_amplitude_rad[ALGO_WING_LEFT] = 0.0f;

    status = mixer_init(
        &state,
        &mixer_config,
        &phase_config
    );

    test_check_status(
        "Zero maximum amplitude rejected",
        status,
        MIXER_STATUS_INVALID_CONFIG
    );

    /* 非法频率范围。 */
    mixer_config =
        (algo_mixer_config_t)ALGO_MIXER_CONFIG_DEFAULT;

    phase_config =
        (algo_phase_config_t)ALGO_PHASE_CONFIG_DEFAULT;

    phase_config.min_frequency_hz = 0.0f;

    status = mixer_init(
        &state,
        &mixer_config,
        &phase_config
    );

    test_check_status(
        "Zero minimum frequency rejected",
        status,
        MIXER_STATUS_INVALID_CONFIG
    );

    /* 配置包含 NaN。 */
    mixer_config =
        (algo_mixer_config_t)ALGO_MIXER_CONFIG_DEFAULT;

    phase_config =
        (algo_phase_config_t)ALGO_PHASE_CONFIG_DEFAULT;

    mixer_config.default_shape_k = NAN;

    status = mixer_init(
        &state,
        &mixer_config,
        &phase_config
    );

    test_check_status(
        "NaN shape rejected",
        status,
        MIXER_STATUS_INVALID_CONFIG
    );
}

/* ============================================================
 * 测试9：结果读取与复位
 *
 * 验证 mixer_get 返回的结果与最近一次 mixer_update 一致，
 * 且 mixer_reset 后结果恢复为安全状态（零幅度、valid 为 false）。
 * ============================================================ */

static void test_get_and_reset(void)
{
    mixer_state_t state;
    control_out_t control;
    safety_directive_t safety;
    flap_wave_cmd_t output;
    flap_wave_cmd_t saved_output;
    mixer_status_t status;

    printf("\n===== Test 9: Get and reset =====\n");

    if (!test_initialize_default(&state)) {
        g_test_failures++;
        return;
    }

    test_set_default_control(&control);
    test_set_default_safety(&safety);

    /* 任意一组非平凡输入，覆盖频率、幅度差动和俯仰偏置。 */
    control.lift = 0.5f;
    control.roll = 0.5f;
    control.pitch = -0.5f;

    mixer_update(
        &state,
        &control,
        &safety,
        &output
    );

    status = mixer_get(&state, &saved_output);

    test_check_status(
        "Get latest output",
        status,
        MIXER_STATUS_OK
    );

    test_check_true(
        "Saved output valid",
        saved_output.valid
    );

    /* mixer_get 不重算，应与 mixer_update 的输出逐项一致。 */
    test_check_close(
        "Saved frequency matches",
        saved_output.frequency_hz,
        output.frequency_hz,
        TEST_TOLERANCE
    );

    test_check_close(
        "Saved left amplitude matches",
        saved_output.amplitude_rad[ALGO_WING_LEFT],
        output.amplitude_rad[ALGO_WING_LEFT],
        TEST_TOLERANCE
    );

    test_check_close(
        "Saved pitch bias matches",
        saved_output.up_down_bias[ALGO_WING_LEFT],
        output.up_down_bias[ALGO_WING_LEFT],
        TEST_TOLERANCE
    );

    status = mixer_reset(&state);

    test_check_status(
        "Mixer reset",
        status,
        MIXER_STATUS_OK
    );

    status = mixer_get(&state, &saved_output);

    test_check_status(
        "Get after reset",
        status,
        MIXER_STATUS_OK
    );

    /* 复位后回到安全状态。 */
    test_check_true(
        "Reset output invalid",
        !saved_output.valid
    );

    test_check_close(
        "Reset left amplitude zero",
        saved_output.amplitude_rad[ALGO_WING_LEFT],
        0.0f,
        TEST_TOLERANCE
    );

    test_check_close(
        "Reset right amplitude zero",
        saved_output.amplitude_rad[ALGO_WING_RIGHT],
        0.0f,
        TEST_TOLERANCE
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
    printf("Mixer unit tests\n");

    test_initialization();
    test_lift_mapping();
    test_roll_differential();
    test_pitch_bias();
    test_output_saturation();
    test_safety_limits();
    test_invalid_inputs();
    test_invalid_configs();
    test_get_and_reset();

    printf("\n========================================\n");

    if (g_test_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return EXIT_SUCCESS;
    }

    printf("TESTS FAILED: %d\n", g_test_failures);
    return EXIT_FAILURE;
}
