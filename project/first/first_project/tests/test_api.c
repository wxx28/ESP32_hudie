/*
 * algo_api 端到端主机测试（不依赖 ESP32）
 *
 * 编译运行（在 first_project 目录下）：
 *   gcc -std=c11 -O2 -Wall -Icomponents/Middlewares/algo \
 *       tests/test_api.c components/Middlewares/algo/[a-z]*.c -lm -o tests/test_api && ./tests/test_api
 *
 * 覆盖：
 *   1. 默认配置（机构未确认）永远不能解锁；
 *   2. 直驱 + 机构确认后：自检 -> 待机 -> 解锁 -> 扑翼；
 *   3. 油门高时拨解锁不会起飞；上电时开关已在解锁位也不会起飞；
 *   4. 遥控中断：300 ms 降级，1 s 失控保护，幅度斜坡收小，禁止差动；
 *   5. 遥控恢复后必须先上锁再重新解锁；
 *   6. 姿态翻转 0.5 s 急停，上锁后可清除；
 *   7. 连杆逆解（四连杆）基本几何。
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "algo_api.h"
#include "algo_linkage.h"

#define DT 0.0025f
#define G  9.80665f

static int s_fail;
#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (!(cond)) {                                                     \
            printf("  FAIL: %s (line %d)\n", msg, __LINE__);               \
            s_fail++;                                                      \
        } else {                                                           \
            printf("  ok  : %s\n", msg);                                   \
        }                                                                  \
    } while (0)

static uint64_t s_now;
static uint64_t s_last_rc_us;
static algo_output_t s_out;

typedef struct {
    float thr, roll, pitch;
    int arm;
    int link;        /* 0 = 遥控帧停止到达 */
    float tilt_rad;  /* 机体滚转角，用来模拟翻转 */
} sim_t;

static void step(const sim_t *s, int n)
{
    for (int i = 0; i < n; ++i) {
        algo_input_t in;
        memset(&in, 0, sizeof(in));
        s_now += 2500;
        in.now_us = s_now;
        in.imu.timestamp_us = s_now;
        in.imu.valid = true;
        /* 机体滚转 tilt：静止比力 = R^T (0,0,-g) */
        in.imu.accel_m_s2.x = 0.0f;
        in.imu.accel_m_s2.y = -G * sinf(s->tilt_rad);
        in.imu.accel_m_s2.z = -G * cosf(s->tilt_rad);
        if (s->link) {
            s_last_rc_us = s_now;
        }
        uint32_t age = (uint32_t)((s_now - s_last_rc_us) / 1000u);
        in.rc.throttle = s->thr;
        in.rc.roll = s->roll;
        in.rc.pitch = s->pitch;
        in.rc.arm_request = s->arm;
        in.rc.stabilize_request = false;
        in.rc.age_ms = age;
        in.rc.valid = age < 100u;
        in.health.imu_ready = true;
        in.health.power_good = true;
        algo_status_t st = algo_api_step(&in, DT, &s_out);
        if (st != ALGO_STATUS_OK) {
            printf("  step status %d\n", st);
            s_fail++;
            return;
        }
    }
}

/* 运行 n 拍，返回左右舵机角的峰峰值 */
static void run_measure(const sim_t *s, int n, float *pp_left, float *pp_right)
{
    float lo[2] = {1e9f, 1e9f}, hi[2] = {-1e9f, -1e9f};
    for (int i = 0; i < n; ++i) {
        step(s, 1);
        for (int w = 0; w < 2; ++w) {
            if (s_out.servo.angle_deg[w] < lo[w]) lo[w] = s_out.servo.angle_deg[w];
            if (s_out.servo.angle_deg[w] > hi[w]) hi[w] = s_out.servo.angle_deg[w];
        }
    }
    *pp_left = hi[0] - lo[0];
    *pp_right = hi[1] - lo[1];
}

static const char *state_name(safety_state_t s)
{
    static const char *n[] = {"BOOT", "SELFTEST", "STANDBY", "ARMED", "DEGRADED", "FAILSAFE", "ESTOP"};
    return (unsigned)s < 7u ? n[s] : "?";
}

static algo_config_t bench_config(void)
{
    algo_config_t c = ALGO_CONFIG_DEFAULT;
    c.linkage.direct_drive = true;
    c.waveform.mechanical_limits_valid = true;
    c.mixer.lift_amplitude_gain_rad = ALGO_DEG_TO_RAD(40.0f);
    c.mixer.max_amplitude_rad[0] = c.mixer.max_amplitude_rad[1] = ALGO_DEG_TO_RAD(40.0f);
    c.mixer.roll_differential_gain_rad = ALGO_DEG_TO_RAD(8.0f);
    c.mixer.max_differential_rad = ALGO_DEG_TO_RAD(8.0f);
    c.waveform.min_wing_angle_rad[0] = c.waveform.min_wing_angle_rad[1] = ALGO_DEG_TO_RAD(-60.0f);
    c.waveform.max_wing_angle_rad[0] = c.waveform.max_wing_angle_rad[1] = ALGO_DEG_TO_RAD(60.0f);
    c.waveform.max_wing_rate_rad_s[0] = c.waveform.max_wing_rate_rad_s[1] = ALGO_DEG_TO_RAD(3000.0f);
    c.phase.min_frequency_hz = 2.0f;
    c.phase.max_frequency_hz = 6.0f;
    c.phase.default_frequency_hz = 2.0f;
    c.phase.slew_hz_per_s = 4.0f;
    c.linkage.side[1].servo_direction = -1.0f;
    return c;
}

static void test_default_never_arms(void)
{
    printf("[1] 默认配置（机构未确认）\n");
    algo_config_t c = ALGO_CONFIG_DEFAULT;
    CHECK(algo_api_init(&c) == ALGO_STATUS_OK, "默认配置可初始化");
    sim_t s = {.thr = 0.0f, .link = 1};
    step(&s, 1200);
    CHECK(s_out.safety.state == ALGO_SAFETY_STANDBY, "自检后进入 STANDBY");
    s.arm = 1;
    step(&s, 400);
    CHECK(s_out.safety.state == ALGO_SAFETY_STANDBY, "机构未确认时拨解锁仍停在 STANDBY");
    CHECK(!s_out.safety.allow_flap, "不允许扑翼");
}

static void test_flight_cycle(void)
{
    printf("[2] 直驱台架配置：解锁 -> 扑翼 -> 失联 -> 恢复\n");
    algo_config_t c = bench_config();
    CHECK(algo_api_init(&c) == ALGO_STATUS_OK, "台架配置可初始化");

    sim_t s = {.thr = 0.0f, .arm = 1, .link = 1};
    step(&s, 1200);
    CHECK(s_out.safety.state == ALGO_SAFETY_STANDBY, "上电时开关已在解锁位：不直接解锁");

    s.arm = 0;
    step(&s, 40);
    s.thr = 0.5f;
    s.arm = 1;
    step(&s, 40);
    CHECK(s_out.safety.state == ALGO_SAFETY_STANDBY, "油门 50% 时拨解锁：拒绝");

    s.arm = 0;
    s.thr = 0.0f;
    step(&s, 40);
    s.arm = 1;
    step(&s, 40);
    CHECK(s_out.safety.state == ALGO_SAFETY_ARMED, "油门低 + 解锁上升沿：ARMED");

    float pl, pr;
    s.thr = 0.8f;
    step(&s, 800);
    run_measure(&s, 800, &pl, &pr);
    printf("      油门 80%%：舵机峰峰 L=%.1f° R=%.1f°，频率 %.2f Hz\n", pl, pr, s_out.phase.f_hz);
    CHECK(pl > 40.0f && pr > 40.0f, "油门 80% 两翼都在扑动");
    CHECK(fabsf(pl - pr) < 1.0f, "不打杆时左右幅度一致");

    s.roll = 1.0f;
    run_measure(&s, 800, &pl, &pr);
    printf("      满右滚杆：L=%.1f° R=%.1f°\n", pl, pr);
    CHECK(pl > pr + 10.0f, "右滚：左翼幅度大于右翼");
    s.roll = 0.0f;
    run_measure(&s, 400, &pl, &pr);
    float pp_armed = pl;

    s.link = 0;
    step(&s, 160); /* 400 ms */
    CHECK(s_out.safety.state == ALGO_SAFETY_DEGRADED, "失联 400 ms：DEGRADED");
    step(&s, 280); /* 累计约 1.1 s */
    CHECK(s_out.safety.state == ALGO_SAFETY_FAILSAFE, "失联 1.1 s：FAILSAFE");
    step(&s, 1200); /* 斜坡 */
    run_measure(&s, 800, &pl, &pr);
    printf("      FAILSAFE 稳定后：L=%.1f° R=%.1f°（ARMED 时 %.1f°），幅度系数 %.2f\n", pl, pr, pp_armed,
           s_out.safety.amplitude_limit);
    CHECK(pl > 1.0f, "失控保护仍在小幅扑翼（缓降，不直接停翅）");
    CHECK(pl < pp_armed * 0.6f, "失控保护幅度明显收小");
    CHECK(s_out.safety.differential_limit == 0.0f, "失控保护禁止差动");

    s.link = 1;
    step(&s, 400);
    CHECK(s_out.safety.state == ALGO_SAFETY_FAILSAFE, "遥控恢复但开关仍在解锁：保持 FAILSAFE");
    s.arm = 0;
    step(&s, 400);
    CHECK(s_out.safety.state == ALGO_SAFETY_STANDBY, "上锁后回到 STANDBY");
    CHECK(!s_out.safety.allow_flap, "STANDBY 不扑翼");
}

static void test_upset_estop(void)
{
    printf("[3] 姿态翻转急停\n");
    algo_config_t c = bench_config();
    algo_api_init(&c);
    sim_t s = {.link = 1};
    step(&s, 1200);
    s.arm = 1;
    step(&s, 40);
    s.thr = 0.5f;
    step(&s, 200);
    CHECK(s_out.safety.state == ALGO_SAFETY_ARMED, "解锁飞行");
    s.tilt_rad = ALGO_DEG_TO_RAD(80.0f);
    step(&s, 2000); /* 5 s，让姿态跟上 */
    CHECK(s_out.safety.state == ALGO_SAFETY_ESTOP, "滚转 80° 持续：ESTOP");
    CHECK(s_out.safety.immediate_stop, "急停立即停翅");
    s.tilt_rad = 0.0f;
    s.arm = 0;
    s.thr = 0.0f;
    step(&s, 2000);
    CHECK(algo_api_clear_latched_faults() == ALGO_STATUS_OK, "上锁后可清除锁存故障");
    step(&s, 1200);
    CHECK(s_out.safety.state == ALGO_SAFETY_STANDBY, "清除后重新自检进入 STANDBY");
}

static void test_four_bar(void)
{
    printf("[4] 四连杆逆解\n");
    algo_linkage_config_t lc = ALGO_LINKAGE_CONFIG_DEFAULT;
    lc.enabled = true;
    for (int w = 0; w < 2; ++w) {
        lc.side[w].ground_length_mm = 20.0f;
        lc.side[w].input_length_mm = 5.0f;
        lc.side[w].coupler_length_mm = 28.0f;
        lc.side[w].output_length_mm = 20.0f;
        lc.side[w].ground_angle_rad = 0.0f;
        lc.side[w].wing_zero_rad = ALGO_DEG_TO_RAD(90.0f);
        lc.side[w].assembly_sign = 1.0f;
        lc.side[w].servo_direction = 1.0f;
    }
    linkage_state_t ls;
    CHECK(linkage_init(&ls, &lc) == LINKAGE_STATUS_OK, "曲柄摇杆几何可初始化");
    wing_target_t wt = {0};
    wt.valid = true;
    servo_target_t st;
    linkage_update(&ls, &wt, &st);
    CHECK(fabsf(st.angle_deg[0] - 90.0f) < 0.01f, "翼角 0 -> 舵机中位 90°");
    wt.angle_rad[0] = wt.angle_rad[1] = ALGO_DEG_TO_RAD(10.0f);
    linkage_update(&ls, &wt, &st);
    printf("      翼角 +10° -> 舵机 %.1f°（传动比约 3）\n", st.angle_deg[0] - 90.0f);
    CHECK(st.reachable[0] && fabsf(st.angle_deg[0] - 90.0f) > 25.0f, "传动比放大方向正确");
    wt.angle_rad[0] = wt.angle_rad[1] = ALGO_DEG_TO_RAD(80.0f);
    linkage_update(&ls, &wt, &st);
    CHECK(!st.valid || st.saturated[0], "超出机构行程：不可达或饱和");
}

int main(void)
{
    test_default_never_arms();
    test_flight_cycle();
    test_upset_estop();
    test_four_bar();
    printf(s_fail ? "\n%d 项失败\n" : "\n全部通过\n", s_fail);
    (void)state_name;
    return s_fail ? 1 : 0;
}
