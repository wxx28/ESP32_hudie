/*
 * 仿生蝴蝶飞行仿真内核（PC 端）
 *
 * 直接链接仓库里的真实代码：
  *   Middlewares/algo/ 全部 .c   滤波 / Mahony / 串级 PID / 混控 / 相位 / 波形 / 连杆 / 安全 / 顶层
 *   flight_control/fc_output.c  安全输出协调（舵机写入换成仿真桩）
 *   flight_control/fc_config.c  V1 板参数（sdkconfig 用 stubs/sdkconfig.h）
 *   common/hudie_link/hudie_protocol.c  遥控帧 / 遥测帧 + CRC
 *
 * 仿真部分（本文件）：
 *   - 遥控器：按 rc_app.c 的逻辑生成 50 Hz 控制帧（解锁保护、模式、微调、校准请求）；
 *   - 无线信道：可断开、可设丢包率，飞控侧按 fc_link.c 的规则收包；
 *   - 飞控任务：按 flight_control.c 的 tick() 调度 400 Hz，含上电 1 s 陀螺校准；
 *   - 舵机：一阶滞后 + 600°/s 速度上限；
 *   - 机体：6 自由度刚体，扑翼升力按准稳态 L = c·ω²（ω 为翼角速度），
 *     升力中心随翼角前后移动（产生俯仰力矩）、左右升力差产生滚转力矩，
 *     升力作用点高于重心（摆锤稳定），线性气动阻尼；
 *   - IMU：真实角速度 + 零偏 + 噪声，比力 = Rᵀ(a - g) + 噪声；扑翼振动自然出现在信号里；
 *   - 电池：1S 500 mAh，电流随升力变化。
 *
 * 物理模型是"定性正确"的简化模型，参数只用于看懂控制链路的行为，不代表真机数值。
 *
 * 与界面的通信：stdin 一行一条命令，每条 "run N" 推进 N 个 2.5 ms 节拍后输出一行 JSON。
 */
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "algo_api.h"
#include "fc_config.h"
#include "fc_output.h"
#include "hudie_protocol.h"
#include "servo.h"

/* ======================= 常量 ======================= */
#define FC_DT        0.0025
#define PHYS_SUB     4
#define PHYS_DT      (FC_DT / PHYS_SUB)
#define D2R          0.017453292519943295
#define R2D          57.29577951308232
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 机体 */
static const double MASS = 0.075, GRAV = 9.80665;
static const double IXX = 1.6e-4, IYY = 1.2e-4, IZZ = 2.4e-4;
static const double B_ROLL = 4.0e-3, B_PITCH = 3.0e-3, B_YAW = 3.0e-3;
static const double LIFT_Y = 0.045;   /* 单侧升力中心到机体中线 (m) */
static const double LIFT_Z = 0.012;   /* 升力中心高于重心 (m)：摆锤稳定 */
/* 翼水平时升力中心位置：与前倾升力的力矩配平（LIFT_Z·tanγ），再略微偏后一点点 */
static const double LIFT_X0 = 0.00075;
static const double LIFT_KX = 0.030;  /* 翼角每 1 rad 升力中心前后移动 (m)，翼越低越靠前 */
static const double LIFT_TILT = 4.0 * D2R;  /* 升力相对机体竖直前倾，产生前飞分量 */
static const double DRAG = 0.15;      /* 线性阻力 N/(m/s) */
static const double DRAG_Z = 0.030;   /* 阻力作用点高于重心 (m)：倾斜后侧滑产生回正力矩（摆锤稳定） */
static const double WING_LIMIT = 70.0 * D2R;
static const double WING_INERTIA = 2.0e-5; /* 单翼绕铰链转动惯量 kg·m²（约 2 g、回转半径 0.1 m） */

/* 舵机 */
static const double SERVO_TAU = 0.006, SERVO_VMAX = 600.0; /* s, deg/s */

/* 电池 */
static const double BAT_CAP_AH = 0.5, BAT_R = 0.08;

/* ======================= 小工具 ======================= */
typedef struct { double x, y, z; } v3;
typedef struct { double w, x, y, z; } quat;

static v3 v3_add(v3 a, v3 b) { return (v3){a.x + b.x, a.y + b.y, a.z + b.z}; }
static v3 v3_scale(v3 a, double s) { return (v3){a.x * s, a.y * s, a.z * s}; }
static v3 v3_cross(v3 a, v3 b) { return (v3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

static quat q_mul(quat a, quat b)
{
    return (quat){a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                  a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
static quat q_norm(quat q)
{
    double n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return (quat){q.w / n, q.x / n, q.y / n, q.z / n};
}
/* 机体 -> NED */
static v3 q_rot(quat q, v3 v)
{
    quat p = {0, v.x, v.y, v.z}, qc = {q.w, -q.x, -q.y, -q.z};
    quat r = q_mul(q_mul(q, p), qc);
    return (v3){r.x, r.y, r.z};
}
static v3 q_rot_inv(quat q, v3 v) { return q_rot((quat){q.w, -q.x, -q.y, -q.z}, v); }
static void q_euler(quat q, double *roll, double *pitch, double *yaw)
{
    *roll = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    double s = 2 * (q.w * q.y - q.z * q.x);
    *pitch = asin(s > 1 ? 1 : (s < -1 ? -1 : s));
    *yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
static quat q_from_euler(double r, double p, double y)
{
    double cr = cos(r / 2), sr = sin(r / 2), cp = cos(p / 2), sp = sin(p / 2), cy = cos(y / 2), sy = sin(y / 2);
    return (quat){cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                  cr * cp * sy - sr * sp * cy};
}

static uint64_t s_rng = 0x2545F4914F6CDD1Dull;
static double urand(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 7;
    s_rng ^= s_rng << 17;
    return (double)(s_rng >> 11) / 9007199254740992.0;
}
static double nrand(void)
{
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) {
        u1 = 1e-12;
    }
    return sqrt(-2 * log(u1)) * cos(2 * M_PI * u2);
}
static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ======================= 消息队列（输出给界面） ======================= */
static char s_msgs[8][160];
static int s_msg_n;
static void msg(const char *fmt, ...)
{
    if (s_msg_n >= 8) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_msgs[s_msg_n++], sizeof(s_msgs[0]), fmt, ap);
    va_end(ap);
}

/* ======================= 舵机桩（替代 BSP/SERVO） ======================= */
static float s_servo_cmd[2] = {90, 90};
esp_err_t servo_init(void) { return ESP_OK; }
esp_err_t servo_set_angle(servo_id_t id, float a)
{
    if (id >= SERVO_COUNT || !isfinite(a)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_servo_cmd[id] = a < 0 ? 0 : (a > 180 ? 180 : a);
    return ESP_OK;
}
esp_err_t servo_set_pulse_us(servo_id_t id, uint32_t us) { return servo_set_angle(id, ((float)us - 500.0f) * 0.09f); }

/* ======================= 世界状态 ======================= */
static struct {
    double t;
    v3 p, v;          /* NED */
    quat q;
    v3 w;             /* 机体角速度 */
    double servo[2];  /* 舵机实际角 deg */
    double wing[2], wing_prev[2], wing_rate[2], wing_acc[2]; /* rad，上为正 */
    double lift[2];
    bool on_ground, crashed;
    v3 gust_torque;
    double gust_left;
    v3 wind;
    v3 gyro_bias;
    double soc, current, vbat, lift_avg;
} W;

/* 故障注入 / 界面设置 */
static struct {
    bool link;
    double loss;       /* 0..1 */
    bool imu_fail;
    bool right_reversed;
    double drain;      /* 电池消耗倍率 */
    bool mech;
    bool stab;
    float angle_p, rate_p, rate_i, rate_d;
    float fs_amp;      /* 失控保护幅度系数，<0 表示用默认配置 */
    float dg_amp;      /* 降级幅度系数，<0 表示用默认配置 */
} U = {.fs_amp = -1, .dg_amp = -1, .link = true, .drain = 1.0, .mech = true, .stab = false, .angle_p = 3.0f, .rate_p = 0.30f, .rate_i = 0.05f,
       .rate_d = 0.002f};

/* ======================= 遥控器（对应 rc_app.c） ======================= */
static struct {
    int16_t thr, roll, pitch, yaw;  /* 摇杆 */
    bool armed, manual;
    int16_t trim_roll, trim_pitch;
    double calibrate_until;
    uint8_t seq;
    double tele_time;
    tele_frame_t tele;
    uint32_t vbat_mv;
} R = {.manual = true, .tele_time = -1};

/* ======================= 飞控（对应 flight_control.c / fc_link.c / fc_input.c） ======================= */
static struct {
    algo_config_t cfg;
    bool booting;
    double boot_left;
    v3 bias_sum;
    int bias_n;
    v3 bias_est;
    bool calib_mode;
    /* fc_link */
    rc_frame_t frame;
    double rx_time;   /* <0 表示从未收到 */
    bool have_seq;
    uint8_t last_seq;
    uint32_t win_exp, win_got;
    uint8_t loss_pct;
    /* fc_input */
    bool prev_cal_flag, cal_request;
    float bat_filt;
    uint32_t bat_div;
    /* tick */
    uint32_t cycles;
    algo_output_t out;
    algo_input_t in;
    algo_status_t last_status;
    safety_state_t prev_state;
    uint8_t tele_seq;
} F;

static const double LIFT_C_REF_THR = 0.65; /* 设计：理想条件下 65% 油门悬停 */
static double s_lift_c;

static void compute_lift_coeff(void)
{
    const algo_config_t *c = &F.cfg;
    double amp = c->mixer.lift_amplitude_gain_rad * LIFT_C_REF_THR;
    double f = c->phase.min_frequency_hz + LIFT_C_REF_THR * (c->phase.max_frequency_hz - c->phase.min_frequency_hz);
    double w2 = pow(amp * 2 * M_PI * f, 2) / 2.0;
    /* 0.96 ≈ 扑动中翼面倾斜导致的竖直分量平均损失 */
    s_lift_c = MASS * GRAV / cos(LIFT_TILT) / 0.96 / (2.0 * w2);
}

static void fc_boot(void)
{
    fc_config_v1(&F.cfg);
    F.cfg.waveform.mechanical_limits_valid = U.mech;
    for (int a = 0; a < 2; ++a) {
        F.cfg.control.angle_pid[a].kp = U.stab ? U.angle_p : 0;
        F.cfg.control.rate_pid[a].kp = U.stab ? U.rate_p : 0;
        F.cfg.control.rate_pid[a].ki = U.stab ? U.rate_i : 0;
        F.cfg.control.rate_pid[a].kd = U.stab ? U.rate_d : 0;
        F.cfg.control.angle_pid[a].ki = 0;
        F.cfg.control.angle_pid[a].kd = 0;
    }
    if (U.fs_amp >= 0) {
        F.cfg.safety.failsafe_amplitude_limit = U.fs_amp;
    }
    if (U.dg_amp >= 0) {
        F.cfg.safety.degraded_amplitude_limit = U.dg_amp;
    }
    if (algo_api_config_validate(&F.cfg) != ALGO_STATUS_OK) {
        msg("参数非法，飞控拒绝启动");
    }
    fc_output_init(&F.cfg.linkage);
    F.booting = true;
    F.boot_left = 1.0;  /* fc_input_init 里的 1 s 陀螺校准 */
    F.bias_sum = (v3){0, 0, 0};
    F.bias_n = 0;
    F.rx_time = -1;
    F.have_seq = false;
    F.cycles = 0;
    F.prev_state = ALGO_SAFETY_BOOT;
    memset(&F.out, 0, sizeof(F.out));
    compute_lift_coeff();
}

static void world_reset(void)
{
    memset(&W, 0, sizeof(W));
    W.q = (quat){1, 0, 0, 0};
    W.servo[0] = W.servo[1] = 90;
    W.on_ground = true;
    W.soc = 1.0;
    W.vbat = 4.15;
    W.gyro_bias = (v3){0.02 * (urand() - 0.5), 0.02 * (urand() - 0.5), 0.02 * (urand() - 0.5)};
    R.armed = false;
    R.tele_time = -1;
    R.trim_roll = R.trim_pitch = 0;
}

/* ---------- 遥控器按键（rc_app.c handle_keys） ---------- */
static void rc_key(const char *k)
{
    if (strcmp(k, "stop") == 0) {
        if (R.armed) {
            R.armed = false;
            msg("遥控器：上锁");
        } else if (R.thr > 50) {
            msg("遥控器：油门没拉到底（%d%%），拒绝解锁", R.thr / 10);
        } else {
            R.armed = true;
            msg("遥控器：解锁");
        }
    } else if (strcmp(k, "mode") == 0) {
        R.manual = !R.manual;
        msg("遥控器：模式 %s", R.manual ? "手动" : "增稳（飞控未整定时退回手动）");
    } else if (strcmp(k, "calib") == 0) {
        if (!R.armed) {
            R.calibrate_until = W.t + 1.0;
            msg("遥控器：请求飞控陀螺校准");
        }
    } else if (strcmp(k, "trim_l") == 0) {
        R.trim_roll = (int16_t)clampd(R.trim_roll - 10, -200, 200);
    } else if (strcmp(k, "trim_r") == 0) {
        R.trim_roll = (int16_t)clampd(R.trim_roll + 10, -200, 200);
    } else if (strcmp(k, "trim_u") == 0) {
        R.trim_pitch = (int16_t)clampd(R.trim_pitch + 10, -200, 200);
    } else if (strcmp(k, "trim_d") == 0) {
        R.trim_pitch = (int16_t)clampd(R.trim_pitch - 10, -200, 200);
    } else if (strcmp(k, "trim_reset") == 0) {
        R.trim_roll = R.trim_pitch = 0;
    }
}

/* ---------- 无线信道 ---------- */
static bool channel_pass(void) { return U.link && urand() >= U.loss; }

/* fc_link.c on_recv（控制帧部分） */
static void fc_on_rc(const rc_frame_t *pkt)
{
    if (!hudie_proto_check(pkt, sizeof(*pkt), sizeof(rc_frame_t), TYPE_RC)) {
        return;
    }
    if (F.have_seq) {
        uint8_t gap = (uint8_t)(pkt->seq - F.last_seq);
        if (gap == 0 || gap > 100) {
            return;
        }
        F.win_exp += gap;
    } else {
        F.win_exp += 1;
        F.have_seq = true;
    }
    F.last_seq = pkt->seq;
    F.win_got++;
    if (F.win_exp >= 50) {
        F.loss_pct = (uint8_t)(100 - F.win_got * 100 / F.win_exp);
        F.win_exp = F.win_got = 0;
    }
    F.frame = *pkt;
    F.rx_time = W.t;
}

/* rc_app.c 发送部分（50 Hz） */
static void rc_send(void)
{
    rc_frame_t f = {0};
    f.seq = R.seq++;
    f.flags = (R.armed ? RC_FLAG_ARMED : 0) | (R.manual ? RC_FLAG_MANUAL : 0) |
              (W.t < R.calibrate_until ? RC_FLAG_CALIBRATE : 0);
    f.ch[RC_CH_THROTTLE] = R.thr;
    f.ch[RC_CH_ROLL] = (int16_t)clampd(R.roll + R.trim_roll, -1000, 1000);
    f.ch[RC_CH_PITCH] = (int16_t)clampd(R.pitch + R.trim_pitch, -1000, 1000);
    f.ch[RC_CH_YAW] = R.yaw;
    f.ch[RC_CH_MODE] = R.manual ? 1000 : 0;
    f.ch[RC_CH_ARM] = R.armed ? 1000 : 0;
    hudie_proto_finalize(&f, sizeof(f), TYPE_RC);
    if (channel_pass()) {
        fc_on_rc(&f);
    }
}

/* fc_link_send_telemetry -> 遥控器 on_recv */
static void fc_send_tele(void)
{
    tele_frame_t t = {0};
    t.seq = F.tele_seq++;
    switch (F.out.safety.state) {
    case ALGO_SAFETY_STANDBY: t.state = TELE_STATE_STANDBY; break;
    case ALGO_SAFETY_ARMED:
    case ALGO_SAFETY_DEGRADED: t.state = TELE_STATE_FLYING; break;
    case ALGO_SAFETY_FAILSAFE: t.state = TELE_STATE_FAILSAFE; break;
    case ALGO_SAFETY_ESTOP: t.state = TELE_STATE_FAULT; break;
    default: t.state = TELE_STATE_SELFTEST; break;
    }
    t.roll = (int16_t)(F.out.attitude.roll_rad * 572.96f);
    t.pitch = (int16_t)(F.out.attitude.pitch_rad * 572.96f);
    t.vbat_mv = (uint16_t)F.bat_filt;
    t.loss_pct = F.loss_pct;
    hudie_proto_finalize(&t, sizeof(t), TYPE_TELEM);
    if (channel_pass() && hudie_proto_check(&t, sizeof(t), sizeof(tele_frame_t), TYPE_TELEM)) {
        R.tele = t;
        R.tele_time = W.t;
    }
}

/* ---------- IMU 采样（BSP/mpu6500 输出：已减去估计零偏） ---------- */
static v3 s_accel_world; /* 最近一次物理步的线加速度（NED） */

static void imu_sample(algo_imu_sample_t *s, v3 *gyro_raw)
{
    v3 g = v3_add(W.w, W.gyro_bias);
    g = v3_add(g, (v3){0.004 * nrand(), 0.004 * nrand(), 0.004 * nrand()});
    *gyro_raw = g;
    v3 sf = q_rot_inv(W.q, v3_add(s_accel_world, (v3){0, 0, -GRAV}));
    sf = v3_add(sf, (v3){0.06 * nrand(), 0.06 * nrand(), 0.06 * nrand()});
    s->timestamp_us = (uint64_t)(W.t * 1e6);
    s->gyro_rad_s.x = (float)(g.x - F.bias_est.x);
    s->gyro_rad_s.y = (float)(g.y - F.bias_est.y);
    s->gyro_rad_s.z = (float)(g.z - F.bias_est.z);
    s->accel_m_s2.x = (float)sf.x;
    s->accel_m_s2.y = (float)sf.y;
    s->accel_m_s2.z = (float)sf.z;
    s->valid = !U.imu_fail;
}

static bool is_flying(safety_state_t s)
{
    return s == ALGO_SAFETY_ARMED || s == ALGO_SAFETY_DEGRADED || s == ALGO_SAFETY_FAILSAFE;
}

static const char *state_cn(safety_state_t s)
{
    static const char *n[] = {"启动", "自检", "待机", "已解锁", "降级", "失控保护", "急停"};
    return (unsigned)s < 7u ? n[s] : "?";
}

/* 飞控一拍（flight_control.c tick + fc_input_collect） */
static void fc_tick(void)
{
    v3 graw;
    algo_imu_sample_t imu;
    imu_sample(&imu, &graw);

    /* 电池 ADC 50 Hz，fc_input 的一阶低通 */
    if ((F.bat_div++ % 8u) == 0u) {
        float mv = (float)(W.vbat * 1000.0 + 8.0 * nrand());
        F.bat_filt = (F.bat_filt <= 0) ? mv : F.bat_filt + 0.04f * (mv - F.bat_filt);
    }

    if (F.booting || F.calib_mode) {
        /* 陀螺校准：板子静止时取平均 */
        fc_output_write_safe();
        if (!U.imu_fail) {
            F.bias_sum = v3_add(F.bias_sum, graw);
            F.bias_n++;
        }
        F.boot_left -= FC_DT;
        if (F.boot_left <= 0) {
            if (F.bias_n > 0) {
                F.bias_est = v3_scale(F.bias_sum, 1.0 / F.bias_n);
            }
            if (F.booting) {
                algo_api_init(&F.cfg);
                msg("飞控：上电自检完成，陀螺零偏 %.2f %.2f %.2f °/s", F.bias_est.x * R2D, F.bias_est.y * R2D,
                    F.bias_est.z * R2D);
            } else {
                algo_api_reset();
                msg("飞控：陀螺重新校准完成");
            }
            F.booting = F.calib_mode = false;
        }
        return;
    }

    algo_input_t in;
    memset(&in, 0, sizeof(in));
    in.now_us = (uint64_t)(W.t * 1e6);
    in.imu = imu;

    if (F.rx_time < 0) {
        in.rc.age_ms = UINT32_MAX;
        in.rc.valid = false;
    } else {
        double age = (W.t - F.rx_time) * 1000.0;
        in.rc.age_ms = age > 4e9 ? UINT32_MAX : (uint32_t)age;
        in.rc.valid = in.rc.age_ms < 100u;
        const rc_frame_t *f = &F.frame;
        in.rc.throttle = (float)clampd(f->ch[RC_CH_THROTTLE] / 1000.0, 0, 1);
        in.rc.roll = (float)clampd(f->ch[RC_CH_ROLL] / 1000.0, -1, 1);
        in.rc.pitch = (float)clampd(f->ch[RC_CH_PITCH] / 1000.0, -1, 1);
        in.rc.yaw = (float)clampd(f->ch[RC_CH_YAW] / 1000.0, -1, 1);
        in.rc.arm_request = (f->flags & RC_FLAG_ARMED) != 0;
        in.rc.stabilize_request = (f->flags & RC_FLAG_MANUAL) == 0;
        bool cal = in.rc.valid && (f->flags & RC_FLAG_CALIBRATE);
        if (cal && !F.prev_cal_flag) {
            F.cal_request = true;
        }
        F.prev_cal_flag = cal;
    }
    in.health.imu_ready = !U.imu_fail;
    in.health.power_good = F.bat_filt >= 3400.0f;

    algo_output_t out;
    algo_status_t st = algo_api_step(&in, (float)FC_DT, &out);
    F.last_status = st;
    if (st != ALGO_STATUS_OK) {
        fc_output_write_safe();
    } else {
        fc_output_write(&out);
    }
    F.in = in;
    F.out = out;

    if (out.safety.state != F.prev_state) {
        msg("飞控：%s → %s", state_cn(F.prev_state), state_cn(out.safety.state));
        F.prev_state = out.safety.state;
    }

    F.cycles++;
    if (F.cycles % 20u == 0u) {
        fc_send_tele();
    }
    if (F.cal_request) {
        F.cal_request = false;
        if (!is_flying(out.safety.state)) {
            F.calib_mode = true;
            F.boot_left = 1.0;
            F.bias_sum = (v3){0, 0, 0};
            F.bias_n = 0;
        }
    }
}

/* ======================= 物理 ======================= */
static void physics_step(void)
{
    const double dt = PHYS_DT;
    const algo_linkage_config_t *lk = &F.cfg.linkage;

    /* 舵机 -> 翼角（直驱；右舵机装反时机构方向和配置不一致） */
    for (int i = 0; i < 2; ++i) {
        double err = s_servo_cmd[i] - W.servo[i];
        double rate = clampd(err / SERVO_TAU, -SERVO_VMAX, SERVO_VMAX);
        W.servo[i] += rate * dt;
        double dir = lk->side[i].servo_direction;
        if (i == 1 && U.right_reversed) {
            dir = -dir;
        }
        W.wing_prev[i] = W.wing[i];
        W.wing[i] = clampd((W.servo[i] - lk->side[i].servo_neutral_deg) / dir * D2R, -WING_LIMIT, WING_LIMIT);
        double wr = (W.wing[i] - W.wing_prev[i]) / dt;
        const double prev_rate = W.wing_rate[i];
        W.wing_rate[i] += 0.5 * (wr - W.wing_rate[i]); /* 去掉离散噪声 */
        W.wing_acc[i] = (W.wing_rate[i] - prev_rate) / dt;
    }

    /* 升力与力矩（机体系 FRD） */
    v3 Fb = {0, 0, 0}, Tb = {0, 0, 0};
    double air = 1.0;
    for (int i = 0; i < 2; ++i) {
        double L = s_lift_c * W.wing_rate[i] * W.wing_rate[i] * air;
        W.lift[i] = L;
        double side = (i == 0) ? -1.0 : 1.0;
        /* 升力垂直于翼面：翼抬起时向机身内侧倾斜（两翼对称时侧向分量抵消） */
        const double cw = cos(W.wing[i]), sw = sin(W.wing[i]);
        v3 f = {L * sin(LIFT_TILT), -side * L * sw * cos(LIFT_TILT), -L * cw * cos(LIFT_TILT)};
        v3 r = {LIFT_X0 - LIFT_KX * sin(W.wing[i]), side * LIFT_Y * cos(W.wing[i]), -LIFT_Z};
        Fb = v3_add(Fb, f);
        Tb = v3_add(Tb, v3_cross(r, f));
    }
    /* 翼的惯性反作用：左翼抬起 = 绕 +X 转，右翼抬起 = 绕 -X 转；对称扑动时抵消 */
    Tb.x -= WING_INERTIA * (W.wing_acc[0] - W.wing_acc[1]);
    Tb.x -= B_ROLL * W.w.x;
    Tb.y -= B_PITCH * W.w.y;
    Tb.z -= B_YAW * W.w.z;
    if (W.gust_left > 0) {
        Tb = v3_add(Tb, W.gust_torque);
        W.gust_left -= dt;
    }

    /* 平动 */
    v3 vrel = {W.v.x - W.wind.x, W.v.y - W.wind.y, W.v.z - W.wind.z};
    {
        /* 阻力作用在翼面（高于重心）：侧滑时产生回正力矩 */
        v3 drag_b = q_rot_inv(W.q, v3_scale(vrel, -DRAG));
        Tb = v3_add(Tb, v3_cross((v3){0, 0, -DRAG_Z}, drag_b));
    }
    v3 Fn = q_rot(W.q, Fb);
    v3 a = {Fn.x / MASS - DRAG * vrel.x / MASS, Fn.y / MASS - DRAG * vrel.y / MASS,
            Fn.z / MASS + GRAV - DRAG * vrel.z / MASS};

    /* 着地：升力不足以托起时地面承重，机身不转 */
    bool ground = (W.p.z >= 0.0) && (a.z >= 0.0 || W.v.z >= 0.0);
    if (ground) {
        if (!W.on_ground) {
            double r, p, y;
            q_euler(W.q, &r, &p, &y);
            if (W.v.z > 2.5 || fabs(r) > 60 * D2R || fabs(p) > 60 * D2R) {
                if (!W.crashed) {
                    msg("仿真：坠地！下落速度 %.1f m/s", W.v.z);
                }
                W.crashed = true;
            }
        }
        W.on_ground = true;
        W.p.z = 0;
        if (W.v.z > 0) {
            W.v.z = 0;
        }
        if (a.z > 0) {
            a.z = 0;
        }
        W.v.x *= 0.97;
        W.v.y *= 0.97;
        a.x = a.y = 0;
    } else {
        W.on_ground = false;
    }
    s_accel_world = a;
    W.v = v3_add(W.v, v3_scale(a, dt));
    W.p = v3_add(W.p, v3_scale(W.v, dt));

    /* 转动 */
    if (W.on_ground) {
        /* 落在地上：机身被地面放平 */
        double r, p, y;
        q_euler(W.q, &r, &p, &y);
        W.q = q_from_euler(r * 0.995, p * 0.995, y);
        W.w.x *= 0.9;
        W.w.y *= 0.9;
        W.w.z *= 0.95;
        W.w.z += Tb.z / IZZ * dt;
    } else {
        v3 Iw = {IXX * W.w.x, IYY * W.w.y, IZZ * W.w.z};
        v3 gyro = v3_cross(W.w, Iw);
        W.w.x += (Tb.x - gyro.x) / IXX * dt;
        W.w.y += (Tb.y - gyro.y) / IYY * dt;
        W.w.z += (Tb.z - gyro.z) / IZZ * dt;
    }
    quat dq = q_mul(W.q, (quat){0, W.w.x, W.w.y, W.w.z});
    W.q = q_norm((quat){W.q.w + 0.5 * dq.w * dt, W.q.x + 0.5 * dq.x * dt, W.q.y + 0.5 * dq.y * dt,
                        W.q.z + 0.5 * dq.z * dt});

    /* 电池 */
    double lift_tot = W.lift[0] + W.lift[1];
    W.lift_avg += 0.002 * (lift_tot - W.lift_avg);
    double load = W.lift_avg / (MASS * GRAV);
    W.current = 0.12 + 2.3 * pow(load > 0 ? load : 0, 1.5);
    W.soc -= W.current * dt / 3600.0 / BAT_CAP_AH * U.drain;
    if (W.soc < 0) {
        W.soc = 0;
    }
    double ocv = 3.3 + 0.9 * W.soc - 0.25 * exp(-W.soc * 20.0);
    W.vbat = ocv - W.current * BAT_R;
    W.t += dt;
}

/* ======================= 一个 2.5 ms 节拍 ======================= */
static uint32_t s_step;
static void step_once(void)
{
    if (s_step % 8u == 0u) {
        rc_send(); /* 遥控器 50 Hz */
    }
    fc_tick();
    for (int i = 0; i < PHYS_SUB; ++i) {
        physics_step();
    }
    s_step++;
}

/* ======================= 指示灯（flight_control.c pattern_for / rc_app.c） ======================= */
static const char *fc_led(void)
{
    if (F.booting || F.calib_mode) {
        return "rapid";
    }
    switch (F.out.safety.state) {
    case ALGO_SAFETY_BOOT:
    case ALGO_SAFETY_SELFTEST: return "rapid";
    case ALGO_SAFETY_STANDBY:
        if (!F.in.rc.valid) return "double";
        return (F.out.fault_flags & ALGO_FAULT_POWER) ? "fast" : "slow";
    case ALGO_SAFETY_ARMED: return "on";
    case ALGO_SAFETY_DEGRADED:
    case ALGO_SAFETY_FAILSAFE: return "fast";
    default: return "sos";
    }
}
static const char *rc_led(bool *link_ok)
{
    *link_ok = R.tele_time >= 0 && (W.t - R.tele_time) < 1.0;
    uint32_t rc_bat = 3900;
    bool low = (*link_ok && R.tele.vbat_mv > 1000 && R.tele.vbat_mv < 3500) || rc_bat < 3500;
    if (!*link_ok) return "fast";
    if (low) return "double";
    return R.armed ? "on" : "slow";
}

/* ======================= 输出 ======================= */
static void print_state(void)
{
    static char buf[8192];
    double r, p, y;
    q_euler(W.q, &r, &p, &y);
    const algo_output_t *o = &F.out;
    bool link_ok;
    const char *rled = rc_led(&link_ok);
    static const char *fault_names[] = {"遥控丢失", "IMU无效", "姿态无效", "相位异常", "左舵机", "右舵机",
                                        "电源低",   "连杆",    "输出饱和", "机构未确认", "内部错误"};
    char faults[400] = "";
    for (int i = 0; i < 11; ++i) {
        if (o->fault_flags & (1u << i)) {
            char tmp[48];
            snprintf(tmp, sizeof(tmp), "%s\"%s\"", faults[0] ? "," : "", fault_names[i]);
            strncat(faults, tmp, sizeof(faults) - strlen(faults) - 1);
        }
    }
    char msgs[1700] = "";
    for (int i = 0; i < s_msg_n; ++i) {
        char tmp[200];
        snprintf(tmp, sizeof(tmp), "%s\"%.160s\"", i ? "," : "", s_msgs[i]);
        strncat(msgs, tmp, sizeof(msgs) - strlen(msgs) - 1);
    }
    s_msg_n = 0;
    const char *state = (F.booting || F.calib_mode) ? "校准" : state_cn(o->safety.state);

    snprintf(buf, sizeof(buf),
             "{\"t\":%.3f,\"state\":\"%s\",\"faults\":[%s],\"msg\":[%s],"
             "\"pos\":[%.3f,%.3f,%.3f],\"vel\":[%.2f,%.2f,%.2f],\"q\":[%.5f,%.5f,%.5f,%.5f],"
             "\"att\":[%.2f,%.2f,%.2f],\"est\":[%.2f,%.2f,%.2f],\"rate\":[%.1f,%.1f,%.1f],"
             "\"lift_avg\":%.3f,\"wing\":[%.2f,%.2f],\"servo_cmd\":[%.1f,%.1f],\"servo\":[%.1f,%.1f],\"lift\":[%.4f,%.4f],"
             "\"f\":%.2f,\"amp_lim\":%.2f,\"diff_lim\":%.2f,\"ctrl\":[%.2f,%.2f,%.2f],"
             "\"allow_flap\":%d,\"allow_stab\":%d,\"converged\":%d,"
             "\"rc\":{\"thr\":%d,\"roll\":%d,\"pitch\":%d,\"yaw\":%d,\"armed\":%d,\"manual\":%d,\"trim\":[%d,%d]},"
             "\"rx_age\":%d,\"loss\":%d,\"vbat\":%.0f,\"soc\":%.3f,\"current\":%.2f,"
             "\"led_fc\":\"%s\",\"led_rc\":\"%s\",\"link_ok\":%d,\"ground\":%d,\"crashed\":%d,"
             "\"cfg\":{\"mech\":%d,\"stab\":%d,\"pid\":[%.3f,%.3f,%.3f,%.4f],\"link\":%d,\"loss\":%.2f,"
             "\"imu_fail\":%d,\"right_rev\":%d,\"drain\":%.1f,\"wind\":[%.1f,%.1f],\"fsamp\":%.2f,\"dgamp\":%.2f}}",
             W.t, state, faults, msgs, W.p.x, W.p.y, -W.p.z, W.v.x, W.v.y, -W.v.z, W.q.w, W.q.x, W.q.y, W.q.z,
             r * R2D, p * R2D, y * R2D, o->attitude.roll_rad * R2D, o->attitude.pitch_rad * R2D,
             o->attitude.yaw_rad * R2D, W.w.x * R2D, W.w.y * R2D, W.w.z * R2D, W.lift_avg / (MASS * GRAV), W.wing[0] * R2D, W.wing[1] * R2D,
             s_servo_cmd[0], s_servo_cmd[1], W.servo[0], W.servo[1], W.lift[0], W.lift[1], o->phase.f_hz,
             o->safety.amplitude_limit, o->safety.differential_limit, o->control.lift, o->control.roll,
             o->control.pitch, o->safety.allow_flap, o->safety.allow_stabilization, o->attitude.converged, R.thr,
             R.roll, R.pitch, R.yaw, R.armed, R.manual, R.trim_roll, R.trim_pitch,
             F.rx_time < 0 ? -1 : (int)((W.t - F.rx_time) * 1000), F.loss_pct, F.bat_filt, W.soc, W.current,
             fc_led(), rled, link_ok, W.on_ground, W.crashed, U.mech, U.stab, U.angle_p, U.rate_p, U.rate_i,
             U.rate_d, U.link, U.loss, U.imu_fail, U.right_reversed, U.drain, W.wind.x, W.wind.y,
             F.cfg.safety.failsafe_amplitude_limit, F.cfg.safety.degraded_amplitude_limit);
    puts(buf);
    fflush(stdout);
}

/* ======================= 命令 ======================= */
static void handle(char *line)
{
    char cmd[32] = "";
    if (sscanf(line, "%31s", cmd) != 1) {
        return;
    }
    if (strcmp(cmd, "run") == 0) {
        int n = 4;
        sscanf(line + 3, "%d", &n);
        for (int i = 0; i < n && i < 4000; ++i) {
            step_once();
        }
        print_state();
    } else if (strcmp(cmd, "stick") == 0) {
        float t, r, p, y;
        if (sscanf(line + 5, "%f %f %f %f", &t, &r, &p, &y) == 4) {
            R.thr = (int16_t)clampd(t * 1000, 0, 1000);
            R.roll = (int16_t)clampd(r * 1000, -1000, 1000);
            R.pitch = (int16_t)clampd(p * 1000, -1000, 1000);
            R.yaw = (int16_t)clampd(y * 1000, -1000, 1000);
        }
    } else if (strcmp(cmd, "key") == 0) {
        char k[32];
        if (sscanf(line + 3, "%31s", k) == 1) {
            rc_key(k);
        }
    } else if (strcmp(cmd, "gust") == 0) {
        float tr, tp;
        if (sscanf(line + 4, "%f %f", &tr, &tp) == 2) {
            W.gust_torque = (v3){tr * 0.004, tp * 0.003, 0};
            W.gust_left = 0.12;
            msg("仿真：阵风扰动");
        }
    } else if (strcmp(cmd, "set") == 0) {
        char k[32];
        float a = 0, b = 0, c = 0, d = 0;
        int n = sscanf(line + 3, "%31s %f %f %f %f", k, &a, &b, &c, &d);
        if (n < 2) {
            return;
        }
        if (!strcmp(k, "link")) {
            U.link = a != 0;
        } else if (!strcmp(k, "loss")) {
            U.loss = clampd(a, 0, 1);
        } else if (!strcmp(k, "imufail")) {
            U.imu_fail = a != 0;
        } else if (!strcmp(k, "rightrev")) {
            U.right_reversed = a != 0;
        } else if (!strcmp(k, "drain")) {
            U.drain = clampd(a, 0, 200);
        } else if (!strcmp(k, "wind")) {
            W.wind = (v3){a, b, 0};
        } else if (!strcmp(k, "mech") || !strcmp(k, "stab") || !strcmp(k, "pid") || !strcmp(k, "fsamp") || !strcmp(k, "dgamp")) {
            if (is_flying(F.out.safety.state) && !W.on_ground) {
                msg("飞行中不能改飞控参数（相当于重新烧录），先上锁落地");
                return;
            }
            if (!strcmp(k, "mech")) {
                U.mech = a != 0;
            } else if (!strcmp(k, "stab")) {
                U.stab = a != 0;
            } else if (!strcmp(k, "fsamp")) {
                U.fs_amp = (float)clampd(a, 0, 1);
            } else if (!strcmp(k, "dgamp")) {
                U.dg_amp = (float)clampd(a, 0, 1);
            } else if (n >= 5) {
                U.angle_p = a;
                U.rate_p = b;
                U.rate_i = c;
                U.rate_d = d;
            }
            msg("飞控重新启动（等同改 menuconfig 后重新烧录）");
            fc_boot();
        }
    } else if (strcmp(cmd, "reset") == 0) {
        world_reset();
        fc_boot();
        msg("仿真复位：飞机放在地上，电池充满");
    } else if (strcmp(cmd, "battery") == 0) {
        W.soc = 1.0;
        msg("仿真：换上满电电池");
    }
}

/* ======================= 自测（--test） ======================= */
static void run_s(double s) { for (int i = 0; i < (int)(s / FC_DT); ++i) step_once(); }
static void report(const char *tag)
{
    double r, p, y;
    q_euler(W.q, &r, &p, &y);
    printf("%-22s L/W=%.2f t=%5.1fs %-6s h=%5.2fm vz=%+5.2f roll=%+6.1f pitch=%+6.1f est=(%+6.1f,%+6.1f) f=%.1fHz amp=%.2f "
           "vbat=%.0f\n",
           tag, W.lift_avg / (MASS * GRAV), W.t, (F.booting ? "校准" : state_cn(F.out.safety.state)), -W.p.z, -W.v.z, r * R2D, p * R2D,
           F.out.attitude.roll_rad * R2D, F.out.attitude.pitch_rad * R2D, F.out.phase.f_hz,
           F.out.safety.amplitude_limit, F.bat_filt);
    for (int i = 0; i < s_msg_n; ++i) {
        printf("    · %s\n", s_msgs[i]);
    }
    s_msg_n = 0;
}

static void self_test(void)
{
    world_reset();
    fc_boot();
    run_s(4.5);
    report("上电 4.5 s");
    rc_key("stop");
    run_s(0.2);
    report("油门 0 解锁");
    for (int t = 30; t <= 100; t += 10) {
        R.thr = (int16_t)(t * 10);
        run_s(2.0);
        char tag[32];
        snprintf(tag, sizeof(tag), "手动 油门 %d%%", t);
        report(tag);
        if (-W.p.z > 3.0) break;
    }
    R.thr = 700;
    R.roll = 500;
    run_s(0.6);
    report("右滚杆 50% 0.6 s");
    R.roll = 0;
    run_s(1.0);
    report("回中 1 s");
    R.pitch = 500;
    run_s(0.6);
    report("抬头杆 50% 0.6 s");
    R.pitch = 0;
    run_s(1.0);
    report("回中 1 s");
    U.link = false;
    run_s(0.5);
    report("断链 0.5 s");
    run_s(2.5);
    report("断链 3 s");
    run_s(4.0);
    report("断链 7 s");
    U.link = true;
    rc_key("stop");
    run_s(1.0);
    report("恢复 + 上锁");

    printf("\n--- 增稳（界面默认 PID）---\n");
    U.stab = true;
    world_reset();
    fc_boot();
    R.manual = false;
    R.thr = 0;
    run_s(4.5);
    rc_key("stop");
    run_s(0.2);
    R.thr = 720;
    run_s(4.0);
    report("增稳 油门 72% 4 s");
    W.gust_torque = (v3){0.004, 0, 0};
    W.gust_left = 0.12;
    run_s(0.3);
    report("阵风后 0.3 s");
    run_s(2.0);
    report("阵风后 2.3 s");
    R.roll = 500;
    run_s(1.0);
    report("增稳 右滚杆 50%");
    R.roll = 0;
    run_s(2.0);
    report("回中");
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOFBF, 1 << 16);
    if (argc > 1 && strcmp(argv[1], "--test") == 0) {
        self_test();
        return 0;
    }
    world_reset();
    fc_boot();
    char line[512];
    while (fgets(line, sizeof(line), stdin)) {
        handle(line);
    }
    return 0;
}
