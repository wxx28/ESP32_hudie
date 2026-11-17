#include "fc_config.h"

#include "sdkconfig.h"

void fc_config_v1(algo_config_t *c)
{
    algo_config_t d = ALGO_CONFIG_DEFAULT;
    *c = d;

    /* ---------- 扑翼频率：受舵机速度限制，比 7.5 Hz 设计点低 ---------- */
    c->phase.min_frequency_hz = (float)CONFIG_FC_FLAP_MIN_HZ_X10 / 10.0f;
    c->phase.max_frequency_hz = (float)CONFIG_FC_FLAP_MAX_HZ_X10 / 10.0f;
    c->phase.default_frequency_hz = c->phase.min_frequency_hz;
    c->phase.slew_hz_per_s = 3.0f;

    /* ---------- 混控：直驱时翼角 = 舵机角 ---------- */
    const float amp = ALGO_DEG_TO_RAD((float)CONFIG_FC_FLAP_AMPLITUDE_DEG);
    c->mixer.lift_amplitude_gain_rad = amp;
    c->mixer.max_amplitude_rad[ALGO_WING_LEFT] = amp;
    c->mixer.max_amplitude_rad[ALGO_WING_RIGHT] = amp;
    c->mixer.roll_differential_gain_rad = ALGO_DEG_TO_RAD((float)CONFIG_FC_ROLL_DIFF_DEG);
    c->mixer.max_differential_rad = ALGO_DEG_TO_RAD((float)CONFIG_FC_ROLL_DIFF_DEG);

    /* ---------- 翼角限位：幅度 + 差动再留 10° 余量 ---------- */
    const float lim = ALGO_DEG_TO_RAD((float)(CONFIG_FC_FLAP_AMPLITUDE_DEG + CONFIG_FC_ROLL_DIFF_DEG + 10));
    for (int w = 0; w < ALGO_WING_COUNT; ++w) {
        c->waveform.min_wing_angle_rad[w] = -lim;
        c->waveform.max_wing_angle_rad[w] = lim;
        /* 舵机 0.1 s/60° ≈ 600°/s；目标角速度超过时只标记饱和，舵机自己跟不上 */
        c->waveform.max_wing_rate_rad_s[w] = ALGO_DEG_TO_RAD(3000.0f);
    }
#if CONFIG_FC_MECH_CONFIRMED
    c->waveform.mechanical_limits_valid = true;
#else
    c->waveform.mechanical_limits_valid = false;
#endif

    /* ---------- 连杆 / 舵机方向 ---------- */
#if CONFIG_FC_DIRECT_DRIVE
    c->linkage.enabled = false;
    c->linkage.direct_drive = true;
#else
    /* 四连杆：在这里填实测尺寸后再打开 enabled */
    c->linkage.enabled = false;
    c->linkage.direct_drive = false;
#endif
    c->linkage.side[ALGO_WING_LEFT].servo_neutral_deg = (float)CONFIG_FC_SERVO_LEFT_NEUTRAL_DEG;
    c->linkage.side[ALGO_WING_RIGHT].servo_neutral_deg = (float)CONFIG_FC_SERVO_RIGHT_NEUTRAL_DEG;
    c->linkage.side[ALGO_WING_LEFT].servo_direction = 1.0f;
#if CONFIG_FC_SERVO_RIGHT_MIRRORED
    c->linkage.side[ALGO_WING_RIGHT].servo_direction = -1.0f;
#else
    c->linkage.side[ALGO_WING_RIGHT].servo_direction = 1.0f;
#endif
    for (int w = 0; w < ALGO_WING_COUNT; ++w) {
        c->linkage.side[w].servo_min_deg = 10.0f;
        c->linkage.side[w].servo_max_deg = 170.0f;
    }

    /* ---------- 失控保护 / 降级 ---------- */
    /* 仿真（project/sim）得出：升力≈幅度²，默认 0.35 会自由落体，改成略低于悬停油门 */
    c->safety.failsafe_amplitude_limit = (float)CONFIG_FC_FAILSAFE_THROTTLE_PCT / 100.0f;
    c->safety.degraded_amplitude_limit = (float)CONFIG_FC_DEGRADED_AMP_PCT / 100.0f;

    /* ---------- 增稳 ---------- */
#if CONFIG_FC_ENABLE_STABILIZE
    /*
     * 起步增益（docs/01 第 4.2 节的量级，按本控制器的归一化输出换算）：
     * 角度环 P=3（误差 rad -> 目标 rad/s），角速度环 P=0.3、I=0.05、D=0.002。
     * 必须系绳试飞：先只开角速度环 P，确认方向正确再加。
     */
    for (int a = 0; a < 2; ++a) {
        c->control.angle_pid[a].kp = 3.0f;
        c->control.rate_pid[a].kp = 0.30f;
        c->control.rate_pid[a].ki = 0.05f;
        c->control.rate_pid[a].kd = 0.002f;
        c->control.rate_pid[a].integral_limit = 0.3f;
    }
#endif
}
