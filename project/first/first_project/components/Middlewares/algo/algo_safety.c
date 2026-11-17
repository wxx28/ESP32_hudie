#include "algo_safety.h"

#include <math.h>
#include <string.h>

static bool is_flying(safety_state_t s)
{
    return s == ALGO_SAFETY_ARMED || s == ALGO_SAFETY_DEGRADED || s == ALGO_SAFETY_FAILSAFE;
}

bool safety_config_is_valid(const algo_safety_config_t *c)
{
    if (c == NULL) {
        return false;
    }
    return c->selftest_timeout_ms > 0u &&
           c->rc_degraded_timeout_ms > 0u &&
           c->rc_failsafe_timeout_ms > c->rc_degraded_timeout_ms &&
           isfinite(c->max_attitude_angle_rad) && c->max_attitude_angle_rad > 0.0f &&
           isfinite(c->max_body_rate_rad_s) && c->max_body_rate_rad_s > 0.0f &&
           isfinite(c->degraded_amplitude_limit) && c->degraded_amplitude_limit >= 0.0f &&
           c->degraded_amplitude_limit <= 1.0f &&
           isfinite(c->degraded_differential_limit) && c->degraded_differential_limit >= 0.0f &&
           c->degraded_differential_limit <= 1.0f &&
           isfinite(c->failsafe_amplitude_limit) && c->failsafe_amplitude_limit >= 0.0f &&
           c->failsafe_amplitude_limit <= 1.0f &&
           isfinite(c->failsafe_differential_limit) && c->failsafe_differential_limit >= 0.0f &&
           c->failsafe_differential_limit <= 1.0f &&
           isfinite(c->failsafe_ramp_time_s) && c->failsafe_ramp_time_s > 0.0f;
}

void safety_make_safe_directive(uint32_t fault_flags, safety_directive_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->state = ALGO_SAFETY_STANDBY;
    out->allow_flap = false;
    out->allow_stabilization = false;
    out->immediate_stop = true;
    out->reset_controller = true;
    out->amplitude_limit = 0.0f;
    out->differential_limit = 0.0f;
    out->fault_flags = fault_flags;
}

void safety_init(safety_state_machine_t *sm, const algo_safety_config_t *config, bool flight_permitted,
                 bool stabilization_tuned)
{
    if (sm == NULL || config == NULL) {
        return;
    }
    memset(sm, 0, sizeof(*sm));
    sm->config = *config;
    sm->flight_permitted = flight_permitted;
    sm->stabilization_tuned = stabilization_tuned;
    sm->state = ALGO_SAFETY_BOOT;
    sm->initialized = true;
}

void safety_reset(safety_state_machine_t *sm)
{
    if (sm == NULL || !sm->initialized) {
        return;
    }
    algo_safety_config_t c = sm->config;
    bool fp = sm->flight_permitted;
    bool st = sm->stabilization_tuned;
    safety_init(sm, &c, fp, st);
}

bool safety_clear_latched(safety_state_machine_t *sm)
{
    if (sm == NULL || !sm->initialized || is_flying(sm->state)) {
        return false;
    }
    sm->latched_faults = 0u;
    if (sm->state == ALGO_SAFETY_ESTOP) {
        sm->state = ALGO_SAFETY_SELFTEST;
        sm->state_elapsed_ms = 0u;
    }
    return true;
}

static void enter(safety_state_machine_t *sm, safety_state_t s)
{
    if (sm->state != s) {
        sm->state = s;
        sm->state_elapsed_ms = 0u;
    }
}

void safety_update(safety_state_machine_t *sm, const safety_input_t *in, uint32_t dt_ms, safety_directive_t *out)
{
    if (out == NULL) {
        return;
    }
    if (sm == NULL || !sm->initialized || in == NULL || in->rc == NULL || in->health == NULL ||
        in->attitude == NULL || in->filter == NULL) {
        safety_make_safe_directive(ALGO_FAULT_INTERNAL, out);
        return;
    }

    const algo_rc_command_t *rc = in->rc;
    const algo_system_health_t *hl = in->health;
    const algo_safety_config_t *c = &sm->config;
    const safety_state_t prev_state = sm->state;

    sm->state_elapsed_ms += dt_ms;

    /* ---------------- 故障汇总 ---------------- */
    uint32_t faults = in->module_faults | hl->fault_flags;
    const bool rc_fresh = rc->valid && rc->age_ms < c->rc_degraded_timeout_ms;
    if (!rc_fresh) {
        faults |= ALGO_FAULT_RC_LOST;
    }
    if (!hl->imu_ready || !in->filter->valid) {
        faults |= ALGO_FAULT_IMU_INVALID;
    }
    if (!in->attitude->valid) {
        faults |= ALGO_FAULT_ATTITUDE_INVALID;
    }
    if (!hl->power_good) {
        faults |= ALGO_FAULT_POWER;
    }
    if (hl->servo_fault[ALGO_WING_LEFT]) {
        faults |= ALGO_FAULT_SERVO_LEFT;
    }
    if (hl->servo_fault[ALGO_WING_RIGHT]) {
        faults |= ALGO_FAULT_SERVO_RIGHT;
    }
    if (!sm->flight_permitted) {
        faults |= ALGO_FAULT_CONFIG;
    }

    sm->rc_good_ms = rc_fresh ? sm->rc_good_ms + dt_ms : 0u;

    /* 姿态翻转判定：仅在姿态可用时检查 */
    bool upset_now = false;
    if (in->attitude->valid && in->attitude->converged) {
        upset_now = fabsf(in->attitude->roll_rad) > c->max_attitude_angle_rad ||
                    fabsf(in->attitude->pitch_rad) > c->max_attitude_angle_rad;
    }
    if (in->filter->gyro_valid) {
        const algo_vec3_t *g = &in->filter->gyro_rad_s;
        if (fabsf(g->x) > c->max_body_rate_rad_s || fabsf(g->y) > c->max_body_rate_rad_s) {
            upset_now = true;
        }
    }
    sm->upset_ms = upset_now ? sm->upset_ms + dt_ms : 0u;

    const bool arm_edge = rc->arm_request && !sm->prev_arm_request;
    if (rc_fresh) {
        sm->prev_arm_request = rc->arm_request;
    }

    /* ---------------- 全局：严重故障 ---------------- */
    if (hl->critical_output_fault || (faults & ALGO_FAULT_INTERNAL)) {
        sm->latched_faults |= faults | ALGO_FAULT_INTERNAL;
        enter(sm, ALGO_SAFETY_ESTOP);
    } else if (is_flying(sm->state) && (sm->upset_ms >= SAFETY_UPSET_HOLD_MS)) {
        sm->latched_faults |= ALGO_FAULT_ATTITUDE_INVALID;
        enter(sm, ALGO_SAFETY_ESTOP);
    } else if (is_flying(sm->state) && (faults & ALGO_FAULT_IMU_INVALID)) {
        /* 没有 IMU 不能安全判断姿态，按急停处理 */
        sm->latched_faults |= ALGO_FAULT_IMU_INVALID;
        enter(sm, ALGO_SAFETY_ESTOP);
    }

    /* ---------------- 状态迁移 ---------------- */
    switch (sm->state) {
    case ALGO_SAFETY_BOOT:
        enter(sm, ALGO_SAFETY_SELFTEST);
        break;

    case ALGO_SAFETY_SELFTEST:
        if (hl->imu_ready && in->filter->valid && in->attitude->valid && in->attitude->converged) {
            enter(sm, ALGO_SAFETY_STANDBY);
        } else if (sm->state_elapsed_ms > c->selftest_timeout_ms) {
            sm->latched_faults |= faults | ALGO_FAULT_IMU_INVALID;
            enter(sm, ALGO_SAFETY_ESTOP);
        }
        break;

    case ALGO_SAFETY_STANDBY: {
        const bool can_arm = rc_fresh && arm_edge && rc->throttle < SAFETY_ARM_THROTTLE_MAX &&
                             sm->flight_permitted && hl->power_good && hl->imu_ready &&
                             in->attitude->converged && !hl->servo_fault[0] && !hl->servo_fault[1] &&
                             !(in->module_faults & ALGO_FAULT_LINKAGE) && sm->latched_faults == 0u;
        if (can_arm) {
            enter(sm, ALGO_SAFETY_ARMED);
        }
        break;
    }

    case ALGO_SAFETY_ARMED:
    case ALGO_SAFETY_DEGRADED:
        if (rc_fresh && !rc->arm_request) {
            enter(sm, ALGO_SAFETY_STANDBY);
        } else if (rc->age_ms >= c->rc_failsafe_timeout_ms || !rc->valid) {
            if (rc->age_ms >= c->rc_failsafe_timeout_ms) {
                sm->ramp_limit = (sm->state == ALGO_SAFETY_DEGRADED) ? c->degraded_amplitude_limit : 1.0f;
                enter(sm, ALGO_SAFETY_FAILSAFE);
            } else {
                enter(sm, ALGO_SAFETY_DEGRADED);
            }
        } else if (!rc_fresh || !hl->power_good) {
            enter(sm, ALGO_SAFETY_DEGRADED);
        } else if (sm->state == ALGO_SAFETY_DEGRADED && sm->rc_good_ms >= c->rc_recover_time_ms &&
                   hl->power_good) {
            enter(sm, ALGO_SAFETY_ARMED);
        }
        break;

    case ALGO_SAFETY_FAILSAFE: {
        /* 幅度线性收到 failsafe 限制 */
        const float step = (1.0f / c->failsafe_ramp_time_s) * ((float)dt_ms / 1000.0f);
        if (sm->ramp_limit > c->failsafe_amplitude_limit) {
            sm->ramp_limit -= step;
            if (sm->ramp_limit < c->failsafe_amplitude_limit) {
                sm->ramp_limit = c->failsafe_amplitude_limit;
            }
        }
        /* 遥控恢复且开关已回到上锁，才能回到待机 */
        if (rc_fresh && !rc->arm_request && sm->rc_good_ms >= c->rc_recover_time_ms) {
            enter(sm, ALGO_SAFETY_STANDBY);
        }
        break;
    }

    case ALGO_SAFETY_ESTOP:
    default:
        break;
    }

    /* ---------------- 生成指令 ---------------- */
    memset(out, 0, sizeof(*out));
    out->state = sm->state;
    out->fault_flags = faults | sm->latched_faults;
    out->reset_controller = (sm->state != prev_state);
    out->reset_attitude = !in->attitude->valid && hl->imu_ready;

    const bool attitude_ok = in->attitude->valid && in->attitude->converged;
    const bool stab = rc->stabilize_request && attitude_ok && sm->stabilization_tuned;

    switch (sm->state) {
    case ALGO_SAFETY_ARMED:
        out->allow_flap = true;
        out->allow_stabilization = stab;
        out->amplitude_limit = 1.0f;
        out->differential_limit = 1.0f;
        break;
    case ALGO_SAFETY_DEGRADED:
        out->allow_flap = true;
        out->allow_stabilization = stab;
        out->amplitude_limit = c->degraded_amplitude_limit;
        out->differential_limit = c->degraded_differential_limit;
        break;
    case ALGO_SAFETY_FAILSAFE:
        out->allow_flap = true;
        /* 失联时如果姿态可用且已整定，就保持水平 */
        out->allow_stabilization = attitude_ok && sm->stabilization_tuned;
        out->amplitude_limit = sm->ramp_limit;
        out->differential_limit = c->failsafe_differential_limit;
        break;
    case ALGO_SAFETY_ESTOP:
        out->immediate_stop = true;
        out->reset_controller = true;
        break;
    default:
        /* BOOT / SELFTEST / STANDBY：不扑翼 */
        out->reset_controller = true;
        break;
    }
}
