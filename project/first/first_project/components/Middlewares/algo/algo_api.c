#include "algo_api.h"

#include <math.h>
#include <string.h>

#include "algo_attitude.h"
#include "algo_control.h"
#include "algo_filter.h"
#include "algo_linkage.h"
#include "algo_mixer.h"
#include "algo_phase.h"
#include "algo_safety.h"
#include "algo_waveform.h"

/*
 * 顶层调度，单周期顺序（与 algo_types.h 的数据流一致）：
 *
 *   filter(上一拍相位) -> attitude -> safety -> control(分频)
 *   -> mixer -> phase -> waveform -> linkage -> algo_output_t
 *
 * 遥控失联处理：
 *   DEGRADED / FAILSAFE 期间遥控帧可能已无效。控制器在 rc.valid == false 时
 *   会输出零控制量（幅度为零），所以这里用"最后一次有效油门 + 杆量回中"
 *   的保持指令喂给控制器，由安全指令中的 amplitude_limit 负责收小幅度。
 */

typedef struct {
    algo_config_t config;

    filter_state_t filter;
    attitude_state_t attitude;
    control_state_t control;
    mixer_state_t mixer;
    phase_gen_t phase;
    waveform_state_t waveform;
    linkage_state_t linkage;
    safety_state_machine_t safety;

    algo_api_runtime_t runtime;

    phase_out_t last_phase;
    control_out_t last_control;
    safety_directive_t last_safety;
    float control_dt_acc_s;
    uint32_t module_faults;     /* 上一周期模块故障，喂给下一周期的安全状态机 */
    algo_rc_command_t last_valid_rc;
    bool has_valid_rc;
} algo_api_ctx_t;

static algo_api_ctx_t s_ctx;

/* 配置检查用的临时模块状态（避免占用栈） */
static filter_state_t s_chk_filter;
static attitude_state_t s_chk_attitude;
static control_state_t s_chk_control;
static mixer_state_t s_chk_mixer;
static phase_gen_t s_chk_phase;
static waveform_state_t s_chk_waveform;
static linkage_state_t s_chk_linkage;

static bool flight_permitted(const algo_config_t *c)
{
    if (!c->safety.require_valid_mechanical_limits) {
        return true;
    }
    return c->waveform.mechanical_limits_valid && (c->linkage.enabled || c->linkage.direct_drive);
}

static bool stabilization_tuned(const algo_config_t *c)
{
    return c->control.rate_pid[ALGO_AXIS_X].kp > 0.0f || c->control.rate_pid[ALGO_AXIS_Y].kp > 0.0f;
}

algo_status_t algo_api_config_validate(const algo_config_t *config)
{
    if (config == NULL) {
        return ALGO_STATUS_INVALID_ARG;
    }
    const algo_scheduler_config_t *s = &config->scheduler;
    if (config->version != ALGO_CONFIG_VERSION ||
        !isfinite(s->fast_dt_s) || s->fast_dt_s <= 0.0f ||
        !isfinite(s->max_dt_s) || s->max_dt_s < s->fast_dt_s ||
        s->control_divider == 0u || s->health_divider == 0u) {
        return ALGO_STATUS_INVALID_CONFIG;
    }
    if (!safety_config_is_valid(&config->safety) || !linkage_config_is_valid(&config->linkage)) {
        return ALGO_STATUS_INVALID_CONFIG;
    }
    if (filter_init(&s_chk_filter, &config->filter, s->fast_dt_s) != FILTER_STATUS_OK ||
        attitude_init(&s_chk_attitude, &config->attitude) != ATTITUDE_STATUS_OK ||
        control_init(&s_chk_control, &config->control) != CONTROL_STATUS_OK ||
        mixer_init(&s_chk_mixer, &config->mixer, &config->phase) != MIXER_STATUS_OK ||
        phase_init(&s_chk_phase, &config->phase) != PHASE_STATUS_OK ||
        waveform_init(&s_chk_waveform, &config->waveform) != WAVEFORM_STATUS_OK ||
        linkage_init(&s_chk_linkage, &config->linkage) != LINKAGE_STATUS_OK) {
        return ALGO_STATUS_INVALID_CONFIG;
    }
    return ALGO_STATUS_OK;
}

static algo_status_t init_modules(void)
{
    const algo_config_t *c = &s_ctx.config;
    if (filter_init(&s_ctx.filter, &c->filter, c->scheduler.fast_dt_s) != FILTER_STATUS_OK ||
        attitude_init(&s_ctx.attitude, &c->attitude) != ATTITUDE_STATUS_OK ||
        control_init(&s_ctx.control, &c->control) != CONTROL_STATUS_OK ||
        mixer_init(&s_ctx.mixer, &c->mixer, &c->phase) != MIXER_STATUS_OK ||
        phase_init(&s_ctx.phase, &c->phase) != PHASE_STATUS_OK ||
        waveform_init(&s_ctx.waveform, &c->waveform) != WAVEFORM_STATUS_OK ||
        linkage_init(&s_ctx.linkage, &c->linkage) != LINKAGE_STATUS_OK) {
        return ALGO_STATUS_INVALID_CONFIG;
    }
    safety_init(&s_ctx.safety, &c->safety, flight_permitted(c), stabilization_tuned(c));
    phase_get(&s_ctx.phase, &s_ctx.last_phase);
    memset(&s_ctx.last_control, 0, sizeof(s_ctx.last_control));
    safety_make_safe_directive(ALGO_FAULT_NONE, &s_ctx.last_safety);
    s_ctx.control_dt_acc_s = 0.0f;
    s_ctx.module_faults = 0u;
    s_ctx.has_valid_rc = false;
    return ALGO_STATUS_OK;
}

algo_status_t algo_api_init(const algo_config_t *config)
{
    if (config == NULL) {
        return ALGO_STATUS_INVALID_ARG;
    }
    memset(&s_ctx, 0, sizeof(s_ctx));
    algo_status_t st = algo_api_config_validate(config);
    if (st != ALGO_STATUS_OK) {
        s_ctx.runtime.last_status = st;
        return st;
    }
    s_ctx.config = *config;
    st = init_modules();
    s_ctx.runtime.config_valid = (st == ALGO_STATUS_OK);
    s_ctx.runtime.initialized = (st == ALGO_STATUS_OK);
    s_ctx.runtime.safety_state = ALGO_SAFETY_BOOT;
    s_ctx.runtime.last_status = st;
    return st;
}

algo_status_t algo_api_reset(void)
{
    if (!s_ctx.runtime.initialized) {
        return ALGO_STATUS_NOT_INITIALIZED;
    }
    algo_status_t st = init_modules();
    algo_api_runtime_t keep = {
        .initialized = (st == ALGO_STATUS_OK),
        .config_valid = (st == ALGO_STATUS_OK),
        .last_status = st,
        .safety_state = ALGO_SAFETY_BOOT,
    };
    s_ctx.runtime = keep;
    return st;
}

algo_status_t algo_api_get_config(algo_config_t *config_out)
{
    if (config_out == NULL) {
        return ALGO_STATUS_INVALID_ARG;
    }
    if (!s_ctx.runtime.initialized) {
        return ALGO_STATUS_NOT_INITIALIZED;
    }
    *config_out = s_ctx.config;
    return ALGO_STATUS_OK;
}

algo_status_t algo_api_make_safe_output(uint64_t timestamp_us, uint32_t fault_flags, algo_output_t *output)
{
    if (output == NULL) {
        return ALGO_STATUS_INVALID_ARG;
    }
    memset(output, 0, sizeof(*output));
    output->timestamp_us = timestamp_us;
    safety_make_safe_directive(fault_flags, &output->safety);
    output->fault_flags = fault_flags;
    if (s_ctx.runtime.initialized) {
        linkage_neutral(&s_ctx.linkage, &output->servo);
    }
    output->valid = false;
    return ALGO_STATUS_OK;
}

static algo_status_t fail(algo_status_t st, const algo_input_t *input, uint32_t faults, algo_output_t *output)
{
    algo_api_make_safe_output(input != NULL ? input->now_us : 0u, faults, output);
    s_ctx.runtime.last_status = st;
    return st;
}

/*
 * 遥控保持指令：
 *   DEGRADED：最后一次有效油门，杆量回中（短时丢包，飞手意图不变）；
 *   FAILSAFE：油门 = 安全指令里的幅度系数（从失联油门过渡到失控设定值），杆量回中。
 */
static algo_rc_command_t effective_rc(const algo_rc_command_t *rc, const safety_directive_t *safety)
{
    const safety_state_t state = safety->state;
    if (rc->valid && state != ALGO_SAFETY_FAILSAFE) {
        return *rc;
    }
    algo_rc_command_t hold = *rc;
    if ((state == ALGO_SAFETY_DEGRADED || state == ALGO_SAFETY_FAILSAFE) && s_ctx.has_valid_rc) {
        hold.throttle = (state == ALGO_SAFETY_FAILSAFE) ? safety->amplitude_limit : s_ctx.last_valid_rc.throttle;
        hold.roll = 0.0f;
        hold.pitch = 0.0f;
        hold.yaw = 0.0f;
        hold.arm_request = true;
        hold.stabilize_request = s_ctx.last_valid_rc.stabilize_request;
        hold.valid = true;
    }
    return hold;
}

algo_status_t algo_api_step(const algo_input_t *input, float dt_s, algo_output_t *output)
{
    if (output == NULL) {
        return ALGO_STATUS_INVALID_ARG;
    }
    if (input == NULL) {
        return fail(ALGO_STATUS_INVALID_ARG, input, ALGO_FAULT_INTERNAL, output);
    }
    if (!s_ctx.runtime.initialized) {
        return fail(ALGO_STATUS_NOT_INITIALIZED, input, ALGO_FAULT_INTERNAL, output);
    }
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > s_ctx.config.scheduler.max_dt_s) {
        return fail(ALGO_STATUS_INVALID_DT, input, ALGO_FAULT_INTERNAL, output);
    }
    if (s_ctx.runtime.fast_update_count > 0u && input->now_us < s_ctx.runtime.last_input_timestamp_us) {
        return fail(ALGO_STATUS_INVALID_TIMESTAMP, input, ALGO_FAULT_INTERNAL, output);
    }

    algo_output_t out;
    memset(&out, 0, sizeof(out));
    out.timestamp_us = input->now_us;
    uint32_t faults = 0u;

    if (input->rc.valid) {
        s_ctx.last_valid_rc = input->rc;
        s_ctx.has_valid_rc = true;
    }

    /* 1. 滤波（陷波跟随上一拍的实际扑翼频率） */
    if (filter_update(&s_ctx.filter, &input->imu, &s_ctx.last_phase, dt_s, &out.filter) != FILTER_STATUS_OK) {
        memset(&out.filter, 0, sizeof(out.filter));
        faults |= ALGO_FAULT_IMU_INVALID;
    }

    /* 2. 姿态 */
    if (s_ctx.last_safety.reset_attitude) {
        attitude_restart_convergence(&s_ctx.attitude);
    }
    if (attitude_update(&s_ctx.attitude, &out.filter, dt_s, &out.attitude) != ATTITUDE_STATUS_OK) {
        memset(&out.attitude, 0, sizeof(out.attitude));
        faults |= ALGO_FAULT_ATTITUDE_INVALID;
    }

    /* 3. 安全状态机 */
    const uint32_t dt_ms = (uint32_t)(dt_s * 1000.0f + 0.5f);
    safety_input_t sin = {
        .rc = &input->rc,
        .health = &input->health,
        .attitude = &out.attitude,
        .filter = &out.filter,
        .module_faults = s_ctx.module_faults,
        .hold_throttle = s_ctx.has_valid_rc ? s_ctx.last_valid_rc.throttle : 0.0f,
    };
    safety_update(&s_ctx.safety, &sin, dt_ms, &out.safety);
    s_ctx.last_safety = out.safety;

    /* 4. 控制器（分频） */
    s_ctx.control_dt_acc_s += dt_s;
    const bool run_control =
        (s_ctx.runtime.fast_update_count % s_ctx.config.scheduler.control_divider) == 0u ||
        out.safety.reset_controller;
    if (run_control) {
        algo_rc_command_t rc = effective_rc(&input->rc, &out.safety);
        if (control_update(&s_ctx.control, &rc, &out.attitude, &out.filter, &out.safety,
                           s_ctx.control_dt_acc_s, &s_ctx.last_control) != CONTROL_STATUS_OK) {
            memset(&s_ctx.last_control, 0, sizeof(s_ctx.last_control));
            faults |= ALGO_FAULT_INTERNAL;
        }
        s_ctx.control_dt_acc_s = 0.0f;
        s_ctx.runtime.control_update_count++;
    } else {
        s_ctx.runtime.control_skip_count++;
    }
    out.control = s_ctx.last_control;

    /* 5. 混控 -> 待提交波形 */
    flap_wave_cmd_t pending;
    if (mixer_update(&s_ctx.mixer, &out.control, &out.safety, &pending) != MIXER_STATUS_OK) {
        faults |= ALGO_FAULT_INTERNAL;
    }

    /* 6. 相位 */
    phase_set_freq(&s_ctx.phase, pending.valid ? pending.frequency_hz : s_ctx.config.phase.min_frequency_hz);
    if (phase_update(&s_ctx.phase, dt_s, &out.phase) != PHASE_STATUS_OK || !out.phase.valid) {
        faults |= ALGO_FAULT_PHASE_INVALID;
    }
    s_ctx.last_phase = out.phase;

    /* 7. 波形（只在整拍边界切换参数） */
    if (waveform_update(&s_ctx.waveform, &out.phase, &pending, &out.wing) != WAVEFORM_STATUS_OK) {
        memset(&out.wing, 0, sizeof(out.wing));
        faults |= ALGO_FAULT_INTERNAL;
    }
    waveform_get_active(&s_ctx.waveform, &out.active_wave);

    /* 8. 连杆逆解 */
    uint32_t next_module_faults = 0u;
    if (out.safety.allow_flap && !out.safety.immediate_stop && out.wing.valid) {
        linkage_update(&s_ctx.linkage, &out.wing, &out.servo);
        if (!out.servo.valid) {
            next_module_faults |= ALGO_FAULT_LINKAGE;
        }
    } else {
        linkage_neutral(&s_ctx.linkage, &out.servo);
    }
    /* 未飞行时也检查机构是否可用，避免带着不可用的逆解解锁 */
    if (!s_ctx.config.linkage.enabled && !s_ctx.config.linkage.direct_drive) {
        next_module_faults |= ALGO_FAULT_LINKAGE;
    }
    if (out.servo.saturated[0] || out.servo.saturated[1]) {
        faults |= ALGO_FAULT_OUTPUT_SATURATED;
    }

    /* 内部错误立即让安全状态机知道（下个周期进入 ESTOP） */
    s_ctx.module_faults = next_module_faults | (faults & ALGO_FAULT_INTERNAL);

    out.fault_flags = faults | out.safety.fault_flags | next_module_faults;
    out.valid = out.servo.valid && out.phase.valid;

    *output = out;

    s_ctx.runtime.fast_update_count++;
    if ((s_ctx.runtime.fast_update_count % s_ctx.config.scheduler.health_divider) == 0u) {
        s_ctx.runtime.health_update_count++;
    }
    s_ctx.runtime.last_dt_s = dt_s;
    s_ctx.runtime.last_input_timestamp_us = input->now_us;
    s_ctx.runtime.safety_state = out.safety.state;
    s_ctx.runtime.fault_flags = out.fault_flags;
    s_ctx.runtime.last_status = ALGO_STATUS_OK;
    return ALGO_STATUS_OK;
}

algo_status_t algo_api_get_runtime(algo_api_runtime_t *runtime_out)
{
    if (runtime_out == NULL) {
        return ALGO_STATUS_INVALID_ARG;
    }
    *runtime_out = s_ctx.runtime;
    return ALGO_STATUS_OK;
}

algo_status_t algo_api_clear_latched_faults(void)
{
    if (!s_ctx.runtime.initialized) {
        return ALGO_STATUS_NOT_INITIALIZED;
    }
    return safety_clear_latched(&s_ctx.safety) ? ALGO_STATUS_OK : ALGO_STATUS_NOT_READY;
}
