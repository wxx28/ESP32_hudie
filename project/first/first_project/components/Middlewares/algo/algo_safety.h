#ifndef ALGO_SAFETY_H
#define ALGO_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_config.h"
#include "algo_types.h"

/*
 * 安全状态机
 *
 *   BOOT ──► SELFTEST ──(IMU 就绪 + 姿态收敛)──► STANDBY
 *                │ 超时                            │ 解锁上升沿 + 油门低 + 允许飞行
 *                ▼                                 ▼
 *              ESTOP ◄──(严重故障/姿态翻转)──── ARMED ⇄ DEGRADED（遥控短时异常/低电）
 *                                                  │
 *                                                  ▼ 遥控失联超过 rc_failsafe_timeout_ms
 *                                               FAILSAFE（幅度斜坡收到 failsafe 限制，禁止差动）
 *
 * 规则：
 *   - 解锁必须是 arm_request 的"上升沿"，且油门 < ARM_THROTTLE_MAX；
 *     上电时开关已经在解锁位不会直接起飞。
 *   - require_valid_mechanical_limits 为 true 时，
 *     waveform.mechanical_limits_valid 和连杆可用性都必须为真才能解锁。
 *   - FAILSAFE 只能通过"遥控恢复 + 解锁开关回到上锁"回到 STANDBY，必须重新解锁。
 *   - ESTOP 锁存，只能在上锁状态下调用 safety_clear_latched() 清除。
 *   - 增稳只是请求：同时满足 stabilize_request、姿态已收敛、
 *     角速度环增益非零（已整定）时才闭环，否则手动直通。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 解锁时允许的最大油门。 */
#define SAFETY_ARM_THROTTLE_MAX      0.05f
/* 姿态/角速度超限持续多久判定翻转（ms）。 */
#define SAFETY_UPSET_HOLD_MS         500u

typedef struct {
    algo_safety_config_t config;

    /* 由顶层在初始化时给出：机构参数是否允许飞行。 */
    bool flight_permitted;
    /* 角速度环增益是否已整定（任一非零即认为已整定）。 */
    bool stabilization_tuned;

    safety_state_t state;
    uint32_t state_elapsed_ms;

    bool prev_arm_request;
    uint32_t rc_good_ms;        /* 遥控连续有效时长 */
    uint32_t upset_ms;          /* 姿态超限持续时长 */
    float ramp_limit;           /* FAILSAFE 当前幅度系数 */

    uint32_t latched_faults;
    bool initialized;
} safety_state_machine_t;

typedef struct {
    const algo_rc_command_t *rc;
    const algo_system_health_t *health;
    const attitude_out_t *attitude;
    const filter_out_t *filter;
    /* 本周期其它模块上报的故障（连杆不可达、内部错误等）。 */
    uint32_t module_faults;
} safety_input_t;

bool safety_config_is_valid(const algo_safety_config_t *config);

void safety_init(
    safety_state_machine_t *sm,
    const algo_safety_config_t *config,
    bool flight_permitted,
    bool stabilization_tuned
);

void safety_reset(safety_state_machine_t *sm);

/* dt_ms：距离上次调用的时间。 */
void safety_update(
    safety_state_machine_t *sm,
    const safety_input_t *in,
    uint32_t dt_ms,
    safety_directive_t *out
);

/* 只在非飞行状态生效；返回是否已清除。 */
bool safety_clear_latched(safety_state_machine_t *sm);

/* 禁止扑翼的安全指令。 */
void safety_make_safe_directive(uint32_t fault_flags, safety_directive_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_SAFETY_H */
