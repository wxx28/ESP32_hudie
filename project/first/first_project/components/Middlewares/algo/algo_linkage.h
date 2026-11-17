#ifndef ALGO_LINKAGE_H
#define ALGO_LINKAGE_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_config.h"
#include "algo_types.h"

/*
 * 连杆逆解模块：翼角 (rad) -> 逻辑舵机角 (deg)
 *
 * 三种工作方式（由 algo_linkage_config_t 决定）：
 *
 *   1. enabled == true：四连杆逆解
 *        舵机转轴 O2 位于原点，机翼转轴 O4 位于
 *        ground_length × (cos ground_angle, sin ground_angle)。
 *        机翼摇臂绝对角 θ4 = wing_zero_rad + 翼角，
 *        求舵机曲柄绝对角 θ2，使 |A - B| = coupler_length，
 *        assembly_sign 选择两个装配分支之一。
 *        逻辑舵机角 = servo_neutral_deg
 *                   + servo_direction × (θ2 - θ2(翼角=0))。
 *
 *   2. enabled == false 且 direct_drive == true：直驱
 *        逻辑舵机角 = servo_neutral_deg + servo_direction × 翼角(deg)。
 *
 *   3. 两者都为 false：输出不可达（reachable = false），
 *      fc_output 会因此保持安全中位。
 *
 * 舵机角超出 [servo_min_deg, servo_max_deg] 时限幅并置 saturated。
 * 不包含 IO1/IO8 光耦的电气反相，也不做脉宽转换。
 *
 * 不访问硬件、不使用动态内存。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LINKAGE_STATUS_OK = 0,
    LINKAGE_STATUS_INVALID_ARG = -1,
    LINKAGE_STATUS_INVALID_CONFIG = -2,
    LINKAGE_STATUS_NOT_INITIALIZED = -3
} linkage_status_t;

typedef struct {
    algo_linkage_config_t config;

    /* 四连杆模式下，翼角为 0 时的舵机曲柄绝对角（rad）。 */
    float crank_zero_rad[ALGO_WING_COUNT];

    /* 最近一次输出。 */
    servo_target_t current;

    bool initialized;
} linkage_state_t;

/* 仅检查配置格式；不判断机构是否已实测。 */
bool linkage_config_is_valid(const algo_linkage_config_t *config);

linkage_status_t linkage_init(
    linkage_state_t *state,
    const algo_linkage_config_t *config
);

/*
 * 根据翼目标生成逻辑舵机目标。
 *
 * wing->valid == false 时输出无效，reachable 全为 false。
 */
linkage_status_t linkage_update(
    linkage_state_t *state,
    const wing_target_t *wing,
    servo_target_t *output
);

/* 生成"两翼都在中位"的输出，valid = true，reachable = true。 */
void linkage_neutral(
    const linkage_state_t *state,
    servo_target_t *output
);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_LINKAGE_H */
