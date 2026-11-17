#ifndef FC_OUTPUT_H
#define FC_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_types.h"
#include "algo_config.h"

/*
 * 飞控输出适配模块
 *
 * 与现有算法的直接对应关系：
 *
 *   1. algo_api_step() 生成 algo_output_t；
 *   2. algo_output_t.servo 保存最终逻辑舵机目标；
 *   3. fc_output_write() 接收完整的 algo_output_t；
 *   4. 根据 output->safety 判断是否允许扑翼；
 *   5. 根据 output->servo 判断是否可输出；
 *   6. 将 angle_deg[] 交给 BSP/SERVO。
 *
 * 配置来源：
 *
 *   fc_output_init() 直接使用 algo_config.linkage 中的：
 *     - servo_neutral_deg：安全逻辑中位；
 *     - servo_min_deg：逻辑最小角；
 *     - servo_max_deg：逻辑最大角。
 *
 *   飞控层不再另外定义一套舵机中位和角度限位，
 *   避免算法与输出层使用不同参数。
 *
 * 第四代 V1 原理图对应关系：
 *
 *   ALGO_WING_LEFT：
 *     output->servo.angle_deg[ALGO_WING_LEFT]
 *       -> BSP 左翼 PWM
 *       -> ESP32-C3 IO1 / PWM2
 *       -> 隔离驱动
 *       -> J3 / PWM_L。
 *
 *   ALGO_WING_RIGHT：
 *     output->servo.angle_deg[ALGO_WING_RIGHT]
 *       -> BSP 右翼 PWM
 *       -> ESP32-C3 IO8 / PWM1
 *       -> 隔离驱动
 *       -> J2 / PWM_R。
 *
 * 本模块不负责：
 *   - 不重新计算混控；
 *   - 不重新进行连杆逆解；
 *   - 不修改算法输出的角度；
 *   - 不操作 PWM 寄存器；
 *   - 不虚构舵机位置或堵转反馈。
 *
 * 方向约定：
 *   - algo_linkage_config_t.servo_direction 属于机构安装方向；
 *   - BSP 的 IO 电气反相属于硬件适配；
 *   - 两种方向处理不能重复应用。
 *
 * 使用限制：
 *   - 单实例设计；
 *   - 不使用动态内存；
 *   - 不允许在 ISR 中调用；
 *   - 只允许飞控任务直接写入舵机目标。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FC_OUTPUT_STATUS_OK = 0,

    /* 必要指针为空。 */
    FC_OUTPUT_STATUS_INVALID_ARG = -1,

    /* 尚未成功调用 fc_output_init()。 */
    FC_OUTPUT_STATUS_NOT_INITIALIZED = -2,

    /* 舵机中位或角度限位配置非法。 */
    FC_OUTPUT_STATUS_INVALID_CONFIG = -3,

    /*
     * 算法输出无效、不可达或包含非法角度。
     * 模块会同时请求安全中位输出。
     */
    FC_OUTPUT_STATUS_INVALID_OUTPUT = -4,

    /* PWM 初始化或硬件写入失败。 */
    FC_OUTPUT_STATUS_HARDWARE_ERROR = -5,

    /* 输出模块内部状态不一致。 */
    FC_OUTPUT_STATUS_INTERNAL_ERROR = -6
} fc_output_status_t;

/* ============================================================
 * 初始化输出模块
 *
 * 参数：
 *   linkage_config：
 *     直接传入 &algo_config.linkage，不允许为空。
 *
 *   本函数复制左右两侧所需的：
 *     servo_neutral_deg；
 *     servo_min_deg；
 *     servo_max_deg。
 *
 * 配置检查：
 *   - 所有角度必须为有限值；
 *   - servo_min_deg <= servo_neutral_deg <= servo_max_deg；
 *   - 左右两翼分别检查，不能共用未确认的限位。
 *
 * linkage_config.enabled == false 时：
 *   - 仍允许初始化 PWM 并保持安全中位；
 *   - 不允许把无效连杆输出当成正常飞行输出；
 *   - 正常输出仍必须通过 output->servo.valid 和 reachable[]。
 *
 * 初始化顺序：
 *   1. 检查并复制算法连杆输出配置；
 *   2. 初始化 BSP 的左右 PWM 通道；
 *   3. 左右翼分别写入配置的 servo_neutral_deg；
 *   4. 保存最近一次安全目标；
 *   5. 标记输出模块初始化完成。
 *
 * 如果任意一路失败：
 *   - 返回 FC_OUTPUT_STATUS_HARDWARE_ERROR；
 *   - 两侧都不得保持随机或未定义输出。
 * ============================================================ */

fc_output_status_t fc_output_init(
    const algo_linkage_config_t *linkage_config
);

/* ============================================================
 * 提交 algo_api_step() 的完整输出
 *
 * 参数：
 *   output：
 *     本周期由 algo_api_step() 生成的完整输出。
 *
 * 正常输出必须同时满足：
 *
 *   output->valid == true；
 *   output->servo.valid == true；
 *   output->servo.reachable[LEFT] == true；
 *   output->servo.reachable[RIGHT] == true；
 *   output->safety.allow_flap == true；
 *   output->safety.immediate_stop == false；
 *   左右舵机角均为有限值；
 *   左右舵机角均位于各自配置的 min/max 范围内。
 *
 * 正常输出映射：
 *
 *   output->servo.angle_deg[ALGO_WING_LEFT]
 *     -> BSP 左翼 -> IO1 / PWM2 -> J3 / PWM_L。
 *
 *   output->servo.angle_deg[ALGO_WING_RIGHT]
 *     -> BSP 右翼 -> IO8 / PWM1 -> J2 / PWM_R。
 *
 * saturated[] 的处理：
 *   - saturated[] 表示算法已经进行了限幅；
 *   - 不属于接口错误；
 *   - 不因此自动丢弃整个输出；
 *   - 后续由安全状态机判断持续饱和是否需要降级。
 *
 * 禁止扑翼：
 *   当 allow_flap == false 或 immediate_stop == true 时：
 *     - 左右翼均输出各自的 servo_neutral_deg；
 *     - 不继续使用 output->servo 中的扑翼角度；
 *     - 安全中位写入成功时返回 FC_OUTPUT_STATUS_OK。
 *
 * 无效算法输出：
 *   当 output 无效、舵机目标无效、不可达或角度非法时：
 *     - 左右翼统一切换到安全中位；
 *     - 返回 FC_OUTPUT_STATUS_INVALID_OUTPUT；
 *     - 不允许一侧继续扑翼而另一侧单独停止。
 *
 * 本函数不重新限幅一个已经超出配置范围的角度，
 * 而是将其视为算法输出异常并切换到安全中位。
 * ============================================================ */

fc_output_status_t fc_output_write(
    const algo_output_t *output
);

/* ============================================================
 * 强制写入安全中位
 *
 * 使用 fc_output_init() 时复制的：
 *
 *   left neutral：
 *     linkage_config.side[ALGO_WING_LEFT].servo_neutral_deg
 *
 *   right neutral：
 *     linkage_config.side[ALGO_WING_RIGHT].servo_neutral_deg
 *
 * 用于：
 *   - 初始化；
 *   - 停止调度；
 *   - 算法输出异常；
 *   - 飞控层紧急停止；
 *   - 其他不可信输出状态。
 *
 * 本函数：
 *   - 不修改算法状态；
 *   - 不清除故障或急停锁存；
 *   - 不停止周期任务；
 *   - 不切断舵机电源。
 * ============================================================ */

fc_output_status_t fc_output_write_safe(void);

/* ============================================================
 * 获取最近一次提交的逻辑舵机目标
 *
 * target->angle_deg[] 使用与 algo_output_t.servo 相同的单位 deg。
 *
 * 注意：
 *   - 这是最近一次提交给 BSP 的目标；
 *   - 不是舵机实际位置；
 *   - 当前原理图没有舵机位置反馈；
 *   - 是否写入成功仍需结合 fc_output_write() 的返回值。
 * ============================================================ */

fc_output_status_t fc_output_get_last(
    servo_target_t *target
);

#ifdef __cplusplus
}
#endif

#endif /* FC_OUTPUT_H */