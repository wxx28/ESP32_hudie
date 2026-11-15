#ifndef ALGO_API_H
#define ALGO_API_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_config.h"
#include "algo_types.h"

/*
 * 仿生蝴蝶算法顶层接口
 *
 * 职责：
 *   1. 保存算法配置和各模块内部状态；
 *   2. 按固定顺序调度滤波、姿态、控制、混控和输出模块；
 *   3. 统一管理快速周期、控制周期和健康检查分频；
 *   4. 向 BSP 提供标准化输入输出；
 *   5. 不包含任何硬件驱动、延时、日志或操作系统调用。
 *
 * 使用限制：
 *   - 单实例设计；
 *   - 不使用动态内存；
 *   - algo_api_step() 只能由一个固定周期任务调用；
 *   - 不允许在 ISR 中调用；
 *   - algo_api_step() 不可重入。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 顶层接口版本号，接口结构或调用约定变化时递增。 */
#define ALGO_API_VERSION    1u

/* ============================================================
 * 接口返回状态
 *
 * 传感器异常、遥控失联等运行故障不作为接口调用错误返回，
 * 而是通过 algo_output_t 中的 safety 和 fault_flags 表达。
 * ============================================================ */

typedef enum {
    ALGO_STATUS_OK = 0,     /* 调用成功，不代表飞行状态正常 */

    /* 输入指针为空、输出指针为空等参数错误。 */
    ALGO_STATUS_INVALID_ARG = -1,

    /* 配置值越界、配置之间存在矛盾。 */
    ALGO_STATUS_INVALID_CONFIG = -2,

    /* 尚未成功调用 algo_api_init()。 */
    ALGO_STATUS_NOT_INITIALIZED = -3,

    /* 当前状态不允许执行所请求的操作。 */
    ALGO_STATUS_NOT_READY = -4,

    /* dt 非有限值、小于等于零或超过配置上限。 */
    ALGO_STATUS_INVALID_DT = -5,

    /* 输入时间戳无效或出现明显回退。 */
    ALGO_STATUS_INVALID_TIMESTAMP = -6,

    /* 算法内部状态不一致。 */
    ALGO_STATUS_INTERNAL_ERROR = -7
} algo_status_t;

/* ============================================================
 * 顶层运行信息
 *
 * 仅用于状态观察和测试，不参与控制计算。
 * ============================================================ */

typedef struct {
    bool initialized;       /* algo_api_init() 已成功完成 */
    bool config_valid;      /* 当前内部配置已通过合法性检查 */

    /* 最近一次接口调用的返回状态。 */
    algo_status_t last_status;

    /* 最近一次实际使用的快速周期。 */
    float last_dt_s;

    /* 最近一次输入时间戳。 */
    uint64_t last_input_timestamp_us;

    /* 顶层调度计数。 */
    uint32_t fast_update_count;     /* 快速周期（400 Hz）累计次数 */
    uint32_t control_update_count;  /* 控制器实际运行次数 */
    uint32_t health_update_count;   /* 健康检查实际运行次数 */

    /* 因分频而跳过控制计算的快速周期数。 */
    uint32_t control_skip_count;

    /* 当前安全状态和故障汇总。 */
    safety_state_t safety_state;
    uint32_t fault_flags;
} algo_api_runtime_t;

/* ============================================================
 * 配置检查
 *
 * 可在 algo_api_init() 前单独调用，便于启动检查和主机测试。
 *
 * 只检查配置本身是否合法：
 *   - 频率、周期和限幅是否有效；
 *   - PID 参数是否为有限值；
 *   - 分频是否非零；
 *   - 启用的连杆几何是否完整；
 *   - 配置版本是否匹配。
 *
 * mechanical_limits_valid == false 不属于配置格式错误，
 * 但会阻止安全状态机进入正常飞行解锁状态。
 *
 * 参数：
 *   config：待检查的配置，不允许为空。
 *
 * 返回：
 *   ALGO_STATUS_OK：配置合法；
 *   ALGO_STATUS_INVALID_ARG：config 为空；
 *   ALGO_STATUS_INVALID_CONFIG：配置值非法或相互矛盾。
 * ============================================================ */

algo_status_t algo_api_config_validate(
    const algo_config_t *config
);

/* ============================================================
 * 初始化
 *
 * 功能：
 *   1. 检查配置；
 *   2. 将配置复制到算法内部；
 *   3. 初始化所有模块；
 *   4. 清空历史故障、积分、滤波和相位状态；
 *   5. 输出默认安全状态；
 *   6. 将安全状态机置为 BOOT。
 *
 * 注意：
 *   - config 可以为 ALGO_CONFIG_DEFAULT 创建的实例；
 *   - 初始化完成后，调用者可以释放或修改自己的 config；
 *   - 重复调用等价于重新初始化整个算法。
 *
 * 参数：
 *   config：初始配置，不允许为空。
 *
 * 返回：
 *   ALGO_STATUS_OK：初始化成功；
 *   ALGO_STATUS_INVALID_ARG：config 为空；
 *   ALGO_STATUS_INVALID_CONFIG：配置未通过合法性检查。
 * ============================================================ */

algo_status_t algo_api_init(
    const algo_config_t *config
);

/* ============================================================
 * 全部复位
 *
 * 保留初始化时使用的配置，但清除：
 *   - 姿态估计；
 *   - 滤波器状态；
 *   - PID 历史；
 *   - 相位和拍数；
 *   - 待提交及当前波形；
 *   - 安全状态和故障计时；
 *   - 运行统计。
 *
 * 复位后重新进入 BOOT，不允许直接恢复到 ARMED。
 *
 * 返回：
 *   ALGO_STATUS_OK：复位成功；
 *   ALGO_STATUS_NOT_INITIALIZED：尚未初始化。
 * ============================================================ */

algo_status_t algo_api_reset(void);

/* ============================================================
 * 获取当前配置副本
 *
 * 第一版运行期间不提供在线修改整个配置的接口。
 * 后续调参接口应单独设计，不能由外部直接修改内部配置指针。
 *
 * 参数：
 *   config_out：输出参数，返回当前内部配置的完整副本。
 *
 * 返回：
 *   ALGO_STATUS_OK：成功；
 *   ALGO_STATUS_INVALID_ARG：config_out 为空；
 *   ALGO_STATUS_NOT_INITIALIZED：尚未初始化。
 * ============================================================ */

algo_status_t algo_api_get_config(
    algo_config_t *config_out
);

/* ============================================================
 * 固定周期更新
 *
 * 调用顺序：
 *
 *   BSP采集IMU、遥控和健康状态
 *              ↓
 *       填充 algo_input_t
 *              ↓
 *       algo_api_step()
 *              ↓
 *   根据 algo_output_t.servo 写入舵机
 *
 * 参数：
 *   input：
 *     当前IMU、遥控和系统健康状态。
 *
 *   dt_s：
 *     距离上次调用的真实时间间隔，单位秒。
 *     正常快速周期为 0.0025s。
 *
 *   output：
 *     始终由本函数完整填充。
 *
 * 返回：
 *   ALGO_STATUS_OK：
 *     本周期计算完成。
 *     不代表飞行状态正常，仍需检查 output->safety。
 *
 *   其他返回值：
 *     表示调用或内部状态错误。
 *     output 将被填充为禁止扑翼的安全输出。
 *
 * 运行故障处理：
 *   IMU无效、遥控失联、姿态异常和连杆不可达等情况，
 *   通过 output->fault_flags 和 output->safety 表达。
 * ============================================================ */

algo_status_t algo_api_step(
    const algo_input_t *input,
    float dt_s,
    algo_output_t *output
);

/* ============================================================
 * 获取运行信息
 *
 * 本接口不改变算法状态，可用于调试、日志和主机侧测试。
 *
 * 参数：
 *   runtime_out：输出参数，返回运行信息快照。
 *
 * 返回：
 *   ALGO_STATUS_OK：成功；
 *   ALGO_STATUS_INVALID_ARG：runtime_out 为空。
 * ============================================================ */

algo_status_t algo_api_get_runtime(
    algo_api_runtime_t *runtime_out
);

/* ============================================================
 * 清除可恢复的锁存故障
 *
 * 仅用于故障恢复流程，不是强制解锁接口。
 *
 * 调用后：
 *   1. 安全状态机重新检查传感器、电源、遥控和机械配置；
 *   2. 仍然存在的故障不会被清除；
 *   3. 严重输出故障不能通过该接口强行忽略；
 *   4. 不会直接进入 ARMED。
 *
 * 返回：
 *   ALGO_STATUS_OK：可恢复故障已清除；
 *   ALGO_STATUS_NOT_INITIALIZED：尚未初始化；
 *   ALGO_STATUS_NOT_READY：当前故障仍不允许恢复。
 * ============================================================ */

algo_status_t algo_api_clear_latched_faults(void);

/* ============================================================
 * 生成安全输出
 *
 * 用途：
 *   1. 初始化失败时由 main 获取禁止扑翼的输出；
 *   2. BSP 检测到算法任务异常时获得统一的安全数据结构；
 *   3. 主机侧测试检查默认输出。
 *
 * 本函数不访问硬件，也不会改变算法当前状态。
 *
 * 参数：
 *   timestamp_us：填入输出的时间戳，us；
 *   fault_flags：触发安全输出的故障标志；
 *   output：输出参数，填充为禁止扑翼的安全输出。
 *
 * 返回：
 *   ALGO_STATUS_OK：成功；
 *   ALGO_STATUS_INVALID_ARG：output 为空。
 * ============================================================ */

algo_status_t algo_api_make_safe_output(
    uint64_t timestamp_us,
    uint32_t fault_flags,
    algo_output_t *output
);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_API_H */