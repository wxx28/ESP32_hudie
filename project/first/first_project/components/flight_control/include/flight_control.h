#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_api.h"
#include "algo_types.h"

/*
 * 仿生蝴蝶飞控应用层公开接口
 *
 * 适配第四代仿生蝴蝶 V1 原理图：
 *
 *   主控：
 *     ESP32-C3-MINI-1-N4
 *
 *   主 IMU：
 *     MPU-6500，使用 SPI。
 *     SDO  -> IO4
 *     SDI  -> IO5
 *     SCLK -> IO6
 *     CS   -> IO7
 *     INT 未连接，因此采用周期轮询，不依赖 IMU 中断。
 *
 *   左右舵机：
 *     左翼：IO1 / PWM2，经隔离和驱动电路连接至 J3 / PWM_L。
 *     右翼：IO8 / PWM1，经隔离和驱动电路连接至 J2 / PWM_R。
 *
 *   遥控：
 *     J6 提供 SBUS 接口。
 *     SBUS 经 Q1 反相后进入 RXD0。
 *     UART0 同时连接调试接口，后续 BSP 必须避免日志与 SBUS 冲突。
 *
 *   电池检测：
 *     ADC_BAT -> IO0。
 *     VBAT_1S 经 100 kΩ / 100 kΩ 分压后进入 ADC。
 *
 *   其他传感器：
 *     LIS2MDLTR 磁力计和 SPL06-001 气压计共用 I2C。
 *     SCL -> IO2，SDA -> IO3。
 *     第一版基础姿态算法不依赖磁力计和气压计。
 *
 * 分层约定：
 *   1. 本头文件不定义任何 GPIO、SPI、UART 或 PWM 的底层寄存器配置；
 *   2. 上述引脚在 BSP 中统一定义，其他模块不得重复定义；
 *   3. 算法计算全部由 Middlewares/algo 完成；
 *   4. 本层只负责硬件数据组装、周期调度、安全输出和状态管理；
 *   5. 左右舵机的电气反相、脉宽转换和 PWM 写入由 BSP 完成。
 *
 * 使用限制：
 *   - 单实例设计；
 *   - 不使用动态内存；
 *   - 不允许在 ISR 中调用本头文件中的接口；
 *   - flight_control_start() 不会自动解锁或让电机开始扑翼；
 *   - 是否允许输出始终由 algo_output_t 中的安全状态决定。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define FLIGHT_CONTROL_API_VERSION       1u

/* 名义快速周期为 2.5 ms，即 400 Hz。 */
#define FLIGHT_CONTROL_FAST_FREQUENCY_HZ 400u
#define FLIGHT_CONTROL_FAST_PERIOD_US    2500u

/* ============================================================
 * 飞控接口返回状态
 *
 * 遥控失联、IMU 采样暂时无效等运行故障，不一定作为接口错误
 * 返回，而是通过快照中的 safety 和 fault_flags 表达。
 * ============================================================ */

typedef enum {
    FC_STATUS_OK = 0,

    /* 必要指针为空或参数不合法。 */
    FC_STATUS_INVALID_ARG = -1,

    /* 算法或飞控配置非法。 */
    FC_STATUS_INVALID_CONFIG = -2,

    /* 尚未成功调用 flight_control_init()。 */
    FC_STATUS_NOT_INITIALIZED = -3,

    /* 当前生命周期状态不允许执行该操作。 */
    FC_STATUS_NOT_READY = -4,

    /* 必需的硬件或驱动初始化失败。 */
    FC_STATUS_HARDWARE_ERROR = -5,

    /* 算法初始化或周期调用发生接口错误。 */
    FC_STATUS_ALGORITHM_ERROR = -6,

    /* 调度任务、定时器或内部状态操作失败。 */
    FC_STATUS_SCHEDULER_ERROR = -7
} fc_status_t;

/* ============================================================
 * 飞控状态快照
 *
 * 仅用于状态观察、日志和测试，不参与实时控制计算。
 * ============================================================ */

typedef struct {
    /* 飞控应用层是否已经完成初始化。 */
    bool initialized;

    /* 400 Hz 周期调度是否正在运行。 */
    bool running;

    /* 飞控层急停是否已锁存。 */
    bool emergency_stop_latched;

    /* 已完成的快速周期数量。 */
    uint32_t fast_cycle_count;

    /* 算法接口异常次数，不包含普通传感器无效和遥控失联。 */
    uint32_t algorithm_error_count;

    /* 最近一次 algo_api_step() 的接口返回状态。 */
    algo_status_t last_algo_status;

    /*
     * 最近一次完整算法输出。
     *
     * output.safety：当前安全状态；
     * output.fault_flags：故障汇总；
     * output.servo：最近一次逻辑舵机目标。
     */
    algo_output_t output;
} flight_control_snapshot_t;

/* ============================================================
 * 初始化飞控
 *
 * 推荐由 main 在完成最基本的系统初始化后调用一次。
 *
 * 初始化职责：
 *   1. 检查并复制算法配置；
 *   2. 初始化安全输出，确保舵机不会收到未定义目标；
 *   3. 调用 algo_api_init() 初始化纯算法；
 *   4. 通过 fc_input 初始化 IMU、SBUS 和电池检测接口；
 *   5. 通过 fc_output 初始化左右舵机输出接口；
 *   6. 初始化周期调度器，但不启动周期任务；
 *   7. 清除运行统计并进入安全状态。
 *
 * 注意：
 *   - 初始化成功不代表已经解锁；
 *   - 磁力计和气压计第一版属于可选传感器；
 *   - 没有收到遥控信号应进入等待或失控状态，
 *     不能直接等同于 UART 驱动初始化失败；
 *   - 机械参数未确认时，算法可以运行测试，
 *     但安全状态机不得允许正常飞行解锁。
 *
 * 参数：
 *   algo_config：
 *     算法配置，不允许为空。
 *     初始化后配置被复制，调用者可以修改或释放自己的副本。
 *
 * 返回：
 *   FC_STATUS_OK：初始化成功；
 *   FC_STATUS_INVALID_ARG：algo_config 为空；
 *   FC_STATUS_INVALID_CONFIG：配置非法；
 *   FC_STATUS_HARDWARE_ERROR：必需硬件初始化失败；
 *   FC_STATUS_ALGORITHM_ERROR：算法初始化失败；
 *   FC_STATUS_NOT_READY：飞控正在运行，不能重新初始化。
 * ============================================================ */

fc_status_t flight_control_init(
    const algo_config_t *algo_config
);

/* ============================================================
 * 启动飞控周期调度
 *
 * 启动后：
 *   - 按名义 400 Hz 周期采集输入；
 *   - 填充 algo_input_t；
 *   - 调用 algo_api_step()；
 *   - 检查安全指令；
 *   - 将有效逻辑舵机目标交给 fc_output。
 *
 * 本函数只启动调度，不执行解锁。
 * 上电后仍必须经过 BOOT、SELFTEST 和 STANDBY，
 * 满足安全条件后才允许进入 ARMED。
 *
 * 重复调用且调度已经运行时，返回 FC_STATUS_OK，
 * 不得重复创建任务或定时器。
 *
 * 返回：
 *   FC_STATUS_OK：调度已运行；
 *   FC_STATUS_NOT_INITIALIZED：尚未初始化；
 *   FC_STATUS_NOT_READY：急停锁存尚未解除；
 *   FC_STATUS_SCHEDULER_ERROR：调度启动失败。
 * ============================================================ */

fc_status_t flight_control_start(void);

/* ============================================================
 * 停止飞控周期调度
 *
 * 停止时：
 *   1. 停止新的硬件采样和算法更新；
 *   2. 输出安全舵机目标；
 *   3. 清除控制器和姿态运行状态；
 *   4. 保留初始化配置；
 *   5. 再次启动后从安全初始状态重新运行。
 *
 * 重复停止属于合法操作，返回 FC_STATUS_OK。
 *
 * 本函数不是 ISR 急停接口，不得在 ISR 中调用。
 * ============================================================ */

fc_status_t flight_control_stop(void);

/* ============================================================
 * 飞控层紧急停止
 *
 * 用于严重硬件异常、调试急停或上层安全保护。
 *
 * 行为：
 *   - 锁存 emergency_stop_latched；
 *   - 禁止后续正常扑翼输出；
 *   - 请求 fc_output 立即写入安全舵机目标；
 *   - 保留故障和最近一次输出，便于事后检查。
 *
 * 急停锁存不能通过遥控指令自动解除。
 * 必须先停止调度，并重新执行完整初始化和自检。
 *
 * 安全舵机目标通常对应零扑翼幅度和逻辑中位，
 * 不表示切断舵机电源，也不包含任何电气反相。
 *
 * 本函数不允许在 ISR 中直接调用。
 * ISR 或底层驱动只能设置急停请求，由任务上下文执行本函数。
 * ============================================================ */

fc_status_t flight_control_emergency_stop(void);

/* ============================================================
 * 清除可恢复的锁存故障
 *
 * 只是将请求传递给算法安全状态机，不是强制解锁接口。
 *
 * 注意：
 *   - 仍然存在的硬件故障不会被清除；
 *   - 不能清除飞控层急停锁存；
 *   - 不会直接进入 ARMED；
 *   - 不会在清故障后立即恢复扑翼输出。
 * ============================================================ */

fc_status_t flight_control_clear_faults(void);

/* ============================================================
 * 获取飞控状态快照
 *
 * 不停止调度，也不改变算法状态。
 *
 * 实现时应通过临界区或其他一致快照机制，
 * 避免调用者读到正在更新的一半数据。
 *
 * 参数：
 *   snapshot：输出快照，不允许为空。
 *
 * 返回：
 *   FC_STATUS_OK：获取成功；
 *   FC_STATUS_INVALID_ARG：snapshot 为空；
 *   FC_STATUS_NOT_INITIALIZED：尚未初始化。
 * ============================================================ */

fc_status_t flight_control_get_snapshot(
    flight_control_snapshot_t *snapshot
);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_CONTROL_H */