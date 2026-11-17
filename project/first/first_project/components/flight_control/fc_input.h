#ifndef FC_INPUT_H
#define FC_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "algo_types.h"

/*
 * 飞控输入适配模块：把硬件数据组装成 algo_input_t
 *
 *   IMU    ：BSP/mpu6500（SPI 轮询），已转换到机体系 SI 单位；
 *   遥控   ：fc_link 收到的最新 rc_frame_t，换算成 [-1, 1] / [0, 1]；
 *   健康   ：IMU 是否就绪、电池电压是否高于降落电压。
 *
 * 舵机没有位置反馈，servo_fault 恒为 false（不虚构堵转检测）。
 * 只能在飞控任务里调用。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FC_INPUT_STATUS_OK = 0,
    FC_INPUT_STATUS_HARDWARE_ERROR = -1,
    FC_INPUT_STATUS_INVALID_ARG = -2
} fc_input_status_t;

/* 遥控帧超过该时间未更新即视为无效（安全状态机按 age_ms 再判断降级/失控）。 */
#define FC_INPUT_RC_VALID_MS      100u
/* 低于该电压 power_good = false（滤波后，mV）。 */
#define FC_INPUT_BAT_LAND_MV      3400u

/* 初始化 IMU / 电池 ADC，并做一次陀螺零偏校准（需静止约 1 s）。 */
fc_input_status_t fc_input_init(void);

/* 采集一次全部输入。now_us 由调度器给出。 */
fc_input_status_t fc_input_collect(uint64_t now_us, algo_input_t *input);

/*
 * 遥控请求的陀螺校准（RC_FLAG_CALIBRATE 上升沿）。
 * 返回 true 表示有新的校准请求需要处理，由飞控任务在上锁时执行 fc_input_calibrate()。
 */
bool fc_input_take_calibrate_request(void);

/* 阻塞约 1 s 重新校准陀螺零偏。 */
fc_input_status_t fc_input_calibrate(void);

/* 最近一次电池电压（mV），用于遥测。 */
uint32_t fc_input_battery_mv(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_INPUT_H */
