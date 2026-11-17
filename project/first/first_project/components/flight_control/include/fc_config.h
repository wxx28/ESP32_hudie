#ifndef FC_CONFIG_H
#define FC_CONFIG_H

#include "algo_config.h"

/*
 * V1 板 + 康师傅 V1 机构的算法配置（在 ALGO_CONFIG_DEFAULT 基础上覆盖）。
 * 数值来源见 fc_config.c 注释；修改请走 menuconfig "蝴蝶飞控参数"。
 */

#ifdef __cplusplus
extern "C" {
#endif

void fc_config_v1(algo_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* FC_CONFIG_H */
