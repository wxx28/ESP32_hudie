# 飞控应用层组件：
# 负责周期调度、算法输入组装、无线链路和安全输出协调。
# 不包含具体控制算法，也不直接实现硬件寄存器驱动。

idf_component_register(
    SRCS
        "flight_control.c"
        "fc_scheduler.c"
        "fc_input.c"
        "fc_output.c"
        "fc_link.c"
        "fc_config.c"

    # 对外公开 flight_control.h、fc_config.h。
    INCLUDE_DIRS
        "include"

    # fc_scheduler.h、fc_input.h、fc_output.h、fc_link.h 仅供组件内部使用。
    PRIV_INCLUDE_DIRS
        "."

    # 对外接口会使用 algo_api.h 和公共算法类型。
    REQUIRES
        Middlewares

    # 仅组件内部实现使用硬件适配、链路、定时器和任务接口。
    PRIV_REQUIRES
        BSP
        hudie_link
        esp_timer
        freertos
        log
)
