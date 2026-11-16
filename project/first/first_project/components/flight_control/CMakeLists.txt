# 飞控应用层组件：
# 负责周期调度、算法输入组装和安全输出协调。
# 不包含具体控制算法，也不直接实现硬件寄存器驱动。

idf_component_register(
    SRCS
        "flight_control.c"
        "fc_scheduler.c"
        "fc_input.c"
        "fc_output.c"

    # 对外公开 flight_control.h。
    INCLUDE_DIRS
        "include"

    # fc_scheduler.h、fc_input.h、fc_output.h 仅供组件内部使用。
    PRIV_INCLUDE_DIRS
        "."

    # 对外接口会使用 algo_api.h 和公共算法类型。
    REQUIRES
        Middlewares

    # 仅组件内部实现使用硬件适配、定时器和任务接口。
    PRIV_REQUIRES
        BSP
        esp_timer
        freertos
)