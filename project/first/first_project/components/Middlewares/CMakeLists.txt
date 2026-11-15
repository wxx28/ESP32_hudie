# 中间件组件：仿生蝴蝶算法与后续公共模块
idf_component_register(
    SRCS "algo/algo_api.c"
         "algo/algo_attitude.c"
         "algo/algo_control.c"
         "algo/algo_filter.c"
         "algo/algo_linkage.c"
         "algo/algo_mixer.c"
         "algo/algo_phase.c"
         "algo/algo_safety.c"
         "algo/algo_waveform.c"
    INCLUDE_DIRS "algo"
)
