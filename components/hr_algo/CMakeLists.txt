# hr_algo 组件：心率 / 血氧算法（纯计算，无 IO 依赖）
#
# 公开头文件只用 stdint / stdbool / esp_err.h，没有任何外设头文件，
# 所以这里不需要 REQUIRES 任何驱动组件。
idf_component_register(
    SRCS         "hr_algo.c"
    INCLUDE_DIRS "include"
)