# 崩溃日志组件：读取 flash 上的 core dump
#
# espcoredump 只在头文件里被用到吗？不是——esp_core_dump_* 的实现也在里面，
# 而且是**私有依赖**（crash_log.h 不暴露 esp_core_dump.h，只用自定义结构体）。
idf_component_register(
    SRCS         "crash_log.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES espcoredump
)