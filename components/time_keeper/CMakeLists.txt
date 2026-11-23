# time_keeper：把系统时间存进 NVS，保证断电后时间不丢
#
# 只依赖 nvs_flash（NVS 读写）。
idf_component_register(
    SRCS         "time_keeper.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES nvs_flash
)