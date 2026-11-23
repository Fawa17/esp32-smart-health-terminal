# ds1302 组件：三线 RTC，GPIO 位操作实现
#
# 公开头文件 ds1302.h 只包含 esp_err.h，没有暴露 driver/ 的头，
# 所以 driver 可以放 PRIV_REQUIRES（不像 bsp_i2c 那样必须公开）。
idf_component_register(
    SRCS         "ds1302.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_rom esp_timer
)