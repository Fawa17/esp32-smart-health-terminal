# display 组件：240x296 ST7789 屏，esp_lcd 直绘
#
# esp_lcd 与 driver 都只在 .c 里用（disp.h 只依赖 esp_err.h），
# 所以可以放 PRIV_REQUIRES —— 和 bsp_i2c 的情况正好相反。
idf_component_register(
    SRCS         "disp.c" "disp_font.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_lcd driver
)