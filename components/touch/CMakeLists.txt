# touch 组件：I2C 触摸，自动识别 CST816 / FT6236
#
# 复用 bsp_i2c 已经初始化好的那条总线，不再单独建 I2C 端口。
idf_component_register(
    SRCS         "touch.c"
    INCLUDE_DIRS "include"
    REQUIRES     bsp_i2c
    # esp_timer：中断里取时间戳做"中断风暴"统计
    # driver    ：gpio_config / gpio_isr_handler_add（红外模式才用，但编译期就得能找到）
    PRIV_REQUIRES esp_timer driver
)