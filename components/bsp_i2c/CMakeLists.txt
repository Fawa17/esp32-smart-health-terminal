# bsp_i2c 组件：只负责 I2C 总线的初始化与收发，不依赖任何具体器件。
#
# 【注意】driver 必须放 REQUIRES 而不是 PRIV_REQUIRES：
#   bsp_i2c.h 是公开头文件，里面 include 了 driver/i2c.h，
#   任何一个 include 了 bsp_i2c.h 的组件都要能顺着依赖找到 driver 的 include 路径。
#   放 PRIV 的后果是编译期直接报
#   "fatal error: driver/i2c.h: No such file or directory"。
#   esp_rom 只在 .c 里用（esp_rom_delay_us），可以继续私有。
idf_component_register(
    SRCS         "bsp_i2c.c"
    INCLUDE_DIRS "include"
    REQUIRES     driver
    PRIV_REQUIRES esp_rom
)