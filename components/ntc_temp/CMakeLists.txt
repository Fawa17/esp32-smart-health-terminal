# ntc_temp 组件：NTC 热敏模块（AO 模拟输出）驱动
#
# 公开头文件只 include 了 esp_err.h，ADC 相关头文件不外泄 → 依赖可以全放 PRIV。
idf_component_register(
    SRCS         "ntc_temp.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_adc driver
)