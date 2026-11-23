# mpu6050 组件
#
# mpu6050.h 只暴露 esp_err_t / stdint 类型，不 include bsp_i2c.h，
# 所以 bsp_i2c 可以放 PRIV_REQUIRES（私有依赖），不污染上层。
idf_component_register(
    SRCS         "mpu6050.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES bsp_i2c  esp_timer
)