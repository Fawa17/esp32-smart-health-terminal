# max30102 组件：心率/血氧传感器驱动（硬件 I2C + GPIO 中断）
#
# driver 走 PRIV_REQUIRES 即可：本组件的**公开头文件**只 include 了 esp_err.h
# 和 FreeRTOS，没有把 driver/gpio.h 暴露出去（引脚号用裸数字宏 MAX30102_INT_GPIO_IO），
# 所以不像 bsp_i2c 那样必须放公开依赖。
idf_component_register(
    SRCS         "max30102.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_timer  bsp_i2c
)