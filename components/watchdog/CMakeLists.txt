# watchdog：任务看门狗 + 应用级心跳 + 复位原因记录
#
# 依赖说明：
#   - esp_timer：用 esp_timer_get_time() 算超时（比 tick 精度高，且不受 HZ 影响）
#   - esp_system / freertos：IDF 的通用依赖，默认就在自动依赖里，不用显式写
idf_component_register(
    SRCS         "watchdog.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer
)