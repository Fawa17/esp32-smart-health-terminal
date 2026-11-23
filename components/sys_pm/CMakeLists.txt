# sys_pm 组件：屏幕电源状态机 + 物理按键唤醒
#
# 依赖很轻：driver（gpio_config / gpio_isr_handler_add）、esp_timer（取毫秒时间戳）。
# 它**不认识** display 和 touch，两者通过 sys_pm_hooks_t 回调接进来 ——
# 这样以后换屏、换触摸方案都不用动这个组件。
idf_component_register(
    SRCS         "sys_pm.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_timer
)