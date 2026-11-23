/*
 * sys_pm.h - 屏幕电源管理 + 按键唤醒
 *
 * 模块 8 针没引出背光脚和触摸 IRQ，屏幕常亮、触摸常轮询，也没有"用户来了"的硬件信号。
 * 加一个物理按键做唤醒源，顺带解决三件事：
 *   1. 提供真实中断（按键下降沿），比触摸轮询灵敏
 *   2. 息屏时触摸降到 0 Hz，I2C 让给 MPU6050 / MAX30102
 *   3. 只有实体按键能唤醒，衣物摩擦不会亮屏
 *
 * SLEEP:  面板 DISPOFF+SLPIN，触摸任务挂起，LVGL 暂停；传感器与网络照常运行
 * ACTIVE: 面板 DISPON，触摸 50 Hz，LVGL 正常刷新
 *
 * 中断里只做 vTaskNotifyGiveFromISR，控制屏幕、调 LVGL 都留在任务上下文。
 * 未使用 esp_light_sleep_start()：手动轻睡会断开 WiFi，需要 CONFIG_PM_ENABLE 那套。
 */
#ifndef SYS_PM_H
#define SYS_PM_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYS_PM_SLEEP  = 0,
    SYS_PM_ACTIVE = 1,
} sys_pm_state_t;

/** 进入/退出休眠时要做的事。注意都会在 sys_pm 任务上下文里被调用。 */
typedef struct {
    void (*on_sleep)(void);
    void (*on_wake)(void);
} sys_pm_hooks_t;

/**
 * 初始化按键中断与状态机任务。
 * @param hooks 可以为 NULL（那就只维护状态，不动硬件）
 *
 * 必须在显示屏初始化之后调用（on_sleep/on_wake 里要操作面板）。
 */
esp_err_t sys_pm_init(const sys_pm_hooks_t *hooks);

/** 当前是否处于活跃状态（触摸任务据此决定轮询还是挂起）。 */
bool sys_pm_is_active(void);

sys_pm_state_t sys_pm_state(void);

/** 有用户操作（触摸/按键），重置无操作计时。可从任意任务调用。 */
void sys_pm_notify_activity(void);

/** 主动唤醒 / 主动息屏。UI 上的电源按钮可以调这两个。 */
void sys_pm_wake(void);
void sys_pm_sleep(void);

/**
 * 注册"现在不能睡"的判据，比如 OTA 下载中。
 * 返回 true 时，超时逻辑和按键都不会让系统进入休眠。
 */
void sys_pm_set_busy_check(bool (*fn)(void));

/** 板上那颗按键原始的按下状态（已按 ACTIVE_LOW 配置换算成"true=按下"）。 */
bool sys_pm_key_pressed(void);

#ifdef __cplusplus
}
#endif

#endif /* SYS_PM_H */