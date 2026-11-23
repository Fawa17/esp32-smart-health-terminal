/*
 * watchdog.h，看门狗与复位原因记录（ESP-IDF 版）
 *
 * 和 STM32 版最本质的区别：
 *
 * STM32 的 IWDG 只有一个计数器：任何一个任务喂一次，计数器就清了。
 * 所以"谁卡死了"这件事，只能靠 BKP 寄存器里自己写标记、下次开机再猜。
 *
 * ESP32-S3 有三个层次，能力逐级更高：
 *   1. 中断看门狗 IWDT，中断/关中断太久，直接 panic（内核自带）。
 *   2. 任务看门狗 TWDT，每订阅一个任务就多一个"欠账人"，谁没按时来
 *      esp_task_wdt_reset() 就 panic，并且 **panic 信息里直接打印卡死的任务名
 *      和调用栈**。这是 STM32 做不到的。
 *   3. RTC 保留内存（RTC_NOINIT_ATTR）—— BKP->DRx 的等价物，软件复位、
 *      看门狗复位、深睡唤醒都不会清，只有彻底掉电才丢。
 *
 * 本组件在 TWDT 之上再加一层"应用级心跳"：每个业务任务按自己的周期来
 * watchdog_kick(slot)，监控任务发现谁超时，先记日志、调一次恢复回调，
 * 连续 N 次还没好就主动复位，并把"是哪个 slot"写进 RTC 内存 ——
 * 这样下次开机能看到"上次是 MAX30102 采样任务卡死"，而不是笼统的"看门狗复位"。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* ==================== 复位原因编码 ====================
 * BKP 记录 */
#define RST_REASON_POWER_ON     0x0001   /* 上电复位：正常启动 */
#define RST_REASON_PIN          0x0002   /* 复位引脚 / 外部复位 */
#define RST_REASON_SOFTWARE     0x0003   /* 软件复位：esp_restart() */
#define RST_REASON_IWDG         0x0004   /* 独立看门狗复位 */
#define RST_REASON_WWDG         0x0005   /* 窗口看门狗复位*/
#define RST_REASON_LOWPOWER     0x0006   /* 低功耗唤醒 */
#define RST_REASON_STACK_OVF    0x0007   /* 栈溢出 */
#define RST_REASON_HARDFAULT    0x0008   /* 硬件错误 / 非法访问 */
#define RST_REASON_TASK_WDT     0x0009   /* 任务看门狗超时 */
#define RST_REASON_INT_WDT      0x000A   /* 中断看门狗超时 */
#define RST_REASON_PANIC        0x000B   /* 内核 panic（含 LoadProhibited 等） */
#define RST_REASON_BROWNOUT     0x000C   /* 掉电复位：电源不稳 */
#define RST_REASON_HEARTBEAT    0x000D   /* 本组件的应用级心跳检测到任务卡死 */

/* ==================== 心跳槽位 ====================
 * 新增受监控的任务就在这里加一个枚举值，编译期会检查数组大小 */
typedef enum {
    WD_SLOT_MPU = 0,     /* MPU6050 采样任务，40ms 一次 */
    WD_SLOT_MAX,         /* MAX30102 采样任务，10ms 一次 */
    WD_SLOT_TEMP,        /* NTC 温度任务，2s 一次 */
    WD_SLOT_NET,         /* MQTT 上行任务，2s 一次 */
    WD_SLOT_UI,          /* 界面刷新，500ms 一次 */
    WD_SLOT_CNT
} wd_slot_t;

/* 恢复回调：任务第一次超时时被调一次（在监控任务上下文里跑，
 * 不能阻塞太久）。返回后如果它还是不来心跳，下一次超时就复位。 */
typedef void (*wd_recover_cb_t)(void);

/** @brief 读复位原因 + 配置 TWDT。要尽早调用（所有业务任务之前） */
esp_err_t watchdog_init(void);

/** @brief 注册一个被监控的任务。timeout_ms 建议 >= 该任务周期的 3 倍 */
esp_err_t watchdog_register(wd_slot_t slot, const char *name, uint32_t timeout_ms);

/** @brief 设置"第一次超时时尝试的恢复动作"，可不设 */
esp_err_t watchdog_set_recover_cb(wd_slot_t slot, wd_recover_cb_t cb);

/** @brief 任务来打卡。未注册的 slot 直接忽略，可以无脑调用 */
void watchdog_kick(wd_slot_t slot);

/** @brief 建监控任务。之后必须保证各业务任务真的在 kick */
esp_err_t watchdog_start(void);

/** @brief 注册"现在是否处于免检状态"的回调（典型用途：OTA 升级期间）。
 *
 *  OTA 写 flash 时每擦一个 4KB 扇区要几十毫秒，期间 cache 全关、中断被推迟，
 *  业务任务连续几秒轮不到 CPU 是正常的。这时候如果照常判"任务卡死"，
 *  会把正在升级的板子复位掉，现象就是"看到 GET 了、板子重启了、版本没变"。
 *  回调返回 true 时：重置所有槽位计时、不判定、不复位。 */
void watchdog_set_pause_check(bool (*fn)(void));

/** @brief 打印所有任务的栈余量（排查栈溢出用） */
void watchdog_dump_tasks(void);

uint16_t    watchdog_last_reset_reason(void);      /* 上次复位原因（编码见上） */
const char *watchdog_reason_text(uint16_t reason); /* 编码转中文描述 */
bool        watchdog_is_running(void);

/* ==================== 兼容 STM32 版接口 ====================
 * 原来的实现是往 BKP->DRx 写，这里映射到 RTC 保留内存，
 * 语义一致：复位后仍然读得到，掉电才丢。 */
void Save_Reset_Reason(uint16_t reason);
void Save_Task_Name(const char *name);
void Save_HardFault_PC(uint32_t pc);
void Print_Last_Reset_Reason(void);