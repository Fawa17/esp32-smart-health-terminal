/*
 * watchdog.c，任务看门狗 + 应用级心跳 + 复位原因记录
 *
 * 三层保护：
 *   ① 内核 IWDT：中断关太久 → panic（不用管，默认开着）
 *   ② TWDT    ：监控任务自己订阅，循环里 reset；同时把空闲任务也订阅上，
 *                用来抓"有任务饿死 IDLE 导致底层驱动饿死"这种隐形卡死
 *   ③ 应用心跳：业务任务定时 kick，监控任务查超时 → 先恢复、后复位
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "sdkconfig.h"

#include "watchdog.h"

static const char *TAG = "watchdog";

/* ==================== 掉电保持区（BKP->DRx 的等价物） ====================
 * RTC_NOINIT_ATTR 的变量放在 RTC 慢速内存里：
 *   - 软件复位、TWDT 复位、panic 复位：内容保留
 *   - 深睡唤醒：保留
 *   - 彻底断电：内容随机 → 所以必须有 magic 校验，不能直接信任 */
#define WD_RTC_MAGIC 0x57445431u   /* 'WDT1' */

RTC_NOINIT_ATTR static uint32_t s_rtc_magic;
RTC_NOINIT_ATTR static uint32_t s_rtc_reason;
RTC_NOINIT_ATTR static uint32_t s_rtc_slot;   /* 哪个 slot 卡死 */
RTC_NOINIT_ATTR static uint32_t s_rtc_name;   /* 任务名前 4 个字符 */
RTC_NOINIT_ATTR static uint32_t s_rtc_pc;     /* 出错 PC（兼容接口用） */

static uint16_t s_last_reason = RST_REASON_POWER_ON;

typedef struct {
    const char     *name;
    uint32_t        timeout_ms;
    uint32_t        last_kick_ms;
    uint8_t         miss;          /* 连续超时次数 */
    bool            registered;
    wd_recover_cb_t recover;
} wd_entry_t;

static wd_entry_t   s_slots[WD_SLOT_CNT];
static TaskHandle_t s_task;
static bool         s_running;

/* 免检回调（OTA 期间用），NULL 表示不启用 */
static bool (*s_pause_check)(void) = NULL;

void watchdog_set_pause_check(bool (*fn)(void))
{
    s_pause_check = fn;
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static uint16_t hw_reason_to_code(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return RST_REASON_POWER_ON;
    case ESP_RST_EXT:      return RST_REASON_PIN;
    case ESP_RST_SW:       return RST_REASON_SOFTWARE;
    case ESP_RST_PANIC:    return RST_REASON_PANIC;
    case ESP_RST_INT_WDT:  return RST_REASON_INT_WDT;
    case ESP_RST_TASK_WDT: return RST_REASON_TASK_WDT;
    case ESP_RST_WDT:      return RST_REASON_IWDG;
    case ESP_RST_BROWNOUT: return RST_REASON_BROWNOUT;
    case ESP_RST_DEEPSLEEP:return RST_REASON_LOWPOWER;
    default:               return 0;
    }
}

const char *watchdog_reason_text(uint16_t reason)
{
    switch (reason) {
    case RST_REASON_POWER_ON:  return "上电复位（正常启动）";
    case RST_REASON_PIN:       return "复位引脚 / 外部复位";
    case RST_REASON_SOFTWARE:  return "软件复位 esp_restart()";
    case RST_REASON_IWDG:      return "看门狗复位";
    case RST_REASON_WWDG:      return "窗口看门狗复位";
    case RST_REASON_LOWPOWER:  return "低功耗唤醒";
    case RST_REASON_STACK_OVF: return "栈溢出";
    case RST_REASON_HARDFAULT: return "硬件错误 / 非法内存访问";
    case RST_REASON_TASK_WDT:  return "任务看门狗超时";
    case RST_REASON_INT_WDT:   return "中断看门狗超时（中断关太久）";
    case RST_REASON_PANIC:     return "内核 panic";
    case RST_REASON_BROWNOUT:  return "掉电复位（电源不稳）";
    case RST_REASON_HEARTBEAT: return "应用心跳超时（业务任务卡死）";
    default:                   return "未知";
    }
}

uint16_t watchdog_last_reset_reason(void)
{
    return s_last_reason;
}

static void print_reset_report(void)
{
    esp_reset_reason_t hw = esp_reset_reason();
    uint16_t hw_code = hw_reason_to_code(hw);
    uint16_t sw_code = 0;
    bool     has_sw  = false;

    /* RTC 内存里有本组件写入的记录时才可信 */
    if (s_rtc_magic == WD_RTC_MAGIC) {
        sw_code = (uint16_t)(s_rtc_reason & 0xFFFFu);
        has_sw  = (sw_code != 0);
    }

    /* 优先相信"软件记录"：它比硬件寄存器知道得更细 */
    s_last_reason = has_sw ? sw_code : hw_code;

    ESP_LOGW(TAG, "########## BOOT ##########");
    ESP_LOGW(TAG, "硬件复位原因：%s（esp_reset_reason=%d）",
             watchdog_reason_text(hw_code), (int)hw);

    if (has_sw) {
        char name[5] = {0};
        name[0] = (char)((s_rtc_name >> 0) & 0xFF);
        name[1] = (char)((s_rtc_name >> 8) & 0xFF);
        name[2] = (char)((s_rtc_name >> 16) & 0xFF);
        name[3] = (char)((s_rtc_name >> 24) & 0xFF);

        ESP_LOGE(TAG, "软件记录原因：%s", watchdog_reason_text(sw_code));
        if (sw_code == RST_REASON_HEARTBEAT) {
            ESP_LOGE(TAG, "卡死的槽位 #%lu（任务名 \"%s\"）",
                     (unsigned long)s_rtc_slot, name);
        }
        if (sw_code == RST_REASON_STACK_OVF && name[0] != '\0') {
            ESP_LOGE(TAG, "栈溢出任务：\"%s\"（%s）", name, "请加大该任务栈");
        }
        if (s_rtc_pc != 0) {
            ESP_LOGE(TAG, "出错地址 PC = 0x%08lx", (unsigned long)s_rtc_pc);
        }
    } else {
        ESP_LOGI(TAG, "无自定义异常记录");
    }
    ESP_LOGW(TAG, "==========================");

    /* 用掉就清，避免下次开机重复显示 */
    s_rtc_magic  = 0;
    s_rtc_reason = 0;
    s_rtc_slot   = 0;
    s_rtc_name   = 0;
    s_rtc_pc     = 0;
}

esp_err_t watchdog_init(void)
{
    memset(s_slots, 0, sizeof(s_slots));

    print_reset_report();

    /* TWDT 配置：
     *   - timeout 取 Kconfig
     *   - idle_core_mask 把两个核的空闲任务都订阅上：
     *     任何任务把它饿死超过超时时间，就会 panic 打印现场，
     *     而不是变成"系统变慢"这种查不出来的问题
     *   - trigger_panic = true：卡死就复位。只在超时那一刻生效，
     *     平时不影响任何东西 */
    uint32_t idle_mask = 0;
#if portNUM_PROCESSORS > 1
    idle_mask = (1u << 0) | (1u << 1);
#else
    idle_mask = (1u << 0);
#endif
    /* 配置 TWDT，内核会在超时那一刻 panic 打印现场，然后复位 */
    esp_task_wdt_config_t cfg = {
        .timeout_ms      = (uint32_t)CONFIG_WATCHDOG_TWDT_TIMEOUT_S * 1000u,
        .idle_core_mask  = idle_mask,
        .trigger_panic   = true,
    };

    /* CONFIG_ESP_TASK_WDT_INIT=y 时，内核已经初始化过 TWDT，
     * 再调 init 会返回 ESP_ERR_INVALID_STATE，这不是错误，改调 reconfigure */
    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        err = esp_task_wdt_reconfigure(&cfg);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWDT 配置失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "TWDT 已配置：%d 秒超时，触发 panic=%s",
             CONFIG_WATCHDOG_TWDT_TIMEOUT_S, "是");
    return ESP_OK;
}

esp_err_t watchdog_register(wd_slot_t slot, const char *name, uint32_t timeout_ms)
{
    if ((int)slot < 0 || slot >= WD_SLOT_CNT || name == NULL || timeout_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    wd_entry_t *e = &s_slots[slot];
    e->name         = name;
    e->timeout_ms   = timeout_ms;
    e->last_kick_ms = now_ms();     /* 从注册这一刻开始计时，别一上来就判超时 */
    e->miss         = 0;
    e->registered   = true;

    ESP_LOGI(TAG, "监控 %s：%lu ms 内必须有心跳",
             name, (unsigned long)timeout_ms);
    return ESP_OK;
}

esp_err_t watchdog_set_recover_cb(wd_slot_t slot, wd_recover_cb_t cb)
{
    if ((int)slot < 0 || slot >= WD_SLOT_CNT) {
        return ESP_ERR_INVALID_ARG;
    }
    s_slots[slot].recover = cb;
    return ESP_OK;
}

/** @brief 任务来打卡 */
void watchdog_kick(wd_slot_t slot)
{
    if ((int)slot < 0 || slot >= WD_SLOT_CNT) {
        return;
    }
    if (!s_slots[slot].registered) {
        return;                     /* 没注册就静默忽略，调用方不用判断 */
    }
    s_slots[slot].last_kick_ms = now_ms();
}

static void monitor_task(void *arg)
{
    (void)arg;

    /* TWDT 订阅监控任务自己，防止它卡死导致整个系统卡死 */
    (void)esp_task_wdt_add(NULL);

    ESP_LOGI(TAG, "监控任务启动（周期 %d ms）", CONFIG_WATCHDOG_PERIOD_MS);
    /* 监控任务循环：
     *   - 每轮先喂 TWDT
     *   - 再检查每个槽位的心跳
     *   - 超时就先尝试恢复，连续 N 次还没好就主动复位 */
    for (;;) {
        (void)esp_task_wdt_reset();

        uint32_t now = now_ms();

        /* 升级（OTA）期间不判卡死。 */
        if (s_pause_check != NULL && s_pause_check()) {
            for (int i = 0; i < (int)WD_SLOT_CNT; i++) {
                s_slots[i].last_kick_ms = now;
                s_slots[i].miss = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(CONFIG_WATCHDOG_PERIOD_MS));
            continue;
        }

        for (int i = 0; i < (int)WD_SLOT_CNT; i++) {
            wd_entry_t *e = &s_slots[i];
            if (!e->registered) {
                continue;
            }

            uint32_t elapsed = now - e->last_kick_ms;

            if (elapsed > e->timeout_ms) {
                if (e->miss == 0) {
                    ESP_LOGW(TAG, "任务 \"%s\" 已 %lu ms 没来心跳（阈值 %lu ms）",
                             e->name, (unsigned long)elapsed,
                             (unsigned long)e->timeout_ms);
                    /* 第一次超时先给一次自愈机会：重开总线、重建任务之类 */
                    if (e->recover != NULL) {
                        ESP_LOGW(TAG, "尝试恢复 \"%s\"…", e->name);
                        e->recover();
                    }
                }

                if (e->miss < 255) {
                    e->miss++;
                }

                if (e->miss >= CONFIG_WATCHDOG_MAX_MISS) {
                    ESP_LOGE(TAG, "\"%s\" 连续 %u 次超时，判定卡死，主动复位",
                             e->name, (unsigned)e->miss);

                    /* 先落盘"是谁卡的"，再复位，这就是 BKP->DR1 的等价操作 */
                    Save_Reset_Reason(RST_REASON_HEARTBEAT);
                    s_rtc_slot = (uint32_t)i;
                    Save_Task_Name(e->name);

                    vTaskDelay(pdMS_TO_TICKS(50));   /* 让日志先吐出去 */
                    esp_restart();
                }
            } else {
                e->miss = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(CONFIG_WATCHDOG_PERIOD_MS));
    }
}

/** @brief 启动看门狗监控 */
esp_err_t watchdog_start(void)
{
    if (s_running) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(monitor_task, "wd_mon",
                                            CONFIG_WATCHDOG_TASK_STACK, NULL,
                                            CONFIG_WATCHDOG_TASK_PRIO, &s_task,
                                            CONFIG_WATCHDOG_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "监控任务创建失败（栈 %d 字节）", CONFIG_WATCHDOG_TASK_STACK);
        return ESP_ERR_NO_MEM;
    }

    s_running = true;
    return ESP_OK;
}

/** @brief 查询看门狗是否正在运行 */
bool watchdog_is_running(void)
{
    return s_running;
}

void watchdog_dump_tasks(void)
{
#if defined(CONFIG_FREERTOS_USE_TRACE_FACILITY) && CONFIG_FREERTOS_USE_TRACE_FACILITY
    static TaskStatus_t st[24];

    /* 一次拼成整段再输出，而不是每个任务打一条。 */
    static char s_buf[1536];
    size_t off = 0;

    UBaseType_t n = uxTaskGetSystemState(st, (UBaseType_t)(sizeof(st) / sizeof(st[0])), NULL);

    off += (size_t)snprintf(s_buf + off, sizeof(s_buf) - off,
                            "任务栈余量（单位：字节，ESP-IDF 的栈深按字节算）：");

    for (UBaseType_t i = 0; i < n && off < sizeof(s_buf) - 80; i++) {
        off += (size_t)snprintf(s_buf + off, sizeof(s_buf) - off,
                                "\n  %-16s prio=%2u 余=%6u  %s",
                                st[i].pcTaskName,
                                (unsigned)st[i].uxCurrentPriority,
                                (unsigned)st[i].usStackHighWaterMark,
                                st[i].usStackHighWaterMark < 512 ? "<== 偏小，注意" : "");
    }
    ESP_LOGI(TAG, "%s", s_buf);
#else
    ESP_LOGW(TAG, "未开启 CONFIG_FREERTOS_USE_TRACE_FACILITY，无法导出任务列表");
#endif
}

/* ==================== 兼容 STM32 版接口 ====================
 * 原实现写 BKP->DR1..DR5，现在写 RTC 保留内存；
 * 唯一区别是老代码必须在复位前调用，这里也一样。 */
void Save_Reset_Reason(uint16_t reason)
{
    s_rtc_reason = reason;
    s_rtc_magic  = WD_RTC_MAGIC;     /* magic 最后写，保证前面先落地 */
}

void Save_Task_Name(const char *name)
{
    if (name == NULL) {
        return;
    }
    uint32_t packed = 0;
    for (int i = 0; i < 4; i++) {
        uint8_t c = (uint8_t)name[i];   /* 遇到 '\0' 自然补零，正好凑满 4 字节 */
        packed |= ((uint32_t)c) << (8 * i);
        if (c == 0) {
            break;
        }
    }
    s_rtc_name  = packed;
    s_rtc_magic = WD_RTC_MAGIC;
}

void Save_HardFault_PC(uint32_t pc)
{
    s_rtc_pc    = pc;
    s_rtc_magic = WD_RTC_MAGIC;
}

void Print_Last_Reset_Reason(void)
{
    print_reset_report();
}