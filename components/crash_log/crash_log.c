/**
 * @file  crash_log.c
 * @brief 用 IDF 的 core dump 机制记录"上次是谁崩的、崩在哪一行"
 *
 * 编译依赖：CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y
 * CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF=y
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"

#include "crash_log.h"

/* 只有开了 core dump 才有这一整套 API 与结构体 */
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
#include "esp_core_dump.h"
#endif

static const char *TAG = "crash";

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH

bool crash_log_has_dump(void)
{
    /* 返回 ESP_OK 表示 flash 上躺着一份**校验通过**的 core dump。
     * 校验含 CRC/SHA256，分区被写过一半（掉电）时这里会失败，这正是期望行为。 */
    return esp_core_dump_image_check() == ESP_OK;
}

esp_err_t crash_log_read(crash_info_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    esp_err_t err = esp_core_dump_image_check();
    if (err != ESP_OK) {
        return err;
    }

    /* ① panic 原因字符串：形如
     *    "Guru Meditation Error: Core 0 panic'ed (LoadProhibited)"
     *    无需自己维护 EXCCAUSE 编号表，IDF 已经把它翻译好了。 */
    if (esp_core_dump_get_panic_reason(out->reason, sizeof(out->reason)) != ESP_OK) {
        snprintf(out->reason, sizeof(out->reason), "(未取到 panic 原因)");
    }

    /* ② 崩溃任务名 / PC / 调用栈
     *    结构体只在 FLASH + ELF 格式时才定义，所以要额外判一次格式。 */
#if CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
    static esp_core_dump_summary_t s_sum;   /* 结构体较大，不放栈上 */
    memset(&s_sum, 0, sizeof(s_sum));

    if (esp_core_dump_get_summary(&s_sum) == ESP_OK) {
        out->pc = s_sum.exc_pc;

        memcpy(out->task, s_sum.exc_task, sizeof(out->task));
        out->task[sizeof(out->task) - 1] = '\0';   /* 保险：万一 ID F 没写终止符 */

        uint32_t depth = s_sum.exc_bt_info.depth;
        if (depth > CRASH_BT_MAX) {
            depth = CRASH_BT_MAX;
        }
        out->bt_depth = depth;
        for (uint32_t i = 0; i < depth; i++) {
            out->bt[i] = s_sum.exc_bt_info.bt[i];
        }
    } else {
        ESP_LOGW(TAG, "core dump 存在，但摘要解析失败（可能不是 ELF 格式或版本不匹配）");
    }
#endif

    out->valid = true;
    return ESP_OK;
}

esp_err_t crash_log_report(void)
{
    crash_info_t ci;
    esp_err_t err = crash_log_read(&ci);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGE(TAG, "======== 检测到上次运行崩溃 ========");
    ESP_LOGE(TAG, "原因 : %s", ci.reason);
    ESP_LOGE(TAG, "任务 : %s", (ci.task[0] != '\0') ? ci.task : "?");
    ESP_LOGE(TAG, "PC   : 0x%08" PRIx32, ci.pc);

    for (uint32_t i = 0; i < ci.bt_depth; i++) {
        ESP_LOGE(TAG, "  #%02" PRIu32 " 0x%08" PRIx32, i, ci.bt[i]);
    }

    /* 光有地址没用，得翻译成函数名和行号。这一步必须在电脑上做，
     * 因为它需要 build 目录里的 ELF（里面有符号表），芯片上没有。 */
    ESP_LOGE(TAG, "查行号：idf.py -p <你的串口> coredump-info");
    ESP_LOGE(TAG, "====================================");
    return ESP_OK;
}

esp_err_t crash_log_erase(void)
{
    esp_err_t err = esp_core_dump_image_erase();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "崩溃记录已清除");
    } else {
        ESP_LOGW(TAG, "清除崩溃记录失败: %s", esp_err_to_name(err));
    }
    return err;
}

void crash_log_init(void)
{
    if (!crash_log_has_dump()) {
        return;                     /* 正常启动，一声不吭 */
    }

    /* 注意：这里**故意不擦除**。
     * 一旦擦掉，`idf.py coredump-info` 就再也读不到了；
     * 保留的代价只是每次启动多打几行日志。 */
    crash_log_report();
}

#else  /* 没开 core dump：给空实现，保证 main.c 不用改结构 */

bool crash_log_has_dump(void)          { return false; }
esp_err_t crash_log_read(crash_info_t *o) { (void)o; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t crash_log_report(void)       { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t crash_log_erase(void)        { return ESP_ERR_NOT_SUPPORTED; }
void crash_log_init(void)
{
    ESP_LOGW(TAG, "未启用 core dump（CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=n），崩溃信息无法保留");
}

#endif /* CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH */