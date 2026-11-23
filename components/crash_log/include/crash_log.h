/**
 * @file  crash_log.h
 * @brief panic 现场记录（coredump 分区）
 *
 * ESP32 的异常统一走 IDF panic 流程，没有可插钩子的 HardFault 入口，
 * 因此不用 STM32 那种 BKP 备份寄存器的做法：panic 时由 IDF 把现场写进
 * coredump 分区，重启后读取。可拿到出错任务名、PC、调用栈和 panic 原因；
 * 需要行号时用 idf.py coredump-info 配合 ELF 解析。
 */
#ifndef CRASH_LOG_H
#define CRASH_LOG_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CRASH_TASK_NAME_LEN 16
#define CRASH_BT_MAX        16
#define CRASH_REASON_LEN    96

/** 一次崩溃的摘要 */
typedef struct {
    bool     valid;                        /**< 读到了有效记录 */
    char     task[CRASH_TASK_NAME_LEN];    /**< 出错的任务名（相当于"哪个任务崩的"） */
    uint32_t pc;                           /**< 异常发生时的 PC */
    uint32_t bt[CRASH_BT_MAX];             /**< 调用栈（PC 数组，越靠前越深） */
    uint32_t bt_depth;                     /**< 调用栈深度 */
    char     reason[CRASH_REASON_LEN];     /**< 崩溃原因 */
} crash_info_t;

/** flash 上是否有未处理的 core dump */
bool crash_log_has_dump(void);

/** 读取上次崩溃摘要（不擦除） */
esp_err_t crash_log_read(crash_info_t *out);

/** 打印到串口（不擦除） */
esp_err_t crash_log_report(void);

/** 擦掉 flash 上的 core dump */
esp_err_t crash_log_erase(void);

/** 启动时调用：有崩溃记录就打印一条摘要 */
void crash_log_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CRASH_LOG_H */