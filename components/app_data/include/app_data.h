/*
 * app_data.h - 全系统唯一的数据中枢
 *
 * 各任务各存一份全局变量时，网络上报要连续读 5 个字段，中途被任务切换打断
 * 会拼出"上一秒心率 + 这一秒血氧"这种自相矛盾的帧。
 *
 * 做法：单写者 + 一致性快照。只有采集汇总任务调 app_data_publish()，
 * 读者调 app_data_read() 拿一份加锁复制的完整结构（几十字节，微秒级）。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 一次采集的完整结果。单位都写在名字里，避免读代码时猜 */
typedef struct {
    uint32_t seq;              /* 第几次发布，用来判断"数据有没有在更新" */
    uint32_t uptime_s;         /* 开机秒数 */

    /* 时间（RTC 没电池 / 没对时时 time_ok = false） */
    bool     time_ok;
    int      year, month, day;
    int      hour, minute, second;

    /* 生命体征 */
    bool     hr_ok;
    int32_t  hr;               /* 次/分 */
    bool     spo2_ok;
    int32_t  spo2;             /* % */
    bool     temp_ok;
    float    temp_c;           /* 摄氏度 */

    /* 运动 */
    uint32_t steps;
    bool     fall;             /* 跌倒状态（锁存到下一次复位） */

    /* 掉线位图：哪一路外设没响应就点哪一位（详见 app_data_fault.c 的说明）
     *   bit0 MPU6050  bit1 MAX30102  bit2 NTC 温度
     *   bit3 DS1302   bit4 ST7789 屏 */
    uint8_t  fault;
} app_snapshot_t;

/* 下行命令能改的阈值。放在这里而不是散在业务代码里，
 * 是为了让"谁改了配置"只有一条路径 */
typedef struct {
    int32_t heart_max;         /* 心率上限，超过就告警 */
    int32_t spo2_min;          /* 血氧下限，低于就告警 */
    float   temp_max;
    float   temp_min;
    uint8_t mode;              /* 工作模式，由下行 {"mode":N} 设置，1=标准运行 */
} app_cfg_t;

/** @brief 建锁并填入默认快照。要在任何任务之前调 */
esp_err_t app_data_init(void);

/** @brief 发布一份新快照（只允许一个写者）。返回 false 表示参数非法 */
bool app_data_publish(const app_snapshot_t *s);

/** @brief 读一份一致性快照。返回 false 表示还没发布过任何数据 */
bool app_data_read(app_snapshot_t *out);

/** @brief 已发布次数，读者可据此判断数据是否停滞 */
uint32_t app_data_seq(void);

/** @brief 阈值配置的读写（下行命令用） */
void app_data_get_cfg(app_cfg_t *out);
void app_data_set_cfg(const app_cfg_t *cfg);

#ifdef __cplusplus
}
#endif