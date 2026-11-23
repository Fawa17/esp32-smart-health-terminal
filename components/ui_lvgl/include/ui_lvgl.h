/*
 * ui_lvgl.h - LVGL 界面层（时钟页 + 三页 + 告警弹窗）
 *
 * 线程模型：业务任务只调 ui_lvgl_set_data()，在 portMUX 保护下拷一份结构体；
 * 所有 lv_* 调用都发生在 ui 任务内。LVGL 不是线程安全的。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 阈值修改回调：用户在设置页点 +/- 时触发。
 * key 取值 "heart_max" / "spo2_min" / "temp_max"（temp_max 单位 0.1℃） */
typedef void (*ui_threshold_cb_t)(const char *key, int value);

/* OTA 按钮回调：由 main 决定从哪个 URL 升级 */
typedef void (*ui_ota_cb_t)(void);

typedef struct {
    /* 时间 */
    bool time_ok;
    int  year, month, day, hour, minute, second;

    /* 传感器（ok=false 时界面显示 --） */
    bool hr_ok;    int hr;
    bool spo2_ok;  int spo2;
    bool temp_ok;  float temp_c;
    uint32_t steps;
    bool fall;

    /* 网络：0 离线 / 1 WiFi 已连 / 2 MQTT 已连 */
    uint8_t net;

    /* 掉线位图：bit0 MPU / bit1 MAX / bit2 NTC / bit3 RTC / bit4 DISPLAY */
    uint8_t fault;

    /* 当前阈值（设置页显示） */
    int heart_max;
    int spo2_min;
    int temp_max_x10;   /* 37.8℃ 存 378，避免界面层做浮点比较 */
} ui_lvgl_data_t;

/** @brief 初始化 LVGL、创建界面、启动 UI 任务（内部会初始化触摸） */
esp_err_t ui_lvgl_start(void);

/** @brief 更新界面数据（只拷结构体，不触发绘制） */
void ui_lvgl_set_data(const ui_lvgl_data_t *d);

/**
 * @brief 暂停/恢复渲染
 *
 * OTA 下载期间必须暂停：写 flash 时两核 cache 关闭，PSRAM 不可访问，
 * 而帧缓冲就在 PSRAM 里，LCD 的 DMA 若还在读它，直接 panic
 * "Cache disabled but cached memory region accessed"。
 */
void ui_lvgl_set_paused(bool paused);

/**
 * @brief 请求下一次渲染整屏重画
 *
 * 息屏唤醒后必须调一次：面板 SLEEP 期间 GDDRAM 内容不可信，
 * 只重画"脏区"会留下半屏旧像素（表现为唤醒后上黑下旧）。
 * 从任意任务调用都安全，它只是置一个标志，真正的 invalidate 在 UI 任务里做。
 */
void ui_lvgl_force_full_refresh(void);

/**
 * @brief 显示"正在升级，请勿断电"的全屏遮罩，并立即同步刷一帧到面板
 *
 * 用途：OTA 期间渲染任务会被 ui_lvgl_set_paused(true) 整个挂起
 * （写 flash 时 cache 关闭、PSRAM 不可访问，帧缓冲碰不得）。如果不先画一屏，
 * 用户按下"开始升级"后画面就定格在设置页，看起来和死机一模一样。
 */
void ui_lvgl_show_ota(const char *msg);

/** @brief 注册阈值修改回调（设置页 +/- 按钮） */
void ui_lvgl_set_threshold_cb(ui_threshold_cb_t cb);

/** @brief 注册 OTA 按钮回调 */
void ui_lvgl_set_ota_cb(ui_ota_cb_t cb);

#ifdef __cplusplus
}
#endif