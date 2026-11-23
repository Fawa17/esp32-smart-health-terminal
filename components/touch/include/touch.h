/*
 * touch.h - 触摸驱动：TK020F1131 上的 CST816S，I2C @0x15
 *
 * 排针 8 针分两组独立信号：
 *   VCC / GND / SCL / SDA   -> 触摸 CST816S 的 I2C
 *   CS / DC / SCK / MOSI    -> 显示 SPI
 * "SCL/SDA" 与 "SCK/MOSI" 同名不同义，接错会整条总线挂掉。
 *
 * 三个信号没引出：RST（板上只有 RC 复位）、IRQ（只能轮询）、BL_CTR（背光常亮）。
 * 因为 IRQ 没引出，touch_init() 会写 0xFA 关掉芯片自动休眠，
 * 否则闲置一段时间后只能断电重启（Kconfig: TOUCH_I2C_DIS_AUTOSLEEP）。
 *
 * 接线：模块 SCL/SDA 并入现有 I2C 总线（GPIO9 / GPIO8），与 MPU6050、MAX30102 共用。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 触摸事件：只在状态变化时给出，方便做"按下/抬起"逻辑 */
typedef enum {
    TOUCH_EVT_NONE = 0,
    TOUCH_EVT_DOWN,     /* 从没按下变成按下 */
    TOUCH_EVT_MOVE,     /* 已经按着，坐标又变了 */
    TOUCH_EVT_UP,       /* 从按下变成松开 */
    TOUCH_EVT_GESTURE,  /* 芯片内置手势识别命中（滑动/单击/双击/长按） */
} touch_event_t;

/* CST816S 手势寄存器 0x01 的取值。
 * 这是芯片内部用专门的检测算法算出来的，比从 50Hz 采样序列
 * 自己拟合方向+速度可靠得多，滑动方向尤其容易在快速划动时判反。 */
typedef enum {
    TOUCH_GEST_NONE = 0,
    TOUCH_GEST_SWIPE_UP,      /* 0x01 上滑（手指从下往上） */
    TOUCH_GEST_SWIPE_DOWN,    /* 0x02 下滑 */
    TOUCH_GEST_SWIPE_LEFT,    /* 0x03 左滑 */
    TOUCH_GEST_SWIPE_RIGHT,   /* 0x04 右滑 */
    TOUCH_GEST_SINGLE_CLICK,  /* 0x05 单击 */
    TOUCH_GEST_DOUBLE_CLICK,  /* 0x0B 双击 */
    TOUCH_GEST_LONG_PRESS,    /* 0x0C 长按 */
} touch_gesture_t;

typedef struct {
    uint16_t       x;        /* 已映射到屏幕坐标系（0~239） */
    uint16_t       y;        /* 0~295 */
    uint8_t        points;   /* 原始触点数，>1 表示多点 */
    bool           pressed;
    touch_event_t  event;    /* */
    touch_gesture_t gesture; /* 事件伴随的手势；没有则为 TOUCH_GEST_NONE */
    bool           gesture_native; /* true = 芯片寄存器给的；false = 驱动按位移/时长推算的 */
} touch_point_t;

/** @brief 手势的中文名，方便直接打日志 */
const char *touch_gesture_name(touch_gesture_t g);

/**
 * @brief 探测并初始化触摸 IC
 * @return ESP_OK 找到并可用；ESP_ERR_NOT_FOUND 三个地址都没人应答（没接或没供电）
 */
esp_err_t touch_init(void);

/** @brief 触摸 IC 的 7 位 I2C 地址（0 表示未初始化） */
uint8_t touch_get_addr(void);

/** @brief 触摸 IC 型号描述字符串，例如 "CST816" / "FT6236" / "未知(0x15)" */
const char *touch_get_model(void);

/**
 * @brief 读一次触摸状态（非阻塞，内部只有一次 5 字节 I2C 读）
 *
 * 用法：调用者按自己的节奏轮询（20~50 Hz 足够），不要放在中断里。
 * @param[out] p 结果；函数保证 event 字段反映"相对上一次调用"的变化
 * @return true 表示这次读到了有效数据（无论是否按下）
 */
bool touch_poll(touch_point_t *p);

/**
 * @brief 注册事件回调（DOWN / MOVE / UP 只会是这三种之一）
 *
 * 回调在触摸任务上下文里执行，别在里面做耗时操作（画屏、发 MQTT）。
 */
typedef void (*touch_event_cb_t)(const touch_point_t *p);
void touch_set_callback(touch_event_cb_t cb);

/**
 * @brief 启动 50Hz 采样任务，去抖后通过回调上报
 * @return ESP_OK；ESP_ERR_INVALID_STATE 表示 touch_init() 没成功
 *
 * 之所以要独立任务：2Hz 轮询会漏掉快速点按，
 * 而且"按下→抬起"会变成同一坐标的两次事件，看起来像误判。
 */
esp_err_t touch_start_task(void);

/**
 * @brief 挂起/恢复触摸采样
 *
 * 息屏时挂起：任务不再碰 I2C（省电，也不会产生"黑屏还有人划"的事件）。
 * 挂起期间不会有任何新事件，唤醒只能靠物理按键。
 * 恢复后第一拍若手指仍按着，会上报一次正常 DOWN，属预期行为。
 */
void touch_set_suspended(bool suspended);

/** @brief 当前是否处于挂起状态 */
bool touch_is_suspended(void);

#ifdef __cplusplus
}
#endif