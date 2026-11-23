/*
 * disp.h，240x296 ST7789 显示屏驱动（esp_lcd 直绘，暂不接 LVGL）
 *
 * 设计要点：
 * 1. 用一整块 RGB565 帧缓冲（240*296*2 = 141.6 KB），放在 PSRAM 里。
 *    所有绘制函数只改内存，disp_flush() 才一次性推给屏。
 *    这样做的好处：没有"画到一半被看到"的撕裂；坏处：一次刷整帧 141.6 KB，
 *    40 MHz SPI 约 28 ms ⇒ 理论上限 ~35 fps。
 * 2. 字库沿用原 STM32 工程的 8x16 点阵（见 disp_font.c），支持整数倍放大。
 * 3. 背光走 LEDC PWM，可调亮度；不用的话给 100% 也行。
 *
 * 接线（可在 menuconfig → Display 里改）：
 *   SCLK=GPIO12  MOSI=GPIO11  CS=GPIO10  DC=GPIO13  RST=GPIO14  BL=GPIO15
 *   触摸先不接；将来接的话建议 INT=GPIO16、RST=GPIO17。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISP_H_RES 240
#define DISP_V_RES 296
#define DISP_FB_BYTES (DISP_H_RES * DISP_V_RES * 2)

#define DISP_BLACK   0x0000
#define DISP_WHITE   0xFFFF
#define DISP_RED     0xF800
#define DISP_GREEN   0x07E0
#define DISP_BLUE    0x001F
#define DISP_YELLOW  0xFFE0
#define DISP_CYAN    0x07FF
#define DISP_MAGENTA 0xF81F
#define DISP_GRAY    0x8410
#define DISP_ORANGE  0xFD20
#define DISP_NAVY    0x000F

/* 把 8 位 RGB 转成 RGB565，方便随手写颜色 */
#define DISP_RGB565(r, g, b) \
    ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

/**
 * @brief 初始化 SPI 总线、面板、背光，并分配帧缓冲（PSRAM）
 * @return ESP_OK 成功；ESP_ERR_NO_MEM 帧缓冲分配失败；其余为 SPI/面板错误
 */
esp_err_t disp_init(void);

/** @brief 是否已初始化成功 */
bool disp_is_ready(void);

void disp_clear(uint16_t color);
void disp_draw_pixel(int x, int y, uint16_t color);
void disp_fill_rect(int x, int y, int w, int h, uint16_t color);
void disp_draw_rect(int x, int y, int w, int h, uint16_t color);
void disp_draw_hline(int x, int y, int w, uint16_t color);
void disp_draw_vline(int x, int y, int h, uint16_t color);

/**
 * @brief 画一个字符
 * @param scale 整数倍放大（1 = 8x16，2 = 16x32）—— 手机上那种大字号靠它
 */
void disp_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, uint8_t scale);

/** @brief 画一串字符（只支持 ASCII 0x20~0x7E，中文需要另取字模） */
void disp_draw_string(int x, int y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale);

/** @brief 这串字符画出来有多宽（像素），方便居中 */
int disp_text_width(const char *s, uint8_t scale);

/** @brief 字符串高度（= 16 * scale），方便排版 */
int disp_text_height(uint8_t scale);

/**
 * @brief 拿到帧缓冲的首地址（RGB565，240*296，行优先连续排布）
 *
 * 给 LVGL 用：把这块 PSRAM 直接当 LVGL 的显示缓冲（DIRECT 模式），
 * LVGL 画完由 disp_flush_rows() 推屏，省掉一次全屏拷贝。
 * 未初始化时返回 NULL。
 */
uint16_t *disp_fb(void);

/**
 * @brief 把整帧推给屏（阻塞到 SPI 传完）
 */
esp_err_t disp_flush(void);

/**
 * @brief 只推若干行（局部刷新，比整帧快很多）
 * @param y 起始行，h 行数；只支持整行宽度（这是由帧缓冲连续排布决定的）
 */
esp_err_t disp_flush_rows(int y, int h);

/**
 * @brief 上电配色自检：画 红/绿/蓝/白/黑/黄 六个竖条，停留 hold_ms
 *
 * 只在 CONFIG_DISP_BOOT_TEST 打开时生效，用来一次性判断
 * 反色 / BGR / 字节序这三个配色开关哪个配错了。
 */
void disp_color_test(uint32_t hold_ms);

/**
 * @brief 面板是否还在正常接收数据
 *
 * 连续 DISP_FAIL_LIMIT 次刷新失败（SPI 出错或等不到传输完成）就置 false，
 * 之后只要有一次成功就自动恢复。用于上报掉线位图的 bit4。
 */
bool disp_is_online(void);

/**
 * @brief 让面板进入/退出休眠（DISPOFF+SLPIN / SLPOUT+DISPON）
 *
 * 这块 8 针模块没引出背光脚，所以"关屏"只能靠面板命令，不能断电。
 * 对比一下三种"关屏"方式的效果：
 *   - 只画黑：面板仍在扫描、仍在耗电，约等于没关
 *   - DISPOFF(0x28)：不再输出图像，但振荡器还开着
 *   - DISPOFF + SLPIN(0x10)：振荡器和驱动全停，只剩 I/O 漏电流 ← 本函数
 *
 * @param off true = 休眠，false = 唤醒（内部已按手册等 120ms 让 SLPOUT 稳定）
 */
esp_err_t disp_display_sleep(bool off);

/** @brief 0 = 全灭，100 = 最亮 */
void disp_backlight_set(uint8_t percent);

#ifdef __cplusplus
}
#endif