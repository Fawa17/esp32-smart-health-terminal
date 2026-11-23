/*
 * disp_test.c —— 屏幕 + 触摸自检（menuconfig 选“只测屏幕”时执行）
 *
 * 分三步，故意做成"先看颜色、再看字、最后摸"：
 *   步骤 1：全屏依次刷 红/绿/蓝/白/黑，每色 400 ms。
 *           —— 接线错、SPI 时钟过高、颜色顺序反了，这一步就能看出来。
 *   步骤 2：画静态界面（顶栏、六色带、三档字号）。
 *           —— 确认字库、坐标、局部刷新都对。
 *   步骤 3：底部做成"触摸板"，手指按上去画十字光标并显示坐标。
 *           —— 确认触摸 IC 找得到、方向没反、量程匹配。
 *
 * 局部刷新只用 disp_flush_rows()，底部这一条约 76 行 ≈ 5.3 ms，
 * 33 Hz 轮询才占百分之十几的 CPU。
 *
 * 说明：字库只有 ASCII 0x20~0x7E，所以界面上全是英文。
 *       要显示中文得另外取 16x16 汉字字模。
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "disp.h"
#include "touch.h"

static const char *TAG = "disp_test";

/* 依次全屏刷一个颜色，并在中间写一句提示 */
static void splash_color(uint16_t bg, const char *label)
{
    disp_clear(bg);
    uint16_t fg = (bg == DISP_WHITE || bg == DISP_YELLOW || bg == DISP_CYAN)
                      ? DISP_BLACK : DISP_WHITE;
    disp_draw_string(24, 140, label, fg, bg, 2);
    disp_flush();
    vTaskDelay(pdMS_TO_TICKS(400));
}

/* 画一个十字光标（触点标记） */
static void draw_crosshair(int x, int y, uint16_t color)
{
    disp_draw_hline(x - 8, y, 17, color);
    disp_draw_vline(x, y - 8, 17, color);
    disp_draw_rect(x - 3, y - 3, 7, 7, color);
}

void run_disp_only(void)
{
    ESP_LOGI(TAG, "=== 模式：只测屏幕 + 触摸（ST7789 %dx%d）===", DISP_H_RES, DISP_V_RES);

    esp_err_t err = disp_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "disp_init 失败: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "排查：1) SCK=%d MOSI=%d CS=%d DC=%d 是否接对"
                 "  2) 模块 VCC/GND  3) 把 DISP_SPI_CLK_HZ 降到 20 MHz",
                 CONFIG_DISP_PIN_SCLK, CONFIG_DISP_PIN_MOSI,
                 CONFIG_DISP_PIN_CS, CONFIG_DISP_PIN_DC);
        return;
    }

    disp_backlight_set(CONFIG_DISP_BL_PERCENT);

    /* ---------- 步骤 1：全屏颜色 ---------- */
    ESP_LOGI(TAG, "步骤 1/3：全屏 R/G/B/W/K 各 400 ms");
    splash_color(DISP_RED,   "RED");
    splash_color(DISP_GREEN, "GREEN");
    splash_color(DISP_BLUE,  "BLUE");
    splash_color(DISP_WHITE, "WHITE");
    splash_color(DISP_BLACK, "BLACK");

    /* ---------- 步骤 2：静态界面 ---------- */
    ESP_LOGI(TAG, "步骤 2/3：界面 + 字号");

    disp_clear(DISP_BLACK);
    disp_fill_rect(0, 0, DISP_H_RES, 34, DISP_NAVY);
    disp_draw_string(6, 9, "ST7789 240x296", DISP_WHITE, DISP_NAVY, 1);

    /* 六条色带：确认 RGB565 与 BGR 顺序 */
    const uint16_t bars[6] = { DISP_RED, DISP_GREEN, DISP_BLUE,
                               DISP_YELLOW, DISP_CYAN, DISP_MAGENTA };
    const char *names[6] = { "R", "G", "B", "Y", "C", "M" };
    for (int i = 0; i < 6; i++) {
        int x = i * 40;
        disp_fill_rect(x, 44, 38, 40, bars[i]);
        disp_draw_string(x + 14, 56, names[i], DISP_BLACK, bars[i], 1);
    }

    /* 字号演示 */
    disp_draw_string(6, 96,  "scale 1", DISP_WHITE, DISP_BLACK, 1);
    disp_draw_string(6, 118, "scale 2", DISP_WHITE, DISP_BLACK, 2);
    disp_draw_string(6, 156, "scale 3", DISP_GREEN, DISP_BLACK, 3);

    /* 触摸板区域的分隔线 */
    disp_draw_hline(0, 208, DISP_H_RES, DISP_GRAY);
    disp_draw_string(6, 212, "TOUCH PAD", DISP_GRAY, DISP_BLACK, 1);

    disp_flush();

    /* ---------- 步骤 3：触摸 ---------- */
    err = touch_init();
    bool touch_ok = (err == ESP_OK);
    if (touch_ok) {
        ESP_LOGI(TAG, "步骤 3/3：触摸就绪（%s @0x%02X），手指按屏幕任意位置",
                 touch_get_model(), touch_get_addr());
    } else {
        ESP_LOGW(TAG, "步骤 3/3：没找到触摸 IC，只做显示自检");
        ESP_LOGW(TAG, "        请确认模块 SCL/SDA 已并到 I2C 总线（GPIO8/9）");
    }

    /* 触摸板：y 从 220 到 296 */
    const int pad_y = 220;
    const int pad_h = DISP_V_RES - pad_y;   /* 76 行 */

    touch_point_t tp;
    uint16_t last_x = DISP_H_RES / 2, last_y = pad_y + pad_h / 2;
    uint32_t frames = 0;

    for (;;) {
        char line[48];
        bool has = touch_ok && touch_poll(&tp);

        /* 整块触摸板重画：先铺底，再画光标和文字 */
        disp_fill_rect(0, pad_y, DISP_H_RES, pad_h, DISP_BLACK);

        if (has && tp.pressed) {
            /* 把光标限制在触摸板范围内显示（手指也可能按在上面区域） */
            int cx = tp.x;
            int cy = tp.y;
            if (cy < pad_y + 10) cy = pad_y + 10;
            if (cy > DISP_V_RES - 10) cy = DISP_V_RES - 10;
            if (cx < 10) cx = 10;
            if (cx > DISP_H_RES - 11) cx = DISP_H_RES - 11;

            draw_crosshair(cx, cy, DISP_YELLOW);
            last_x = tp.x;
            last_y = tp.y;

            if (tp.event == TOUCH_EVT_DOWN) {
                ESP_LOGI(TAG, "触摸按下 (%u,%u) 点数=%u", tp.x, tp.y, tp.points);
            } else if (tp.event == TOUCH_EVT_UP) {
                ESP_LOGI(TAG, "触摸抬起 (%u,%u)", tp.x, tp.y);
            }

            (void)snprintf(line, sizeof(line), "X=%3u Y=%3u  p=%u",
                           tp.x, tp.y, tp.points);
            disp_draw_string(6, pad_y + 2, line, DISP_CYAN, DISP_BLACK, 1);
        } else {
            (void)snprintf(line, sizeof(line), "X=%3u Y=%3u", last_x, last_y);
            disp_draw_string(6, pad_y + 2, line, DISP_GRAY, DISP_BLACK, 1);
            disp_draw_string(120, pad_y + 2,
                             touch_ok ? "touch me" : "no touch IC",
                             DISP_GRAY, DISP_BLACK, 1);
        }

        /* 秒计数放右下角，方便判断刷新还活着 */
        (void)snprintf(line, sizeof(line), "%lu", (unsigned long)(frames / 33));
        disp_draw_string(DISP_H_RES - 30, DISP_V_RES - 18, line,
                         DISP_GRAY, DISP_BLACK, 1);

        esp_err_t e = disp_flush_rows(pad_y, pad_h);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "局部刷新失败: %s", esp_err_to_name(e));
        }

        frames++;
        /* 33 Hz 轮询：够顺滑，又不至于把 SPI 占满 */
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}