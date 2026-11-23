/*
 * disp.c，240x296 ST7789 屏驱动（esp_lcd + SPI + PSRAM 帧缓冲）
 *
 * 说明为什么这么写：
 * 1. 用 esp_lcd 的 SPI Panel IO，SPI 时钟 40 MHz。240*296*2 = 141.6 KB 一帧，
 *    理论耗时 141.6*8/40e6 ≈ 28 ms，所以整帧刷新上限约 35 fps。
 * 2. 帧缓冲放 PSRAM（sdkconfig 里 CONFIG_SPIRAM=y，八线 40 MHz）。
 *    esp_lcd 的 SPI 驱动内部会分块拷到内部 RAM 再走 DMA，
 *    所以 max_transfer_sz 给 40 行（19200 字节）就够，不会一次要 141 KB 的 DMA 缓冲。
 * 3. 绘制全部在内存里做，disp_flush() 一次性推走，避免撕裂。
 * 4. 局部刷新只支持"整行宽度"的条带：帧缓冲是行优先连续排布，
 *    只有 x 从头到尾时数据才是连续的一段，可以直接当 DMA 源用。
 */

#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_check.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "disp.h"
#include "disp_font.h"

static const char *TAG = "disp";

/* SPI 用 SPI2_HOST：SPI1 一般留给 Flash，不要碰 */
#define DISP_SPI_HOST SPI2_HOST
/* 一次 DMA 传输最多 40 行，够大又不浪费内部 RAM */
#define DISP_MAX_TRANS_LINES 40

static esp_lcd_panel_io_handle_t s_io;
static uint16_t                 *s_fb;          /* 帧缓冲，RGB565 */
static SemaphoreHandle_t         s_flush_done;  /* SPI 传完的信号量 */
static bool                      s_inited;


/* SPI 传输完成回调（在中断上下文里跑） */
static bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                          esp_lcd_panel_io_event_data_t *edata,
                                          void *user_ctx)
{
    (void)io;
    (void)edata;
    (void)user_ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_flush_done, &woken);
    return woken == pdTRUE;
}

/* 背光 PWM：用 LEDC 通道 0，10 位分辨率，5 kHz
 *
 * 注意：很多 7 针 SPI 模块（VCC/GND/SCL/SDA/RES/DC/CS）根本没有 BL 脚，
 * 背光在模块内部直接接常亮。这时 DISP_PIN_BL 填 -1，整个 LEDC 都不初始化。
 */
#if (CONFIG_DISP_PIN_BL >= 0)
static esp_err_t backlight_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&tcfg), TAG, "ledc timer");

    ledc_channel_config_t ccfg = {
        .gpio_num   = CONFIG_DISP_PIN_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,          /* 先灭，等画面准备好再亮 */
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ccfg), TAG, "ledc channel");
    return ESP_OK;
}
#endif /* CONFIG_DISP_PIN_BL >= 0 */


/* 每次 SPI 事务最多搬几行。8 行 = 240*2*8 = 3840 字节：
 * 小到内部 DMA 缓冲一定申请得到，大到事务开销可忽略（整屏 296 行 = 37 次）。 */
#define DISP_FLUSH_CHUNK_LINES 8

/* 内部 RAM 的 DMA 中转缓冲。 */
static uint8_t *s_dma_buf;

esp_err_t disp_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    /* 1. 帧缓冲：先要 PSRAM，失败退回内部 RAM（内部只剩约 100 KB，可能不够） */
    s_fb = heap_caps_malloc(DISP_FB_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_fb == NULL) {
        ESP_LOGW(TAG, "PSRAM 分配 %d 字节失败，尝试内部 RAM（可能不够）", DISP_FB_BYTES);
        s_fb = heap_caps_malloc(DISP_FB_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (s_fb == NULL) {
        ESP_LOGE(TAG, "帧缓冲 %d 字节分配失败，检查 CONFIG_SPIRAM 是否打开", DISP_FB_BYTES);
        return ESP_ERR_NO_MEM;
    }
    memset(s_fb, 0x00, DISP_FB_BYTES);

    s_dma_buf = heap_caps_malloc(DISP_H_RES * 2 * DISP_FLUSH_CHUNK_LINES,
                                 MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (s_dma_buf == NULL) {
        ESP_LOGE(TAG, "DMA 中转缓冲分配失败（需要 %d 字节内部 RAM）",
                 DISP_H_RES * 2 * DISP_FLUSH_CHUNK_LINES);
        heap_caps_free(s_fb);
        s_fb = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_flush_done = xSemaphoreCreateBinary();
    if (s_flush_done == NULL) {
        ESP_LOGE(TAG, "创建信号量失败");
        heap_caps_free(s_fb);
        s_fb = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* 2. SPI 总线：只发不收，MISO 给 -1 */
    spi_bus_config_t bus_cfg = {
        .sclk_io_num     = CONFIG_DISP_PIN_SCLK,
        .mosi_io_num     = CONFIG_DISP_PIN_MOSI,
        .miso_io_num     = -1,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = DISP_H_RES * 2 * DISP_MAX_TRANS_LINES,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(DISP_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
                        TAG, "spi_bus_initialize");

    /* 3. Panel IO：DC 区分命令/数据，CS 由驱动管理 */
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num         = CONFIG_DISP_PIN_CS,
        .dc_gpio_num         = CONFIG_DISP_PIN_DC,
        .spi_mode            = 0,
        .pclk_hz             = CONFIG_DISP_SPI_CLK_HZ,
        .trans_queue_depth   = 10,
        .on_color_trans_done = on_color_trans_done,
        .user_ctx            = NULL,
        .lcd_cmd_bits        = 8,
        .lcd_param_bits      = 8,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)DISP_SPI_HOST,
                                                 &io_cfg, &s_io),
                        TAG, "new_panel_io_spi");

    /* 4. 面板初始化，TK020F1131 的驱动 IC 是 **TK0134**，不是 ST7789！
     *
     * 这是整块屏各种怪现象（颜色像负片、文字发虚、怎么调反色/BGR 都不对）的根因：
     * esp_lcd_new_panel_st7789 会往芯片里灌一整套 ST7789 的命令
     * （0xB0~0xB4 伽马、0xBB、0xC0、porch 0xB2…）。而 TK0134 对这些编号的解释
     * 完全不同，它的 0xB0 是 RAMCTRL（显存访问控制，含字节序位），
     * 被写成伽马值之后显存读写方式就乱了，颜色和笔画自然全不对。
     *
     * 下面这段取自厂家参考工程 lcd_tk0202.c 的 panel_tk0134_init()，
     * 是这块屏真正需要的最小命令集。 */
    (void)esp_lcd_panel_io_tx_param(s_io, 0x01, NULL, 0);            /* SWRESET */
    vTaskDelay(pdMS_TO_TICKS(120));

    (void)esp_lcd_panel_io_tx_param(s_io, 0x3A, (uint8_t[]){0x05}, 1);   /* COLMOD: 16bit RGB565 */
    /* MADCTL：0x00 = RGB 顺序、不镜像、不交换 XY（厂家参考工程就是这个值）。
     * 只有红蓝互换时才置 0x08（BGR 位）。
     * 注意 CONFIG_xxx 在"未设置"时是**根本不存在**的宏，不能写进运行期表达式，
     * 必须用 #ifdef 分开编译。 */
#ifdef CONFIG_DISP_BGR_ORDER
    uint8_t madctl = 0x08;
#else
    uint8_t madctl = 0x00;
#endif
    (void)esp_lcd_panel_io_tx_param(s_io, 0x36, &madctl, 1);         /* MADCTL */
#if CONFIG_DISP_INVERT_COLOR
    (void)esp_lcd_panel_io_tx_param(s_io, 0x21, NULL, 0);            /* INVON  */
#else
    (void)esp_lcd_panel_io_tx_param(s_io, 0x20, NULL, 0);            /* INVOFF */
#endif
    (void)esp_lcd_panel_io_tx_param(s_io, 0x11, NULL, 0);            /* SLPOUT */
    vTaskDelay(pdMS_TO_TICKS(120));
    (void)esp_lcd_panel_io_tx_param(s_io, 0x29, NULL, 0);            /* DISPON */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 6. 背光：模块没有 BL 脚时背光常亮，这一步跳过 */
#if (CONFIG_DISP_PIN_BL >= 0)
    ESP_RETURN_ON_ERROR(backlight_init(), TAG, "backlight");
#else
    ESP_LOGI(TAG, "未配置 BL 引脚（模块背光常亮），跳过 LEDC 初始化");
#endif

    s_inited = true;
    ESP_LOGI(TAG, "屏幕就绪：%dx%d，SPI %d MHz，帧缓冲 %d 字节",
             DISP_H_RES, DISP_V_RES, CONFIG_DISP_SPI_CLK_HZ / 1000000, DISP_FB_BYTES);
    return ESP_OK;
}

bool disp_is_ready(void)
{
    return s_inited;
}

esp_err_t disp_display_sleep(bool off)
{
    if (!s_inited || s_io == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (off) {
        /* 先 DISPOFF 再 SLPIN：手册要求这个顺序。
         * SLPIN 之后 5ms 内不能再发其他命令，所以这里补一个短延时。 */
        (void)esp_lcd_panel_io_tx_param(s_io, 0x28, NULL, 0);   /* DISPOFF */
        vTaskDelay(pdMS_TO_TICKS(10));
        (void)esp_lcd_panel_io_tx_param(s_io, 0x10, NULL, 0);   /* SLPIN   */
        ESP_LOGI(TAG, "面板已休眠（DISPOFF + SLPIN）");
    } else {
        (void)esp_lcd_panel_io_tx_param(s_io, 0x11, NULL, 0);   /* SLPOUT  */
        vTaskDelay(pdMS_TO_TICKS(120));                         /* 手册：120ms */
        (void)esp_lcd_panel_io_tx_param(s_io, 0x29, NULL, 0);   /* DISPON  */
        ESP_LOGI(TAG, "面板已唤醒（SLPOUT + DISPON）");
    }
    return ESP_OK;
}


void disp_draw_pixel(int x, int y, uint16_t color)
{
    /* 越界直接丢弃：调用者写成 x<0 也不会把内存踩坏 */
    if (!s_inited || x < 0 || y < 0 || x >= DISP_H_RES || y >= DISP_V_RES) {
        return;
    }
    s_fb[y * DISP_H_RES + x] = color;
}

void disp_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (!s_inited || w <= 0 || h <= 0) {
        return;
    }
    /* 先裁剪成屏内矩形 */
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = x + w; if (x1 > DISP_H_RES) x1 = DISP_H_RES;
    int y1 = y + h; if (y1 > DISP_V_RES) y1 = DISP_V_RES;
    if (x0 >= x1 || y0 >= y1) {
        return;
    }

    for (int yy = y0; yy < y1; yy++) {
        uint16_t *row = &s_fb[yy * DISP_H_RES + x0];
        for (int xx = x0; xx < x1; xx++) {
            *row++ = color;
        }
    }
}

void disp_draw_hline(int x, int y, int w, uint16_t color)
{
    disp_fill_rect(x, y, w, 1, color);
}

void disp_draw_vline(int x, int y, int h, uint16_t color)
{
    disp_fill_rect(x, y, 1, h, color);
}

void disp_draw_rect(int x, int y, int w, int h, uint16_t color)
{
    if (w <= 0 || h <= 0) {
        return;
    }
    disp_draw_hline(x, y, w, color);             /* 上 */
    disp_draw_hline(x, y + h - 1, w, color);     /* 下 */
    disp_draw_vline(x, y, h, color);             /* 左 */
    disp_draw_vline(x + w - 1, y, h, color);     /* 右 */
}

void disp_clear(uint16_t color)
{
    if (!s_inited) {
        return;
    }
    /* 一行一行填，比逐像素快很多 */
    disp_fill_rect(0, 0, DISP_H_RES, DISP_V_RES, color);
}

/* */
void disp_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (!s_inited) {
        return;
    }
    if (scale == 0) {
        scale = 1;
    }

    uint8_t ch = (uint8_t)c;
    if (ch < DISP_FONT_FIRST || ch > DISP_FONT_LAST) {
        ch = ' ';   /* 不认识的字符一律画空格，避免乱码 */
    }
    const uint8_t *glyph = disp_font8x16[ch - DISP_FONT_FIRST];

    for (int row = 0; row < DISP_FONT_H; row++) {
        /* 上半部分取 glyph[0..7]，下半部分取 glyph[8..15] */
        const uint8_t *line = glyph + ((row < 8) ? 0 : 8);
        int bit_row = row & 0x07;

        for (int col = 0; col < DISP_FONT_W; col++) {
            bool on = (line[col] >> bit_row) & 0x01;
            uint16_t color = on ? fg : bg;

            if (scale == 1) {
                disp_draw_pixel(x + col, y + row, color);
            } else {
                disp_fill_rect(x + col * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}

void disp_draw_string(int x, int y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (!s_inited || s == NULL) {
        return;
    }
    if (scale == 0) {
        scale = 1;
    }
    int step_x = DISP_FONT_W * scale;
    int cx = x;
    while (*s) {
        disp_draw_char(cx, y, *s, fg, bg, scale);
        cx += step_x;
        s++;
        /* 右边出界就别画了，省时间 */
        if (cx >= DISP_H_RES) {
            break;
        }
    }
}

int disp_text_width(const char *s, uint8_t scale)
{
    if (s == NULL) {
        return 0;
    }
    if (scale == 0) {
        scale = 1;
    }
    return (int)strlen(s) * DISP_FONT_W * scale;
}

int disp_text_height(uint8_t scale)
{
    if (scale == 0) {
        scale = 1;
    }
    return DISP_FONT_H * scale;
}


/* ---------------- 面板健康状态 ----------------
 * 屏幕没有引 RST，硬件救不回来；但"静默黑屏"和"其实还在画"必须能区分开，
 * 否则界面不动了根本不知道是程序没跑还是 SPI 断了。 */
#define DISP_FAIL_LIMIT 5

static volatile bool s_ok = true;
static uint32_t      s_fail_cnt;
static bool          s_fail_logged;

static void note_flush(esp_err_t err)
{
    if (err == ESP_OK) {
        if (!s_ok) {
            s_ok = true;
            s_fail_logged = false;
            ESP_LOGI(TAG, "ST7789 刷新恢复正常");
        }
        s_fail_cnt = 0;
        return;
    }

    if (++s_fail_cnt >= DISP_FAIL_LIMIT && !s_fail_logged) {
        s_fail_logged = true;
        s_ok = false;
        ESP_LOGE(TAG, "ST7789 连续 %u 次刷新失败：检查 SPI 接线与模块供电"
                      "（模块没引 RES，软件只能计数，无法硬复位）",
                 (unsigned)s_fail_cnt);
    }
}

bool disp_is_online(void)
{
    return s_ok;
}

uint16_t *disp_fb(void)
{
    return s_fb;
}

esp_err_t disp_flush(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 整帧刷新就是"从头刷 296 行"，走同一条分段路径，
     * 避免再出现一次 141 KB 的临时 DMA 缓冲申请 */
    return disp_flush_rows(0, DISP_V_RES);
}

esp_err_t disp_flush_rows(int y, int h)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (y >= DISP_V_RES || h <= 0) {
        return ESP_OK;
    }
    if (y + h > DISP_V_RES) {
        h = DISP_V_RES - y;
    }

    /* 分段搬：每段 8 行，先 memcpy 进内部 DMA 缓冲再发。
     * 直接传 PSRAM 指针会让 esp_lcd 去 malloc 等大的内部缓冲，必然失败。 */
    int done = 0;
    while (done < h) {
        int n = h - done;
        if (n > DISP_FLUSH_CHUNK_LINES) {
            n = DISP_FLUSH_CHUNK_LINES;
        }

        const uint16_t *src = &s_fb[(y + done) * DISP_H_RES];
#if CONFIG_DISP_SWAP_BYTES
        /* ST7789 要"高字节先发"，而 uint16 在内存里是低字节在前，
         * 不换就会整体偏色、细笔画发虚。 */
        uint16_t *dst = (uint16_t *)s_dma_buf;
        const int n_px = n * DISP_H_RES;
        for (int i = 0; i < n_px; i++) {
            dst[i] = (uint16_t)((src[i] >> 8) | (src[i] << 8));
        }
#else
        memcpy(s_dma_buf, src, (size_t)n * DISP_H_RES * 2);
#endif

        xSemaphoreTake(s_flush_done, 0);

        /* TK0134 没有 esp_lcd 的 panel 对象，直接发三条命令：
         *   0x2A CASET 列范围、0x2B RASET 行范围、0x2C RAMWR 写显存 */
        const int xs = CONFIG_DISP_OFFSET_X;
        const int ys = y + done + CONFIG_DISP_OFFSET_Y;
        const int ye = ys + n - 1;
        const uint8_t caset[4] = {
            (uint8_t)(xs >> 8), (uint8_t)(xs & 0xFF),
            (uint8_t)((xs + DISP_H_RES - 1) >> 8), (uint8_t)((xs + DISP_H_RES - 1) & 0xFF),
        };
        const uint8_t raset[4] = {
            (uint8_t)(ys >> 8), (uint8_t)(ys & 0xFF),
            (uint8_t)(ye >> 8), (uint8_t)(ye & 0xFF),
        };
        esp_err_t err = esp_lcd_panel_io_tx_param(s_io, 0x2A, caset, 4);
        if (err == ESP_OK) {
            err = esp_lcd_panel_io_tx_param(s_io, 0x2B, raset, 4);
        }
        if (err == ESP_OK) {
            err = esp_lcd_panel_io_tx_color(s_io, 0x2C, s_dma_buf,
                                            (size_t)n * DISP_H_RES * 2);
        }
        if (err != ESP_OK) {
            note_flush(err);
            return err;
        }
        if (xSemaphoreTake(s_flush_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
            note_flush(ESP_ERR_TIMEOUT);
            return ESP_ERR_TIMEOUT;
        }
        done += n;
    }

    note_flush(ESP_OK);
    return ESP_OK;
}


void disp_color_test(uint32_t hold_ms)
{
#if CONFIG_DISP_BOOT_TEST
    if (!s_inited) {
        return;
    }
    static const uint16_t colors[6] = {
        DISP_RED, DISP_GREEN, DISP_BLUE, DISP_WHITE, DISP_BLACK, DISP_YELLOW
    };
    static const char *names[6] = { "R", "G", "B", "W", "K", "Y" };

    const int w = DISP_H_RES / 6;
    for (int i = 0; i < 6; i++) {
        disp_fill_rect(i * w, 0, w, DISP_V_RES, colors[i]);
    }
    /* 每个色块中间写一个字母：亮块用黑字，黑块用白字 */
    for (int i = 0; i < 6; i++) {
        uint16_t fg = (i == 3 || i == 5) ? DISP_BLACK : DISP_WHITE;   /* 白/黄上用黑字 */
        if (i == 4) {
            fg = DISP_WHITE;                                          /* 黑块上用白字 */
        }
        disp_draw_string(i * w + (w - 8) / 2, DISP_V_RES / 2 - 8, names[i], fg, colors[i], 1);
    }

    ESP_LOGW(TAG, "配色自检：屏幕上从左到右应该是 红 R / 绿 G / 蓝 B / 白 W / 黑 K / 黄 Y");
    ESP_LOGW(TAG, "  R=青 改 INVERT；红蓝反 改 BGR；灰阶发花 开 SWAP_BYTES");

    (void)disp_flush();
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
#endif
}


void disp_backlight_set(uint8_t percent)
{
#if (CONFIG_DISP_PIN_BL >= 0)
    if (percent > 100) {
        percent = 100;
    }
    uint32_t duty = (1023u * percent) / 100u;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
#else
    /* 没有 BL 引脚，亮度不可调（模块背光直接接 3V3） */
    (void)percent;
#endif
}