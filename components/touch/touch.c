/*
 * touch.c，触摸驱动：TK020F1131 模块上的 CST816S（I2C @0x15）
 *
 * 依据原理图 SCH_TK020F1131_PB（嘉立创 EDA，V1.0）确定的事实：
 *   U2 = CST816S，地址 0x15；SCL=U2.18、SDA=U2.19；
 *   RST(U2.17) 只接了 R3 100k + C12/C13 5.6nF 的"硬件阻容复位"，
 *       没有引到排针，所以软件永远无法主动复位它；
 *   IRQ(U2.20) 同样没有引出，所以只能轮询，不能等中断。
 *   排针 P1/P2 共 8 针：3.3V / GND / SCL / SDA（触摸 I2C）
 *                          + CS / SCK / DC / MOSI（显示 SPI）
 *   注意：排针上的 SCL/SDA 是触摸的 I2C，SCK/MOSI 是屏的 SPI，是两组独立信号。
 *
 * 寄存器（CST816S）：
 *   0x01  手势编号
 *   0x02  触点数（低 4 位，0 = 无触摸）
 *   0x03  X 高 4 位（低 4 位是事件标志）
 *   0x04  X 低 8 位
 *   0x05  Y 高 4 位
 *   0x06  Y 低 8 位
 *   0xA7  芯片 ID（CST816S = 0xB5，CST716 = 0xB6）
 *   0xA9  固件版本
 *   0xFA  DisAutoSleep（bit0 = 1 关闭自动休眠）
 *
 * 一次连读 0x01~0x06 六个字节就能拿到全部信息。
 */

#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "bsp_i2c.h"
#include "touch.h"

static const char *TAG = "touch";

/* Kconfig 里的量程，缩短一点方便下面写 #if */
#define TOUCH_RAW_W CONFIG_TOUCH_I2C_RAW_W
#define TOUCH_RAW_H CONFIG_TOUCH_I2C_RAW_H

#define TOUCH_REG_GESTURE  0x01
#define TOUCH_REG_STATUS   0x02
#define TOUCH_REG_XH       0x03
#define TOUCH_REG_XL       0x04
#define TOUCH_REG_YH       0x05
#define TOUCH_REG_YL       0x06

/* 各家的芯片 ID 寄存器（读不到也不影响用，只是打印用） */
#define CST816_REG_CHIP_ID 0xA7
#define CST816_REG_PROJ_ID 0xA8
#define CST816_REG_FW_VER  0xA9
#define CST816_REG_DIS_AUTOSLEEP 0xFA   /* bit0 = 1：关闭自动休眠 */
#define FT6236_REG_CHIP_ID 0xA8

static uint8_t     s_addr;      /* 0 = 还没探测成功 */
static const char *s_model = "未识别";
static uint8_t     s_chip_id;   /* CST816S = 0xB5 */
static bool        s_last_pressed; 
static uint16_t    s_last_x, s_last_y;

#define TOUCH_DEBOUNCE_N   2    /* 连续 2 次一致才认"按下/松开"变化（50Hz 下 = 40ms） */
#define TOUCH_MOVE_EPS     2    /* 坐标变化超过 2px 才算移动，滤掉抖动 */

static bool        s_pressed_stable;   /* 去抖后的稳定状态 */
static uint8_t     s_debounce;         /* 连续"与稳定状态不一致"的次数 */
static bool        s_task_started;
static void      (*s_cb)(const touch_point_t *);

/* 手势去重：寄存器 0x01 会一直保持上一次的手势值，
 * 不处理的话每次 20ms 采样都会拿到同一个值、日志刷屏。
 * 规则：值变化才报；归零之后才允许下一次相同手势重新报。 */
static touch_gesture_t s_last_gesture;
static touch_gesture_t s_pending_gesture;   /* 本次按压期间出现的最后一个手势，随 UP 一起交出去 */

/* 按压起点，用来在"芯片没识别出手势"时自己推算一个（兜底）。
 * 实测这颗 CST816S 的滑动识别偏保守：划得慢一点、距离短一点就不给结果，
 * 所以需要这一层补充。 */
static uint16_t s_down_x, s_down_y;
static int64_t  s_down_ms;

/* 待确认的单击：抬手后先挂起，等双击窗口过去还没等到第二次点击，
 * 才真正当作"单击"报出去，否则一次双击会被拆成两次单击，UI 会连点两下。 */
static int64_t  s_click_ms;
static uint16_t s_click_x, s_click_y;

/** @brief 将芯片手势值解码为标准手势 */
static touch_gesture_t gesture_decode(uint8_t raw)
{
    touch_gesture_t g;
    switch (raw) {
        case 0x01: g = TOUCH_GEST_SWIPE_UP;     break;
        case 0x02: g = TOUCH_GEST_SWIPE_DOWN;   break;
        case 0x03: g = TOUCH_GEST_SWIPE_LEFT;   break;
        case 0x04: g = TOUCH_GEST_SWIPE_RIGHT;  break;
        case 0x05: g = TOUCH_GEST_SINGLE_CLICK; break;
        case 0x0B: g = TOUCH_GEST_DOUBLE_CLICK; break;
        case 0x0C: g = TOUCH_GEST_LONG_PRESS;   break;
        default:   return TOUCH_GEST_NONE;
    }

    /* 下面的镜像/旋转处理是针对"屏幕坐标系"的，而不是芯片坐标系。对照你屏幕的方向，决定是否要翻转/旋转。
    *  map_xy() 之后再做这些镜像变换。 */
#if CONFIG_TOUCH_I2C_SWAP_XY
    switch (g) {
        case TOUCH_GEST_SWIPE_UP:    g = TOUCH_GEST_SWIPE_LEFT;  break;
        case TOUCH_GEST_SWIPE_DOWN:  g = TOUCH_GEST_SWIPE_RIGHT; break;
        case TOUCH_GEST_SWIPE_LEFT:  g = TOUCH_GEST_SWIPE_UP;    break;
        case TOUCH_GEST_SWIPE_RIGHT: g = TOUCH_GEST_SWIPE_DOWN;  break;
        default: break;
    }
#endif
#if CONFIG_TOUCH_I2C_MIRROR_Y
    switch (g) {
        case TOUCH_GEST_SWIPE_UP:   g = TOUCH_GEST_SWIPE_DOWN;  break;
        case TOUCH_GEST_SWIPE_DOWN: g = TOUCH_GEST_SWIPE_UP;    break;
        default: break;
    }
#endif
#if CONFIG_TOUCH_I2C_MIRROR_X
    switch (g) {
        case TOUCH_GEST_SWIPE_LEFT:  g = TOUCH_GEST_SWIPE_RIGHT; break;
        case TOUCH_GEST_SWIPE_RIGHT: g = TOUCH_GEST_SWIPE_LEFT;  break;
        default: break;
    }
#endif
    return g;
}

/* 芯片没给手势时，用起点→终点自己判一个。
 * 坐标已经是屏幕坐标系（map_xy 之后），所以不用再考虑镜像。 */
static touch_gesture_t gesture_from_motion(void)
{
    const int64_t dt = esp_timer_get_time() / 1000 - s_down_ms;
    if (dt > CONFIG_TOUCH_I2C_SWIPE_MAX_MS) {
        return TOUCH_GEST_NONE;               /* 慢慢拖的不算滑动 */
    }

    const int dx = (int)s_last_x - (int)s_down_x;
    const int dy = (int)s_last_y - (int)s_down_y;
    /* 计算绝对值 */
    const int adx = dx < 0 ? -dx : dx;
    const int ady = dy < 0 ? -dy : dy;

    if (adx < CONFIG_TOUCH_I2C_SWIPE_MIN_PX && ady < CONFIG_TOUCH_I2C_SWIPE_MIN_PX) {
        return TOUCH_GEST_NONE;               /* 位移太小，当单击处理 */
    }

    if (adx >= ady) {
        return dx > 0 ? TOUCH_GEST_SWIPE_RIGHT : TOUCH_GEST_SWIPE_LEFT;
    }
    return dy > 0 ? TOUCH_GEST_SWIPE_DOWN : TOUCH_GEST_SWIPE_UP;
}

/** @brief 将手势枚举值转换为中文名称 */
const char *touch_gesture_name(touch_gesture_t g)
{
    switch (g) {
        case TOUCH_GEST_SWIPE_UP:     return "上滑";
        case TOUCH_GEST_SWIPE_DOWN:   return "下滑";
        case TOUCH_GEST_SWIPE_LEFT:   return "左滑";
        case TOUCH_GEST_SWIPE_RIGHT:  return "右滑";
        case TOUCH_GEST_SINGLE_CLICK: return "单击";
        case TOUCH_GEST_DOUBLE_CLICK: return "双击";
        case TOUCH_GEST_LONG_PRESS:   return "长按";
        default:                      return "无";
    }
}

/* ---------------- 中断模式（可选） ----------------
 * 本模块（TK020F1131）的 IRQ 脚 U2.20 **没有引到 8 针排针**，
 * 默认 CONFIG_TOUCH_I2C_IRQ_GPIO = -1，走定时轮询。
 * 如果从 U2 飞线到某个 GPIO，把那个脚号填进 Kconfig,或者模块有 IRQ 引出，就可以走中断模式。
 * 这样可以省掉 50Hz 的轮询任务，节省 CPU 
 * 下面这套代码就自动切成"中断唤醒 + I2C 读取 + 任务处理"。
 *
 * 中断风暴保护：CST816S 的 IRQ 是电平/边沿混合行为，接触不良时可能
 * 在几毫秒内甩出上百个下降沿，中断里只做"计数 + 给信号量"，
 * 一旦 1 秒内超过阈值就置标记，由任务关掉这个中断源、退回轮询。 */
#if CONFIG_TOUCH_I2C_IRQ_GPIO >= 0
static SemaphoreHandle_t s_irq_sem;
static volatile bool     s_irq_storm;
static bool              s_irq_disabled;
static volatile uint32_t s_irq_count;
static volatile int64_t  s_irq_win_start_us;

static void IRAM_ATTR touch_isr(void *arg)
{
    (void)arg;

    const int64_t now = esp_timer_get_time();
    if (now - s_irq_win_start_us > 1000000LL) {   /* 1 秒滑动窗口 */
        s_irq_win_start_us = now;
        s_irq_count = 0;
    }
    if (++s_irq_count > (uint32_t)CONFIG_TOUCH_I2C_IRQ_STORM_LIMIT) {
        s_irq_storm = true;                        /* 只置标记，日志留给任务打 */
    }

    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_irq_sem, &hp);
    if (hp == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}
#endif

/* 连续读失败计数：芯片没有 RST 引出，掉线了软件救不回来，只能提示断电 */
static uint32_t    s_fail_cnt;
static bool        s_fail_warned;

/* 尝试在某个地址上读一个字节：能读到就当这个地址有器件 */
static bool probe_addr(uint8_t addr, uint8_t reg, uint8_t *val)
{
    if (bsp_i2c_probe(addr) != ESP_OK) {
        return false;
    }
    /* 探测通了还不够，再读一个寄存器的值，排除"地址碰巧应答"的情况 */
    return bsp_i2c_read_reg(addr, reg, val, 1) == ESP_OK;
}

esp_err_t touch_init(void)
{
    uint8_t v = 0;

    /* ---- 1. CST816S：常见地址 0x15，但确实存在 0x2A 的板子 ----
     * 所以这里两个都试，探测记录实际地址，后面全用 s_addr。 */
    uint8_t sa = 0;
    if (probe_addr(0x15, CST816_REG_CHIP_ID, &v)) {
        sa = 0x15;
    } else if (probe_addr(0x2A, CST816_REG_CHIP_ID, &v)) {
        sa = 0x2A;
    }

    if (sa != 0) {
        s_addr    = sa;
        s_model   = "CST816S";
        s_chip_id = v;

        uint8_t proj = 0, fw = 0;
        (void)bsp_i2c_read_reg(sa, CST816_REG_PROJ_ID, &proj, 1);
        (void)bsp_i2c_read_reg(sa, CST816_REG_FW_VER,  &fw,   1);

        ESP_LOGI(TAG, "触摸 IC：CST816S @0x%02X（芯片ID=0x%02X 工程号=0x%02X 固件=v%u.%u）",
                 sa, v, proj, (unsigned)(fw >> 4), (unsigned)(fw & 0x0F));
        if (v != 0xB5 && v != 0xB6) {
            ESP_LOGW(TAG, "芯片 ID 不是预期的 0xB5(CST816S)，但寄存器布局一致，仍按 CST816 处理");
        }

        /* 关自动休眠，这是没有 IRQ 引出时的必需项。 */
#if CONFIG_TOUCH_I2C_DIS_AUTOSLEEP
        uint8_t dis = 0x01, back = 0xFF;
        if (bsp_i2c_write_reg(sa, CST816_REG_DIS_AUTOSLEEP, dis) == ESP_OK &&
            bsp_i2c_read_reg(sa, CST816_REG_DIS_AUTOSLEEP, &back, 1) == ESP_OK) {
            if (back & 0x01) {
                ESP_LOGI(TAG, "已关闭自动休眠（0xFA=0x%02X）", back);
            } else {
                ESP_LOGW(TAG, "关自动休眠未生效（0xFA 读回 0x%02X）", back);
            }
        } else {
            ESP_LOGW(TAG, "寄存器 0xFA 读写失败，跳过自动休眠设置");
        }
#else
        uint8_t fb = 0xFF;
        if (bsp_i2c_read_reg(sa, CST816_REG_DIS_AUTOSLEEP, &fb, 1) == ESP_OK) {
            ESP_LOGI(TAG, "0xFA 当前值=0x%02X（未改动自动休眠设置）", fb);
        }
#endif
        goto found;
    }

    if (probe_addr(0x38, FT6236_REG_CHIP_ID, &v)) {
        s_addr  = 0x38;
        s_model = "FT6236";
        ESP_LOGI(TAG, "检测到触摸 IC：FT6236 @0x38，芯片 ID=0x%02X", v);
        goto found;
    }

    if (bsp_i2c_probe(0x5D) == ESP_OK) {
        s_addr  = 0x5D;
        s_model = "FT62xx(0x5D)";
        ESP_LOGW(TAG, "0x5D 有应答，按 FT 系列处理（没读到 ID，可能是备用地址）");
        goto found;
    }

    ESP_LOGE(TAG, "没找到触摸 IC（试过 0x15/0x38/0x5D）");
    ESP_LOGE(TAG, "排查：1) 模块 SCL/SDA 是否并到 I2C 总线(GPIO8/9) 2) 模块是否上电"
                  " 3) 先跑『只扫 I2C 总线』模式看有没有多出来一个地址");
    return ESP_ERR_NOT_FOUND;

found:
    s_last_pressed = false;
    s_last_x = s_last_y = 0;
    s_fail_cnt = 0;
    s_fail_warned = false;
    return ESP_OK;
}

uint8_t touch_get_addr(void)
{
    return s_addr;
}

const char *touch_get_model(void)
{
    return s_model;
}

/* 把 IC 的原始坐标映射到屏幕坐标系，并做可配的翻转 */
static void map_xy(uint16_t *x, uint16_t *y)
{
    uint16_t rx = *x;
    uint16_t ry = *y;

#if CONFIG_TOUCH_I2C_SWAP_XY
    uint16_t t = rx; rx = ry; ry = t;
#endif
#if CONFIG_TOUCH_I2C_MIRROR_X
    rx = (rx >= TOUCH_RAW_W) ? 0 : (TOUCH_RAW_W - 1 - rx);
#endif
#if CONFIG_TOUCH_I2C_MIRROR_Y
    ry = (ry >= TOUCH_RAW_H) ? 0 : (TOUCH_RAW_H - 1 - ry);
#endif

    /* 仿射映射：把实测的"有效触摸区间"搬到整屏。
     * 面板的触摸层往往比显示层小一圈（本例实测：X 只到 10~229、Y 只到 25~253），
     * 不做这一步，屏幕最右和最下的按钮永远点不到，表现为"点了没反应"。
     * 全程整数运算，不用浮点（ISR/任务里都安全）。 */
    const int32_t xmin = CONFIG_TOUCH_I2C_X_MIN;
    const int32_t xmax = CONFIG_TOUCH_I2C_X_MAX;
    const int32_t ymin = CONFIG_TOUCH_I2C_Y_MIN;
    const int32_t ymax = CONFIG_TOUCH_I2C_Y_MAX;

    int32_t tx = rx;
    int32_t ty = ry;
    if (xmax > xmin) {
        tx = (tx - xmin) * 239 / (xmax - xmin);
    }
    if (ymax > ymin) {
        ty = (ty - ymin) * 295 / (ymax - ymin);
    }

    /* 夹到屏幕范围内：手指按在有效区外时也不会报出越界坐标 */
    if (tx < 0)   tx = 0;   else if (tx > 239) tx = 239;
    if (ty < 0)   ty = 0;   else if (ty > 295) ty = 295;

    *x = (uint16_t)tx;
    *y = (uint16_t)ty;
}

bool touch_poll(touch_point_t *p)
{
    if (p == NULL) {
        return false;
    }
    memset(p, 0, sizeof(*p));
    p->event = TOUCH_EVT_NONE;

    if (s_addr == 0) {
        return false;
    }

    /* 一次连读 6 字节：手势、点数、XH、XL、YH、YL */
    uint8_t buf[6] = { 0 };
    if (bsp_i2c_read_reg(s_addr, TOUCH_REG_GESTURE, buf, sizeof(buf)) != ESP_OK) {
        /* 偶尔一次读失败是总线抖动，下个周期再试；连续失败才是真掉线 */
        if (++s_fail_cnt == 20 && !s_fail_warned) {
            s_fail_warned = true;
            ESP_LOGE(TAG, "触摸 IC 连续 20 次无响应，已经掉线");
            ESP_LOGE(TAG, "这块模块没有引出 RST（复位只在板上靠 R3+C 做上电复位），"
                          "软件无法唤醒，只能断电重新上电");
        }
        return false;
    }
    s_fail_cnt = 0;
    s_fail_warned = false;

    const uint8_t points = buf[TOUCH_REG_STATUS - TOUCH_REG_GESTURE] & 0x0F;
    uint16_t x = (uint16_t)(((buf[TOUCH_REG_XH - TOUCH_REG_GESTURE] & 0x0F) << 8) |
                            buf[TOUCH_REG_XL - TOUCH_REG_GESTURE]);
    uint16_t y = (uint16_t)(((buf[TOUCH_REG_YH - TOUCH_REG_GESTURE] & 0x0F) << 8) |
                            buf[TOUCH_REG_YL - TOUCH_REG_GESTURE]);
    const bool raw_pressed = (points > 0);
    const touch_gesture_t gest = gesture_decode((uint8_t)(buf[0] & 0x0F));
    if (raw_pressed) {
        map_xy(&x, &y);
    }

    if (raw_pressed != s_pressed_stable) {
        if (++s_debounce < TOUCH_DEBOUNCE_N) {
            /* 还没攒够两拍，这一轮不产生事件（手指还在，别急着报抬起） */
            p->x = s_last_x;
            p->y = s_last_y;
            p->pressed = s_pressed_stable;
            return true;
        }
        s_debounce = 0;
        if (raw_pressed) {
            s_pressed_stable = true;
            s_last_x = x;
            s_last_y = y;
            p->x = x;
            p->y = y;
            p->points = points;
            p->pressed = true;
            p->event = TOUCH_EVT_DOWN;
            s_pending_gesture = TOUCH_GEST_NONE;   /* 新手势从这一次按压开始记 */
            s_down_x  = x;
            s_down_y  = y;
            s_down_ms = esp_timer_get_time() / 1000;
        } else {
            s_pressed_stable = false;
            p->x = s_last_x;
            p->y = s_last_y;
            p->pressed = false;
            p->event = TOUCH_EVT_UP;
            /* 抬起时把这次按压期间识别到的最后一个手势带出去；
             * 芯片没识别出来的话，用起点→终点自己推一个（单击/滑动都靠它） */
            p->gesture = s_pending_gesture;
            p->gesture_native = (s_pending_gesture != TOUCH_GEST_NONE);
            if (p->gesture == TOUCH_GEST_NONE) {
                const int64_t now_ms = esp_timer_get_time() / 1000;
                const int dur = (int)(now_ms - s_down_ms);
                const int dx  = (int)s_last_x - (int)s_down_x;
                const int dy  = (int)s_last_y - (int)s_down_y;
                const int adx = dx < 0 ? -dx : dx;
                const int ady = dy < 0 ? -dy : dy;

                if (adx < CONFIG_TOUCH_I2C_SWIPE_MIN_PX &&
                    ady < CONFIG_TOUCH_I2C_SWIPE_MIN_PX) {
                    /* 位移很小 → 这是一次点击。
                     * 芯片的 0x05/0x0B 经常不给（轻点太快、或者它认为不够"手势"），
                     * 所以点击一律由软件判：窗口内第二次 → 双击，否则挂起等超时 → 单击。 */
                    if (dur <= CONFIG_TOUCH_I2C_CLICK_MAX_MS) {
                        if (s_click_ms != 0 &&
                            (now_ms - s_click_ms) <= CONFIG_TOUCH_I2C_CLICK_DOUBLE_MS) {
                            p->gesture = TOUCH_GEST_DOUBLE_CLICK;
                            s_click_ms = 0;         /* 双击确认，取消挂起的单击 */
                        } else {
                            s_click_ms = now_ms;    /* 先挂起 */
                            s_click_x  = s_last_x;
                            s_click_y  = s_last_y;
                        }
                    }
                } else {
                    p->gesture = gesture_from_motion();
                }
                p->gesture_native = false;
            }
            s_pending_gesture = TOUCH_GEST_NONE;
        }
        return true;
    }
    s_debounce = 0;

    if (s_pressed_stable) {
        const int dx = (int)x - (int)s_last_x;
        const int dy = (int)y - (int)s_last_y;
        if (dx >= TOUCH_MOVE_EPS || dx <= -TOUCH_MOVE_EPS ||
            dy >= TOUCH_MOVE_EPS || dy <= -TOUCH_MOVE_EPS) {
            s_last_x = x;
            s_last_y = y;
            p->x = x;
            p->y = y;
            p->points = points;
            p->pressed = true;
            p->event = TOUCH_EVT_MOVE;
        } else {
            /* 抖动范围内：保持按下，但不报 MOVE */
            p->x = s_last_x;
            p->y = s_last_y;
            p->pressed = true;
        }
    }

    /* ---- 手势寄存器 0x01 ----
     * 芯片内部识别，手指划完立刻就能给出结果（往往还早于手指抬起），
     * 所以这里单独报一条 TOUCH_EVT_GESTURE，同时记到 pending 里，
     * 抬手的 UP 事件也会带上它，上层按 "手势优先、坐标兜底" 处理即可。
     * 同一个手势值只报一次：该寄存器会保持上一次的值不自动清零。 */
    if (gest != TOUCH_GEST_NONE) {
        s_pending_gesture = gest;
        s_click_ms = 0;                     /* 芯片已经给出判定，取消挂起的单击 */
        if (gest != s_last_gesture && p->event == TOUCH_EVT_NONE) {
            p->x = s_last_x;
            p->y = s_last_y;
            p->pressed = s_pressed_stable;
            p->gesture = gest;
            p->event = TOUCH_EVT_GESTURE;
            p->gesture_native = true;
        }
        s_last_gesture = gest;
    } else {
        s_last_gesture = TOUCH_GEST_NONE;   /* 归零，允许下一次相同手势重新报 */
    }

    /* ---- 挂起的单击到期 ----
     * 抬手之后双击窗口内一直没有第二次点击，就确认这是一次单击，补报一条
     * GESTURE 事件。放在这里而不是抬手那一刻，是为了不把双击拆成两次单击。 */
    if (s_click_ms != 0 && !s_pressed_stable && p->event == TOUCH_EVT_NONE &&
        (esp_timer_get_time() / 1000 - s_click_ms) >= CONFIG_TOUCH_I2C_CLICK_DOUBLE_MS) {
        s_click_ms = 0;
        p->x = s_click_x;
        p->y = s_click_y;
        p->pressed = false;
        p->gesture = TOUCH_GEST_SINGLE_CLICK;
        p->gesture_native = false;
        p->event = TOUCH_EVT_GESTURE;
    }

    s_last_pressed = raw_pressed;
    return true;
}

void touch_set_callback(void (*cb)(const touch_point_t *))
{
    s_cb = cb;
}

/* 息屏时由 sys_pm 置位；touch_task 看到它就整拍跳过，一个 I2C 事务都不发 */
static volatile bool s_suspended;

void touch_set_suspended(bool suspended)
{
    s_suspended = suspended;
    ESP_LOGD(TAG, "触摸采样已%s", suspended ? "挂起" : "恢复");
}

bool touch_is_suspended(void)
{
    return s_suspended;
}

static void touch_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    touch_point_t p;

    for (;;) {
        /* 息屏期间整拍跳过：不读 I2C、不产生事件。
         * 必须重置 last，否则 vTaskDelayUntil 会把挂起期间"欠"的拍数
         * 一次性补跑，恢复瞬间连发一串事件。 */
        if (s_suspended) {
            vTaskDelay(pdMS_TO_TICKS(200));
            last = xTaskGetTickCount();
            continue;
        }
#if CONFIG_TOUCH_I2C_IRQ_GPIO >= 0
        /* 中断模式：等 IRQ 唤醒。超时值取 3 个周期做兜底 ——
         * 中断丢了、或者去抖把这一拍吞掉，也还能自己转起来。 */
        bool timed = s_irq_disabled;
        if (!timed) {
            (void)xSemaphoreTake(s_irq_sem, pdMS_TO_TICKS(CONFIG_TOUCH_I2C_POLL_MS * 3));
        }

        if (s_irq_storm && !s_irq_disabled) {
            s_irq_disabled = true;
            gpio_intr_disable(CONFIG_TOUCH_I2C_IRQ_GPIO);
            ESP_LOGW(TAG, "中断风暴：1 秒内超过 %d 次 IRQ，已关闭该中断源、退回定时轮询",
                     CONFIG_TOUCH_I2C_IRQ_STORM_LIMIT);
        }
#else
        const bool timed = true;
#endif
        if (timed) {
            vTaskDelayUntil(&last, pdMS_TO_TICKS(CONFIG_TOUCH_I2C_POLL_MS));
        }

        if (touch_poll(&p) && p.event != TOUCH_EVT_NONE && s_cb != NULL) {
            s_cb(&p);
        }
    }
}

esp_err_t touch_start_task(void)
{
    if (s_addr == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_task_started) {
        return ESP_OK;
    }
    s_task_started = true;

#if CONFIG_TOUCH_I2C_IRQ_GPIO >= 0
    s_irq_sem = xSemaphoreCreateBinary();
    if (s_irq_sem == NULL) {
        s_task_started = false;
        return ESP_ERR_NO_MEM;
    }

    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_TOUCH_I2C_IRQ_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,     /* CST816S 的 IRQ 是低有效、开漏 */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        s_task_started = false;
        return err;
    }

    /* max30102 也是用 gpio_install_isr_service(0)，重复安装会返回
     * ESP_ERR_INVALID_STATE，那说明服务已经在跑，直接用即可 */
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        s_task_started = false;
        return err;
    }
    err = gpio_isr_handler_add(CONFIG_TOUCH_I2C_IRQ_GPIO, touch_isr, NULL);
    if (err != ESP_OK) {
        s_task_started = false;
        return err;
    }
    ESP_LOGI(TAG, "触摸 IRQ 挂在 GPIO%d（下降沿），任务改为中断唤醒",
             CONFIG_TOUCH_I2C_IRQ_GPIO);
#endif

    BaseType_t ok;
#if CONFIG_TOUCH_I2C_TASK_CORE < 0
    ok = xTaskCreate(touch_task, "touch", CONFIG_TOUCH_I2C_TASK_STACK,
                     NULL, CONFIG_TOUCH_I2C_TASK_PRIO, NULL);
#else
    ok = xTaskCreatePinnedToCore(touch_task, "touch", CONFIG_TOUCH_I2C_TASK_STACK,
                                 NULL, CONFIG_TOUCH_I2C_TASK_PRIO, NULL,
                                 CONFIG_TOUCH_I2C_TASK_CORE);
#endif
    if (ok != pdPASS) {
        s_task_started = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}