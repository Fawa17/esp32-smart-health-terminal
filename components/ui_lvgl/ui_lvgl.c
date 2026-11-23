/* */

#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "disp.h"
#include "sys_pm.h"
#include "touch.h"
#include "ui_lvgl.h"

static const char *TAG = "ui";

#define COL_BG      0x0A1018   /* 屏幕底色（深藏青） */
#define COL_TOPBAR  0x111C2B
#define COL_CARD    0x16202E
#define COL_LINE    0x223247   /* 卡片描边 */
#define COL_TEXT    0xE9F0F7
#define COL_DIM     0x8A98A8
#define COL_GREEN   0x35E06A   /* 心率 / 在线 */
#define COL_CYAN    0x35D6E0   /* 血氧 */
#define COL_AMBER   0xFFB454   /* 体温 */
#define COL_BLUE    0x7AA2F7   /* 步数 */
#define COL_RED     0xE23B3B
#define COL_RED_BG  0xB01E1E

/* 数字字体：同样用 lv_font_conv 生成的**子集**，只含 `0123456789.:+-%v/ ` */
LV_FONT_DECLARE(font_num_48);
LV_FONT_DECLARE(font_num_28);
LV_FONT_DECLARE(font_num_20);

#define FONT_HUGE (&font_num_48)    /* 心率 / 血氧 / 时钟大字 */
#define FONT_NUM  (&font_num_28)    /* 体温 / 步数 */
#define FONT_MID  (&font_num_20)    /* 顶栏时刻、阈值、版本号 */

/* 中文字体：用 lv_font_conv 从 Windows 的“等线”生成的**子集**， */
LV_FONT_DECLARE(font_cjk_16);
#define FONT_CJK  (&font_cjk_16)

#define FONT_SMALL LV_FONT_DEFAULT

static ui_lvgl_data_t s_data;
static portMUX_TYPE   s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool  s_paused;
/* 从"暂停/息屏"恢复时要整屏重画：面板 SLEEP 期间 GDDRAM 内容不可信，
 * 只重画脏区会留下半屏旧像素。 */
static volatile bool  s_need_full;

/* OTA 遮罩：s_ota_pending 由 ui_lvgl_show_ota() 置位，真正绘制在 ui_task 里做
 * （LVGL 的所有调用都必须在同一个任务里，不能在别的任务/回调里直接画）。
 * s_ota_mask 留着是为了升级失败恢复渲染时把它删掉。 */
static volatile bool  s_ota_pending;
static char           s_ota_msg[64];
static lv_obj_t      *s_ota_mask;
static int64_t        s_ota_mask_us;     /* 遮罩出现时刻，用于超时自动撤掉 */

/* 遮罩最长时间：升级没真的起来（服务器没开、没 WiFi、服务器说已是最新）时，
 * 必须自己消失，否则就变成"黑屏卡死，只能等息屏或按按键"。
 * 真升级起来时 s_paused 会变 true，这条超时判断就自动失效了。 */
#define OTA_MASK_TIMEOUT_MS 4000

static ui_threshold_cb_t s_th_cb;
static ui_ota_cb_t       s_ota_cb;

static lv_display_t *s_disp;

/* 顶栏 */
static lv_obj_t *s_lbl_time;
static lv_obj_t *s_lbl_dot;
static lv_obj_t *s_lbl_net;

/* 四张卡片的数值 */
static lv_obj_t *s_lbl_hr, *s_lbl_spo2, *s_lbl_temp, *s_lbl_steps;
static lv_obj_t *s_bar_status;

/* 页面容器 */
static lv_obj_t *s_page_clock, *s_page_home, *s_page_set, *s_page_log;
static lv_obj_t *s_tabbar;
/* 顶栏只建一份、挂在屏幕层：以前每个页面各建一份，而 s_lbl_net/s_lbl_time
 * 是单个静态指针，建到最后一个页面时才指向它，结果只有最后那个页面的
 * 顶栏会刷新，其它页面的顶栏永远停在初始值"离线"。 */
static lv_obj_t *s_topbar;
static lv_obj_t *s_tab[3];
static int       s_page_idx;          /* 0=时钟 1=主页 2=设置 3=告警 */

/* 时钟起始页 */
static lv_obj_t *s_clock_hm, *s_clock_date, *s_clock_status, *s_clock_dot;

/* 设置页 */
static lv_obj_t *s_lbl_set_hr, *s_lbl_set_spo2, *s_lbl_set_temp, *s_lbl_set_net;
static lv_obj_t *s_lbl_ota_state;

/* 告警弹窗 */
static lv_obj_t *s_modal;
static lv_obj_t *s_lbl_modal_msg;
static lv_obj_t *s_lbl_modal_val;
static int       s_alarm_code;      /* 当前告警码 */
static int       s_alarm_ack;       /* 用户已确认过的告警码 */

/* 告警记录（环形，最多 5 条） */
#define LOG_MAX 5
static lv_obj_t *s_lbl_log[LOG_MAX];
static char      s_log[LOG_MAX][48];
static int       s_log_count;
static int       s_log_next;


static uint32_t tick_cb(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    (void)px_map;   /* DIRECT 模式：像素已经在 disp_fb() 里了 */
    disp_flush_rows(area->y1, area->y2 - area->y1 + 1);
    lv_display_flush_ready(disp);
}

/* 触摸驱动推过来的事件：只记坐标和按下状态，绝不在这里碰 LVGL */
static volatile bool     s_pressed;
static volatile uint16_t s_x, s_y;
static volatile int      s_nav_req;      /* 手势翻页请求：+1 左滑、-1 右滑 */

static void on_touch(const touch_point_t *p)
{
    /* 任何触摸都算"用户在场"，重置息屏倒计时。
     * 放在最前面：即使下面某个分支 return 了，活跃时间也已经刷新。 */
    sys_pm_notify_activity();

    if (p->event == TOUCH_EVT_DOWN || p->event == TOUCH_EVT_MOVE) {
        s_x = p->x;
        s_y = p->y;
        s_pressed = true;
    } else if (p->event == TOUCH_EVT_UP) {
        s_x = p->x;
        s_y = p->y;
        s_pressed = false;
    }

    /* 翻页只能"提请求"，真正切页在 ui 任务里做，这是触摸任务上下文 */
    if (p->gesture == TOUCH_GEST_SWIPE_LEFT) {
        s_nav_req = +1;
    } else if (p->gesture == TOUCH_GEST_SWIPE_RIGHT) {
        s_nav_req = -1;
    }
}

static void indev_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    data->point.x = s_x;
    data->point.y = s_y;
    data->state = s_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}


static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, uint32_t color,
                            const char *text)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_color(l, lv_color_hex(color), LV_PART_MAIN);
    if (font != NULL) {
        lv_obj_set_style_text_font(l, font, LV_PART_MAIN);
    }
    return l;
}

static lv_obj_t *make_card(lv_obj_t *parent, int x, int y, int w, int h, uint32_t bg)
{
    lv_obj_t *c = lv_obj_create(parent);
    lv_obj_set_pos(c, x, y);
    lv_obj_set_size(c, w, h);
    lv_obj_set_scrollable(c, false);
    lv_obj_set_style_pad_all(c, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(c, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(c, lv_color_hex(bg), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(c, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(c, lv_color_hex(COL_LINE), LV_PART_MAIN);
    return c;
}

/* 一张数据卡：标题（中文小字）+ 大数字 + 单位。
 * 数字字号由调用方给：心率/血氧用 48，体温/步数用 28（否则 4 个字符会挤满卡片）。 */
static lv_obj_t *value_card(lv_obj_t *parent, int x, int y, int w, int h,
                            uint32_t accent, const lv_font_t *font,
                            const char *title, const char *unit)
{
    lv_obj_t *card = make_card(parent, x, y, w, h, COL_CARD);

    lv_obj_t *t = make_label(card, FONT_CJK, COL_DIM, title);
    lv_obj_set_pos(t, 9, 7);

    /* 单位用中文字体：因为“步”和“℃”都不在 Montserrat 里，
     * 用 FONT_SMALL 会显示成空方框。 */
    lv_obj_t *u = make_label(card, FONT_CJK, COL_DIM, unit);
    lv_obj_align(u, LV_ALIGN_BOTTOM_RIGHT, -9, -9);

    lv_obj_t *v = make_label(card, font, accent, "--");
    lv_obj_align(v, LV_ALIGN_BOTTOM_LEFT, 9, -4);

    return v;
}


static const char *alarm_title(int code)
{
    switch (code) {
    case 1:  return "跌倒";
    case 2:  return "心率极度异常";
    case 3:  return "血氧过低";
    case 4:  return "体温过高";
    default: return "异常";
    }
}

static void modal_close_cb(lv_event_t *e)
{
    (void)e;
    s_alarm_ack = s_alarm_code;          /* 记住"这条已确认"，避免一直弹 */
    lv_obj_set_hidden(s_modal, true);
}

static void build_modal(void)
{
    /* 挂在顶层图层，盖住所有页面 */
    s_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_modal, 240, 296);
    lv_obj_set_pos(s_modal, 0, 0);
    lv_obj_set_scrollable(s_modal, false);
    lv_obj_set_style_pad_all(s_modal, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_modal, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_modal, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_modal, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_modal, LV_OPA_60, LV_PART_MAIN);

    lv_obj_t *box = make_card(s_modal, 20, 48, 200, 200, COL_RED_BG);
    lv_obj_set_style_border_width(box, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(box, 14, LV_PART_MAIN);

    lv_obj_t *title = make_label(box, FONT_CJK, COL_TEXT, "警告");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

    s_lbl_modal_msg = make_label(box, FONT_CJK, COL_TEXT, "--");
    lv_obj_align(s_lbl_modal_msg, LV_ALIGN_TOP_MID, 0, 52);

    /* 这一行放的是中文提示"请立即确认"。
     * 之前用 FONT_HUGE（Montserrat 48）—— 那个字体里一个汉字都没有，
     * 所以整行变成 5 个空方框，跟字库里有没有这几个字毫无关系。
     * 教训：字号字体必须跟着内容走，数字用 Montserrat，中文必须用 FONT_CJK。 */
    s_lbl_modal_val = make_label(box, FONT_CJK, COL_DIM, "--");
    lv_obj_align(s_lbl_modal_val, LV_ALIGN_TOP_MID, 0, 86);

    lv_obj_t *btn = lv_button_create(box);
    lv_obj_set_size(btn, 150, 40);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_obj_set_style_radius(btn, 20, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xE45555), LV_PART_MAIN);
    lv_obj_add_event_cb(btn, modal_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *bl = make_label(btn, FONT_CJK, COL_TEXT, "确认 / 关闭");
    lv_obj_center(bl);

    lv_obj_set_hidden(s_modal, true);
}


static void build_topbar(lv_obj_t *parent)
{
    /* 顶栏做成内缩的胶囊：面板四角是圆的（机构图上 AA 区圆角明显），
     * 贴边的内容会被切掉，所以左右各留 8px、上下各留 6px。 */
    lv_obj_t *bar = make_card(parent, 8, 6, 224, 26, COL_TOPBAR);
    lv_obj_set_style_radius(bar, 13, LV_PART_MAIN);
    lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
    s_topbar = bar;

    /* 左侧：圆点 + 网络文字。用一个小圆角方块当"点" */
    s_lbl_dot = lv_obj_create(bar);
    lv_obj_set_size(s_lbl_dot, 8, 8);
    lv_obj_align(s_lbl_dot, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_set_scrollable(s_lbl_dot, false);
    lv_obj_set_style_radius(s_lbl_dot, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_lbl_dot, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_lbl_dot, lv_color_hex(COL_DIM), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_lbl_dot, LV_OPA_COVER, LV_PART_MAIN);

    s_lbl_net = make_label(bar, FONT_CJK, COL_TEXT, "离线");
    lv_obj_align(s_lbl_net, LV_ALIGN_LEFT_MID, 24, 0);

    s_lbl_time = make_label(bar, FONT_MID, COL_TEXT, "--:--");
    /* 给定宽度 + 右对齐：不写死宽度时标签会撑到屏幕外被裁掉（"17:06" 变 "17:0"） */
    lv_obj_set_width(s_lbl_time, 62);
    lv_obj_set_style_text_align(s_lbl_time, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_align(s_lbl_time, LV_ALIGN_RIGHT_MID, -12, 0);
}

static void build_home(lv_obj_t *page)
{
    

    /* 4 张卡：左右各留 8px、卡间 8px；上下留白让圆角不切内容 */
    s_lbl_hr    = value_card(page, 8,   38, 108, 88, COL_GREEN, FONT_HUGE, "心率", "bpm");
    s_lbl_spo2  = value_card(page, 124, 38, 108, 88, COL_CYAN,  FONT_HUGE, "血氧", "%");
    s_lbl_temp  = value_card(page, 8,  132, 108, 88, COL_AMBER, FONT_NUM,  "体温", "℃");
    s_lbl_steps = value_card(page, 124,132, 108, 88, COL_BLUE,  FONT_NUM,  "步数", "步");

    s_bar_status = make_label(page, FONT_CJK, COL_GREEN, "一切正常");
    lv_obj_align(s_bar_status, LV_ALIGN_TOP_MID, 0, 228);
}


static void th_adj_cb(lv_event_t *e);

static lv_obj_t *make_row(lv_obj_t *page, int y, const char *title,
                          lv_obj_t **value_lbl, int idx)
{
    lv_obj_t *card = make_card(page, 8, y, 224, 44, COL_CARD);

    lv_obj_t *t = make_label(card, FONT_CJK, COL_DIM, title);
    lv_obj_align(t, LV_ALIGN_LEFT_MID, 8, 0);

    *value_lbl = make_label(card, FONT_MID, COL_TEXT, "--");
    lv_obj_align(*value_lbl, LV_ALIGN_LEFT_MID, 84, 0);

    /* 两个按钮：+ / -，用 user_data 编码 (idx, 方向) */
    const int bx[2] = { 194, 158 };
    const char *txt[2] = { "+", "-" };
    for (int i = 0; i < 2; i++) {
        lv_obj_t *b = lv_button_create(card);
        lv_obj_set_size(b, 28, 28);
        lv_obj_set_pos(b, bx[i], 8);
        lv_obj_set_style_radius(b, 14, LV_PART_MAIN);
        lv_obj_set_style_bg_color(b, lv_color_hex(0x24405C), LV_PART_MAIN);
        lv_obj_add_event_cb(b, th_adj_cb, LV_EVENT_CLICKED,
                            (void *)(intptr_t)(idx * 2 + (i == 0 ? 1 : 0)));
        lv_obj_t *bl = make_label(b, FONT_MID, COL_TEXT, txt[i]);
        lv_obj_center(bl);
    }
    return card;
}

static void ota_btn_cb(lv_event_t *e)
{
    (void)e;
    if (s_ota_cb != NULL) {
        s_ota_cb();
    } else {
        ESP_LOGW(TAG, "没注册 OTA 回调");
    }
}

static void build_settings(lv_obj_t *page)
{
    

    make_row(page, 34,  "心率上限", &s_lbl_set_hr,   0);
    make_row(page, 82,  "血氧下限", &s_lbl_set_spo2, 1);
    make_row(page, 130, "体温上限", &s_lbl_set_temp, 2);

    /* OTA 卡片 */
    lv_obj_t *card = make_card(page, 6, 178, 228, 44, COL_CARD);
    lv_obj_t *t = make_label(card, FONT_CJK, COL_DIM, "固件");
    lv_obj_align(t, LV_ALIGN_LEFT_MID, 8, 0);

    char ver[40];
    snprintf(ver, sizeof(ver), "v%s", esp_app_get_description()->version);
    lv_obj_t *v = make_label(card, FONT_MID, COL_TEXT, ver);
    lv_obj_align(v, LV_ALIGN_LEFT_MID, 52, 0);

    lv_obj_t *btn = lv_button_create(card);
    lv_obj_set_size(btn, 96, 30);
    lv_obj_set_pos(btn, 124, 7);
    lv_obj_set_style_radius(btn, 15, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x1F7A5A), LV_PART_MAIN);
    lv_obj_add_event_cb(btn, ota_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = make_label(btn, FONT_CJK, COL_TEXT, "开始升级");
    lv_obj_center(bl);

    /* 网络 */
    lv_obj_t *nc = make_card(page, 6, 226, 228, 24, COL_CARD);
    lv_obj_t *nt = make_label(nc, FONT_CJK, COL_DIM, "网络");
    lv_obj_align(nt, LV_ALIGN_LEFT_MID, 8, -6);
    s_lbl_set_net = make_label(nc, FONT_CJK, COL_TEXT, "--");
    lv_obj_align(s_lbl_set_net, LV_ALIGN_LEFT_MID, 52, -6);

    /* 这里会显示"正在下载 / 校验中"等中文，必须用中文字体。
     * FONT_SMALL 是 LV_FONT_DEFAULT（Montserrat 14，只有 ASCII），
     * 用它显示中文会变成一串方框。 */
    s_lbl_ota_state = make_label(page, FONT_CJK, COL_DIM, "");
    lv_obj_align(s_lbl_ota_state, LV_ALIGN_TOP_MID, 0, 252);
    lv_obj_set_hidden(s_lbl_ota_state, true);
}


static void build_log(lv_obj_t *page)
{
    

    lv_obj_t *title = make_label(page, FONT_CJK, COL_DIM, "最近告警");
    lv_obj_set_pos(title, 8, 34);

    for (int i = 0; i < LOG_MAX; i++) {
        s_lbl_log[i] = make_label(page, FONT_CJK, COL_TEXT, "");
        lv_obj_set_pos(s_lbl_log[i], 8, 58 + i * 30);
        lv_label_set_text(s_lbl_log[i], "");
    }

    lv_obj_t *empty = make_label(page, FONT_CJK, COL_DIM, "暂无记录");
    lv_obj_align(empty, LV_ALIGN_TOP_MID, 0, 200);
}


static void build_clock(lv_obj_t *page)
{
    /* 只显示时间：大字居中，日期在下面，最底一行提示怎么进入 */
    s_clock_hm = make_label(page, FONT_HUGE, COL_TEXT, "--:--");
    lv_obj_align(s_clock_hm, LV_ALIGN_TOP_MID, 0, 62);

    s_clock_date = make_label(page, FONT_CJK, COL_DIM, "----/--/--");
    lv_obj_align(s_clock_date, LV_ALIGN_TOP_MID, 0, 128);

    s_clock_dot = lv_obj_create(page);
    lv_obj_set_size(s_clock_dot, 8, 8);
    lv_obj_set_scrollable(s_clock_dot, false);
    lv_obj_set_style_radius(s_clock_dot, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_clock_dot, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_clock_dot, lv_color_hex(COL_GREEN), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_clock_dot, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_align(s_clock_dot, LV_ALIGN_TOP_MID, -52, 172);

    s_clock_status = make_label(page, FONT_CJK, COL_GREEN, "一切正常");
    lv_obj_align(s_clock_status, LV_ALIGN_TOP_MID, 8, 166);

    lv_obj_t *hint = make_label(page, FONT_CJK, COL_DIM, "向左滑动进入");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -16);
}


/* 统一入口：只有这里能切页，标签栏点击和手势翻页都走它 */
static void show_page(int idx)
{
    if (idx < 0) {
        idx = 3;
    }
    if (idx > 3) {
        idx = 0;
    }
    s_page_idx = idx;

    lv_obj_set_hidden(s_page_clock, idx != 0);
    lv_obj_set_hidden(s_page_home,  idx != 1);
    lv_obj_set_hidden(s_page_set,   idx != 2);
    lv_obj_set_hidden(s_page_log,   idx != 3);

    /* 时钟页要保持纯时间，不显示顶栏和底部标签栏 */
    lv_obj_set_hidden(s_tabbar, idx == 0);
    lv_obj_set_hidden(s_topbar, idx == 0);

    for (int i = 0; i < 3; i++) {
        lv_obj_set_style_bg_color(s_tab[i],
                                  lv_color_hex((idx - 1) == i ? 0x1D4E6B : COL_TOPBAR),
                                  LV_PART_MAIN);
    }
}

static void tab_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    show_page(idx + 1);
}

static void build_tabbar(lv_obj_t *parent)
{
    static const char *names[3] = { "主页", "设置", "告警" };
    /* 底部标签栏同样内缩 8px，避开圆角；高度收到 38 给内容腾地方 */
    lv_obj_t *bar = make_card(parent, 8, 250, 224, 38, COL_TOPBAR);
    lv_obj_set_style_radius(bar, 19, LV_PART_MAIN);
    lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
    s_tabbar = bar;

    for (int i = 0; i < 3; i++) {
        lv_obj_t *b = lv_button_create(bar);
        lv_obj_set_size(b, 64, 30);
        lv_obj_set_pos(b, 8 + i * 72, 4);
        lv_obj_set_style_radius(b, 15, LV_PART_MAIN);
        lv_obj_set_style_bg_color(b, lv_color_hex(i == 0 ? 0x1D4E6B : COL_TOPBAR),
                                  LV_PART_MAIN);
        lv_obj_add_event_cb(b, tab_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        lv_obj_t *l = make_label(b, FONT_CJK, COL_TEXT, names[i]);
        lv_obj_center(l);
        s_tab[i] = b;
    }
}


static int eval_alarm(const ui_lvgl_data_t *d)
{
    if (d->fall) {
        return 1;
    }
    if (d->hr_ok && d->heart_max > 0 && d->hr > d->heart_max) {
        return 2;
    }
    if (d->spo2_ok && d->spo2_min > 0 && d->spo2 < d->spo2_min) {
        return 3;
    }
    if (d->temp_ok && d->temp_max_x10 > 0 && (int)(d->temp_c * 10) > d->temp_max_x10) {
        return 4;
    }
    return 0;
}

static void log_push(const char *text)
{
    strncpy(s_log[s_log_next], text, sizeof(s_log[0]) - 1);
    s_log[s_log_next][sizeof(s_log[0]) - 1] = '\0';
    s_log_next = (s_log_next + 1) % LOG_MAX;
    if (s_log_count < LOG_MAX) {
        s_log_count++;
    }

    /* 最新一条显示在最上面 */
    for (int i = 0; i < LOG_MAX; i++) {
        int idx = (s_log_next - 1 - i + LOG_MAX * 2) % LOG_MAX;
        lv_label_set_text(s_lbl_log[i], (i < s_log_count) ? s_log[idx] : "");
    }
}

/* 0=周日 … 6=周六（Sakamoto 公式） */
static int weekday_of(int y, int m, int d)
{
    static const int t[12] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    int yy = y - (m < 3);
    int w = (yy + yy / 4 - yy / 100 + yy / 400 + t[(m - 1) % 12] + d) % 7;
    return (w + 7) % 7;
}

/*
 * LVGL 自带的 lv_snprintf **不支持 %f**（除非打开 LV_SPRINTF_USE_FLOAT），
 * 用 lv_label_set_text_fmt(lbl, "%.1f", x) 会打出乱码，这就是之前
 * "体温上限 1□" 的根因。所有小数都在这里按整数拼。
 */
static void fmt_temp(char *buf, size_t n, float t)
{
    int t10 = (int)(t * 10.0f + (t >= 0.0f ? 0.5f : -0.5f));
    const char *sign = (t10 < 0) ? "-" : "";
    int a = (t10 < 0) ? -t10 : t10;
    /* 先拼到本地的 24 字节缓冲再拷贝：直接 snprintf 到调用者的短缓冲时，
     * GCC 会按 int 的最坏位数(11)推算总长度并报 -Werror=format-truncation。 */
    char tmp[24];
    snprintf(tmp, sizeof(tmp), "%s%d.%d", sign, a / 10, a % 10);
    const size_t len = strlen(tmp);
    const size_t cp  = (len < n - 1) ? len : (n - 1);
    memcpy(buf, tmp, cp);
    buf[cp] = '\0';
}

static void refresh(void)
{
    ui_lvgl_data_t d;
    taskENTER_CRITICAL(&s_lock);
    d = s_data;
    taskEXIT_CRITICAL(&s_lock);

    if (d.time_ok) {
        static const char *const wkn[7] = { "日", "一", "二", "三", "四", "五", "六" };
        lv_label_set_text_fmt(s_clock_hm, "%02d:%02d", d.hour, d.minute);
        lv_label_set_text_fmt(s_clock_date, "%04d-%02d-%02d 星期%s",
                              d.year, d.month, d.day, wkn[weekday_of(d.year, d.month, d.day)]);
    } else {
        lv_label_set_text(s_clock_hm, "--:--");
        lv_label_set_text(s_clock_date, "等待对时");
    }

    if (d.time_ok) {
        lv_label_set_text_fmt(s_lbl_time, "%02d:%02d", d.hour, d.minute);
    } else {
        lv_label_set_text(s_lbl_time, "--:--");
    }

    switch (d.net) {
    case 2:
        lv_label_set_text(s_lbl_net, "已联网");
        lv_obj_set_style_bg_color(s_lbl_dot, lv_color_hex(COL_GREEN), LV_PART_MAIN);
        break;
    case 1:
        lv_label_set_text(s_lbl_net, "连接中");
        lv_obj_set_style_bg_color(s_lbl_dot, lv_color_hex(COL_AMBER), LV_PART_MAIN);
        break;
    default:
        lv_label_set_text(s_lbl_net, "离线");
        lv_obj_set_style_bg_color(s_lbl_dot, lv_color_hex(COL_DIM), LV_PART_MAIN);
        break;
    }

    if (d.hr_ok) {
        lv_label_set_text_fmt(s_lbl_hr, "%d", d.hr);
    } else {
        lv_label_set_text(s_lbl_hr, "--");
    }
    if (d.spo2_ok) {
        lv_label_set_text_fmt(s_lbl_spo2, "%d", d.spo2);
    } else {
        lv_label_set_text(s_lbl_spo2, "--");
    }
    if (d.temp_ok) {
        char tb[12];
        fmt_temp(tb, sizeof(tb), d.temp_c);
        lv_label_set_text(s_lbl_temp, tb);
    } else {
        lv_label_set_text(s_lbl_temp, "--");
    }
    {
        char sb[16];
        snprintf(sb, sizeof(sb), "%lu", (unsigned long)d.steps);
        lv_label_set_text(s_lbl_steps, sb);
    }

    const int code = eval_alarm(&d);

    /* 时钟页的状态行与主页状态条同步 */
    if (code != 0) {
        lv_label_set_text(s_clock_status, alarm_title(code));
        lv_obj_set_style_text_color(s_clock_status, lv_color_hex(COL_RED), LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_clock_dot, lv_color_hex(COL_RED), LV_PART_MAIN);
    } else if (d.fault != 0) {
        lv_label_set_text(s_clock_status, "传感器掉线");
        lv_obj_set_style_text_color(s_clock_status, lv_color_hex(COL_AMBER), LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_clock_dot, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    } else {
        lv_label_set_text(s_clock_status, "一切正常");
        lv_obj_set_style_text_color(s_clock_status, lv_color_hex(COL_GREEN), LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_clock_dot, lv_color_hex(COL_GREEN), LV_PART_MAIN);
    }

    if (code != 0) {
        lv_label_set_text(s_bar_status, alarm_title(code));
        lv_obj_set_style_text_color(s_bar_status, lv_color_hex(COL_RED), LV_PART_MAIN);
    } else if (d.fault != 0) {
        lv_label_set_text(s_bar_status, "传感器掉线");
        lv_obj_set_style_text_color(s_bar_status, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    } else {
        lv_label_set_text(s_bar_status, "一切正常");
        lv_obj_set_style_text_color(s_bar_status, lv_color_hex(COL_GREEN), LV_PART_MAIN);
    }

    if (code != 0 && code != s_alarm_ack) {
        char msg[64];
        if (code == 2) {
            snprintf(msg, sizeof(msg), "心率  %d bpm", d.hr);
        } else if (code == 3) {
            snprintf(msg, sizeof(msg), "血氧  %d %%", d.spo2);
        } else if (code == 4) {
            char tb[12];
            fmt_temp(tb, sizeof(tb), d.temp_c);
            snprintf(msg, sizeof(msg), "体温  %.11s℃", tb);
        } else {
            snprintf(msg, sizeof(msg), "检测到跌倒");
        }
        lv_label_set_text(s_lbl_modal_msg, msg);
        lv_label_set_text(s_lbl_modal_val, "请立即确认");
        lv_obj_set_hidden(s_modal, false);
    } else if (code == 0) {
        s_alarm_ack = 0;
        lv_obj_set_hidden(s_modal, true);
    }
    if (code != 0 && code != s_alarm_code) {
        char line[48];
        snprintf(line, sizeof(line), "%02d:%02d  %s", d.hour, d.minute, alarm_title(code));
        log_push(line);
    }
    s_alarm_code = code;

    lv_label_set_text_fmt(s_lbl_set_hr, "%d", d.heart_max);
    lv_label_set_text_fmt(s_lbl_set_spo2, "%d", d.spo2_min);
    {
        /* 缓冲要留够：编译器按 int 的最坏情况（10 位）算，16 字节会被判可能截断 */
        char tb[40];
        snprintf(tb, sizeof(tb), "%d.%d℃", d.temp_max_x10 / 10, d.temp_max_x10 % 10);
        lv_label_set_text(s_lbl_set_temp, tb);
    }
    lv_label_set_text(s_lbl_set_net,
                      d.net == 2 ? "已联网" : (d.net == 1 ? "连接中" : "离线"));
}


static void th_adj_cb(lv_event_t *e)
{
    intptr_t code = (intptr_t)lv_event_get_user_data(e);
    const int idx   = (int)(code / 2);
    const int delta = (code & 1) ? +1 : -1;

    ui_lvgl_data_t d;
    taskENTER_CRITICAL(&s_lock);
    d = s_data;
    taskEXIT_CRITICAL(&s_lock);

    const char *key = NULL;
    int v = 0;

    switch (idx) {
    case 0:
        v = d.heart_max + delta * 5;
        if (v < 60)  v = 60;
        if (v > 200) v = 200;
        key = "heart_max";
        break;
    case 1:
        v = d.spo2_min + delta * 1;
        if (v < 85) v = 85;
        if (v > 99) v = 99;
        key = "spo2_min";
        break;
    default:
        v = d.temp_max_x10 + delta * 5;
        if (v < 350) v = 350;
        if (v > 420) v = 420;
        key = "temp_max";
        break;
    }

    /* 先更新本地副本，界面立刻有反馈；再交给 main 落盘/上报 */
    taskENTER_CRITICAL(&s_lock);
    if (idx == 0)      s_data.heart_max = v;
    else if (idx == 1) s_data.spo2_min  = v;
    else               s_data.temp_max_x10 = v;
    taskEXIT_CRITICAL(&s_lock);

    if (s_th_cb != NULL) {
        s_th_cb(key, v);
    }
    refresh();
}


/* 建"正在升级"全屏遮罩。只在 UI 任务里调用。 */
static void show_ota_screen(void)
{
    if (s_ota_mask == NULL) {
        s_ota_mask = lv_obj_create(lv_layer_top());
        lv_obj_set_size(s_ota_mask, DISP_H_RES, DISP_V_RES);
        lv_obj_set_pos(s_ota_mask, 0, 0);
        lv_obj_set_style_bg_color(s_ota_mask, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(s_ota_mask, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(s_ota_mask, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(s_ota_mask, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(s_ota_mask, 0, LV_PART_MAIN);
        lv_obj_set_scrollable(s_ota_mask, false);

        lv_obj_t *t = lv_label_create(s_ota_mask);
        lv_label_set_text(t, s_ota_msg[0] ? s_ota_msg : "正在升级，请勿断电");
        lv_obj_set_style_text_color(t, lv_color_hex(0xE6EDF3), LV_PART_MAIN);
        lv_obj_set_style_text_font(t, FONT_CJK, LV_PART_MAIN);
        lv_obj_set_style_text_align(t, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_width(t, DISP_H_RES - 32);
        lv_obj_align(t, LV_ALIGN_CENTER, 0, 0);
    } else {
        /* 已经建过：只更新文字（进度之类的场景） */
        lv_obj_t *t = lv_obj_get_child(s_ota_mask, 0);
        if (t != NULL && s_ota_msg[0]) {
            lv_label_set_text(t, s_ota_msg);
        }
    }
}

static void ui_task(void *arg)
{
    (void)arg;
    uint32_t n = 0;

    for (;;) {
        /* OTA 提示必须抢在 s_paused 之前处理：一旦升级起来渲染就被挂起了，
         * 这里用 lv_refr_now() 同步刷一帧，保证用户看到的是"正在升级"
         * 而不是定格在原页面。 */
        if (s_ota_pending) {
            s_ota_pending = false;
            show_ota_screen();
            s_ota_mask_us = esp_timer_get_time();
            lv_refr_now(NULL);
        }

        /* 升级没起来就把遮罩撤掉，避免黑屏卡住。
         * 一旦 s_paused 为真（真的在下载），这里不再插手。 */
        if (s_ota_mask != NULL && !s_paused &&
            (esp_timer_get_time() - s_ota_mask_us) > (int64_t)OTA_MASK_TIMEOUT_MS * 1000) {
            lv_obj_delete(s_ota_mask);
            s_ota_mask = NULL;
            s_need_full = true;
        }

        if (s_paused) {
            /* OTA 写 flash 期间不能碰 PSRAM 里的帧缓冲，整个渲染循环挂起。
             * 息屏时也走这条分支：面板已经 SLPIN，画了也看不见，纯浪费 CPU。 */
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (s_need_full) {
            s_need_full = false;
            lv_obj_invalidate(lv_screen_active());   /* 整屏标记为脏 */
            n = 0;                                   /* 让下面立刻 refresh 一次 */
        }

        /* 手势翻页请求（由触摸任务置位），在这里才真正切页 */
        int nav = s_nav_req;
        if (nav != 0) {
            s_nav_req = 0;
            show_page(s_page_idx + nav);
        }

        if ((n % 5) == 0) {     /* 约 150ms 刷一次数据 */
            refresh();
        }
        n++;

        uint32_t d = lv_timer_handler();
        if (d < 5) {
            d = 5;
        }
        if (d > 30) {
            d = 30;
        }
        vTaskDelay(pdMS_TO_TICKS(d));
    }
}


void ui_lvgl_set_data(const ui_lvgl_data_t *d)
{
    taskENTER_CRITICAL(&s_lock);
    s_data = *d;
    taskEXIT_CRITICAL(&s_lock);
}

void ui_lvgl_set_paused(bool paused)
{
    if (s_paused && !paused) {
        /* 升级失败/结束，恢复渲染：先把"正在升级"遮罩拆掉，再整屏重画 */
        if (s_ota_mask != NULL) {
            lv_obj_delete(s_ota_mask);
            s_ota_mask = NULL;
        }
        s_need_full = true;   /* 恢复渲染：下一拍整屏重画 */
    }
    s_paused = paused;
}

void ui_lvgl_force_full_refresh(void)
{
    s_need_full = true;
}

void ui_lvgl_show_ota(const char *msg)
{
    snprintf(s_ota_msg, sizeof(s_ota_msg), "%s", (msg != NULL) ? msg : "正在升级，请勿断电");
    s_ota_pending = true;
}

void ui_lvgl_set_threshold_cb(ui_threshold_cb_t cb)
{
    s_th_cb = cb;
}

void ui_lvgl_set_ota_cb(ui_ota_cb_t cb)
{
    s_ota_cb = cb;
}

esp_err_t ui_lvgl_start(void)
{
    lv_init();
    lv_tick_set_cb(tick_cb);

    s_disp = lv_display_create(DISP_H_RES, DISP_V_RES);
    if (s_disp == NULL) {
        ESP_LOGE(TAG, "lv_display_create 失败");
        return ESP_ERR_NO_MEM;
    }
    lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(s_disp, flush_cb);
    lv_display_set_buffers(s_disp, disp_fb(), NULL, DISP_FB_BYTES,
                           LV_DISPLAY_RENDER_MODE_DIRECT);

    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(COL_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_scrollable(scr, false);

    /* 四个页面（先全部建好，切换靠 hidden） */
    s_page_clock = lv_obj_create(scr);
    lv_obj_set_size(s_page_clock, 240, 296);
    lv_obj_set_pos(s_page_clock, 0, 0);
    lv_obj_set_scrollable(s_page_clock, false);
    lv_obj_set_style_pad_all(s_page_clock, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_page_clock, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_page_clock, LV_OPA_TRANSP, LV_PART_MAIN);
    build_clock(s_page_clock);

    s_page_home = lv_obj_create(scr);
    lv_obj_set_size(s_page_home, 240, 296);
    lv_obj_set_pos(s_page_home, 0, 0);
    lv_obj_set_scrollable(s_page_home, false);
    lv_obj_set_style_pad_all(s_page_home, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_page_home, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_page_home, LV_OPA_TRANSP, LV_PART_MAIN);
    build_home(s_page_home);

    s_page_set = lv_obj_create(scr);
    lv_obj_set_size(s_page_set, 240, 296);
    lv_obj_set_pos(s_page_set, 0, 0);
    lv_obj_set_scrollable(s_page_set, false);
    lv_obj_set_style_pad_all(s_page_set, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_page_set, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_page_set, LV_OPA_TRANSP, LV_PART_MAIN);
    build_settings(s_page_set);
    lv_obj_set_hidden(s_page_set, true);

    s_page_log = lv_obj_create(scr);
    lv_obj_set_size(s_page_log, 240, 296);
    lv_obj_set_pos(s_page_log, 0, 0);
    lv_obj_set_scrollable(s_page_log, false);
    lv_obj_set_style_pad_all(s_page_log, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_page_log, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_page_log, LV_OPA_TRANSP, LV_PART_MAIN);
    build_log(s_page_log);
    lv_obj_set_hidden(s_page_log, true);

    build_tabbar(scr);              /* 标签栏建在屏幕层，切页时不会被一起藏起来 */
    build_topbar(scr);              /* 顶栏同样只建一份，跟着屏幕层走 */

    build_modal();

    show_page(0);                   /* 开机停在时钟页，左滑进入主页 */

    /* 输入设备 */
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, indev_read_cb);

    /* 触摸：驱动把事件推给 on_touch，indev 只读共享变量 */
    esp_err_t terr = touch_init();
    if (terr == ESP_OK) {
        ESP_LOGI(TAG, "触摸就绪：%s @0x%02X", touch_get_model(), touch_get_addr());
        touch_set_callback(on_touch);
        (void)touch_start_task();
    } else {
        ESP_LOGW(TAG, "触摸不可用（%s）：界面能看，但点不了", esp_err_to_name(terr));
    }

    /* 栈 8192：LVGL 的绘制函数是**递归**的（控件树遍历 + 样式继承），
     * 深度随界面嵌套层数变化，不是固定值。实测 6144 字节时历史最小余量
     * 只剩 652 字节（约 89% 已用），一旦碰到更深的界面或更长的文本就会
     * 栈溢出 panic（***ERROR*** A stack overflow in task ui）。
     * 这里直接给到 8192，留一倍余量。 */
    if (xTaskCreatePinnedToCore(ui_task, "ui", 8192, NULL, 4, NULL, 1) != pdPASS) {
        /* 创建失败（内部 RAM 不够）时任务根本不存在，屏幕上会停在最后一帧、
         * 触摸也永远读不到，看着就像"死机"。必须显式报出来，否则无从排查。 */
        ESP_LOGE(TAG, "界面任务创建失败：内部 RAM 不足（需要 8192 字节栈）");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "LVGL 界面已启动（时钟页 + 3 页 + 告警弹窗，左/右滑动翻页）");
    return ESP_OK;
}