/**
 * @file    main.c
 * @brief   上电自检 / 演示程序
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "bsp_i2c.h"
#include "mpu6050.h"
#include "max30102.h"
#include "ntc_temp.h"
#include "ds1302.h"
#include "disp.h"
#include "touch.h"
#include "ui_lvgl.h"
#include "app_data.h"
#include "net_app.h"
#include "watchdog.h"
#include "time_keeper.h"
#include "crash_log.h"
#include "ota_app.h"
#include "sys_pm.h"
#include "esp_timer.h"

/* 看门狗心跳回调：各组件在自己的任务循环里调这个 cb，我们转手给对应槽位打卡。
 * 回调签名是 void(void)，所以只能一个槽位一个函数（C 没有闭包）。 */
__attribute__((unused)) static void hb_mpu(void)  { watchdog_kick(WD_SLOT_MPU); }
__attribute__((unused)) static void hb_max(void)  { watchdog_kick(WD_SLOT_MAX); }
__attribute__((unused)) static void hb_temp(void) { watchdog_kick(WD_SLOT_TEMP); }

/* 某些运行模式下这个 TAG 用不到，加属性避免 -Wunused 告警 */
__attribute__((unused))
static const char *TAG = "app";

/* 息屏 / 唤醒时要做的硬件动作。 */
__attribute__((unused)) static void on_pm_sleep(void)
{
    ui_lvgl_set_paused(true);          /* 先停渲染：不再往帧缓冲写 */
    touch_set_suspended(true);         /* 再停触摸：不再产生事件 */
    (void)disp_display_sleep(true);    /* 最后关面板：DISPOFF + SLPIN */
}

__attribute__((unused)) static void on_pm_wake(void)
{
    (void)disp_display_sleep(false);   /* SLPOUT 要等 120ms 才稳定 */
    touch_set_suspended(false);        /* 面板亮了再恢复 50Hz 轮询 */
    ui_lvgl_set_paused(false);
    /* 面板睡过一觉，GDDRAM 里的旧内容不可信，
     * 必须整屏重画一次，否则会看到"上黑下旧"的残影。 */
    ui_lvgl_force_full_refresh();
}

/* 屏幕自检在单独的文件里（disp_test.c），这里只声明 */
extern void run_disp_only(void);

/* 取当前时间。
 *
 * 顺序是"系统时间优先"，这一点和直觉相反，但必须这样：
 *   SNTP 对上时之后，系统时间就是准的，而且它不依赖 DS1302 写成功；
 *   而 DS1302 里如果还停在我们 init 时写的默认值（2000-01-01），
 *   或者写保护位没清掉导致写不进去，它就会一直报 2000-01-01，
 *   屏幕和网页上就永远是 2000 年 —— 这正是"网络校时成功了但时间还是不对"的原因。
 *
 * 所以：系统时间 >= 2020 就用系统时间；还没对过时（1970）才退回 DS1302。 */
static bool read_time_any(ds1302_time_t *out)
{
    time_t sys_now = time(NULL);
    struct tm t;
    localtime_r(&sys_now, &t);

    if (t.tm_year + 1900 >= 2020) {
        out->second = (uint8_t)t.tm_sec;
        out->minute = (uint8_t)t.tm_min;
        out->hour   = (uint8_t)t.tm_hour;
        out->date   = (uint8_t)t.tm_mday;
        out->month  = (uint8_t)(t.tm_mon + 1);
        out->week   = (uint8_t)(t.tm_wday == 0 ? 7 : t.tm_wday);
        out->year   = (uint8_t)((t.tm_year + 1900) % 100);
        return true;
    }

    /* 系统时间还没对过时：只能信 DS1302（它可能也不对，但至少有秒在走） */
    return ds1302_read_time(out) == ESP_OK;
}

/* 打印时间前缀。RTC 没对时也能用，只是会显示"RTC 不可用" */
__attribute__((unused))
static void make_time_prefix(char *buf, size_t len)
{
    ds1302_time_t now;
    if (read_time_any(&now)) {
        (void)snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d",
                       2000 + now.year, now.month, now.date,
                       now.hour, now.minute, now.second);
    } else {
        (void)snprintf(buf, len, "RTC 不可用");
    }
}


/* ==================================================================
 * 全部运行（正常使用）
 * ================================================================== */
#if CONFIG_APP_RUN_MODE_ALL
/* ---- 设置页的两个回调 ---- */

/* 阈值 +/-：改的是设备侧配置（报警判据用的就是这份），网页那边仍可覆盖 */
static void on_ui_threshold(const char *key, int value)
{
    app_cfg_t cfg;
    app_data_get_cfg(&cfg);

    if (strcmp(key, "heart_max") == 0) {
        cfg.heart_max = value;
    } else if (strcmp(key, "spo2_min") == 0) {
        cfg.spo2_min = value;
    } else if (strcmp(key, "temp_max") == 0) {
        cfg.temp_max = value / 10.0f;    /* 界面传的是 0.1℃ 整数 */
    } else {
        ESP_LOGW(TAG, "未知阈值项 %s", key);
        return;
    }
    app_data_set_cfg(&cfg);
    ESP_LOGI(TAG, "阈值已更新：%s=%d", key, value);
}

/* OTA 按钮：地址在 menuconfig → OTA 里配，改地址不用改代码 */
static void on_ui_ota(void)
{
    if (ota_app_in_progress()) {
        ESP_LOGW(TAG, "OTA 正在进行中，忽略本次点击");
        return;
    }
    /* 先给用户一个明确反馈再发请求。
     * 否则按下按钮后界面毫无变化（升级起来渲染就被挂起了），
     * 看起来和死机一模一样 —— 这个坑我们踩过一次。 */
    ui_lvgl_show_ota("正在检查新固件…");

    /* 不再依赖预先配置的 URL：向服务器发一次"有没有新固件"的询问。
     * 服务器把本机 IP + 固件 URL 回在 <prefix>/ota 上，设备收到就自动下载。
     * 好处是换电脑、换 WiFi 段都不用改固件。 */
    if (net_app_request_ota_check() < 0) {
        ESP_LOGW(TAG, "离线：连不上服务器，没法检查新固件");
        ui_lvgl_show_ota("离线：连不上服务器");
        return;
    }
    ESP_LOGI(TAG, "已向服务器查询新固件，等回复…");
}

static void run_all(void)
{
    ESP_LOGW(TAG, "===== 模式：全部运行 =====");

    /* 1) 总线先起来。同一条线上挂 OLED(0x3C)、MPU6050(0x68)、MAX30102(0x57) */
    ESP_ERROR_CHECK(bsp_i2c_init());

    /* 2) 扫一遍总线，现场排线问题一眼就能看出来 */
    bsp_i2c_scan();

    /* 3) MPU6050。失败不 return —— 任务里还有自动恢复逻辑 */
    esp_err_t err = mpu6050_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mpu6050_init 失败: %s（任务仍会启动并尝试恢复）",
                 esp_err_to_name(err));
    }
    mpu6050_start_task();

    /* 4) MAX30102。同样失败不 return */
    err = max30102_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "max30102_init 失败: %s（采样任务仍会启动）",
                 esp_err_to_name(err));
    }
    max30102_start_tasks();

    /* 5) NTC 温度模块。走 ADC1，不占 I2C 总线，也不参与总线恢复 */
    err = ntc_temp_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ntc_temp_init 失败: %s", esp_err_to_name(err));
    }
    ntc_temp_start_task();

    /* 6) DS1302 三线 RTC。走 GPIO 位操作，不占 I2C，也不占 UART */
    err = ds1302_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ds1302_init 失败: %s", esp_err_to_name(err));
    } else {
        ds1302_time_t now;
        if (read_time_any(&now)) {
            ESP_LOGI(TAG, "RTC 当前时间 %04d-%02d-%02d %02d:%02d:%02d",
                     2000 + now.year, now.month, now.date,
                     now.hour, now.minute, now.second);
        } else {
            /* 电池掉过电就会走到这里：CH=1 或 BCD 非法，需要重新对时 */
            ESP_LOGW(TAG, "RTC 时间不可用，等待网络对时（请检查纽扣电池）");
        }
    }

    /* 7) 显示屏 + LVGL 界面（ui_lvgl_start 内部会顺带初始化触摸并建 UI 任务） */
    bool disp_ok = false;
    /* 上电先显示 1.5 秒配色自检色块（CONFIG_DISP_BOOT_TEST），
     * 用来确认 反色/BGR/字节序 这三个开关配得对不对，确认完可以关掉。 */
    if (disp_init() == ESP_OK) {
        disp_color_test(1500);
    }

    if (disp_is_ready() && ui_lvgl_start() == ESP_OK) {
        disp_ok = true;
        ui_lvgl_set_threshold_cb(on_ui_threshold);
        ui_lvgl_set_ota_cb(on_ui_ota);
        ESP_LOGI(TAG, "显示屏就绪，LVGL 界面已启动");
    } else {
        ESP_LOGE(TAG, "显示屏初始化失败，数据只在串口输出");
    }

    /* 7.5) 电源管理：物理按键做唤醒源 + 无操作自动息屏。 */
    if (disp_ok) {
        sys_pm_hooks_t pm = { .on_sleep = on_pm_sleep, .on_wake = on_pm_wake };
        sys_pm_set_busy_check(ota_app_in_progress);   /* 升级中不许息屏 */
        esp_err_t perr = sys_pm_init(&pm);
        if (perr != ESP_OK) {
            ESP_LOGW(TAG, "电源管理启动失败（%s），屏幕将常亮", esp_err_to_name(perr));
        }
    }

    /* 8) 网络：WiFi + MQTT。这一步不阻塞 —— 连不上就自己退避重连，
     *    设备照样采集、照样显示，只是不上报 */
    (void)app_data_init();
    esp_err_t nerr = net_app_start();
    if (nerr != ESP_OK) {
        ESP_LOGE(TAG, "网络启动失败（%s），设备进入离线模式", esp_err_to_name(nerr));
    }

    /* 9) 看门狗：注册真正的心跳源。
     *    超时值给的是"任务周期的 5~8 倍"，不是周期本身 ——
     *    偶尔被更高优先级的任务抢占（比如 MAX 抢 I2C 总线）不能算卡死。
     *    界面和网络上行的任务没注册：它们卡死会由 TWDT 的空闲任务订阅抓出来，
     *    不需要应用级心跳。 */
    (void)watchdog_register(WD_SLOT_MPU,  "mpu", 1000);
    (void)watchdog_register(WD_SLOT_MAX,  "max",  500);
    (void)watchdog_register(WD_SLOT_TEMP, "ntc", 8000);
    mpu6050_set_heartbeat_cb(hb_mpu);
    max30102_set_heartbeat_cb(hb_max);
    ntc_temp_set_heartbeat_cb(hb_temp);

    /* OTA 升级期间免检：写 flash 时 cache 全关、中断被推迟，业务任务连续几秒
     * 轮不到 CPU 是正常的。不挂这个钩子，升级过程会被看门狗当成"任务卡死"复位掉
     * —— 现象就是"看到 GET 了、板子重启了、版本没变"。 */
    watchdog_set_pause_check(ota_app_in_progress);

    /* 11) 触摸已由 ui_lvgl_start() 内部初始化并注册回调（LVGL 的输入设备在读它），
     *     这里不再重复 touch_init / touch_set_callback —— 回调是独占的，
     *     后注册的会把 LVGL 那个覆盖掉，表现就是"界面点不动"。 */

    /* 10) 主循环：采集在任务里，这里只做"取快照 → 发布 → 画屏 → 打日志" */
    uint32_t steps  = 0;
    uint8_t  tumble = 0, last_tumble = 0;
    uint8_t  heart = 0, spo2 = 0, hr_ok = 0, spo2_ok = 0;
    uint32_t tick = 0;

    while (true) {
        /* 500ms 一帧：秒针看起来是连续的；整帧 141.6KB 约 28ms，占 CPU 不到 6% */
        vTaskDelay(pdMS_TO_TICKS(500));
        tick++;

        mpu6050_get_motion(&steps, &tumble);
        max30102_get_heart(&heart, &spo2, &hr_ok, &spo2_ok);

        /* NTC 的角度：温度是不是有效，和温度值本身一样重要 */
        float temp_c  = ntc_temp_get_celsius();
        bool  temp_ok = ntc_temp_is_valid();

        /* RTC：一次突发读，时分秒来自同一瞬间，不会跨秒撕裂 */
        ds1302_time_t now;
        bool time_ok = (read_time_any(&now));

        /* 先发布到数据中枢，再让各个"消费者"（屏幕 / 上报任务）自己去读。
         * 这一步必须在画屏和上报之前，否则这一轮上报的是上一帧的数据。 */
        /* 掉线位图：哪一路没响应就点哪一位。
         * 判据是"通信 / ID 校验"层面的离线，不是"数据超标"（那是告警，两回事）。
         * bit0 MPU6050 / bit1 MAX30102 / bit2 NTC / bit3 DS1302 / bit4 ST7789 */
        const uint8_t fault =
            (uint8_t)((mpu6050_is_online()  ? 0u : (1u << 0)) |
                      (max30102_is_online() ? 0u : (1u << 1)) |
                      (ntc_temp_is_online() ? 0u : (1u << 2)) |
                      (ds1302_is_online()   ? 0u : (1u << 3)) |
                      ((disp_ok && disp_is_online()) ? 0u : (1u << 4)));

        app_snapshot_t sn = {
            .uptime_s = (uint32_t)(esp_timer_get_time() / 1000000),
            .time_ok  = time_ok,
            .year     = time_ok ? 2000 + now.year : 0,
            .month    = time_ok ? now.month : 0,
            .day      = time_ok ? now.date  : 0,
            .hour     = time_ok ? now.hour   : 0,
            .minute   = time_ok ? now.minute : 0,
            .second   = time_ok ? now.second : 0,
            .hr_ok    = hr_ok,
            .hr       = heart,
            .spo2_ok  = spo2_ok,
            .spo2     = spo2,
            .temp_ok  = temp_ok,
            .temp_c   = temp_c,
            .steps    = steps,
            .fall     = (tumble != 0),
            .fault    = fault,
        };
        (void)app_data_publish(&sn);

        /* OTA 下载期间不要碰屏幕。 */
        ui_lvgl_set_paused(ota_app_in_progress() || !sys_pm_is_active());

        if (disp_ok) {
            app_cfg_t cfg;
            app_data_get_cfg(&cfg);

            ui_lvgl_data_t ui = {
                .time_ok = time_ok,
                .year    = time_ok ? 2000 + now.year : 0,
                .month   = time_ok ? now.month : 0,
                .day     = time_ok ? now.date  : 0,
                .hour    = time_ok ? now.hour   : 0,
                .minute  = time_ok ? now.minute : 0,
                .second  = time_ok ? now.second : 0,
                .hr_ok   = hr_ok,
                .hr      = heart,
                .spo2_ok = spo2_ok,
                .spo2    = spo2,
                .temp_ok = temp_ok,
                .temp_c  = temp_c,
                .steps   = steps,
                .fall    = (tumble != 0),
                .net     = (uint8_t)net_app_state(),
                /* 掉线位图直接复用上面算好的那份，避免两处判据不一致 */
                .fault   = fault,
                /* 阈值给设置页显示；temp 存 0.1℃ 整数，避免界面层做浮点比较 */
                .heart_max    = cfg.heart_max,
                .spo2_min     = cfg.spo2_min,
                .temp_max_x10 = (int)(cfg.temp_max * 10.0f + 0.5f),
            };
            ui_lvgl_set_data(&ui);
        }

        /* 每拍检查一次：时间可信就按 5 分钟节奏写回 NVS，
         * 掉电最多丢 5 分钟，但不至于退回 1970/2000。 */
        time_keeper_tick();

        #if CONFIG_APP_LOG_STACK
        /* 每 30 秒打一次全任务栈余量。 */
        static uint32_t t_dump = 0;
        if (++t_dump >= 60) {          /* 60 × 500ms = 30 秒 */
            t_dump = 0;
            watchdog_dump_tasks();
        }
#endif
        /* 跌倒是一个"事件"而不是"状态"，只在 0→1 跳变时报警，
         * 否则每 2 秒刷一条，日志里根本看不出发生过几次。
         * 所以这个判断放在每 500ms 都跑的位置，不能塞进下面的日志分支。 */
        if (tumble && !last_tumble) {
            ESP_LOGE(TAG, ">>> 检测到跌倒事件，应在此处触发告警上报 <<<");
        }
        last_tumble = tumble;

        /* 串口日志每 2 秒一条，和屏幕刷新解耦 */
        if ((tick % 4) != 0) {
            continue;
        }

        /* 每 2 秒一条汇总日志。
         * 默认关闭：它在串口里每 2 秒刷一行，看 OTA / MQTT 的日志时特别碍事。
         * 需要盯着数据时到 menuconfig 里打开“应用模式 → 打印每次采集的传感器数据”。 */
#if CONFIG_APP_LOG_VERBOSE
        char ts[40];
        if (temp_ok) {
            (void)snprintf(ts, sizeof(ts), "%.2f℃", temp_c);
        } else {
            (void)snprintf(ts, sizeof(ts), "--（未接或超量程）");
        }

        char tbuf[40];
        make_time_prefix(tbuf, sizeof(tbuf));

        ESP_LOGI(TAG, "[%s] 步数=%lu 跌倒=%u | 心率=%u(%s) 血氧=%u(%s) | 温度=%s",
                 tbuf, (unsigned long)steps, tumble,
                 heart, hr_ok ? "有效" : "无效",
                 spo2,  spo2_ok ? "有效" : "无效",
                 ts);
#endif
    }
}
#endif


/* ==================================================================
 * 只扫 I2C 总线
 * ================================================================== */
#if CONFIG_APP_RUN_MODE_I2C_ONLY
static void run_i2c_only(void)
{
    ESP_LOGW(TAG, "===== 模式：只扫 I2C 总线 =====");
    ESP_LOGW(TAG, "期望看到 0x3C(OLED) / 0x57(MAX30102) / 0x68(MPU6050)");

    ESP_ERROR_CHECK(bsp_i2c_init());

    int round = 0;
    for (;;) {
        ESP_LOGI(TAG, "-------- 第 %d 轮 --------", ++round);
        bsp_i2c_scan();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif

/* ==================================================================
 * 只测 MPU6050
 * ================================================================== */
#if CONFIG_APP_RUN_MODE_MPU_ONLY
static void run_mpu_only(void)
{
    ESP_LOGW(TAG, "===== 模式：只测 MPU6050 =====");
    ESP_LOGW(TAG, "拿起板子晃动，加速度和步数应该跟着变");

    ESP_ERROR_CHECK(bsp_i2c_init());

    uint8_t id = 0;
    if (mpu6050_read_id(&id) == ESP_OK) {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%02X（正常是 0x68）", id);
    } else {
        ESP_LOGE(TAG, "读 WHO_AM_I 失败（器件没响应，先查 RST 有没有松开）");
    }

    esp_err_t err = mpu6050_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mpu6050_init 失败: %s", esp_err_to_name(err));
    }

    /* 关键：计步不是 mpu6050_read_raw() 干的： */
    mpu6050_start_task();

    MPU6050_t raw;
    uint32_t steps = 0;
    uint8_t  tumble = 0;
    int32_t  dyn = 0;

    for (;;) {
        err = mpu6050_read_raw(&raw);
        if (err == ESP_OK) {
            mpu6050_get_motion(&steps, &tumble);
            dyn = mpu6050_get_last_dyn();
            ESP_LOGI(TAG, "ax=%6d ay=%6d az=%6d | gx=%6d gy=%6d gz=%6d "
                          "| 步数=%lu 跌倒=%u | 动态=%ld/2000",
                     raw.ax, raw.ay, raw.az, raw.gx, raw.gy, raw.gz,
                     (unsigned long)steps, tumble, (long)dyn);
        } else {
            ESP_LOGE(TAG, "读取失败: %s（总线可能挂了，会自动尝试恢复）",
                     esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
#endif

/* ==================================================================
 * 只测 MAX30102
 * ================================================================== */
#if CONFIG_APP_RUN_MODE_MAX_ONLY
static void run_max_only(void)
{
    ESP_LOGW(TAG, "===== 模式：只测 MAX30102 =====");
    ESP_LOGW(TAG, "手指轻轻放上去（别按太紧），红外 DC 应该明显抬升");

    ESP_ERROR_CHECK(bsp_i2c_init());

    uint8_t id = 0;
    if (max30102_read_part_id(&id) == ESP_OK) {
        ESP_LOGI(TAG, "PART_ID = 0x%02X（正常是 0x15）", id);
    } else {
        ESP_LOGE(TAG, "读 PART_ID 失败（器件没响应，先查供电和上拉）");
    }

    esp_err_t err = max30102_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "max30102_init 失败: %s", esp_err_to_name(err));
    }
    max30102_start_tasks();

    uint8_t heart = 0, spo2 = 0, hr_ok = 0, spo2_ok = 0;
    for (;;) {
        max30102_get_heart(&heart, &spo2, &hr_ok, &spo2_ok);
        ESP_LOGI(TAG, "心率=%3u(%s)  血氧=%3u(%s)",
                 heart, hr_ok ? "有效" : "无效",
                 spo2,  spo2_ok ? "有效" : "无效");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

/* ==================================================================
 * 只测 NTC 温度
 * ================================================================== */
#if CONFIG_APP_RUN_MODE_NTC_ONLY
static void run_ntc_only(void)
{
    ESP_LOGW(TAG, "===== 模式：只测 NTC 温度 =====");
    ESP_LOGW(TAG, "手指捏住探头，温度应该上升；两指直接捏金属壳会更快");

    esp_err_t err = ntc_temp_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ntc_temp_init 失败: %s", esp_err_to_name(err));
    }
    ntc_temp_start_task();

    for (;;) {
        int   raw = 0;
        float c   = ntc_temp_get_celsius();
        bool  ok  = ntc_temp_is_valid();

        if (ntc_temp_read_raw(&raw) != ESP_OK) {
            raw = -1;
        }

        if (ok) {
            ESP_LOGI(TAG, "ADC=%4d  温度=%.2f ℃", raw, c);
        } else {
            /* 打印原始值而不是只报错：0 说明短路，4095 说明开路，
             * 中间值说明是换算问题（多半是 R25 / B 值填错） */
            ESP_LOGE(TAG, "ADC=%4d  温度无效（0=短路 4095=开路 中间=参数配错）", raw);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

/* ==================================================================
 * 只测 DS1302
 * ================================================================== */
#if CONFIG_APP_RUN_MODE_RTC_ONLY
static void run_rtc_only(void)
{
    ESP_LOGW(TAG, "===== 模式：只测 DS1302 =====");
    ESP_LOGW(TAG, "看三件事：①秒位是否每秒 +1（不走 = 晶振没起振）"
                  "②年月日是否为 00-00-00（全 0 = 芯片从没被写成功）"
                  "③停振位 CH 是否为 1（=1 表示时钟没启动）");

    esp_err_t err = ds1302_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ds1302_init 失败: %s", esp_err_to_name(err));
    }

    /* 这里必须直接读 ds1302，不能走 read_time_any()。
     * read_time_any() 是"系统时间优先"：WiFi 一旦对上 SNTP，它返回的是网络时间，
     * 测出来的是系统时间而不是这颗芯片 —— 那这个自测就白做了。 */
    for (;;) {
        ds1302_time_t now;
        const bool halted = ds1302_is_halted();

        if (ds1302_read_time(&now) == ESP_OK) {
            ESP_LOGI(TAG, "%04d-%02d-%02d 周%d %02d:%02d:%02d  停振位=%d  合法=%d",
                     2000 + now.year, now.month, now.date, now.week,
                     now.hour, now.minute, now.second, (int)halted,
                     (int)ds1302_time_is_sane(&now));
        } else {
            ESP_LOGE(TAG, "读时间失败：BCD 非法或总线无响应（停振位=%d）。"
                          "上面那条带 hex 的日志就是原始 8 字节", (int)halted);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif


/* SNTP 对时成功后把时间写进 DS1302 —— 这样断电重启（有纽扣电池时）
 * 也能从 RTC 拿到正确时间，不用每次都等网络。
 * net_app 不认识 ds1302 组件，所以用回调把两件事解开。 */
static void app_rtc_sync(int year, int mon, int day, int hour, int min, int sec)
{
    /* Sakamoto 算法算星期：0 = 周日 */
    static const int t[12] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    int y = year - (mon < 3);
    int w = (y + y / 4 - y / 100 + y / 400 + t[mon - 1] + day) % 7;

    const ds1302_time_t tt = {
        .second = (uint8_t)sec,
        .minute = (uint8_t)min,
        .hour   = (uint8_t)hour,
        .date   = (uint8_t)day,
        .month  = (uint8_t)mon,
        .week   = (uint8_t)(w == 0 ? 7 : w),   /* DS1302 的星期是 1~7 */
        .year   = (uint8_t)(year % 100),
    };

    if (ds1302_write_time(&tt) == ESP_OK) {
        (void)ds1302_clock_start();
        ESP_LOGI(TAG, "RTC 已用网络时间校准：%04d-%02d-%02d %02d:%02d:%02d",
                 year, mon, day, hour, min, sec);
    } else {
        ESP_LOGW(TAG, "RTC 写入失败，时间只保留在系统里（检查 DS1302 接线）");
    }
}

void app_main(void)
{
    /* 看门狗要最先起来：后面任何一个初始化卡住（比如 I2C 被拉死、
     * 某个 while 等应答），都能被 TWDT 抓出来复位，
     * 而不是黑屏不动、只能拔电。 */
    /* 第一件事：查上次是不是崩过。放在看门狗之前，
     * 这样"崩溃"和"看门狗复位"两种原因会前后脚各报一条，不会混淆。 */
    crash_log_init();

    if (watchdog_init() == ESP_OK) {
        (void)watchdog_start();
    }

    /* 时间兜底：系统时间存在 RTC 内存里，断电就没了；SNTP 要等联网。
     * 这里在 NVS 里留一份，下次上电先恢复，至少不会退回 1970。
     * 必须在任何"读时间"之前调用。 */
    time_keeper_init();

    /* 网络对时成功后会回调到这里，把时间写进 RTC */
    net_app_set_time_cb(app_rtc_sync);

#if CONFIG_APP_RUN_MODE_ALL
    /* OTA 回滚保护：新固件第一次启动是"待验证"状态， */
    ota_app_set_ready_check(net_app_mqtt_connected);
    (void)ota_app_init();
#endif

#if CONFIG_APP_RUN_MODE_ALL
    run_all();
#elif CONFIG_APP_RUN_MODE_I2C_ONLY
    run_i2c_only();
#elif CONFIG_APP_RUN_MODE_MPU_ONLY
    run_mpu_only();
#elif CONFIG_APP_RUN_MODE_MAX_ONLY
    run_max_only();
#elif CONFIG_APP_RUN_MODE_NTC_ONLY
    run_ntc_only();
#elif CONFIG_APP_RUN_MODE_RTC_ONLY
    run_rtc_only();
#elif CONFIG_APP_RUN_MODE_DISP_ONLY
    run_disp_only();
#else
    ESP_LOGE(TAG, "没有选中任何运行模式，请到 menuconfig 的『应用模式』里选一个");
#endif
}