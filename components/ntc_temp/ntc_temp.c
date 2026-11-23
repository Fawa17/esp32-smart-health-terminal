/**
 * @file    ntc_temp.c
 * @brief   NTC 热敏模块（4 针 VCC/GND/DO/AO）驱动，ESP-IDF 版
 */

#include "ntc_temp.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "ntc_temp";

#ifndef CONFIG_NTC_TEMP_ADC_CHANNEL
#define CONFIG_NTC_TEMP_ADC_CHANNEL     3
#endif
#ifndef CONFIG_NTC_TEMP_R_FIXED
#define CONFIG_NTC_TEMP_R_FIXED         10000
#endif
#ifndef CONFIG_NTC_TEMP_R25
#define CONFIG_NTC_TEMP_R25             10000
#endif
#ifndef CONFIG_NTC_TEMP_B_VALUE
#define CONFIG_NTC_TEMP_B_VALUE         3950
#endif
#ifndef CONFIG_NTC_TEMP_NTC_TO_GND
#define CONFIG_NTC_TEMP_NTC_TO_GND      1
#endif
#ifndef CONFIG_NTC_TEMP_VCC_MV
#define CONFIG_NTC_TEMP_VCC_MV          3300
#endif
#ifndef CONFIG_NTC_TEMP_TASK_STACK
#define CONFIG_NTC_TEMP_TASK_STACK      3072
#endif
#ifndef CONFIG_NTC_TEMP_TASK_PRIO
#define CONFIG_NTC_TEMP_TASK_PRIO       4
#endif
#ifndef CONFIG_NTC_TEMP_TASK_CORE
#define CONFIG_NTC_TEMP_TASK_CORE       1
#endif
#ifndef CONFIG_NTC_TEMP_PERIOD_MS
#define CONFIG_NTC_TEMP_PERIOD_MS       2000
#endif

/* 注意：ADC_ATTEN_DB_12 是 adc_atten_t 的**枚举成员**，不是宏。
 * 所以不能用 #ifndef 去探测它（预处理器看不见枚举），这里直接使用；
 * IDF 5.0 之前叫 ADC_ATTEN_DB_11，两者在 5.x 里是同一个值 (=3)，已标记 deprecated。 */

#define NTC_T0_K                        298.15f     /* 25℃ 的开氏温度 */
#define NTC_TEMP_MIN_C                  (-40.0f)
#define NTC_TEMP_MAX_C                  (125.0f)
#define NTC_TEMP_EMA_ALPHA              0.35f       /* 一阶滤波，抑制 NTC 自身噪声 */


static adc_oneshot_unit_handle_t s_adc_unit = NULL;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
static adc_cali_handle_t s_cali = NULL;
#endif

static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;
static float     s_celsius   = NAN;
static bool      s_valid     = false;
static bool      s_inited    = false;
static bool      s_task_up   = false;
#if (NTC_TEMP_DO_GPIO_IO >= 0)
static uint8_t   s_do_last   = 1;       /* DO 脚未启用时不需要这个变量 */
#endif
static void    (*s_hb_cb)(void) = NULL;

/* 器件在线标志：AO 线松了 ADC 会读到 0mV 或接近 VCC，读数是"错"的但任务照转。
 * 连续 3 次判失败就当器件掉线，上层据此显示告警。 */
static volatile bool s_online = true;

bool ntc_temp_is_online(void)
{
    return s_online;
}


static esp_err_t adc_setup(void)
{
    /* 1) 建 ADC1 单元。注意必须是 ADC_UNIT_1：
     *    ESP32-S3 的 ADC2 与 WiFi 共用硬件，连上 WiFi 后 ADC2 直接读不到值。 */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc_unit),
                        TAG, "ADC1 单元创建失败");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,        /* 约 0~3.1V 量程 */
        .bitwidth = ADC_BITWIDTH_12,        /* 0~4095 */
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_unit,
                                                   (adc_channel_t)CONFIG_NTC_TEMP_ADC_CHANNEL,
                                                   &chan_cfg),
                        TAG, "ADC 通道配置失败");

    /* 2) 电压校准。出厂 eFuse 里烧了每个芯片的 ADC 修正曲线，
     *    不做校准的话，12dB 衰减下典型误差 50mV 以上，换算成温度就是好几度。 */
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = (adc_channel_t)CONFIG_NTC_TEMP_ADC_CHANNEL,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ADC 曲线校准已启用");
    } else {
        s_cali = NULL;
        ESP_LOGW(TAG, "ADC 校准不可用(%s)，退回线性换算，读数可能偏差几度",
                 esp_err_to_name(err));
    }
#endif

    return ESP_OK;
}

/** 原始码值 → 毫伏 */
static int raw_to_mv(int raw)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    int mv = 0;
    if (s_cali != NULL && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
        return mv;
    }
#endif
    return (int)((int32_t)raw * CONFIG_NTC_TEMP_VCC_MV / 4095);
}

esp_err_t ntc_temp_read_raw(int *raw)
{
    if (raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_adc_unit == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t sum = 0;
    for (int i = 0; i < NTC_TEMP_AVG_SAMPLES; i++) {
        int v = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc_unit,
                                            (adc_channel_t)CONFIG_NTC_TEMP_ADC_CHANNEL, &v),
                            TAG, "ADC 采样失败");
        sum += v;
    }
    *raw = (int)(sum / NTC_TEMP_AVG_SAMPLES);
    return ESP_OK;
}


/**
 * @brief 采一次 → 反算电阻 → Beta 公式 → 摄氏度。
 *
 * 分压反算（模块上 Rfix 与 NTC 串在 VCC 与 GND 之间，AO 取中点）：
 *   NTC 在 GND 侧：R = Rfix * Vout / (Vcc - Vout)
 *   NTC 在 VCC 侧：R = Rfix * (Vcc - Vout) / Vout
 *
 * Beta 公式：1/T = 1/T0 + (1/B)·ln(R/R25)，T 单位开尔文。
 */
esp_err_t ntc_temp_read_celsius(float *celsius)
{
    if (celsius == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_adc_unit == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    ESP_RETURN_ON_ERROR(ntc_temp_read_raw(&raw), TAG, "采样失败");

    const int mv  = raw_to_mv(raw);
    const int vcc = CONFIG_NTC_TEMP_VCC_MV;

    /* 开路/短路判据：
     *  - mv 贴着 0 或贴着 Vcc，说明分压中点被拉到了轨上（探头没接、线断了、短路）；
     *  - 12dB 衰减上限约 3.1V，若 Vcc=3.3V，贴着上限也可能是探针温度太低导致饱和。 */
    if (mv < 10 || mv > vcc - 10) {
        ESP_LOGW(TAG, "AO 电压异常：%d mV（raw=%d），检查探头接线与供电", mv, raw);
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float vout = mv / 1000.0f;
    const float vccf = vcc / 1000.0f;
    float r_ntc;

#if CONFIG_NTC_TEMP_NTC_TO_GND
    r_ntc = (float)CONFIG_NTC_TEMP_R_FIXED * vout / (vccf - vout);
#else
    r_ntc = (float)CONFIG_NTC_TEMP_R_FIXED * (vccf - vout) / vout;
#endif

    if (!(r_ntc > 0.0f) || isinf(r_ntc)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float inv_t = 1.0f / NTC_T0_K
                      + logf(r_ntc / (float)CONFIG_NTC_TEMP_R25) / (float)CONFIG_NTC_TEMP_B_VALUE;
    if (inv_t <= 0.0f) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float t_c = 1.0f / inv_t - 273.15f;
    if (t_c < NTC_TEMP_MIN_C || t_c > NTC_TEMP_MAX_C) {
        ESP_LOGW(TAG, "温度超出量程：%.1f℃（R=%.0fΩ, %d mV）", t_c, r_ntc, mv);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 体表温度 → 估算核心温度。 */
    const float t_corr = t_c + (float)CONFIG_NTC_TEMP_OFFSET_X10 / 10.0f;
    if (t_corr < NTC_TEMP_MIN_C || t_corr > NTC_TEMP_MAX_C) {
        ESP_LOGW(TAG, "温度超量程：%.1f℃（原始 %.1f℃，偏移 %+.1f℃）",
                 t_corr, t_c, (float)CONFIG_NTC_TEMP_OFFSET_X10 / 10.0f);
        return ESP_ERR_INVALID_RESPONSE;
    }

    *celsius = t_corr;
    return ESP_OK;
}


esp_err_t ntc_temp_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(adc_setup(), TAG, "ADC 初始化失败");

    /* DO 脚：模块上的比较器输出，超阈值时为低。不接（-1）就不碰。 */
#if (NTC_TEMP_DO_GPIO_IO >= 0)
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << NTC_TEMP_DO_GPIO_IO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "DO 脚配置失败");
    s_do_last = (uint8_t)gpio_get_level(NTC_TEMP_DO_GPIO_IO);
#endif

    s_inited = true;
    ESP_LOGI(TAG, "初始化完成：ADC1_CH%d(GPIO%d)  Rfix=%dΩ R25=%dΩ B=%d 供电=%dmV",
             CONFIG_NTC_TEMP_ADC_CHANNEL, CONFIG_NTC_TEMP_ADC_CHANNEL + 1,
             CONFIG_NTC_TEMP_R_FIXED, CONFIG_NTC_TEMP_R25,
             CONFIG_NTC_TEMP_B_VALUE, CONFIG_NTC_TEMP_VCC_MV);
    return ESP_OK;
}


float ntc_temp_get_celsius(void)
{
    float v;
    portENTER_CRITICAL(&s_state_mux);
    v = s_celsius;
    portEXIT_CRITICAL(&s_state_mux);
    return v;
}

bool ntc_temp_is_valid(void)
{
    bool v;
    portENTER_CRITICAL(&s_state_mux);
    v = s_valid;
    portEXIT_CRITICAL(&s_state_mux);
    return v;
}

void ntc_temp_set_heartbeat_cb(void (*cb)(void))
{
    s_hb_cb = cb;
}


void vTemperature_Task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   fail_cnt  = 0;
    uint32_t   last_log  = 0;

    for (;;) {
        float t = 0.0f;
        if (ntc_temp_read_celsius(&t) == ESP_OK) {
            if (fail_cnt > 0) {
                ESP_LOGI(TAG, "[Temp] 恢复正常");
                fail_cnt = 0;
            }

            portENTER_CRITICAL(&s_state_mux);
            /* 一阶滤波：首读直接取原值，之后滑动 */
            s_celsius = isnan(s_celsius) ? t
                                         : (s_celsius + NTC_TEMP_EMA_ALPHA * (t - s_celsius));
            s_valid   = true;
            portEXIT_CRITICAL(&s_state_mux);

            ESP_LOGD(TAG, "[Temp] %.2f℃", t);

#if (NTC_TEMP_DO_GPIO_IO >= 0)
            /* DO 是"阈值告警"的数字输出，只在跳变时打印，避免刷屏 */
            uint8_t do_now = (uint8_t)gpio_get_level(NTC_TEMP_DO_GPIO_IO);
            if (do_now != s_do_last) {
                s_do_last = do_now;
                ESP_LOGW(TAG, "[Temp] 模块 DO 电平变化：%s（超过电位器设定阈值）",
                         do_now == 0 ? "低=超温" : "高=回到正常");
            }
#endif
        } else {
            fail_cnt++;
            /* 日志限流：5 秒最多一条，否则传感器没接时会把串口刷爆 */
            uint32_t now = esp_log_timestamp();
            if (now - last_log >= 5000) {
                last_log = now;
                ESP_LOGW(TAG, "[Temp] 读取失败，累计 %lu 次", (unsigned long)fail_cnt);
            }

            portENTER_CRITICAL(&s_state_mux);
            s_valid = false;        /* 保留上次温度值，只把有效位清零 */
            portEXIT_CRITICAL(&s_state_mux);
        }

        /* 打卡规则：读成功 → 打卡；连败 3 次 → 判器件离线，也打卡
         * （任务活着，断的是线）；只有"刚失败 1~2 次"这几拍不打卡。
         * 3 × 2000ms = 6s，仍在 8s 超时之内，不会误复位。 */
        if (fail_cnt == 0) {
            s_online = true;
            if (s_hb_cb) {
                s_hb_cb();
            }
        } else if (fail_cnt >= 3) {
            if (s_online) {
                s_online = false;
                ESP_LOGE(TAG, "[Temp] NTC 离线：读数为开路/短路（检查 AO 线与供电）");
            }
            if (s_hb_cb) {
                s_hb_cb();
            }
        }

        /* 周期 2000ms，与原工程一致。NTC 本身响应就要几秒，没必要更快 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_NTC_TEMP_PERIOD_MS));
    }
}

esp_err_t ntc_temp_start_task(void)
{
    if (s_task_up) {
        return ESP_OK;
    }
    if (!s_inited) {
        ESP_RETURN_ON_ERROR(ntc_temp_init(), TAG, "任务启动前初始化失败");
    }

    BaseType_t ret = xTaskCreatePinnedToCore(vTemperature_Task, "ntc_temp",
                                             CONFIG_NTC_TEMP_TASK_STACK, NULL,
                                             CONFIG_NTC_TEMP_TASK_PRIO, NULL,
                                             CONFIG_NTC_TEMP_TASK_CORE);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "采样任务创建失败");
        return ESP_ERR_NO_MEM;
    }
    s_task_up = true;
    return ESP_OK;
}


float ds18b20_get_temperature(void)
{
    return ntc_temp_get_celsius();
}

float DS18B20_GetTemp(void)
{
    return ntc_temp_get_celsius();
}

float DS18B20_ReadTemperature(void)
{
    /* 语义变了：原来要 750ms 转换 + 读两字节，现在是一次 ADC 采样，立即返回。
     * 失败时返回 -999 与原工程的错误哨兵值保持一致，App 层判据不用改。 */
    float t = 0.0f;
    if (ntc_temp_read_celsius(&t) != ESP_OK) {
        return -999.0f;
    }
    return t;
}

void DS18B20_StartConversion(void)
{
    /* NTC 没有"启动转换"这个动作，空实现仅为兼容原调用点。 */
}